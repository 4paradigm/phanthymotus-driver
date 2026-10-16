#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G1 award-ceremony card — a standalone MCP tool server for the canvas.

Exposes ONE tool, `award_ceremony`, that runs the whole per-award sequence
    announce name -> turn to winner -> clap -> turn back
deterministically, by calling the G1 driver MCP tools (tts/arm/loco) over
JSON-RPC. The canvas only has to call this one tool with the award list, so the
order and the wait-for-speech timing are enforced here instead of by the LLM.

Register: the server POSTs itself to agent-core (/api/mcp) and heartbeats, so it
shows up on the canvas as a card.

Run (on the robot host):
    python3 ceremony_card.py            # listens on :15799, registers itself
Tool args:
    awards : "奖项=获奖人;奖项=获奖人;..."   (required)
    turn   : true/false  (default true)  turn body between faces
    action : "run" (default) | "stop" | "status"
"""
import json
import os
import ssl
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

G1_MCP = os.environ.get("G1_MCP_URL", "http://localhost:15701/mcp")  # g1 driver bundle
AGENT_CORE = os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
PORT = 15799
SERVER_NAME = "g1-award-ceremony-card"
TURN_RATE = 0.5      # rad/s
TURN_TIME = 3.0      # seconds
DEFAULT_VOLUME = 60  # 0-100; lower than the robot default so it is not loud

_ctx = ssl.create_default_context()
_ctx.check_hostname = False
_ctx.verify_mode = ssl.CERT_NONE

_run_lock = threading.Lock()
_state = {"state": "idle"}


def g1_call(tool, args, timeout=60):
    body = {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
            "params": {"name": tool, "arguments": args}}
    req = urllib.request.Request(
        G1_MCP, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            res = json.load(r)
    except Exception as e:
        return {"_error": "http: %s" % e}
    if isinstance(res, dict) and "error" in res:
        return {"_error": "rpc: %s" % res.get("error")}
    try:
        txt = res["result"]["content"][0]["text"]
        return json.loads(txt)
    except Exception as e:
        return {"_error": "parse: %s" % e}


def speak(text, wait=True):
    out = g1_call("tts", {"action": "speak", "text": text})
    if wait:
        time.sleep(max(1.0, len(text) * 0.20))
    return out


def turn(rate):
    out = g1_call("loco", {"action": "move", "vyaw": rate, "duration": TURN_TIME})
    _state["last_turn"] = {"vyaw": rate, "raw": out}
    time.sleep(TURN_TIME + 0.3)
    g1_call("loco", {"action": "stop_move"})


def release_teleop():
    """End any active teleop session so this card regains motion ownership.

    The G1 arm/loco tools reject motion with `motion_owned_by_teleop` while a
    teleop (PICO) session holds the arm stream. `teleop_control stop` ends that
    session; it does not move the robot.
    """
    out = g1_call("teleop_control", {"action": "stop"})
    _state["teleop_release"] = out
    return out


def do_gesture(gesture):
    """Execute an arm gesture exactly once.

    The G1 arm action is a blocking RPC and its return code is unreliable: a
    non-zero code (3104/740x) is returned even when the gesture *does* play, so
    retrying would replay the same gesture several times. Call it once and let
    the operator judge by eye.
    """
    last = g1_call("arm", {"action": "execute", "gesture": gesture})
    ret = last.get("ret") if isinstance(last, dict) else None
    _state["last_gesture"] = {"gesture": gesture, "ret": ret, "raw": last}
    return last


def parse_awards(text):
    """Each entry: 奖项=获奖人  |  optional praise after a pipe:  奖项=获奖人|表扬词"""
    out = []
    # Only split entries on semicolons (half/full width) or newlines — NOT on
    # commas, because praise text legitimately contains commas.
    for part in text.replace("；", ";").replace("\n", ";").split(";"):
        part = part.strip()
        if not part:
            continue
        praise = ""
        if "|" in part:
            part, praise = part.split("|", 1)
        award, name = (part.split("=", 1) if "=" in part else (part, part))
        out.append((award.strip(), name.strip(), praise.strip()))
    return out


def run_ceremony(awards_text, do_turn=True, do_wave=True, volume=DEFAULT_VOLUME):
    awards = parse_awards(awards_text)
    if not awards:
        _state.update({"state": "error", "detail": "no awards"})
        return
    _state.update({"state": "running", "step": "release_teleop", "index": 0, "total": len(awards)})
    release_teleop()          # 先结束遥操会话，拿回运动控制权
    time.sleep(0.5)
    _state.update({"step": ""})
    g1_call("tts", {"action": "set_volume", "volume": int(volume)})
    time.sleep(0.4)
    speak("颁奖典礼现在开始，请嘉宾就座，我们即将揭晓各个奖项。")
    if do_wave:
        _state.update({"step": "open_wave"})
        do_gesture("high wave")   # 开场挥手（带重试）
        time.sleep(1.0)
    for i, (award, name, praise) in enumerate(awards, 1):
        if _state.get("stop"):
            break
        _state.update({"index": i, "step": "announce", "current": f"{award}={name}"})
        speak("三、二、一")
        speak("%s 的获得者是 %s" % (award, name))
        if praise:
            _state.update({"step": "praise"})
            speak(praise)
        if do_turn:
            _state.update({"step": "turn"})
            turn(TURN_RATE)
        _state.update({"step": "clap"})
        do_gesture("clap")   # 鼓掌（带重试）
        if do_turn:
            _state.update({"step": "turn_back"})
            turn(-TURN_RATE)
        time.sleep(0.6)
    speak("颁奖典礼到此结束，感谢大家，明年再见。")
    _state.update({"state": "idle", "step": "", "index": 0})


def dispatch(args):
    action = args.get("action", "run")
    # Lifecycle actions sent by agent-core when a project starts/stops.
    if action in ("start", "ping"):
        return {"state": "ready"}
    if action == "stop":
        _state["stop"] = True
        return {"state": "idle"}
    if action == "status":
        return dict(_state)
    if action != "run":
        return {"error": "unknown action: %s" % action}
    if _run_lock.locked():
        return {"state": "busy", "detail": "ceremony already running"}
    awards = args.get("awards", "")
    do_turn = args.get("turn", True)
    do_wave = args.get("wave", True)
    volume = args.get("volume", DEFAULT_VOLUME)
    if isinstance(do_turn, str):
        do_turn = do_turn.strip().lower() not in ("false", "0", "no")
    if isinstance(do_wave, str):
        do_wave = do_wave.strip().lower() not in ("false", "0", "no")
    try:
        volume = int(volume)
    except (TypeError, ValueError):
        volume = DEFAULT_VOLUME
    _state["stop"] = False

    def _worker():
        with _run_lock:
            try:
                run_ceremony(awards, do_turn, do_wave, volume)
            except Exception as e:
                _state.update({"state": "error", "detail": str(e)})

    threading.Thread(target=_worker, daemon=True).start()
    return {"state": "started", "awards": parse_awards(awards), "turn": do_turn}


TOOL = {
    "name": "award_ceremony",
    "type": "actuator",
    "multiInstance": False,
    "description": "G1 颁奖流程卡片：开场播报后挥手，随后每个奖项按 念名字→（表扬词）→转身面向获奖者→鼓掌→转回正前方 "
                   "固定顺序执行。输入奖项列表（可带表扬词）即可整场运行，顺序由本卡片保证，不依赖大模型。",
    "inputSchema": {
        "type": "object",
        "properties": {
            "awards": {"type": "string",
                       "description": "奖项列表：奖项1=获奖人1|表扬词1;奖项2=获奖人2|表扬词2；表扬词可省略。用分号分隔多个奖项。"},
            "turn": {"type": "boolean", "description": "是否在念完名字后转身（默认 true）"},
            "wave": {"type": "boolean", "description": "开场播报后是否挥手（默认 true）"},
            "volume": {"type": "integer", "description": "播报音量 0-100（默认 60，比机器人默认音量小）"},
            "action": {"type": "string", "enum": ["run", "stop", "status", "start", "ping"],
                       "description": "run=开始整场；stop=停止；status=查询进度"},
        },
        "required": ["awards"],
    },
}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _send(self, status, body):
        b = body.encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Accept")
        self.end_headers()
        self.wfile.write(b)

    def do_OPTIONS(self):
        self._send(204, "")

    def do_GET(self):
        self._send(404, "{}")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            rpc = json.loads(self.rfile.read(length))
        except Exception:
            self._send(400, json.dumps({"jsonrpc": "2.0", "id": None,
                                        "error": {"code": -32700, "message": "Parse error"}}))
            return
        rid, method = rpc.get("id"), rpc.get("method", "")
        params = rpc.get("params") or {}
        if rid is None:
            self.send_response(202)
            self.end_headers()
            return

        def ok(result):
            self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid, "result": result}))

        def err(code, msg):
            self._send(200, json.dumps({"jsonrpc": "2.0", "id": rid,
                                        "error": {"code": code, "message": msg}}))
        try:
            if method == "initialize":
                ok({"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                    "serverInfo": {"name": SERVER_NAME, "version": "1.0.0"}})
            elif method == "tools/list":
                ok({"tools": [TOOL]})
            elif method == "tools/call":
                result = dispatch(params.get("arguments") or {})
                ok({"content": [{"type": "text", "text": json.dumps(result, ensure_ascii=False)}]})
            else:
                err(-32601, "Method not found: %s" % method)
        except Exception as e:
            err(-32603, str(e))


def _register():
    payload = json.dumps({"name": "G1 Award Ceremony Card",
                          "url": "http://localhost:%d/mcp" % PORT,
                          "category": "driver"}).encode()
    while True:
        try:
            req = urllib.request.Request(AGENT_CORE + "/api/mcp", data=payload,
                                         headers={"Content-Type": "application/json"}, method="POST")
            urllib.request.urlopen(req, timeout=3, context=_ctx).read()
            time.sleep(30)
        except Exception:
            time.sleep(5)


def main():
    threading.Thread(target=_register, daemon=True, name="register").start()
    print("[award-card] listening on :%d, registering to agent-core" % PORT, flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
