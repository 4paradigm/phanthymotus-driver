#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Deterministic G1 award ceremony runner.

Bypasses the LLM: directly calls the G1 driver MCP tools in the fixed order
    announce name -> turn to winner -> clap -> turn back
for each award, plus opening and closing lines. Prints every tool result.

Usage:
    python3 ceremony_run.py "最佳创意奖=张三;最佳团队奖=李四"
    python3 ceremony_run.py "奖项=得主;..." --no-turn     # skip turning
"""
import json
import ssl
import sys
import time
import urllib.error
import urllib.request

TOKEN = "eXFNVNBw4geoffy_T-KZGM1v8hYFlLfk"
BASE = "https://localhost:15678"
MCP = "driver-unitree-g1"
TURN_RATE = 0.5      # rad/s
TURN_TIME = 3.0      # seconds

ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE


def call(tool, args):
    r = urllib.request.Request(
        BASE + "/api/mcp/%s/call" % MCP,
        data=json.dumps({"tool": tool, "arguments": args}).encode(),
        method="POST",
        headers={"Authorization": "Bearer " + TOKEN,
                 "Content-Type": "application/json"})
    try:
        res = json.load(urllib.request.urlopen(r, context=ctx, timeout=40))
        return res["data"][0]["text"]
    except urllib.error.HTTPError as e:
        return {"http_error": e.code, "body": e.read().decode()[:200]}
    except Exception as e:
        return {"error": str(e)}


def speak(text, wait=True):
    out = call("tts", {"action": "speak", "text": text})
    print("  tts    :", out)
    # Let the utterance actually play before the next call; calling speak again
    # too soon silently truncates the previous line on the G1 TTS engine.
    if wait:
        time.sleep(max(1.0, len(text) * 0.20))
    return out


def clap():
    out = call("arm", {"action": "execute", "gesture": "clap"})
    print("  clap   :", out)
    return out


def turn(rate):
    a = call("loco", {"action": "move", "vyaw": rate, "duration": TURN_TIME})
    print("  turn   :", a)
    time.sleep(TURN_TIME + 0.3)
    b = call("loco", {"action": "stop_move"})
    print("  stop   :", b)


def parse(text):
    out = []
    for part in text.replace("，", ";").replace(",", ";").replace("；", ";").split(";"):
        part = part.strip()
        if not part:
            continue
        if "=" in part:
            a, n = part.split("=", 1)
        else:
            a, n = part, part
        out.append((a.strip(), n.strip()))
    return out


def main():
    awards_text = sys.argv[1] if len(sys.argv) > 1 else ""
    do_turn = "--no-turn" not in sys.argv
    awards = parse(awards_text)
    if not awards:
        raise SystemExit("no awards given")

    print("volume :", call("tts", {"action": "set_volume", "volume": 100}))
    time.sleep(0.5)
    speak("颁奖典礼现在开始，请嘉宾就座，我们即将揭晓各个奖项。")
    time.sleep(0.6)
    for i, (award, name) in enumerate(awards, 1):
        print("[%d] %s = %s" % (i, award, name))
        speak("三、二、一")
        speak("%s 的获得者是 %s" % (award, name))
        if do_turn:
            turn(TURN_RATE)          # face winner
        clap()                        # clap after the name
        if do_turn:
            turn(-TURN_RATE)          # turn back
        time.sleep(1)
    speak("颁奖典礼到此结束，感谢大家，明年再见。")


if __name__ == "__main__":
    main()
