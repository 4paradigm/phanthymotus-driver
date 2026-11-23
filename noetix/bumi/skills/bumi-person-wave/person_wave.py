"""Bumi doorway greeting workflow; reuses vop, wave and TTS/speaker.

check/observe never submit wave or speech; prepare connects the audio route.
Only start/run enable automatic greeting; voice-test speaks once without waving.
Runs inside Agent Core using its existing aiohttp dependency and local API.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
from pathlib import Path
import signal
import ssl
import subprocess
import sys
import time
import uuid
from urllib.parse import quote, urlsplit

SLUG = "bumi-person-wave"
INSTANCE = "skill-bumi-person-wave"
GREETING = "您好，欢迎光临！"
ROOT = Path(__file__).resolve().parent
RUNTIME = ROOT / "runtime"


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


class PresenceGate:
    """One candidate per occupied interval; silence never means an empty scene."""

    def __init__(self, hold=1.0, clear=3.0, cooldown=30.0, max_gap=2.0):
        if not all(finite(x) and x > 0 for x in (hold, clear, cooldown, max_gap)):
            raise ValueError("Gate durations must be finite and positive")
        self.hold, self.clear, self.cooldown, self.max_gap = hold, clear, cooldown, max_gap
        self.candidate = self.absent = self.previous = None
        self.latched = False
        self.last_attempt = -math.inf

    def update(self, visible, now):
        if type(visible) is not bool or not finite(now):
            raise ValueError("Expected boolean visibility and finite observation time")
        if self.previous is not None and now <= self.previous:
            return False
        if self.previous is not None and now - self.previous > self.max_gap:
            self.candidate = self.absent = None
        self.previous = now
        if visible:
            self.absent = None
            if self.latched:
                return False
            if self.candidate is None:
                self.candidate = now
                return False
            if now - self.candidate < self.hold:
                return False
            self.latched = True
            return now - self.last_attempt >= self.cooldown
        self.candidate = None
        if self.latched:
            if self.absent is None:
                self.absent = now
            elif now - self.absent >= self.clear:
                self.latched = False
                self.absent = None
        return False

    def attempted(self, now):
        self.last_attempt = now


def person_visible(payload, wall_now, confidence=0.5):
    if not isinstance(payload, dict) or not isinstance(payload.get("objects"), list):
        raise ValueError("Expected the vop objects payload")
    stamp = payload.get("timestamp")
    if not finite(stamp) or not -0.5 <= wall_now - stamp <= 2.0:
        raise ValueError("Person observation is stale or has no valid timestamp")
    for item in payload["objects"]:
        if not isinstance(item, dict):
            raise ValueError("Invalid object observation")
        if str(item.get("name", "")).strip().casefold() != "person":
            continue
        score = item.get("confidence")
        if not finite(score) or not 0 <= score <= 1:
            raise ValueError("Person confidence is invalid")
        if score >= confidence:
            return True
    return False


def motion_ready(payload, age):
    if not isinstance(payload, dict) or not 0 <= age <= 2:
        return False
    mode = payload.get("workmode")
    return (payload.get("fresh") is True and isinstance(mode, dict)
            and mode.get("code") == 2 and mode.get("protection") is False
            and payload.get("activity") == "stationary"
            and payload.get("motor_faults") == [])


def audio_topics(info, direction):
    return {p["topic"] for p in info.get(direction, [])
            if p.get("format") == "audio/pcm-16k" and p.get("topic")}


def voice_route_ready(tts, speaker, mixer=None):
    outputs = audio_topics(tts, "topic_out")
    if tts.get("state") != "running" or speaker.get("state") not in ("running", "playing") or len(outputs) != 1:
        return False
    inputs = audio_topics(speaker, "topic_in")
    if outputs & inputs:
        return True
    return bool(mixer and mixer.get("state") == "running"
                and outputs & audio_topics(mixer, "topic_in")
                and inputs & audio_topics(mixer, "topic_out"))


def unpack(response):
    if not isinstance(response, dict) or response.get("code") != 200:
        raise RuntimeError("API request failed: " + str(response.get("message", "invalid response") if isinstance(response, dict) else "invalid response"))
    data = response.get("data")
    if isinstance(data, list):
        texts = [x.get("text") for x in data if isinstance(x, dict) and x.get("type") == "text"]
        if len(texts) != 1:
            raise RuntimeError("Expected one MCP business result")
        data = json.loads(texts[0])
    elif isinstance(data, str):
        data = json.loads(data)
    if not isinstance(data, dict) or data.get("ok") is False or data.get("state") == "error":
        raise RuntimeError("Tool reported failure: " + str(data))
    return data


def read_token():
    token = os.environ.get("PHANTHY_ACCESS_TOKEN", "")
    if token:
        return token
    # Same source and parsing as the running app's auth.py. Never copied to logs.
    path = Path("/opt/phanthy-motus/.env")
    if path.is_file():
        for line in path.read_text().splitlines():
            line = line.strip()
            if line.startswith("ACCESS_TOKEN="):
                return line.split("=", 1)[1].strip()
    raise RuntimeError("Dashboard authentication is unavailable")


def read_state():
    try:
        return json.loads((RUNTIME / "state.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def own_process(pid):
    if type(pid) is not int or pid <= 1 or os.name != "posix":
        return False
    try:
        parts = Path("/proc/%d/cmdline" % pid).read_bytes().split(b"\0")
        return os.fsencode(str(Path(__file__).resolve())) in parts and b"run" in parts
    except OSError:
        return False


def write_state(state):
    RUNTIME.mkdir(parents=True, exist_ok=True)
    state["heartbeat"] = time.time()
    tmp = RUNTIME / ("state.%d.tmp" % os.getpid())
    tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(RUNTIME / "state.json")


class Client:
    def __init__(self, base, token):
        parts = urlsplit(base)
        if parts.scheme != "https" or parts.hostname not in ("127.0.0.1", "localhost", "::1") or parts.query or parts.fragment or parts.username:
            raise ValueError("This workflow only uses the local HTTPS API")
        self.base, self.token, self.session = base.rstrip("/"), token, None

    async def __aenter__(self):
        import aiohttp
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        self.session = aiohttp.ClientSession(
            connector=aiohttp.TCPConnector(ssl=context),
            headers={"Authorization": "Bearer " + self.token},
            timeout=aiohttp.ClientTimeout(total=15))
        return self

    async def __aexit__(self, *args):
        await self.session.close()
        # Let TLS transports close before asyncio.run tears down the event loop.
        await asyncio.sleep(0.25)

    async def json(self, method, path, body=None):
        async with self.session.request(method, self.base + path, json=body, allow_redirects=False) as response:
            if response.status != 200:
                raise RuntimeError("API HTTP %d" % response.status)
            return await response.json()

    async def call(self, mcp, tool, args):
        return unpack(await self.json("POST", "/api/mcp/" + quote(mcp, safe="") + "/call",
                                      {"tool": tool, "arguments": args}))

    async def discover(self):
        registry = await self.json("GET", "/api/mcp")
        if registry.get("code") != 200:
            raise RuntimeError("MCP registry is unavailable")
        devices = registry.get("data", [])
        robots = [x for x in devices if any(t.get("name") == "semantic_action" and "wave" in t.get("inputSchema", {}).get("properties", {}).get("action", {}).get("enum", []) for t in x.get("tools", []))]
        detectors = [x for x in devices if any(t.get("name") == "vop" for t in x.get("tools", []))]
        if len(robots) != 1 or len(detectors) != 1:
            raise RuntimeError("Need exactly one semantic_action.wave provider and one running vop service")
        robot, detector = robots[0], detectors[0]
        def topic(tool_name, fmt):
            values = [p["topic"] for t in robot["tools"] if t.get("name") == tool_name for p in t.get("topic_out", []) if p.get("format") == fmt and p.get("topic")]
            if len(values) != 1:
                raise RuntimeError("Cannot resolve " + tool_name + " data topic")
            return values[0]
        info = await self.call(detector["id"], "vop", {"action": "info", "instance_id": INSTANCE, "input_topic": topic("camera", "image/jpeg")})
        return {"robot": robot["id"], "detector": detector["id"], "camera": topic("camera", "image/jpeg"),
                "motion": topic("motion_state", "data/json"), "detector_info": info}

    async def conditions(self, robot):
        running = await self.json("GET", "/api/config/project-running")
        if running.get("running") is not True:
            raise RuntimeError("Intelligent control is stopped")
        skill = await self.json("GET", "/api/skills/" + SLUG)
        if skill.get("code") != 200 or skill.get("data", {}).get("active") is not True:
            raise RuntimeError("Greeting skill is disabled or uninstalled")
        canvas = (await self.json("GET", "/api/canvas/layout")).get("data", {})
        controllers = {c["id"] for c in canvas.get("cards", []) if c.get("toolName") == "decision_core"}
        if not any(c.get("fromCardId") in controllers and c.get("toMcpId") == robot and c.get("toToolName") == "semantic_action" for c in canvas.get("execConnections", [])):
            raise RuntimeError("Decision core is not bound to semantic_action")

    async def voice_info(self, robot):
        registry = await self.json("GET", "/api/mcp")
        devices = registry.get("data", [])
        providers = [x for x in devices if any(t.get("name") == "tts" for t in x.get("tools", []))]
        if len(providers) != 1:
            raise RuntimeError("Need exactly one TTS provider for the welcome phrase")
        provider = providers[0]
        tts = await self.call(provider["id"], "tts", {"action": "info"})
        speaker = await self.call(robot, "speaker", {"action": "info"})
        mixer = None
        if any(t.get("name") == "audio_mixer" for t in provider.get("tools", [])):
            mixer = await self.call(provider["id"], "audio_mixer", {"action": "info"})
        return {"provider": provider["id"], "tts": tts, "speaker": speaker,
                "ready": voice_route_ready(tts, speaker, mixer)}

    async def prepare_voice(self, robot):
        voice = await self.voice_info(robot)
        if voice["tts"].get("state") != "running":
            await self.call(voice["provider"], "tts", {"action": "start"})
            for _ in range(180):
                voice = await self.voice_info(robot)
                state = voice["tts"].get("state")
                if state == "running":
                    break
                if state in ("error", "failed"):
                    raise RuntimeError("Welcome TTS model failed to prepare")
                await asyncio.sleep(1)
            else:
                raise RuntimeError("Welcome TTS preparation timed out")
        if voice["speaker"].get("state") == "idle":
            outputs = audio_topics(voice["tts"], "topic_out")
            if len(outputs) != 1:
                raise RuntimeError("Cannot resolve one TTS output for the robot speaker")
            await self.call(robot, "speaker", {"action": "start", "input_topic": next(iter(outputs))})
            voice = await self.voice_info(robot)
        if not voice["ready"]:
            raise RuntimeError("Connect the running TTS output to the robot speaker; existing active audio routing was preserved")
        return voice

    async def speak_greeting(self, robot):
        voice = await self.voice_info(robot)
        if not voice["ready"]:
            raise RuntimeError("Welcome speech output is not ready; run prepare")
        result = await self.call(voice["provider"], "tts", {"action": "speak", "text": GREETING})
        if result.get("status") != "queued" or not result.get("action_id"):
            raise RuntimeError("Welcome speech was not confirmed queued; no retry sent")
        return result

    async def reader(self, topic, frames, stopped):
        import aiohttp
        while not stopped.is_set():
            first = True
            try:
                async with self.session.ws_connect(self.base.replace("https:", "wss:", 1) + "/ws/bus" + topic,
                                                   params={"token": self.token}, heartbeat=10, receive_timeout=10) as ws:
                    async for message in ws:
                        if stopped.is_set():
                            return
                        if message.type not in (aiohttp.WSMsgType.TEXT, aiohttp.WSMsgType.BINARY):
                            break
                        item = json.loads(message.data)
                        if item.get("type") in ("meta", "ping"):
                            continue
                        if item.get("type") == "error":
                            raise RuntimeError("Data topic is unavailable")
                        if first:
                            first = False  # May be the server's old cached snapshot.
                            continue
                        frames[topic] = (time.monotonic(), item)
            except Exception:
                frames.pop(topic, None)
                if not stopped.is_set():
                    await asyncio.sleep(1)


async def prepare(client):
    info = await client.discover()
    await client.call(info["detector"], "vop", {"action": "config", "instance_id": INSTANCE,
                                             "confidence": 0.5, "fps": 5})
    await client.call(info["detector"], "vop", {"action": "start", "instance_id": INSTANCE, "input_topic": info["camera"]})
    for _ in range(180):
        result = await client.discover()
        state = result["detector_info"].get("state")
        if state == "running":
            await client.prepare_voice(result["robot"])
            return result
        if state in ("error", "failed"):
            raise RuntimeError("Detector failed: " + str(result["detector_info"].get("error", "unknown")))
        await asyncio.sleep(1)
    raise RuntimeError("Detector preparation timed out; inspect vop status")


def detection_topic(info):
    outputs = [p["topic"] for p in info["detector_info"].get("topic_out", []) if p.get("format") == "data/json" and p.get("topic")]
    if info["detector_info"].get("state") != "running" or len(outputs) != 1:
        raise RuntimeError("Prepare the vop camera instance first")
    return outputs[0]


async def monitor(client, seconds, live):
    info = await client.discover()
    people_topic = detection_topic(info)
    if live:
        await client.conditions(info["robot"])
        if not (await client.voice_info(info["robot"]))["ready"]:
            raise RuntimeError("Prepare welcome speech before starting greeting")
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    if live:
        for sig in (signal.SIGTERM, signal.SIGINT):
            loop.add_signal_handler(sig, stopped.set)
    frames, readers = {}, []
    for topic in (people_topic, info["motion"]):
        readers.append(asyncio.create_task(client.reader(topic, frames, stopped)))
    gate = PresenceGate()
    old = read_state() if live else {}
    last_wall = old.get("last_wave_wall", 0)
    if finite(last_wall) and last_wall > 0:
        gate.last_attempt = time.monotonic() - max(0, time.time() - last_wall)
    state = {"pid": os.getpid(), "running": live, "phase": "waiting_for_data", "session": uuid.uuid4().hex,
             "last_wave_wall": last_wall, "dry_run": not live, "people_topic": people_topic, "motion_topic": info["motion"]}
    started, next_check, last_frame = time.monotonic(), 0.0, None
    pending_since = None
    last_source_stamp = -math.inf
    diagnostics = {"fresh_person_observation": False, "fresh_motion_observation": False, "person_seen": False, "motion_ready": False, "wave_was_called": False, "speech_was_queued": False}
    try:
        while not stopped.is_set() and (seconds == 0 or time.monotonic() - started < seconds):
            now = time.monotonic()
            if live and now >= next_check:
                await client.conditions(info["robot"])
                write_state(state)
                next_check = now + 1
            motion = frames.get(info["motion"])
            ready = bool(motion and motion_ready(motion[1], now - motion[0]))
            diagnostics["fresh_motion_observation"] = bool(motion and now - motion[0] <= 2)
            diagnostics["motion_ready"] = ready
            if pending_since is not None:
                mode = motion[1].get("workmode", {}) if motion and now - motion[0] <= 2 else {}
                if ready:
                    pending_since = None
                    state["phase"] = "waiting_for_departure"
                elif now - pending_since > 20:
                    raise RuntimeError("Wave completion was not confirmed within 20 seconds; no repeat sent")
                elif mode.get("code") not in (2, 8, None):
                    raise RuntimeError("Robot mode changed during greeting; workflow stopped")
            frame = frames.get(people_topic)
            if frame is not None and frame[0] != last_frame:
                last_frame = frame[0]
                try:
                    visible = person_visible(frame[1], time.time())
                    stamp = frame[1]["timestamp"]
                    if stamp <= last_source_stamp:
                        await asyncio.sleep(0.1)
                        continue
                    last_source_stamp = stamp
                except ValueError:
                    gate.candidate = gate.absent = None
                    diagnostics["fresh_person_observation"] = False
                else:
                    diagnostics["fresh_person_observation"] = True
                    diagnostics["person_seen"] = diagnostics["person_seen"] or visible
                    candidate = gate.update(visible, now)
                    if candidate:
                        if live and ready and pending_since is None:
                            # Reserve before sending; a timeout must never produce a retry storm.
                            gate.attempted(now)
                            state.update(last_wave_wall=time.time(), phase="submitting_wave")
                            write_state(state)
                            result = await client.call(info["robot"], "semantic_action", {"action": "wave"})
                            diagnostics["wave_was_called"] = True
                            if result.get("command_sent") is not True or result.get("confirmed_started") is not True:
                                raise RuntimeError("Wave result is unconfirmed; stopped without retry")
                            pending_since = time.monotonic()
                            # Forget the pre-command standing snapshot. Await new telemetry.
                            frames.pop(info["motion"], None)
                            state.update(phase="waving", last_action_result="confirmed_started")
                            state["last_speech_status"] = "submitting"
                            write_state(state)
                            speech = await client.speak_greeting(info["robot"])
                            diagnostics["speech_was_queued"] = True
                            state.update(last_speech_status="queued", last_speech_action_id=speech["action_id"],
                                         greeting_text=GREETING)
                            write_state(state)
                        else:
                            state["phase"] = "person_confirmed_no_action" if not live else "blocked_by_motion_state"
            await asyncio.sleep(0.1)
    except Exception as error:
        state["error"] = str(error).replace(client.token, "[redacted]").replace(quote(client.token, safe=""), "[redacted]")
        raise
    finally:
        stopped.set()
        for task in readers:
            task.cancel()
        await asyncio.gather(*readers, return_exceptions=True)
        if live:
            state.update(running=False, phase="error" if "error" in state else "stopped", wave_may_still_be_finishing=pending_since is not None)
            write_state(state)
    diagnostics["wave_was_called"] = bool(diagnostics["wave_was_called"])
    return diagnostics


async def async_main(args):
    token = read_token()
    try:
        async with Client(args.base_url, token) as client:
            if args.command == "check":
                info = await client.discover()
                voice = await client.voice_info(info["robot"])
                return {"ok": True, "robot": info["robot"], "detector": info["detector"],
                        "detector_state": info["detector_info"].get("state"), "motion_topic": info["motion"],
                        "voice_ready": voice["ready"], "greeting_text": GREETING, "wave_was_called": False, "speech_was_queued": False}
            if args.command == "prepare":
                info = await prepare(client)
                return {"ok": True, "people_topic": detection_topic(info), "voice_ready": True, "wave_was_called": False, "speech_was_queued": False}
            if args.command == "voice-test":
                info = await client.discover()
                speech = await client.speak_greeting(info["robot"])
                return {"ok": True, "speech_was_queued": True, "wave_was_called": False,
                        "action_id": speech["action_id"], "greeting_text": GREETING}
            if args.command == "start":
                info = await client.discover()
                await client.conditions(info["robot"])
                detection_topic(info)
                if not (await client.voice_info(info["robot"]))["ready"]:
                    raise RuntimeError("Prepare welcome speech before starting greeting")
                if own_process(read_state().get("pid")):
                    return {"ok": True, "already_running": True}
                RUNTIME.mkdir(parents=True, exist_ok=True)
                with (RUNTIME / "runner.log").open("ab") as log:
                    child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "run", "--base-url", args.base_url, "--seconds", str(args.seconds)],
                                             stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                             start_new_session=True)
                await asyncio.sleep(0.6)
                if child.poll() is not None:
                    raise RuntimeError("Workflow did not remain running; inspect runner.log")
                return {"ok": True, "pid": child.pid, "running": True}
            return await monitor(client, args.seconds if args.command == "run" else (args.seconds or 10), args.command == "run")
    except Exception as error:
        # WebSocket errors may contain a query-string credential: never log it.
        message = str(error).replace(token, "[redacted]").replace(quote(token, safe=""), "[redacted]")
        raise RuntimeError(message) from None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check", "prepare", "observe", "voice-test", "start", "run", "status", "stop"))
    parser.add_argument("--base-url", default="https://127.0.0.1:15678")
    parser.add_argument("--seconds", type=int, default=0)
    args = parser.parse_args()
    if args.seconds < 0:
        parser.error("--seconds cannot be negative")
    try:
        if args.command in ("status", "stop"):
            state = read_state()
            active = own_process(state.get("pid"))
            if args.command == "stop" and active:
                os.kill(state["pid"], signal.SIGTERM)
            print(json.dumps({"running": active, "stop_requested": args.command == "stop" and active, "state": state}, ensure_ascii=False, indent=2))
            return 0
        lock = None
        if args.command in ("start", "run"):
            if os.name != "posix":
                raise RuntimeError("Run live greeting inside the Linux Agent Core container")
            if args.command == "run":
                import fcntl
                RUNTIME.mkdir(parents=True, exist_ok=True)
                lock = (RUNTIME / "runner.lock").open("a")
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            result = asyncio.run(async_main(args))
            print(json.dumps(result, ensure_ascii=False, indent=2))
        finally:
            if lock:
                lock.close()
        return 0
    except Exception as error:
        print(json.dumps({"ok": False, "error": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
