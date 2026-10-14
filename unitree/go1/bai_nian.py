#!/usr/bin/env python3
"""
bai_nian.py — Go1 拜年讨红包卡(actuator)。

「检测到人 → 原地作揖拜年 + 头部扬声器播报『恭喜发财，红包拿来！』」一体化表演卡。

  detect_once  抓一帧相机 JPEG → OpenCV HOG 行人检测（内置模型，离线）
  greet        原地拜年：前倾作揖 ×greet_times + 并行播报预生成 PCM
  arm/disarm   自动守护：循环检测，检测到人且过冷却期即自动 greet（arm 须 confirm=true）
  start/stop   生命周期（start 加载播报音频；stop 停守护/打断拜年并回正站立）
  info         状态：守护开关 / 最近检测 / 最近拜年 / 冷却剩余 / 音频就绪

实现要点：
- 取帧协议与 vision_capture 相同：TCP 连 Nano rgb_stream（:9201~9205），
  读 [4B 大端长度][JPEG] 一帧即断；经 camera._CAMERA_LOCK 与推流/拍照互斥。
- 播报走 speaker_adapter v1 HTTP base64 play（:18083 /v1/speaker/actions），
  PCM 为 16k/mono/S16LE（resource/gongxi_facai_16k.pcm）。
- 拜年动作用共享 SDK client 的 set_posture 原语（幅度与 gesture.greet 一致：
  pitch 0.4 / bodyHeight -0.05，smoothstep 插值 50Hz），不依赖 gesture 卡。
- 降级：cv2 缺失 → detect 返回 NO_DEPENDENCY；STUB（无 fresh HighState）→
  arm/greet 拒绝执行；speaker 不可达 → 仅拜年，播报失败记入结果不致命。
"""

from __future__ import annotations

import base64
import json
import socket
import struct
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from uuid import uuid4

try:
    import cv2
    import numpy as np
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

if __package__:
    from . import camera
else:
    import camera


CARD_BAI_NIAN = "bai_nian"
TYPE_BAI_NIAN = "actuator"
CONTROL_LEVEL_BN = "HIGHLEVEL"

M_FORCE_STAND_BN = 1
_ATT_MAX_BN = 0.55
_H_MAX_BN = 0.12
_DETECTION_MAX_WIDTH = 640
_AUDIO_SAMPLE_RATE = 16000
_AUDIO_CHANNELS = 1
_SPEAK_HTTP_TIMEOUT = 15.0
_JOIN_SLICE_S = 0.2

DESC_BN = (
    "Go1 拜年讨红包 — HOG 人形检测到人后原地作揖拜年并播报『恭喜发财，红包拿来！』。"
    "detect_once 单次检测(离线 OpenCV HOG)；greet 立即拜年+播报；"
    "arm 启动自动守护(循环检测,须 confirm=true)；disarm 停守护；"
    "stop 打断并回正站立。前置：狗须已站立、front 机位未被推流占用。"
)


def _ms_bn():
    return int(time.time() * 1000)


def _err_bn(code, message, **extra):
    return {"ok": False, "code": code, "message": message, **extra}


def _clamp_bn(v, lim):
    return max(-lim, min(lim, float(v)))


def _iou_bn(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy
    if inter <= 0.0:
        return 0.0
    return inter / (aw * ah + bw * bh - inter)


def _detect_people(jpeg, min_confidence):
    """JPEG → HOG 行人检测；返回按置信度降序的人框（归一化 x/y/w/h，0~1）。"""
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("JPEG decode failed")
    src_h, src_w = img.shape[:2]
    if src_w > _DETECTION_MAX_WIDTH:
        scale = _DETECTION_MAX_WIDTH / src_w
        img = cv2.resize(img, (int(src_w * scale), int(src_h * scale)),
                         interpolation=cv2.INTER_AREA)
    rh, rw = img.shape[:2]
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor.getDefaultPeopleDetector())
    rects, weights = hog.detectMultiScale(img, winStride=(8, 8),
                                          padding=(4, 4), scale=1.05)
    people = []
    for (x, y, w, h), weight in zip(rects, weights):
        conf = float(np.atleast_1d(weight)[0])
        if conf < min_confidence:
            continue
        people.append({"box": [round(x / rw, 3), round(y / rh, 3),
                               round(w / rw, 3), round(h / rh, 3)],
                       "confidence": round(conf, 3)})
    people.sort(key=lambda p: p["confidence"], reverse=True)
    kept = []
    for cand in people:
        if all(_iou_bn(cand["box"], k["box"]) <= 0.5 for k in kept):
            kept.append(cand)
    return kept


class BaiNianPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        cfg = plugin_config or {}

        self._position = cfg.get("camera_position", "front")
        if self._position not in camera._VALID_POSITIONS:
            raise ValueError("camera_position must be one of %s" % (camera._VALID_POSITIONS,))
        self._endpoints = {
            position: (endpoint["board_ip"], int(endpoint["image_port"]))
            for position, endpoint in camera._resolve_positions_raw(cfg).items()
        }
        self._interval_s = max(0.2, float(cfg.get("detect_interval_s", 1.0)))
        self._min_confidence = float(cfg.get("min_confidence", 0.5))
        self._close_ratio = float(cfg.get("close_ratio", 0.45))
        self._greet_times = max(1, min(4, int(cfg.get("greet_times", 2))))
        self._cooldown_s = max(0.0, float(cfg.get("cooldown_s", 30.0)))
        self._stabilize_timeout_s = float(cfg.get("stabilize_timeout_s", 3.0))
        self._speaker_host = cfg.get("speaker_host", "192.168.123.13")
        self._speaker_port = int(cfg.get("speaker_port", 18083))
        volume = cfg.get("volume_percent", None)
        self._volume_percent = None if volume is None else max(0, min(100, int(volume)))
        pcm_path = Path(cfg.get("audio_pcm", "resource/gongxi_facai_16k.pcm"))
        if not pcm_path.is_absolute():
            pcm_path = Path(__file__).resolve().parent / pcm_path
        self._audio_path = pcm_path
        self._audio_bytes = None

        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._auto_cancel = threading.Event()
        self._auto_thread = None
        self._cur = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "h": 0.0}
        self._last_greet_monotonic = None
        self._last_greet = None
        self._last_speak = None
        self._last_detection = None

    # ── 生命周期 ────────────────────────────────────────────────────────────

    def start(self):
        self._audio_bytes = self._load_audio()
        return {"state": "ready"}

    def stop(self):
        self._auto_cancel.set()
        self._cancel.set()
        thread = self._auto_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._auto_thread = None
        try:
            self._client.stop_move()
            self._client.set_posture(M_FORCE_STAND_BN)
        except Exception:
            pass
        self._cancel.clear()
        return {"state": "idle"}

    def _load_audio(self):
        try:
            pcm = self._audio_path.read_bytes()
        except OSError as exc:
            print(f"[{CARD_BAI_NIAN}] audio not loaded ({exc}); greet will bow without speech",
                  flush=True)
            return None
        if not pcm or len(pcm) % 2 != 0:
            print(f"[{CARD_BAI_NIAN}] audio invalid (empty or not 16-bit aligned): {self._audio_path}",
                  flush=True)
            return None
        print(f"[{CARD_BAI_NIAN}] audio loaded: {self._audio_path.name} "
              f"({len(pcm)} bytes ≈ {len(pcm) / 2 / _AUDIO_SAMPLE_RATE:.1f}s)", flush=True)
        return pcm

    # ── 工具声明 ────────────────────────────────────────────────────────────

    def get_tool(self):
        return {
            "name": CARD_BAI_NIAN, "type": TYPE_BAI_NIAN, "multiInstance": False,
            "description": DESC_BN,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "arm", "disarm",
                                        "detect_once", "greet", "info"],
                               "description": "要执行的动作"},
                    "confirm": {"type": "boolean",
                                "description": "arm 启动自动守护须传 true（机器人将自主检测并作揖）"},
                    "position": {"type": "string", "enum": list(camera._VALID_POSITIONS),
                                 "description": "detect_once 检测机位；省略时用卡配置 camera_position"},
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": [], "description": "生命周期：就绪（加载播报音频）"},
                    "stop": {"params": [], "description": "停止自动守护、打断进行中的拜年并回正站立"},
                    "arm": {"params": ["confirm"],
                            "description": "启动自动守护：循环检测，检测到人即拜年+播报（须 confirm=true）"},
                    "disarm": {"params": [], "description": "停止自动守护（不打断进行中的拜年）"},
                    "detect_once": {"params": ["position"],
                                    "description": "抓一帧做 HOG 人形检测，返回 person_detected/count/people"},
                    "greet": {"params": [], "description": "不检测，立即原地拜年+播报一轮"},
                    "info": {"params": [], "description": "状态：守护开关/最近检测/最近拜年/冷却剩余/音频就绪"},
                },
            },
        }

    # ── 取帧（协议同 vision_capture）────────────────────────────────────────

    @staticmethod
    def _receive_exact(connection, size, deadline):
        chunks = bytearray()
        while len(chunks) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("camera frame deadline exceeded")
            connection.settimeout(remaining)
            chunk = connection.recv(size - len(chunks))
            if not chunk:
                raise OSError("camera stream closed before a complete JPEG arrived")
            chunks.extend(chunk)
        return bytes(chunks)

    def _capture_jpeg(self, position):
        host, port = self._endpoints[position]
        # 连接会让 Nano 按需开启相机；读完一帧即断开并释放机位。
        with socket.create_connection((host, port), timeout=8) as connection:
            deadline = time.monotonic() + 20
            length = struct.unpack(">I", self._receive_exact(connection, 4, deadline))[0]
            if not 0 < length <= 5_000_000:
                raise OSError(f"invalid JPEG frame length: {length}")
            return self._receive_exact(connection, length, deadline)

    # ── 检测 ────────────────────────────────────────────────────────────────

    def _detect_internal(self, position):
        if not _HAS_CV2:
            return _err_bn("NO_DEPENDENCY",
                           "cv2 不可用：容器内 pip install opencv-python-headless")
        with camera._CAMERA_LOCK:
            if position in camera._SNAPSHOT_POSITIONS or camera.running_stream(position) is not None:
                return _err_bn("RESOURCE_BUSY",
                               f"camera position {position!r} is occupied; stop its stream first")
            camera._SNAPSHOT_POSITIONS.add(position)
        try:
            jpeg = self._capture_jpeg(position)
        except OSError as exc:
            return _err_bn("CAMERA_UNAVAILABLE", f"cannot capture from {position}: {exc}")
        finally:
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard(position)
        if not jpeg or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            return _err_bn("CAMERA_UNAVAILABLE", f"invalid JPEG from {position}")
        try:
            people = _detect_people(jpeg, self._min_confidence)
        except Exception as exc:
            return _err_bn("DETECT_FAILED", f"HOG detection failed: {exc}")
        result = {
            "ok": True, "card": CARD_BAI_NIAN, "action": "detect_once",
            "control_level": CONTROL_LEVEL_BN,
            "position": position,
            "person_detected": bool(people),
            "count": len(people),
            "people": people,
            "max_confidence": people[0]["confidence"] if people else 0.0,
            "close_enough": bool(people) and max(p["box"][3] for p in people) >= self._close_ratio,
            "timestamp_ms": _ms_bn(),
        }
        self._last_detection = result
        return result

    # ── 播报（speaker_adapter v1 HTTP base64 play）──────────────────────────

    def _speaker_request(self, payload):
        url = "http://%s:%s/v1/speaker/actions" % (self._speaker_host, self._speaker_port)
        request = urllib.request.Request(
            url, data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(request, timeout=_SPEAK_HTTP_TIMEOUT) as response:
            body = response.read().decode("utf-8", "replace")
        return json.loads(body) if body else {}

    def _speak(self):
        """播报预生成 PCM；返回结果 dict。音频缺失/适配器不可达不致命。"""
        pcm = self._audio_bytes
        if not pcm:
            result = _err_bn("AUDIO_UNAVAILABLE", "audio not loaded (start first / file missing)")
        else:
            try:
                if self._volume_percent is not None:
                    self._speaker_request({"card": CARD_BAI_NIAN, "action": "set_volume",
                                           "volume_percent": self._volume_percent,
                                           "request_id": uuid4().hex})
                result = self._speaker_request({
                    "card": CARD_BAI_NIAN, "action": "play",
                    "pcm_base64": base64.b64encode(pcm).decode("ascii"),
                    "sample_rate": _AUDIO_SAMPLE_RATE, "channels": _AUDIO_CHANNELS,
                    "request_id": uuid4().hex})
                if not result.get("ok", False):
                    result = _err_bn(result.get("code", "SPEAK_FAILED"),
                                     result.get("message", "speaker adapter rejected play"))
            except Exception as exc:
                result = _err_bn("SPEAK_FAILED", f"speaker adapter unreachable: {exc}")
        self._last_speak = {"ok": bool(result.get("ok")),
                            "code": result.get("code"),
                            "message": result.get("message"),
                            "timestamp": datetime.now().astimezone().isoformat(timespec="seconds")}
        return result

    # ── 拜年动作（set_posture 原语，幅度同 gesture.greet）────────────────────

    def _glide(self, pitch=0.0, roll=0.0, yaw=0.0, h=0.0, dur=0.5, hz=50):
        tgt = {"roll": _clamp_bn(roll, _ATT_MAX_BN), "pitch": _clamp_bn(pitch, _ATT_MAX_BN),
               "yaw": _clamp_bn(yaw, _ATT_MAX_BN), "h": _clamp_bn(h, _H_MAX_BN)}
        start = dict(self._cur)
        steps = max(1, int(dur * hz))
        for i in range(1, steps + 1):
            if self._cancel.is_set():
                return False
            t = i / steps
            e = t * t * (3.0 - 2.0 * t)
            r = start["roll"] + (tgt["roll"] - start["roll"]) * e
            p = start["pitch"] + (tgt["pitch"] - start["pitch"]) * e
            y = start["yaw"] + (tgt["yaw"] - start["yaw"]) * e
            hh = start["h"] + (tgt["h"] - start["h"]) * e
            self._client.set_posture(M_FORCE_STAND_BN, euler=(r, p, y), body_height=hh)
            time.sleep(1.0 / hz)
        self._cur = tgt
        return True

    def _hold(self, seconds):
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            if self._cancel.is_set():
                return False
            time.sleep(0.03)
        return True

    def _bow(self):
        for _ in range(self._greet_times):
            if not self._glide(pitch=0.4, h=-0.05, dur=0.6):
                return False
            if not self._hold(0.5):
                return False
            if not self._glide(0, 0, 0, 0, dur=0.5):
                return False
        return True

    def _require_live(self):
        snap = self._client.snapshot()
        if not snap.get("fresh"):
            return _err_bn("STUB_MODE",
                           "no fresh HighState (SDK unavailable / STUB); refuse bai_nian motion")
        return None

    def _wait_stand(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._client.snapshot().get("mode") == M_FORCE_STAND_BN:
                return True
            time.sleep(0.05)
        return self._client.snapshot().get("mode") == M_FORCE_STAND_BN

    def _perform_greet(self, source):
        steps = {}
        client = self._client
        client.set_posture(M_FORCE_STAND_BN)
        stood = self._wait_stand(self._stabilize_timeout_s)
        steps["balance_stand_confirmed"] = stood
        if not stood:
            client.set_posture(M_FORCE_STAND_BN)
            return {"ok": False, "code": "PRECONDITION_FAILED",
                    "message": (f"robot did not reach balance_stand (mode=1) within "
                                f"{self._stabilize_timeout_s}s; bai_nian aborted"),
                    "card": CARD_BAI_NIAN, "action": "greet",
                    "control_level": CONTROL_LEVEL_BN, "steps": steps,
                    "timestamp_ms": _ms_bn()}

        speech_box = {}

        def _speak_worker():
            speech_box["result"] = self._speak()

        speak_thread = threading.Thread(target=_speak_worker, daemon=True,
                                       name="bai_nian_speak")
        speak_thread.start()
        completed = self._bow()
        interrupted = self._cancel.is_set()
        if not interrupted:
            self._glide(0, 0, 0, 0, dur=0.4)
            client.set_posture(M_FORCE_STAND_BN)
            self._cur = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "h": 0.0}

        audio_s = (len(self._audio_bytes) / 2 / _AUDIO_SAMPLE_RATE) if self._audio_bytes else 0.0
        speak_thread.join(timeout=audio_s + 10.0)
        speech = speech_box.get("result") or {"ok": False, "code": "SPEAK_PENDING",
                                             "message": "speech still playing (or not started)"}

        self._last_greet_monotonic = time.monotonic()
        self._last_greet = {"source": source, "completed": completed,
                            "interrupted": interrupted,
                            "timestamp": datetime.now().astimezone().isoformat(timespec="seconds")}
        return {
            "ok": True, "card": CARD_BAI_NIAN, "action": "greet",
            "control_level": CONTROL_LEVEL_BN, "source": source,
            "applied": {"greet_times": self._greet_times,
                        "completed": completed,
                        "interrupted": interrupted,
                        "speech": {"ok": bool(speech.get("ok")),
                                    "code": speech.get("code"),
                                    "message": speech.get("message")}},
            "steps": steps, "timestamp_ms": _ms_bn(),
        }

    def _greet_dispatch(self, source):
        stub = self._require_live()
        if stub:
            return stub
        if not self._lock.acquire(blocking=False):
            return _err_bn("PRECONDITION_FAILED", "另一轮拜年正在进行,请先 stop")
        self._cancel.clear()
        try:
            return self._perform_greet(source)
        finally:
            self._lock.release()

    # ── 自动守护 ────────────────────────────────────────────────────────────

    def _auto_loop(self, position):
        while not self._auto_cancel.is_set():
            in_cooldown = (self._last_greet_monotonic is not None and
                           time.monotonic() - self._last_greet_monotonic < self._cooldown_s)
            if not in_cooldown and self._lock.acquire(blocking=False):
                self._lock.release()
                detection = self._detect_internal(position)
                if detection.get("ok") and detection.get("person_detected"):
                    print(f"[{CARD_BAI_NIAN}] auto: person detected "
                          f"(count={detection['count']}) → greet", flush=True)
                    self._greet_dispatch(source="auto")
            deadline = time.monotonic() + self._interval_s
            while time.monotonic() < deadline and not self._auto_cancel.is_set():
                time.sleep(min(_JOIN_SLICE_S, max(0.05, deadline - time.monotonic())))

    # ── dispatch ────────────────────────────────────────────────────────────

    def dispatch(self, action, args):
        args = args or {}
        if action == "start":
            return self.start()
        if action == "stop":
            self.stop()
            return {"ok": True, "card": CARD_BAI_NIAN, "action": "stop",
                    "control_level": CONTROL_LEVEL_BN, "state": "idle",
                    "applied": {"stopped": True, "auto_disarmed": True},
                    "timestamp_ms": _ms_bn()}
        if action == "info":
            return self._info()
        if action == "detect_once":
            position = args.get("position", self._position)
            if position not in camera._VALID_POSITIONS:
                return _err_bn("INVALID_ARGUMENT", f"unknown camera position: {position!r}")
            return self._detect_internal(position)
        if action == "greet":
            return self._greet_dispatch(source="manual")
        if action == "arm":
            if not args.get("confirm"):
                return _err_bn("PRECONDITION_FAILED",
                               "arm requires confirm=true (autonomous motion)")
            stub = self._require_live()
            if stub:
                return stub
            if self._auto_thread is not None and self._auto_thread.is_alive():
                return _err_bn("RESOURCE_BUSY", "auto guard already armed")
            if self._audio_bytes is None:
                self._audio_bytes = self._load_audio()
            self._auto_cancel.clear()
            self._auto_thread = threading.Thread(
                target=self._auto_loop, args=(self._position,),
                daemon=True, name="bai_nian_auto")
            self._auto_thread.start()
            return {"ok": True, "card": CARD_BAI_NIAN, "action": "arm",
                    "control_level": CONTROL_LEVEL_BN,
                    "applied": {"auto": True, "position": self._position,
                                "detect_interval_s": self._interval_s,
                                "cooldown_s": self._cooldown_s},
                    "timestamp_ms": _ms_bn()}
        if action == "disarm":
            self._auto_cancel.set()
            thread = self._auto_thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=3.0)
            self._auto_thread = None
            return {"ok": True, "card": CARD_BAI_NIAN, "action": "disarm",
                    "control_level": CONTROL_LEVEL_BN,
                    "applied": {"auto": False}, "timestamp_ms": _ms_bn()}
        return None

    def _info(self):
        armed = bool(self._auto_thread is not None and self._auto_thread.is_alive())
        cooldown_remaining = 0.0
        if self._last_greet_monotonic is not None:
            cooldown_remaining = round(max(
                0.0, self._cooldown_s - (time.monotonic() - self._last_greet_monotonic)), 1)
        return {
            "ok": True, "card": CARD_BAI_NIAN, "action": "info",
            "control_level": CONTROL_LEVEL_BN,
            "state": "running" if armed else "ready",
            "auto_armed": armed,
            "position": self._position,
            "cv2_available": _HAS_CV2,
            "audio_loaded": bool(self._audio_bytes),
            "audio_file": str(self._audio_path),
            "last_detection": self._last_detection,
            "last_greet": self._last_greet,
            "last_speak": self._last_speak,
            "cooldown_remaining_s": cooldown_remaining,
            "config": {"detect_interval_s": self._interval_s,
                       "min_confidence": self._min_confidence,
                       "close_ratio": self._close_ratio,
                       "greet_times": self._greet_times,
                       "cooldown_s": self._cooldown_s,
                       "speaker": "%s:%s" % (self._speaker_host, self._speaker_port)},
            "timestamp_ms": _ms_bn(),
        }


def make_bai_nian(plugin_config, namespace, executor, client):
    return BaiNianPlugin(plugin_config, namespace, executor, client)
