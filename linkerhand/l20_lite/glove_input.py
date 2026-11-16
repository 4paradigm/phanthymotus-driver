"""Headless Hexacercle glove input for hand_teach.

The vendor C ABI handles receiver framing and mocap. We own neutral calibration
workflow, the basic angle-to-stroke mapping and the UDP packet.
"""
import argparse
import ctypes as C
import json
import math
import os
from pathlib import Path
import socket
import threading
import time
from collections import Counter
from contextlib import ExitStack, nullcontext

from hand_trajectory import GLOVE_LABELS, SOURCE_FORMAT
from teach_messages import error_text

SDK_API = 0x00010400
HAND_INDEX = {'left': 0, 'right': 1}
STALE_S = 0.1
R_WARMUP_S = 0.3


class RCapture(C.Structure):
    _fields_ = [("struct_size", C.c_uint32), ("state", C.c_int32),
                ("hand_mask", C.c_int32), ("progress", C.c_float),
                ("fresh_hand_mask", C.c_uint32)]


class Angles(C.Structure):
    """HexAnglesSnapshot from the public Hexacercle C headers."""
    _fields_ = [
        ("struct_size", C.c_uint32), ("version", C.c_uint32),
        ("valid_mask", C.c_uint32), ("calibrated", C.c_uint32),
        ("timestamp_ms", C.c_int64), ("frame_sequence", C.c_uint32),
        ("flags", C.c_uint32), ("finger", (C.c_float * 4) * 5),
        ("ui_finger", (C.c_float * 4) * 5), ("palm_twist_degrees", C.c_float * 3),
        ("thumb_twist_degrees", C.c_float), ("thumb_plane_degrees", C.c_float),
        ("thumb_hardware_smoothed_degrees", C.c_float), ("thumb_hardware_255", C.c_int32),
        ("thumb_flexion_readout_degrees", C.c_float), ("thumb_to_tip_mm", C.c_float * 4),
        ("wrist_speed_deg_sec", C.c_float), ("wrist_smooth_blend", C.c_float),
        ("finger_speed_deg_sec", C.c_float * 5), ("flex_hold_mask", C.c_uint32),
        ("spread_hold_mask", C.c_uint32), ("static_hold_mask", C.c_uint32),
        ("thumb_plane_valid", C.c_uint32),
    ]


def _bind(lib, name, result, *args):
    fn = getattr(lib, name)
    fn.restype, fn.argtypes = result, args
    return fn


class HexReceiver:
    def __init__(self, library, port, profile):
        self.library, self.port, self.profile = library, port, profile
        self._resources = ExitStack()
        self.last_angle_issue = None

    def __enter__(self):
        if not Path(self.profile).is_file():
            raise ValueError("请选择与当前手套及佩戴状态匹配的 .hexprofile")
        with ExitStack() as resources:
            path = Path(self.library).resolve(strict=True)
            if os.name == "nt":
                resources.enter_context(os.add_dll_directory(str(path.parent)))
            lib = C.CDLL(str(path))
            handle, integer, uint = C.c_void_p, C.c_int, C.c_uint32
            version = _bind(lib, "hex_mocap_get_api_version", uint)
            actual = version()
            if actual != SDK_API:
                raise RuntimeError(f"手套 SDK ABI 为 0x{actual:08x}，需要 0x{SDK_API:08x}")
            create = _bind(lib, "hex_glove_create", handle)
            destroy = _bind(lib, "hex_glove_destroy", None, handle)
            abort = _bind(lib, "hex_glove_abort", None, handle)
            start = _bind(lib, "hex_glove_start", integer, handle, C.c_char_p)
            mocap_create = _bind(lib, "hex_mocap_create", handle, handle)
            mocap_destroy = _bind(lib, "hex_mocap_destroy", None, handle)
            load = _bind(lib, "hex_mocap_profile_load", integer, handle, C.c_char_p)
            self._update = _bind(lib, "hex_mocap_update", integer, handle)
            self._fresh = None
            if hasattr(lib, "hex_mocap_get_fresh_hand_mask"):
                self._fresh = _bind(lib, "hex_mocap_get_fresh_hand_mask", uint, handle)
            else:
                self._sequence = {side: _bind(lib, f"hex_glove_get_frame_seq_{side}", uint, handle)
                                  for side in HAND_INDEX}
            self._r = _bind(lib, "hex_mocap_start_r_capture", integer, handle, integer, C.c_float)
            self._r_info = _bind(lib, "hex_mocap_get_r_capture_info", integer, handle, C.POINTER(RCapture))
            self._zero = _bind(lib, "hex_mocap_calibrate_angles", integer, handle, integer)
            self._angles = _bind(lib, "hex_mocap_get_precise_angles", integer, handle, integer, C.POINTER(Angles))
            glove = create()
            if not glove:
                raise RuntimeError("手套 SDK 无法创建接收器会话")
            resources.callback(destroy, glove)
            resources.callback(abort, glove)
            self._mocap = mocap_create(glove)
            if not self._mocap:
                raise RuntimeError("手套 SDK 无法创建解算会话")
            resources.callback(mocap_destroy, self._mocap)
            if not load(self._mocap, str(Path(self.profile).resolve()).encode("utf-8")):
                raise RuntimeError("手套 SDK 无法加载校准档案")
            serial = os.path.realpath(self.port) if os.name == "posix" else self.port
            if not start(glove, serial.encode("utf-8")):
                raise RuntimeError(f"无法打开手套串口 {self.port}，请检查连接或串口占用")
            self._glove = glove
            if self._fresh is None:
                self._last_sequence = {side: read(glove) for side, read in self._sequence.items()}
            self._resources = resources.pop_all()
        return self

    def __exit__(self, *_exc):
        self._resources.close()

    def tick(self):
        updated = self._update(self._mocap)
        if not updated:
            return set()
        if self._fresh is not None:
            mask = self._fresh(self._mocap)
            return {side for side, index in HAND_INDEX.items() if mask & (1 << index)}
        current = {side: read(self._glove) for side, read in self._sequence.items()}
        fresh = {side for side, seq in current.items() if seq != self._last_sequence[side]}
        self._last_sequence = current
        return fresh

    def start_r(self, mask):
        if not self._r(self._mocap, mask, 1.0):
            raise RuntimeError("手套 SDK 未能开始 R 校准")

    def r_state(self):
        info = RCapture(struct_size=C.sizeof(RCapture))
        if not self._r_info(self._mocap, C.byref(info)):
            raise RuntimeError("无法读取 R 校准状态")
        return info.state

    def zero(self, mask):
        if not self._zero(self._mocap, mask):
            raise RuntimeError("伸直零位校准失败，请保持手指伸直后重试")

    def angles(self, side):
        snap = Angles(struct_size=C.sizeof(Angles))
        if not self._angles(self._mocap, HAND_INDEX[side], C.byref(snap)):
            self.last_angle_issue = "snapshot_unavailable"
        elif not snap.calibrated or not snap.valid_mask:
            self.last_angle_issue = "invalid_angles"
        # Wrist freeze holds the wrist estimate. Only finger channels are used
        # here; baseline realignment can affect those too.
        elif snap.flags & 2:
            self.last_angle_issue = f"wrist_flags_{snap.flags & 3}"
        else:
            self.last_angle_issue = None
            return snap
        return None


def make_packet(angles, side="right"):
    """Basic retargeting. Deliberately omits the Unity pinch refinement."""
    finger = angles.finger
    spans = (70, 90, 100, 100, 100, 100)
    values = (angles.thumb_flexion_readout_degrees, abs(finger[0][3]),
              *(row[0] + 0.2 * row[1] for row in finger[1:]))
    if not all(math.isfinite(value) for value in values):
        return None
    stroke = [int(255 * min(1, max(0, value / span))) for value, span in zip(values, spans)]
    stroke[0] = 255 - stroke[0]
    return json.dumps({"schemaId": SOURCE_FORMAT, "handType": "LinkerHand", "dof": 6,
                       "timestampMs": time.time_ns() // 1_000_000, "labels": GLOVE_LABELS,
                       "availableSides": [side],
                       "leftHand": stroke if side == "left" else [0] * 6,
                       "rightHand": stroke if side == "right" else [0] * 6}, separators=(",", ":")).encode()


class GloveInput:
    """One receiver owner; all native calls live on the same worker thread."""
    def __init__(self, *, udp_output=False):
        self._udp_output = udp_output
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._state = "idle"
        self._error = None
        self._broadcast_to = None
        self._selected = ()
        self._last_gap = {}
        self._latest_packet = {}

    def status(self, side=None):
        with self._lock:
            now = time.monotonic()
            fresh_sides = [key for key, (ts, _) in self._latest_packet.items() if now - ts <= STALE_S]
            fresh = side in fresh_sides if side else bool(fresh_sides)
            calibrated = list(self._selected) if self._state == "ready" else []
            messages = {
                "idle": "手套会话未启动",
                "waiting_for_glove": "正在等待所选手套的数据",
                "calibrating": "正在校准，请保持所选手的手指自然伸直",
                "ready": "手套已校准，可以开始录制" if fresh else "所选手没有有效的新帧",
            }
            gap = self._last_gap.get(side) if side else next(iter(self._last_gap.values()), None)
            return {"state": self._state, "message": self._error or messages.get(self._state),
                    "fresh": fresh, "calibrated_sides": calibrated, "fresh_sides": fresh_sides,
                    "error": self._error, "last_gap": gap}

    def latest_packet(self, side="right"):
        with self._lock:
            return self._latest_packet.get(side)

    def calibrate(self, library, serial_port, profile, sides=("right",)):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("请先执行 disconnect_glove 结束当前手套会话")
            self._stop.clear()
            self._state, self._error = "waiting_for_glove", None
            self._selected = tuple(sides)
            self._broadcast_to = None
            self._last_gap.clear()
            self._latest_packet.clear()
            self._thread = threading.Thread(target=self._run, args=(library, serial_port, profile, self._selected),
                                            name="hand_teach_glove", daemon=True)
            self._thread.start()
        return self.status()

    def broadcast(self, target, port):
        if not self._udp_output:
            raise RuntimeError("当前手套会话仅提供本机数据")
        with self._lock:
            if self._state != "ready" or not any(time.monotonic() - ts <= STALE_S
                                                   for ts, _ in self._latest_packet.values()):
                raise RuntimeError("手套未完成校准，或没有有效的新帧")
            endpoint = target, port
            if self._broadcast_to is not None and self._broadcast_to != endpoint:
                raise RuntimeError("请先停止当前手套广播")
            self._broadcast_to = endpoint
            self._error = None
        return self.status()

    def stop_broadcast(self):
        with self._lock:
            self._broadcast_to = None
        return self.status()

    def close(self):
        with self._lock:
            worker = self._thread
            self._broadcast_to = None
            self._stop.set()
        if worker and worker.is_alive():
            worker.join(3)
            if worker.is_alive():
                raise RuntimeError("手套 SDK 尚未退出")
        with self._lock:
            if self._thread is worker:
                self._state, self._error = "idle", None

    def _run(self, library, serial_port, profile, sides):
        try:
            output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self._udp_output else nullcontext()
            with output as udp, HexReceiver(library, serial_port, profile) as receiver:
                if udp is not None:
                    udp.setblocking(False)
                required = set(sides)
                mask = sum(1 << HAND_INDEX[side] for side in sides)
                deadline = time.monotonic() + 8
                first_fresh, last_fresh, last_id, last_valid = {}, {}, {}, {}
                next_send = dict.fromkeys(sides, 0.0)
                skipped = {side: Counter() for side in sides}
                while not self._stop.is_set():
                    fresh = receiver.tick() & required
                    now = time.monotonic()
                    for side in fresh:
                        if side not in last_fresh or now - last_fresh[side] > STALE_S:
                            first_fresh[side] = now
                        last_fresh[side] = now
                    online = {side for side, ts in last_fresh.items() if now - ts <= STALE_S}
                    if self._state == "waiting_for_glove":
                        if online == required and all(now - first_fresh[side] >= R_WARMUP_S for side in sides):
                            receiver.start_r(mask)
                            with self._lock:
                                self._state = "calibrating"
                            deadline = now + 6
                    elif self._state == "calibrating":
                        r_state = receiver.r_state()
                        if r_state in (3, 4):
                            raise RuntimeError("手套 R 校准未完成，请检查追踪并重试")
                        if r_state == 2 and online == required and fresh:
                            receiver.zero(mask)
                            with self._lock:
                                self._state = "ready"
                    if self._state != "ready" and now > deadline:
                        if self._state == "waiting_for_glove":
                            missing = "、".join("左手" if side == "left" else "右手"
                                                for side in sides if side not in online)
                            raise TimeoutError(f"等待{missing or '所选手套'}有效数据超时，请检查电源、配对和接收器")
                        raise TimeoutError("R 校准超时，请保持所选手伸直并检查手套追踪")
                    if self._state == "ready":
                        for side in sides:
                            counts = skipped[side]
                            if side not in fresh:
                                counts["polls_without_fresh_frame"] += 1
                                continue
                            angles = receiver.angles(side)
                            if angles is None:
                                counts[receiver.last_angle_issue] += 1
                                continue
                            if angles.frame_sequence == last_id.get(side):
                                counts["duplicate_angle_frame"] += 1
                                continue
                            last_id[side] = angles.frame_sequence
                            packet = make_packet(angles, side)
                            if packet is None:
                                counts["non_finite_angles"] += 1
                                continue
                            with self._lock:
                                previous = last_valid.get(side)
                                if previous is not None and now - previous > STALE_S:
                                    gap = {"duration_ms": round((now - previous) * 1000), "skipped": dict(counts)}
                                    self._last_gap[side] = gap
                                    print(f"[glove_input] {side} frame gap: {gap}", flush=True)
                                counts.clear()
                                last_valid[side] = now
                                if now + 0.002 >= next_send[side]:
                                    self._latest_packet[side] = (now, packet)
                                    if self._broadcast_to is not None:
                                        udp.sendto(packet, self._broadcast_to)
                                    period = 1 / 60
                                    next_send[side] = max(now + period / 2, next_send[side] + period)
                    self._stop.wait(0.003)
        except (OSError, AttributeError, RuntimeError, ValueError) as exc:
            if isinstance(exc, AttributeError):
                print(f"[glove_input] 缺少 SDK 接口：{exc!r}", flush=True)
                message = "手套 SDK 缺少所需接口，请检查库版本"
            elif isinstance(exc, OSError) and not isinstance(exc, TimeoutError):
                message = "手套 SDK 或串口访问失败：" + error_text(exc, "请检查库路径、架构、依赖和串口")
            else:
                message = error_text(exc, "手套校准失败，请查看驱动日志")
            with self._lock:
                self._state, self._error = "error", message
        finally:
            with self._lock:
                if self._state != "error":
                    self._state = "idle"
                self._broadcast_to = None
                self._latest_packet.clear()


def main():
    parser = argparse.ArgumentParser(description="Hexacercle 手套无界面校准及 UDP 输出")
    parser.add_argument("--library", required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--side", choices=tuple(HAND_INDEX), default="right")
    parser.add_argument("--profile", required=True)
    parser.add_argument("--target", default="127.0.0.1")
    parser.add_argument("--port", default=9012, type=int)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")
    try:
        socket.inet_pton(socket.AF_INET, args.target)
    except OSError:
        parser.error("--target must be an IPv4 address")
    glove = GloveInput(udp_output=True)
    try:
        input("请戴好所选手套，保持手指伸直中立后按回车。")
        glove.calibrate(args.library, args.serial, args.profile, (args.side,))
        started = time.monotonic()
        sent_started = False
        while True:
            state = glove.status()
            if state["state"] == "error":
                raise RuntimeError(state["error"])
            if not sent_started and state["fresh"]:
                glove.broadcast(args.target, args.port)
                sent_started = True
                print("校准完成，正在发送六通道 UDP 数据。", flush=True)
            if not sent_started and time.monotonic() - started > 16:
                raise TimeoutError("校准后没有收到所选手套的有效角度")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        glove.close()


if __name__ == "__main__":
    main()
