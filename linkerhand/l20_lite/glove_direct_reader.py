"""Native Hex receiver: no vendor SDK, Mono, or hand serial access."""

import argparse
import json
import math
from pathlib import Path
import signal
import stat
import time
from glove_protocol import Decoder, normalized


def diagnostic_angles(palm, fingers):
    """Relative rotation-vector projections; control uses quaternions only."""
    px, py, pz, pw = palm
    pitch = []
    yaw = []
    for x, y, z, w in fingers:
        q = [
            pw * x - px * w - py * z + pz * y,
            pw * y + px * z - py * w - pz * x,
            pw * z - px * y + py * x - pz * w,
            pw * w + px * x + py * y + pz * z,
        ]
        if q[3] < 0:
            q = [-v for v in q]
        norm = math.sqrt(sum(v * v for v in q[:3]))
        scale = math.degrees(2 * math.atan2(norm, q[3])) / norm if norm > 1e-10 else 0.0
        pitch.append(q[0] * scale)
        yaw.append(q[2] * scale)
    return pitch, yaw


class GloveAssembler:
    def __init__(self, max_age=0.2):
        self.max_age = max_age
        self.channels = {}
        self.counter = 0
        self.emitted = {}
        self.last_emit = {}
        self.diagnostics = {}

    def update(self, frame, now):
        self.counter += 1
        self.channels[(frame["side"], frame["channel"])] = {
            **frame,
            "quaternions": normalized(frame["quaternions"]),
            "received": now,
            "counter": self.counter,
        }

    def sample(self, side, now, wall_ms=None):
        root = self.channels.get((side, "root"))
        top = self.channels.get((side, "top"))
        if root is None or top is None:
            self.diagnostics[side] = {"status": "waiting_for_root_and_top"}
            return None
        if (
            max(now - root["received"], now - top["received"]) > self.max_age
            or abs(root["received"] - top["received"]) > self.max_age
        ):
            self.diagnostics[side] = {"status": "root_top_out_of_sync"}
            return None
        roots = root["quaternions"]
        tips = top["quaternions"]
        fingers = [roots[1]] + [roots[i] or tips[i] for i in range(2, 6)]
        if roots[0] is None or any(q is None for q in fingers):
            self.diagnostics[side] = {"status": "invalid_root_top_quaternions"}
            return None
        marker = (root["counter"], top["counter"])
        if (
            marker == self.emitted.get(side)
            or now - self.last_emit.get(side, -1e9) < 0.02
        ):
            return None
        pitch, yaw = diagnostic_angles(roots[0], fingers)
        self.emitted[side] = marker
        self.last_emit[side] = now
        self.diagnostics[side] = {"status": "valid_sample", "backend": "native_serial"}
        return {
            "side": side,
            "sequence": top["sequence"],
            "frame_marker": f"{marker[0]}:{marker[1]}",
            "timestamp_ms": int(time.time() * 1000) if wall_ms is None else wall_ms,
            "layout": "new",
            "quaternion_layout": "hex-root-top-v1",
            "root_quaternions": roots,
            "tip_quaternions": tips,
            "pitch": pitch,
            "yaw": yaw,
            "angle_source": "relative_rotation_vector_diagnostic",
            "backend": "native_serial",
        }


def main():
    import serial

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--max-seconds", type=float, default=0)
    args = ap.parse_args()
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def emit(data):
        print(json.dumps(data, separators=(",", ":"), allow_nan=False), flush=True)

    started = time.monotonic()
    while not stopped and (
        args.max_seconds <= 0 or time.monotonic() - started < args.max_seconds
    ):
        receiver = None
        try:
            port = Path(args.port)
            device = port.resolve(strict=True)
            if not stat.S_ISCHR(device.stat().st_mode):
                raise RuntimeError("configured receiver must be a character device")
            receiver = serial.Serial(
                port=None,
                baudrate=921600,
                timeout=0.01,
                write_timeout=0.5,
                exclusive=True,
            )
            receiver.dtr = False
            receiver.rts = True
            receiver.port = str(device)
            receiver.open()
            decoder = Decoder()
            assembler = GloveAssembler()
            connected = time.monotonic()
            latest = connected
            last_status = connected
            heartbeat = connected
            init_sent = False
            seen_frames = 0
            emit(
                {
                    "state": "connected",
                    "port": args.port,
                    "device": str(device),
                    "runtime": "python-native-serial",
                    "sdk_required": False,
                }
            )
            while not stopped and (
                args.max_seconds <= 0 or time.monotonic() - started < args.max_seconds
            ):
                now = time.monotonic()
                if not port.exists() or port.resolve() != device:
                    raise RuntimeError("receiver USB path changed")
                if now - connected > 2 and not init_sent and seen_frames == 0:
                    command = bytes.fromhex("a0 ff ff 28 01 00 89 0a")
                    if receiver.write(command) != len(command):
                        raise RuntimeError("incomplete receiver initialization")
                    init_sent = True
                    emit({"state": "receiver_init_sent_once"})
                data = receiver.read(min(max(receiver.in_waiting, 1), 65536))
                now = time.monotonic()
                for frame in decoder.feed(data):
                    assembler.update(frame, now)
                    seen_frames += 1
                for side in ("left", "right"):
                    sample = assembler.sample(side, now)
                    if sample is not None:
                        emit(sample)
                        latest = now
                if now - latest > 0.8 and now - last_status > 5:
                    emit(
                        {
                            "state": "waiting_for_glove",
                            "idle_seconds": round(now - latest, 1),
                            "diagnostics": assembler.diagnostics,
                        }
                    )
                    last_status = now
                if now - heartbeat >= 0.1:
                    emit({"state": "heartbeat"})
                    heartbeat = now
        except (OSError, RuntimeError, serial.SerialException) as error:
            emit(
                {
                    "error": f"{type(error).__name__}: {error}",
                    "backend": "native_serial",
                }
            )
        finally:
            if receiver is not None:
                receiver.close()
        deadline = time.monotonic() + 1
        while not stopped and time.monotonic() < deadline:
            time.sleep(0.05)


if __name__ == "__main__":
    main()
