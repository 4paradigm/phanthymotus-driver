"""Read-only A3 Ultra O10Hand tactile data and schematic preview."""

from __future__ import annotations

import io
import json
import math
import threading
import time

from common.vendor_runtime import tool

# AimDK A3 Ultra v3.3 sends the complete left hand, then the right hand.
SIDES = ("left", "right")
REGIONS = (("thumb", 16), ("index", 18), ("middle", 18), ("ring", 18),
           ("pinky", 18), ("palm", 78), ("back", 102))
TAXEL_COUNT = len(SIDES) * sum(count for _, count in REGIONS)
PUBLISH_INTERVAL_S = 0.2
STALE_AFTER_S = 1.0


def _source_stamp(sample):
    header = sample.get("header") if isinstance(sample, dict) else None
    stamp = header.get("stamp") if isinstance(header, dict) else None
    if not isinstance(stamp, dict):
        return None
    sec, nanosec = stamp.get("sec"), stamp.get("nanosec")
    if (type(sec) is int and sec >= 0 and type(nanosec) is int
            and 0 <= nanosec < 1_000_000_000 and (sec or nanosec)):
        return sec, nanosec
    return None


class TactileReader:
    def read(self, sample, now, received_at=None):
        result = {"state": "waiting", "hand_type": "O10Hand", "source_stamp": None,
                  "sample_age_ms": None, "hands": None}
        if not sample:
            return result
        header = sample.get("header") if isinstance(sample, dict) else None
        if not isinstance(header, dict) or header.get("frame_id") != "O10Hand":
            return {**result, "state": "unsupported_hand"}
        stamp = _source_stamp(sample)
        if stamp is None:
            return {**result, "state": "invalid_stamp"}
        result["source_stamp"] = {"sec": stamp[0], "nanosec": stamp[1]}
        values = sample.get("effort")
        if not isinstance(values, (list, tuple)) or len(values) != TAXEL_COUNT:
            return {**result, "state": "unsupported_layout"}
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            return {**result, "state": "invalid_readings"}
        if received_at is None:
            return result
        age_s = max(0.0, now - received_at)
        result["sample_age_ms"] = round(age_s * 1000)
        if age_s >= STALE_AFTER_S:
            return {**result, "state": "stale"}

        hands = {}
        offset = 0
        for side in SIDES:
            hands[side] = {}
            for region, count in REGIONS:
                points = list(values[offset:offset + count])
                hands[side][region] = {"raw": points, "peak_raw": max(points, key=abs),
                                       "nonzero_points": sum(value != 0.0 for value in points)}
                offset += count
        return {**result, "state": "fresh", "hands": hands}


class HandTactileSource:
    """A latest-sample subscription isolated from aggregate joint callbacks."""

    def __init__(self, context, message_type):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy

        self._lock = threading.Lock()
        self._sample = None
        self._received_at = None
        self._halt = threading.Event()
        self._thread = None
        self._node = Node("agibot_a3_hand_tactile", context=context)
        self._executor = SingleThreadedExecutor(context=context)
        try:
            self._node.create_subscription(
                message_type, "/motion/control/hand_joint_state", self._receive,
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
            self._executor.add_node(self._node)
        except Exception:
            self._executor.shutdown()
            self._node.destroy_node()
            raise

    def _receive(self, message):
        received_at = time.monotonic()
        header = message.header
        sample = {"header": {"frame_id": header.frame_id,
                             "stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec}},
                  "effort": tuple(message.effort)}
        with self._lock:
            self._sample, self._received_at = sample, received_at

    def snapshot(self):
        with self._lock:
            return self._sample, self._received_at

    def start(self):
        self._thread = threading.Thread(target=self._spin, name="a3-tactile-receiver", daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            while not self._halt.is_set():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            if not self._halt.is_set():
                print(f"[hand-tactile] receiver stopped error={type(exc).__name__}", flush=True)

    def stop(self):
        if self._node is None:
            return
        self._halt.set()
        self._executor.wake()
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise RuntimeError("hand_tactile receiver did not stop")
        self._executor.shutdown()
        self._node.destroy_node()
        self._node = None


def _color(raw):
    if raw is None:
        return (39, 43, 52)
    # A fixed visual scale keeps a contact comparable across frames. Raw data
    # and numeric labels keep the original value, including values above 255.
    level = math.sqrt(min(abs(raw), 255) / 255)
    return tuple(round(low + level * (high - low))
                 for low, high in zip((26, 49, 67), (255, 151, 66)))


class TactilePreview:
    def __init__(self):
        from PIL import Image, ImageDraw, ImageFont

        self._image = Image
        self._draw = ImageDraw
        self._title = ImageFont.load_default(size=26)
        self._label = ImageFont.load_default(size=24)
        self._small = ImageFont.load_default(size=18)
        self._value = ImageFont.load_default(size=36)

    def render(self, reading):
        canvas = self._image.new("RGB", (800, 600), (12, 20, 31))
        draw = self._draw.Draw(canvas)
        draw.text((16, 12), "O10Hand tactile", font=self._title, fill="#f0f5fb")
        status = reading["state"].replace("_", " ").upper()
        draw.text((784, 15), status, anchor="ra", font=self._label,
                  fill="#6ee7b7" if reading["state"] == "fresh" else "#fbbf24")
        draw.text((16, 49), "Raw peak values  |  Schematic regions", font=self._small, fill="#b4c2d4")
        for side, x in zip(SIDES, (12, 406)):
            regions = (reading["hands"] or {}).get(side, {})
            draw.rounded_rectangle((x, 84, x + 382, 588), radius=12, fill="#152235")
            draw.text((x + 191, 99), side.upper(), anchor="ma", font=self._title, fill="#f0f5fb")
            for index, (region, _) in enumerate(REGIONS):
                data = regions.get(region)
                peak = data["peak_raw"] if data is not None else None
                if index < 5:
                    column = 4 - index if side == "left" else index
                    left = x + 18 + column * 70
                    top = (183, 155, 139, 155, 175)[index]
                    box = (left, top, left + 66, 319)
                    label_font, value_font = self._small, self._label
                else:
                    top = 333 if region == "palm" else 459
                    box = (x + 18, top, x + 364, top + 110)
                    label_font, value_font = self._label, self._value
                # One named tile per region; physical taxel coordinates are unknown.
                draw.rounded_rectangle(box, radius=9, fill=_color(peak), outline="#67839a", width=2)
                foreground = "#101b29" if peak is not None and abs(peak) >= 80 else "#f0f5fb"
                center = (box[0] + box[2]) / 2
                draw.text((center, box[1] + 12), region.title(), anchor="ma", font=label_font, fill=foreground)
                value = f"{peak:.3g}" if peak is not None else "--"
                draw.text((center, (box[1] + box[3]) / 2 + 16), value,
                          anchor="mm", font=value_font, fill=foreground)
        result = io.BytesIO()
        canvas.save(result, format="JPEG", quality=85)
        return result.getvalue()


class HandTactilePlugin:
    def __init__(self, nodes):
        self.nodes = nodes
        prefix = f"/{nodes.namespace.strip('/')}/hand"
        self.topic_out = [{"topic": f"{prefix}/tactile_preview", "format": "image/jpeg"},
                          {"topic": f"{prefix}/tactile", "format": "data/json"}]
        self._lifecycle = threading.Lock()
        self._halt = threading.Event()
        self._thread = None
        self._publishers = None
        self._source = None
        self._data_state = "waiting"

    def get_tool(self):
        return tool("hand_tactile", "sensor", "O10Hand 双手触觉原始读数与分区强度预览（只读）",
                    topic_out=self.topic_out)

    def start(self):
        # Bundle startup registers the card. The canvas starts its output.
        pass

    def _start_stream(self):
        with self._lifecycle:
            if self._thread is not None and self._thread.is_alive():
                return
            preview = TactilePreview()
            if self._publishers is None:
                if self.nodes.core_bridge is not None:
                    self.nodes.core_bridge.enable_hand_tactile()
                self._publishers = (
                    self.nodes.core.create_publisher(self.nodes._String, self.topic_out[1]["topic"], 1),
                    self.nodes.core.create_publisher(self.nodes._CompressedImage, self.topic_out[0]["topic"], 1),
                )
            if self._source is not None:
                self._source.stop()
            source = HandTactileSource(self.nodes.robot.context, self.nodes._JointState)
            self._source = source
            self._halt.clear()
            self._data_state = "waiting"
            try:
                source.start()
                self._thread = threading.Thread(target=self._publish, args=(source, preview),
                                                name="a3-hand-tactile", daemon=True)
                self._thread.start()
            except Exception:
                source.stop()
                self._source = None
                raise

    def _publish(self, source, preview):
        reader = TactileReader()
        data_pub, preview_pub = self._publishers
        last_state = None
        try:
            while not self._halt.is_set():
                started_at = time.monotonic()
                feedback, received_at = source.snapshot()
                sample = reader.read(feedback, time.monotonic(), received_at)
                self._data_state = sample["state"]
                if self._data_state != last_state:
                    print(f"[hand-tactile] state={self._data_state} sample_age_ms={sample['sample_age_ms']}", flush=True)
                    last_state = self._data_state
                data_msg = self.nodes._String()
                data_msg.data = json.dumps(sample, allow_nan=False, separators=(",", ":"))
                preview_msg = self.nodes._CompressedImage()
                preview_msg.format = "jpeg"
                preview_msg.data = preview.render(sample)
                if self._halt.is_set():
                    break
                data_pub.publish(data_msg)
                preview_pub.publish(preview_msg)
                self._halt.wait(max(0.0, PUBLISH_INTERVAL_S - (time.monotonic() - started_at)))
        finally:
            source.stop()

    def stop(self):
        with self._lifecycle:
            self._halt.set()
            if self._thread is not None:
                self._thread.join(timeout=2)
                if self._thread.is_alive():
                    raise RuntimeError("hand_tactile preview did not stop")
            if self._source is not None:
                self._source.stop()
                self._source = None

    def dispatch(self, action, args):
        if action == "start":
            self._start_stream()
        elif action == "stop":
            self.stop()
        elif action not in ("info", "hand_tactile"):
            return None
        running = self._thread is not None and self._thread.is_alive()
        return {"state": "running" if running else "idle", "topic_out": self.topic_out,
                "data_state": self._data_state if running else "idle"}
