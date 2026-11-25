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
    def __init__(self, cached_stamp=None):
        self._last_stamp = cached_stamp
        self._last_advance = None

    def read(self, sample, now):
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
        if stamp != self._last_stamp:
            self._last_stamp = stamp
            self._last_advance = now
        values = sample.get("effort")
        if not isinstance(values, (list, tuple)) or len(values) != TAXEL_COUNT:
            return {**result, "state": "unsupported_layout"}
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            return {**result, "state": "invalid_readings"}
        if self._last_advance is None:
            return result
        age_s = now - self._last_advance
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


def _color(raw):
    if raw is None:
        return (39, 43, 52)
    # A fixed visual scale keeps a contact comparable across frames. Raw data
    # and numeric labels keep the original value, including values above 255.
    level = math.sqrt(min(abs(raw), 255) / 255)
    return tuple(round(low + level * (high - low))
                 for low, high in zip((26, 49, 67), (255, 151, 66)))


class TactilePreview:
    # Each shape stands for one entire region, not the sensor's physical grid.
    HAND_SHAPES = ((8, 65, 35, 93), (36, 16, 53, 66), (56, 6, 73, 66),
                   (76, 13, 93, 66), (96, 26, 113, 66), (36, 69, 113, 130),
                   (36, 156, 113, 217))

    def __init__(self):
        from PIL import Image, ImageDraw, ImageFont

        self._image = Image
        self._draw = ImageDraw
        self._title = ImageFont.load_default(size=22)
        self._label = ImageFont.load_default(size=15)
        self._small = ImageFont.load_default(size=12)

    def render(self, reading):
        canvas = self._image.new("RGB", (1120, 462), (12, 20, 31))
        draw = self._draw.Draw(canvas)
        draw.text((22, 14), "O10Hand tactile", font=self._title, fill="#f0f5fb")
        state = reading["state"]
        status = state.replace("_", " ").upper()
        draw.text((22, 45), f"{status}  |  Raw sensor values  |  Region map is schematic",
                  font=self._label, fill="#b4c2d4")
        for side, x in zip(SIDES, (16, 568)):
            regions = (reading["hands"] or {}).get(side, {})
            draw.rounded_rectangle((x, 80, x + 536, 441), radius=12, fill="#152235")
            draw.text((x + 14, 91), side.upper(), font=self._label, fill="#f0f5fb")
            draw.text((x + 13, 128), "Region map", font=self._small, fill="#aebdd0")
            draw.text((x + 147, 110), "Points in source order · peak / active points",
                      font=self._small, fill="#aebdd0")
            for index, (region, count) in enumerate(REGIONS):
                data = regions.get(region)
                peak = data["peak_raw"] if data is not None else None
                x0, y0, x1, y1 = self.HAND_SHAPES[index]
                if side == "left":
                    x0, x1 = 120 - x1, 120 - x0
                cx, cy = x + 12, 153
                draw.rounded_rectangle((cx + x0, cy + y0, cx + x1, cy + y1), radius=5,
                                       fill=_color(peak), outline="#67839a")
                draw.text((cx + (x0 + x1) // 2 - 3, cy + (y0 + y1) // 2 - 6),
                          str(index + 1), fill="#f0f5fb", font=self._small)
                y = 136 + index * 42
                label = f"{index + 1}  {region.title()}"
                if data is not None:
                    label += f"  {peak:g} / {data['nonzero_points']}"
                draw.text((x + 147, y), label, fill="#f0f5fb", font=self._small)
                for point, raw in enumerate(data["raw"] if data is not None else [None] * count):
                    left = x + 147 + point * 3
                    draw.rectangle((left, y + 19, left + 1, y + 29), fill=_color(raw))
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
            reader = TactileReader(_source_stamp(self.nodes.snapshot("hand_state")))
            self._halt.clear()
            self._data_state = "waiting"
            self._thread = threading.Thread(target=self._publish, args=(reader, preview),
                                            name="a3-hand-tactile", daemon=True)
            self._thread.start()

    def _publish(self, reader, preview):
        data_pub, preview_pub = self._publishers
        while not self._halt.is_set():
            sample = reader.read(self.nodes.snapshot("hand_state"), time.monotonic())
            self._data_state = sample["state"]
            data_msg = self.nodes._String()
            data_msg.data = json.dumps(sample, allow_nan=False, separators=(",", ":"))
            preview_msg = self.nodes._CompressedImage()
            preview_msg.format = "jpeg"
            preview_msg.data = preview.render(sample)
            if self._halt.is_set():
                break
            data_pub.publish(data_msg)
            preview_pub.publish(preview_msg)
            self._halt.wait(PUBLISH_INTERVAL_S)

    def stop(self):
        with self._lifecycle:
            self._halt.set()
            if self._thread is not None:
                self._thread.join(timeout=2)

    def dispatch(self, action, args):
        if action == "start":
            self._start_stream()
        elif action == "stop":
            self.stop()
        elif action not in ("info", "hand_tactile"):
            raise ValueError(f"hand_tactile: unknown action {action!r}")
        running = self._thread is not None and self._thread.is_alive()
        return {"state": "running" if running else "idle", "topic_out": self.topic_out,
                "data_state": self._data_state if running else "idle"}
