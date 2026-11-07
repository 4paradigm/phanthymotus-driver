"""Read-only PCM statistics. No SDK, microphone ownership or audio persistence."""

from __future__ import annotations

import array
import math
import sys
import statistics
import json
import threading
import time


class AudioMonitor:
    def __init__(
        self,
        calibration_s=5,
        hold_s=2,
        recover_s=2,
        rise_db=12,
        fall_db=6,
        mode="relative",
        threshold_dbfs=-20,
        comparison="above",
        hysteresis_db=3,
    ):
        values = (calibration_s, hold_s, recover_s, rise_db, fall_db)
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("settings must be finite numbers")
        if not (
            all(0.25 <= v <= 60 for v in values[:3]) and 0 <= fall_db < rise_db <= 60
        ):
            raise ValueError("invalid durations or hysteresis thresholds")
        self.calibration_s, self.hold_s, self.recover_s = values[:3]
        self.rise_db, self.fall_db = rise_db, fall_db
        if mode not in ("relative", "threshold") or comparison not in (
            "above",
            "below",
        ):
            raise ValueError("invalid detection mode or comparison")
        if any(
            type(v) not in (int, float) or not math.isfinite(v)
            for v in (threshold_dbfs, hysteresis_db)
        ):
            raise ValueError("threshold and hysteresis must be finite numbers")
        if not (-160 <= threshold_dbfs <= 0 and 0 <= hysteresis_db <= 30):
            raise ValueError("invalid dBFS threshold or hysteresis")
        if (
            mode == "threshold"
            and comparison == "below"
            and threshold_dbfs + hysteresis_db > 0
        ):
            raise ValueError("below-mode recovery threshold must not exceed 0 dBFS")
        self.mode, self.threshold_dbfs = mode, threshold_dbfs
        self.comparison, self.hysteresis_db = comparison, hysteresis_db
        self.sequence = 0
        self.reset()

    def settings(self):
        return {
            key: getattr(self, key)
            for key in (
                "calibration_s",
                "hold_s",
                "recover_s",
                "rise_db",
                "fall_db",
                "mode",
                "threshold_dbfs",
                "comparison",
                "hysteresis_db",
            )
        }

    def reset(self):
        self.metrics = dict(rms=None, peak=None, rms_dbfs=None, clip_ratio=None)
        self.status = "unknown"
        self.baseline = None
        self.levels = []
        self.last = None
        self.pending = 0
        self._count = self._energy = self._clips = self._peak = 0

    def feed(self, payload, now):
        if not payload or len(payload) % 2 or len(payload) > 16000:
            self.reset()
            raise ValueError("expected 1–8000 PCM16 samples")
        # Maximum valid packet duration is .5 s; allow .25 s delivery jitter.
        if self.last is not None and (now <= self.last or now - self.last > 0.75):
            self.reset()
        self.last = now
        samples = array.array("h")
        samples.frombytes(payload)
        if sys.byteorder != "little":
            samples.byteswap()
        events = []
        for offset in range(0, len(samples), 4000):
            # A packet may cross a window boundary. Never weight short packets
            # like full windows or depend on how the upstream splits its audio.
            block = samples[offset : offset + 4000]
            while block:
                take = min(len(block), 4000 - self._count)
                part, block = block[:take], block[take:]
                self._count += take
                self._energy += sum(x * x for x in part)
                self._clips += sum(abs(x) >= 32760 for x in part)
                self._peak = max(self._peak, max(abs(x) for x in part))
                if self._count != 4000:
                    continue
                rms = math.sqrt(self._energy / 4000) / 32768
                db = 20 * math.log10(rms) if rms else None
                self.metrics = dict(
                    rms=rms,
                    peak=self._peak / 32768,
                    rms_dbfs=db,
                    clip_ratio=self._clips / 4000,
                )
                level = max(-90, db if db is not None else -90)
                self._count = self._energy = self._clips = self._peak = 0
                if self.mode == "threshold":
                    # Valid all-zero PCM is below every finite threshold. Keep
                    # its public dBFS null; never serialize negative infinity.
                    absolute = db if db is not None else -math.inf
                    active = self.status == "triggered"
                    if self.status == "unknown":
                        self.status = "normal"
                    if self.comparison == "above":
                        condition = (
                            absolute <= self.threshold_dbfs - self.hysteresis_db
                            if active
                            else absolute > self.threshold_dbfs
                        )
                    else:
                        condition = (
                            absolute >= self.threshold_dbfs + self.hysteresis_db
                            if active
                            else absolute < self.threshold_dbfs
                        )
                    self.pending = self.pending + 0.25 if condition else 0
                    if self.pending >= (self.recover_s if active else self.hold_s):
                        self.status = "normal" if active else "triggered"
                        self.pending = 0
                        self.sequence += 1
                        events.append(
                            dict(
                                event=(
                                    "threshold_cleared"
                                    if active
                                    else "threshold_entered"
                                ),
                                sequence=self.sequence,
                                priority=1,
                                rms_dbfs=db,
                                threshold_dbfs=self.threshold_dbfs,
                                comparison=self.comparison,
                            )
                        )
                    continue
                if self.baseline is None:
                    self.status = "calibrating"
                    self.levels.append(level)
                    if len(self.levels) * 0.25 >= self.calibration_s:
                        self.baseline = statistics.median(self.levels)
                        self.levels.clear()
                        self.status = "normal"
                    continue
                delta = level - self.baseline
                condition = (
                    delta >= self.rise_db
                    if self.status == "normal"
                    else delta <= self.fall_db
                )
                self.pending = self.pending + 0.25 if condition else 0
                duration = self.hold_s if self.status == "normal" else self.recover_s
                if self.pending >= duration:
                    self.status = "loud" if self.status == "normal" else "normal"
                    self.pending = 0
                    self.sequence += 1
                    events.append(
                        dict(
                            event="loud" if self.status == "loud" else "recovered",
                            sequence=self.sequence,
                            priority=1,
                            relative_db=delta,
                            rms_dbfs=db,
                        )
                    )
        return events

    def read(self, now):
        fresh = self.last is not None and 0 <= now - self.last <= 2
        metrics = self.metrics if fresh else {k: None for k in self.metrics}
        db = metrics["rms_dbfs"]
        return dict(
            metrics,
            status=self.status if fresh else "unknown",
            fresh=fresh,
            baseline_dbfs=self.baseline if fresh else None,
            relative_db=(
                (max(-90, db if db is not None else -90) - self.baseline)
                if fresh and self.baseline is not None
                else None
            ),
            window_s=0.25,
            age_s=now - self.last if self.last is not None else None,
            units="digital_full_scale_not_SPL",
            settings=self.settings(),
        )


def _ros_runtime():
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from audio_msgs.msg import AudioChunk
    from std_msgs.msg import String

    return (
        Node("bumi_audio_environment"),
        AudioChunk,
        String,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
    )


class AudioEnvironmentPlugin:
    """Canvas-controlled subscriber; bundle startup never takes the microphone."""

    def __init__(
        self, config, namespace, executor, *, runtime=_ros_runtime, clock=time.monotonic
    ):
        keys = (
            "calibration_s",
            "hold_s",
            "recover_s",
            "rise_db",
            "fall_db",
            "mode",
            "threshold_dbfs",
            "comparison",
            "hysteresis_db",
        )
        self.monitor = AudioMonitor(**{k: config[k] for k in keys if k in config})
        self.executor, self.runtime, self.clock = executor, runtime, clock
        self.prefix = f"/{namespace}/audio_environment"
        self.lock = threading.RLock()
        self.node = None
        self.topic = ""
        self.source_stamp = None
        self.error = ""

    def get_tool(self):
        settings = self.monitor.settings()
        labels = {
            "mode": "检测模式",
            "threshold_dbfs": "触发界限（dBFS，负数）",
            "comparison": "触发方向",
            "hysteresis_db": "恢复滞回（dB）",
            "hold_s": "触发持续时间（秒）",
            "recover_s": "恢复持续时间（秒）",
            "calibration_s": "背景校准时间（秒）",
            "rise_db": "相对增强阈值（dB）",
            "fall_db": "相对恢复阈值（dB）",
        }
        properties = {
            key: dict(type="number", default=value, title=labels[key], scope="shared")
            for key, value in settings.items()
        }
        properties["mode"].update(
            type="string",
            enum=["relative", "threshold"],
            oneOf=[
                {"const": "relative", "title": "相对安静背景（原有模式）"},
                {"const": "threshold", "title": "固定 dBFS 界限"},
            ],
        )
        properties["comparison"].update(
            type="string",
            enum=["above", "below"],
            oneOf=[
                {"const": "above", "title": "大于界限触发"},
                {"const": "below", "title": "小于界限触发"},
            ],
        )
        for key in ("calibration_s", "hold_s", "recover_s"):
            properties[key].update(minimum=0.25, maximum=60)
        properties["threshold_dbfs"].update(
            minimum=-160,
            maximum=0,
            description="数字电平，不是声压 dB SPL。-20 比 -40 更响；与界限相等不触发。",
        )
        properties["hysteresis_db"].update(
            minimum=0,
            maximum=30,
            description="大于模式在界限减去此值后恢复；小于模式在界限加上此值后恢复，恢复值不能超过 0 dBFS。",
        )
        for key in ("rise_db", "fall_db"):
            properties[key].update(minimum=0, maximum=60)
        return dict(
            name="audio_environment",
            type="processor",
            multiInstance=False,
            description="Read-only PCM level monitoring: relative background or configurable above/below dBFS threshold. Digital levels, not dB SPL. Connect events to Decision Core, state to dashboard.",
            inputSchema={
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "start",
                            "stop",
                            "info",
                            "read",
                            "calibrate",
                            "config",
                        ],
                    },
                    "input_topic": {"type": "string"},
                },
                "required": ["action"],
            },
            configSchema={"type": "object", "properties": properties},
            topic_in=[{"format": "audio/pcm-16k"}],
            topic_out=[
                {"topic": self.prefix + "/state", "format": "data/json"},
                {"topic": self.prefix + "/events", "format": "data/json"},
            ],
        )

    def start(self):
        # Bundle lifecycle only; the canvas supplies the subscribed topic.
        pass

    def stop(self):
        with self.lock:
            node, self.node = self.node, None
            self.monitor.reset()
            self.source_stamp = None
            self.topic = ""
            if node is not None:
                try:
                    self.executor.remove_node(node)
                finally:
                    node.destroy_node()

    def dispatch(self, action, args):
        with self.lock:
            if action == "config":
                current = self.monitor.settings()
                unknown = (
                    set(args) - set(current) - {"action", "_tool_name", "instance_id"}
                )
                if unknown:
                    return {
                        "ok": False,
                        "state": "error",
                        "message": "Unknown settings: " + ", ".join(sorted(unknown)),
                    }
                updated = dict(
                    current, **{k: v for k, v in args.items() if k in current}
                )
                try:
                    replacement = AudioMonitor(**updated)
                except ValueError as exc:
                    return {"ok": False, "state": "error", "message": str(exc)}
                if updated != current:
                    replacement.sequence = self.monitor.sequence
                    self.monitor = replacement
                    self.error = ""
                return {
                    "ok": True,
                    "state": "running" if self.node else "idle",
                    "settings": self.monitor.settings(),
                }
            if action == "stop":
                self.stop()
                return {"state": "idle"}
            if action == "start":
                inputs = args.get("input_topics") or []
                topic = args.get("input_topic") or (inputs[0] if inputs else "")
                if (
                    not isinstance(topic, str)
                    or not topic.startswith("/")
                    or len(topic) > 256
                ):
                    return {
                        "state": "error",
                        "message": "Connect a PCM16k audio input_topic",
                    }
                if self.node is not None:
                    return {
                        "state": "running" if topic == self.topic else "error",
                        "message": "Stop before changing input_topic",
                        "topic_out": self.get_tool()["topic_out"],
                    }
                node = None
                try:
                    node, audio_type, self.string_type, qos = self.runtime()
                    self.state_pub = node.create_publisher(
                        self.string_type, self.prefix + "/state", 10
                    )
                    self.event_pub = node.create_publisher(
                        self.string_type, self.prefix + "/events", 10
                    )
                    node.create_subscription(
                        audio_type, topic, lambda msg: self._receive(msg, node), qos
                    )
                    node.create_timer(0.25, lambda: self._publish_state(node))
                    self.monitor.reset()
                    self.source_stamp = None
                    self.error = ""
                    self.node, self.topic = node, topic
                    self.executor.add_node(node)
                except Exception as exc:
                    if node is not None:
                        try:
                            self.executor.remove_node(node)
                        finally:
                            node.destroy_node()
                    self.node = None
                    return {"state": "error", "message": str(exc)}
                return {"state": "running", "topic_out": self.get_tool()["topic_out"]}
            if action == "calibrate":
                if self.monitor.mode == "threshold":
                    return {
                        "ok": False,
                        "state": "error",
                        "message": "Fixed dBFS mode needs no baseline calibration",
                    }
                self.monitor.reset()
                self.error = ""
                return {
                    "state": "running" if self.node else "idle",
                    "message": "Quiet baseline will be collected from new audio",
                }
            if action in ("read", "info", "audio_environment"):
                return dict(
                    self.monitor.read(self.clock()),
                    state="running" if self.node else "idle",
                    input_topic=self.topic,
                    last_error=self.error,
                    topic_out=self.get_tool()["topic_out"],
                )
            return None

    def _emit(self, publisher, data):
        message = self.string_type()
        message.data = json.dumps(data, allow_nan=False)
        publisher.publish(message)

    def _receive(self, message, owner):
        with self.lock:
            if self.node is not owner:
                return
            try:
                # Bumi's existing mic declares the standard port format but
                # still writes this legacy, equivalent encoding in AudioChunk.
                if message.format not in ("audio/pcm-16k", "pcm_16k_16bit_mono"):
                    raise ValueError("unsupported audio format")
                stamp = getattr(getattr(message, "header", None), "stamp", None)
                if stamp is not None:
                    stamp = stamp.sec * 1_000_000_000 + stamp.nanosec
                    if stamp > 0:
                        if self.source_stamp is not None and stamp <= self.source_stamp:
                            raise ValueError(
                                "duplicate or reversed source timestamp; restart input/Card after clock reset"
                            )
                        self.source_stamp = stamp
                events = self.monitor.feed(bytes(message.data), self.clock())
                self.error = ""
                for event in events:
                    self._emit(self.event_pub, event)
            except (ValueError, TypeError, OverflowError) as exc:
                self.monitor.reset()
                self.error = str(exc)

    def _publish_state(self, owner):
        with self.lock:
            if self.node is owner:
                self._emit(self.state_pub, self.dispatch("read", {}))
