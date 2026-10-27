"""One-way raw audio bridge to an isolated instance of the vendor DOA model.

The vendor worker has no device access or behavior tree actions. It runs on
private domain 43. This bridge publishes only PCM and actual device metadata;
it subscribes only to its typed DOA result. It never sends wakeup/motion RPCs.
"""
from __future__ import annotations

import threading
import time
from collections import deque

DOMAIN = 43
RAW_TOPIC = "/phanthy/u1_doa/raw"
DEVICE_TOPIC = "/sys/device/audio_in/cur_device_info"
EVENT_TOPIC = "/phanthy/u1_doa/event"
INPUT_TOPIC = "/sys/device/audio_in/raw"


class NativeDirectionSource:
    def __init__(self, nodes):
        self.nodes = nodes
        self._lock = threading.RLock()
        self._events = deque(maxlen=32)
        self._stop = threading.Event()
        self._context = self._node = self._executor = self._thread = None
        self._audio_sub = self._device_sub = None
        self._raw_pub = self._device_pub = None
        self.running = False
        self.audio_frames = self.device_updates = self.events_received = 0
        self._last_audio = self._ready_after = None
        self.device = None
        self.error = ""
        self._worker_present = False

    def start(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from audio_msgs.msg import AudioInData, CurrentAudioDeviceInfo, DoaEvent
        self._rclpy = rclpy
        try:
            self._context = Context()
            rclpy.init(context=self._context, domain_id=DOMAIN)
            self._node = Node("u1_pro_native_doa_bridge", context=self._context)
            self._raw_pub = self._node.create_publisher(AudioInData, RAW_TOPIC, 10)
            device_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._device_pub = self._node.create_publisher(CurrentAudioDeviceInfo, DEVICE_TOPIC, device_qos)
            self._node.create_subscription(DoaEvent, EVENT_TOPIC, self._on_direction, self.nodes._sensor_qos)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self.running = True
            self._device_sub = self.nodes.audio_device.create_subscription(
                CurrentAudioDeviceInfo, DEVICE_TOPIC, self._on_device, device_qos)
            self._audio_sub = self.nodes.audio_device.create_subscription(
                AudioInData, INPUT_TOPIC, self._on_audio, self.nodes._sensor_qos)
            self._thread = threading.Thread(target=self._spin, daemon=True, name="u1-native-doa")
            self._thread.start()
        except Exception:
            self.stop()
            raise

    def _spin(self):
        try:
            while not self._stop.is_set() and self._rclpy.ok(context=self._context):
                self._executor.spin_once(timeout_sec=.1)
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self.error = str(exc)[:256]
                    self.running = False
                    self._events.clear()

    def _on_device(self, message):
        with self._lock:
            if not self.running:
                return
            device = {"device_id": message.device_id, "device_name": message.device_name,
                      "channels": message.channels, "is_built_in": message.is_built_in,
                      "sample_rate": message.sample_rate, "sample_format": message.sample_format}
            if self.device != device:
                self._events.clear()
                self._events.append((None, time.monotonic(), time.time_ns() // 1_000_000))
                self._ready_after = time.monotonic() + 1.0
                self._last_audio = None
            # This firmware reports is_built_in=False for its real internal
            # array. Forward it unchanged; the vendor IsInternalMicNode uses
            # the actual internal_mics.yml layout as the authority.
            self.device = device
            try:
                self._device_pub.publish(message)
                self.device_updates += 1
            except Exception as exc:
                self.error = str(exc)[:256]

    def _on_audio(self, message):
        with self._lock:
            if not self.running or not self.device:
                return
            if not message.data.data:
                return
            now = time.monotonic()
            if self._last_audio is None or now - self._last_audio > .5:
                # Do not revive a buffered direction after input loss. This is
                # receipt-clock warmup, never claimed as a capture-time window.
                self._ready_after = now + 1.0
                self._events.clear()
                self._events.append((None, now, time.time_ns() // 1_000_000))
            try:
                self._raw_pub.publish(message)
                self.audio_frames += 1
                self._last_audio = now
            except Exception as exc:
                self.error = str(exc)[:256]

    def _on_direction(self, message):
        with self._lock:
            now = time.monotonic()
            self.events_received += 1
            if (not self.running or self.error or self._last_audio is None
                    or not self._worker_present
                    or now - self._last_audio > .5 or now < (self._ready_after or 0)):
                return
            self._events.append(({"azimuth": message.azimuth, "confidence": message.confidence},
                                 now, time.time_ns() // 1_000_000))

    def drain(self):
        with self._lock:
            present = bool(self._raw_pub is not None and self._raw_pub.get_subscription_count())
            if present != self._worker_present:
                self._events.clear()
                self._events.append((None, time.monotonic(), time.time_ns() // 1_000_000))
                self._ready_after = time.monotonic() + 1.0
                self._worker_present = present
            elif not present:
                self._events.clear()
            events = list(self._events)
            self._events.clear()
            return events

    def info(self):
        with self._lock:
            age = None if self._last_audio is None else time.monotonic() - self._last_audio
            subscribers = self._raw_pub.get_subscription_count() if self._raw_pub is not None else 0
            state = ("error" if self.error else "stopped" if not self.running else
                     "waiting_for_device" if not self.device else
                     "worker_not_discovered" if not subscribers else
                     "waiting_for_audio" if age is None or age > .5 else "receiving")
            return {"state": state, "domain": DOMAIN, "device": self.device,
                    "audio_frames_forwarded": self.audio_frames, "device_updates": self.device_updates,
                    "native_events_received": self.events_received,
                    "audio_age_seconds": age, "worker_subscribers": subscribers, "error": self.error}

    def stop(self):
        with self._lock:
            self.running = False
            self._stop.set()
            self._events.clear()
        errors = []
        for attribute in ("_audio_sub", "_device_sub"):
            sub = getattr(self, attribute)
            if sub is not None:
                try:
                    self.nodes.audio_device.destroy_subscription(sub)
                except Exception as exc:
                    errors.append(exc)
                else:
                    setattr(self, attribute, None)
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2)
            except Exception as exc:
                errors.append(exc)
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                errors.append(RuntimeError("native DOA executor did not stop"))
        if not errors:
            if self._node is not None:
                self._node.destroy_node()
                self._node = self._raw_pub = self._device_pub = None
            if self._context is not None and self._rclpy.ok(context=self._context):
                self._rclpy.shutdown(context=self._context)
            self._context = self._executor = self._thread = None
        if errors:
            raise RuntimeError(f"native DOA cleanup failed: {errors[0]}") from errors[0]
