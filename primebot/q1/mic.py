"""Capture Q1 array audio through the official PulseAudio interface."""

import math
import os
import selectors
import struct
import subprocess
import time
from threading import Event, Lock, Thread


AUDIO_FORMAT = "audio/pcm-16k"
SOURCE_RATE = 48_000
OUTPUT_RATE = 16_000
SOURCE_CHANNELS = 8
OUTPUT_SAMPLES = 512
DECIMATION = SOURCE_RATE // OUTPUT_RATE
INPUT_SAMPLES = OUTPUT_SAMPLES * DECIMATION
INPUT_BYTES = INPUT_SAMPLES * SOURCE_CHANNELS * 2
SOURCE_TIMEOUT = 5.0
PULSE_SOURCE = "alsa_input.platform-aw89403_sound.pro-input-0"
PULSE_SERVER = "unix:/run/user/1000/pulse/native"


def _lowpass_coefficients():
    # Windowed sinc at 7 kHz preserves the speech band and attenuates aliases
    # before decimation. Scale the coefficients for unity gain at DC.
    half_width = 31
    cutoff = 7_000 / SOURCE_RATE
    taps = [
        (math.sin(2 * math.pi * cutoff * offset) / (math.pi * offset) if offset else 2 * cutoff)
        * (0.54 + 0.46 * math.cos(math.pi * offset / half_width))
        for offset in range(-half_width, half_width + 1)
    ]
    gain = sum(taps)
    return tuple(tap / gain for tap in taps)


FILTER = _lowpass_coefficients()


class PcmConverter:
    """Mix the selected channels and decimate without resetting at chunk edges."""

    def __init__(self, channels):
        if (
            not isinstance(channels, (list, tuple))
            or len(channels) != 2
            or any(type(channel) is not int or not 3 <= channel <= 8 for channel in channels)
        ):
            raise ValueError("mic.channels must contain two Q1 microphone channels numbered 3–8")
        self.left, self.right = (channel - 1 for channel in channels)
        self.pending = bytearray()
        self.history = [0.0] * (len(FILTER) - 1)

    def feed(self, data):
        self.pending.extend(data)
        while len(self.pending) >= INPUT_BYTES:
            interleaved = struct.unpack_from(f"<{INPUT_SAMPLES * SOURCE_CHANNELS}h", self.pending)
            del self.pending[:INPUT_BYTES]
            left = interleaved[self.left::SOURCE_CHANNELS]
            right = interleaved[self.right::SOURCE_CHANNELS]
            samples = self.history + [(a + b) * 0.5 for a, b in zip(left, right)]
            output = []
            for offset in range(0, INPUT_SAMPLES, DECIMATION):
                window = samples[offset:offset + len(FILTER)]
                result = round(sum(coefficient * sample for coefficient, sample in zip(FILTER, window)))
                output.append(max(-32768, min(32767, result)))
            self.history = samples[-(len(FILTER) - 1):]
            yield struct.pack(f"<{OUTPUT_SAMPLES}h", *output)


class MicCard:
    def __init__(self, config, namespace, ros):
        from audio_msgs.msg import AudioChunk
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data

        self.message_type = AudioChunk
        self.topic = f"/{namespace}/mic/audio"
        self.output = [{"topic": self.topic, "format": AUDIO_FORMAT}]
        self.node = Node("primebot_q1_mic", context=ros.ctx_core)
        self.publisher = self.node.create_publisher(AudioChunk, self.topic, qos_profile_sensor_data)
        ros.executor_core.add_node(self.node)

        self.channels = config.get("channels", (5, 6))
        self.source = os.environ.get("Q1_MIC_SOURCE") or config.get("source", PULSE_SOURCE)
        self.server = os.environ.get("PULSE_SERVER") or PULSE_SERVER
        self.lifecycle = Lock()
        self.status_lock = Lock()
        self.stopping = Event()
        self.worker = None
        self.state = "idle"
        self.last_frame = None
        self.error = None

    def get_tool(self):
        from common.vendor_runtime import action_schema, tool

        return tool(
            "mic", "sensor", "启元 Q1 内置麦克风。向语音识别卡输出 16 kHz 单声道 PCM 音频。",
            action_schema({"info": ([], "查看麦克风状态与输出话题")}, {}),
            topic_out=self.output,
        )

    def start(self):
        with self.lifecycle:
            if self.worker and self.worker.is_alive():
                return
            self.stopping.clear()
            with self.status_lock:
                self.state = "waiting"
                self.last_frame = None
                self.error = None
            self.worker = Thread(target=self._capture, name="q1-mic-capture", daemon=True)
            self.worker.start()

    def stop(self):
        with self.lifecycle:
            self.stopping.set()
            if self.worker:
                self.worker.join()
                self.worker = None
            with self.status_lock:
                self.state = "idle"

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        elif action not in ("info", "mic"):
            return None
        with self.status_lock:
            result = {"state": self.state, "topic_out": self.output}
            if self.last_frame is not None:
                result["age_s"] = round(time.monotonic() - self.last_frame, 3)
            if self.error:
                result["error"] = self.error
            return result

    def _capture(self):
        process = None
        try:
            converter = PcmConverter(self.channels)
            # Keep the vendor channel order rather than letting PulseAudio remix it.
            command = [
                "parec", "--raw", "--format=s16le", f"--rate={SOURCE_RATE}",
                f"--channels={SOURCE_CHANNELS}",
                "--channel-map=aux0,aux1,aux2,aux3,aux4,aux5,aux6,aux7",
                f"--device={self.source}",
                "--latency-msec=50",
            ]
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, env={**os.environ, "PULSE_SERVER": self.server},
            )
            deadline = time.monotonic() + SOURCE_TIMEOUT
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout, selectors.EVENT_READ)
                while not self.stopping.is_set():
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f"no complete PulseAudio frame received for {SOURCE_TIMEOUT:g} s")
                    if not selector.select(timeout=0.2):
                        continue
                    data = os.read(process.stdout.fileno(), INPUT_BYTES)
                    if not data:
                        raise RuntimeError("parec closed the audio stream")
                    for frame in converter.feed(data):
                        if self.stopping.is_set():
                            break
                        self._publish(frame)
                        deadline = time.monotonic() + SOURCE_TIMEOUT
        except (OSError, RuntimeError, ValueError) as exc:
            if not self.stopping.is_set():
                with self.status_lock:
                    self.state = "error"
                    self.error = f"{exc} (PulseAudio {self.server}, source {self.source})"
                self.node.get_logger().error(self.error)
        finally:
            if process:
                process.stdout.close()
                process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

    def _publish(self, frame):
        message = self.message_type()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.format = AUDIO_FORMAT
        message.data = frame
        self.publisher.publish(message)
        with self.status_lock:
            self.state = "running"
            self.last_frame = time.monotonic()
