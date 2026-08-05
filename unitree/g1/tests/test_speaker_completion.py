import importlib.util
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock


G1_DIR = Path(__file__).resolve().parents[1]


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _Timer:
    def __init__(self, callback):
        self.callback = callback

    def cancel(self):
        pass


class _Node:
    def __init__(self, name):
        self.name = name
        self.logger = _Logger()

    def create_subscription(self, _msg_type, topic, callback, _qos):
        return types.SimpleNamespace(topic=topic, callback=callback)

    def destroy_subscription(self, _subscription):
        pass

    def create_publisher(self, *_args, **_kwargs):
        return types.SimpleNamespace(publish=lambda _msg: None)

    def create_timer(self, _period, callback):
        return _Timer(callback)

    def destroy_timer(self, _timer):
        pass

    def get_logger(self):
        return self.logger


class _Header:
    def __init__(self):
        self.frame_id = ""


class _AudioChunk:
    def __init__(self, data=b""):
        self.header = _Header()
        self.format = "audio/pcm-16k"
        self.data = list(data)


class _QoSProfile:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def _package(name):
    module = types.ModuleType(name)
    module.__path__ = []
    return module


def _load_device_module():
    rclpy = _package("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _Node
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(
        BEST_EFFORT="best_effort", RELIABLE="reliable",
    )
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(
        VOLATILE="volatile", TRANSIENT_LOCAL="transient_local",
    )

    std_msgs = _package("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Header = _Header
    std_msgs_msg.String = type("String", (), {})
    audio_msgs = _package("audio_msgs")
    audio_msgs_msg = types.ModuleType("audio_msgs.msg")
    audio_msgs_msg.AudioChunk = _AudioChunk

    sdk = _package("unitree_sdk2py")
    sdk_g1 = _package("unitree_sdk2py.g1")
    sdk_audio = _package("unitree_sdk2py.g1.audio")
    sdk_client = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    sdk_client.AudioClient = object
    pointcloud = types.ModuleType("pointcloud_utils")
    pointcloud.gravity_align_inplace = lambda *_args, **_kwargs: None
    playback_state = types.ModuleType("playback_state")
    playback_state.G1PlaybackStateMonitor = object

    stubs = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "audio_msgs": audio_msgs,
        "audio_msgs.msg": audio_msgs_msg,
        "unitree_sdk2py": sdk,
        "unitree_sdk2py.g1": sdk_g1,
        "unitree_sdk2py.g1.audio": sdk_audio,
        "unitree_sdk2py.g1.audio.g1_audio_client": sdk_client,
        "pointcloud_utils": pointcloud,
        "playback_state": playback_state,
        "numpy": types.ModuleType("numpy"),
    }
    spec = importlib.util.spec_from_file_location(
        "g1_device_under_test", G1_DIR / "device.py",
    )
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


DEVICE = _load_device_module()


class _AudioClient:
    def __init__(self):
        self.stream_calls = []
        self.stop_calls = []
        self.stream_code = 0
        self.stream_codes = []
        self.stream_exceptions = []
        self.stop_code = 0
        self.block_stream = False
        self.block_stream_call = None
        self.stream_entered = threading.Event()
        self.stream_release = threading.Event()
        self.tts_calls = []

    def PlayStream(self, app_name, stream_id, pcm):
        self.stream_calls.append((app_name, stream_id, bytes(pcm)))
        self.stream_entered.set()
        if (self.block_stream
                or self.block_stream_call == len(self.stream_calls)):
            self.stream_release.wait(timeout=5)
        if self.stream_exceptions:
            exception = self.stream_exceptions.pop(0)
            if exception is not None:
                raise exception
        code = (
            self.stream_codes.pop(0)
            if self.stream_codes else self.stream_code
        )
        return code, {}

    def PlayStop(self, app_name):
        self.stop_calls.append(app_name)
        return self.stop_code

    def TtsMaker(self, text, voice):
        self.tts_calls.append((text, voice))
        return 0


class _Monitor:
    def __init__(self, results=None):
        self.results = list(results or ["completed"])
        self.wait_calls = []

    def checkpoint(self):
        return 7

    def wait_for_stable_idle(self, **kwargs):
        self.wait_calls.append(kwargs)
        state = self.results.pop(0) if self.results else "completed"
        if state == "completed":
            return {
                "state": "completed",
                "reason": "",
                "playing_seq": 8,
                "idle_seq": 9,
                "playing_ts": 1.0,
                "idle_ts": 2.0,
            }
        return {
            "state": state,
            "reason": "play_state_idle_timeout",
            "playing_seq": 8,
            "idle_seq": None,
        }

    def status(self):
        return {"available": True, "current_state": 0, "event_seq": 9}


class _ForcedCleanupMonitor(_Monitor):
    def __init__(self):
        super().__init__()
        self.cleanup_entered = threading.Event()
        self.call_count = 0

    def wait_for_stable_idle(self, **kwargs):
        self.wait_calls.append(kwargs)
        self.call_count += 1
        if self.call_count == 1:
            return {
                "state": "timeout",
                "reason": "play_state_idle_timeout",
                "playing_seq": 8,
                "idle_seq": None,
            }
        self.cleanup_entered.set()
        cancelled = kwargs.get("cancelled", lambda: False)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if cancelled():
                return {"state": "interrupted", "reason": "interrupted"}
            time.sleep(0.01)
        return {"state": "timeout", "reason": "play_state_idle_timeout"}


class _GatedNaturalTimeoutMonitor(_Monitor):
    def __init__(self, cleanup_state="timeout"):
        super().__init__()
        self.cleanup_state = cleanup_state
        self.natural_wait_entered = threading.Event()
        self.natural_wait_release = threading.Event()
        self.call_count = 0

    def wait_for_stable_idle(self, **kwargs):
        self.wait_calls.append(kwargs)
        self.call_count += 1
        if self.call_count == 1:
            self.natural_wait_entered.set()
            self.natural_wait_release.wait(timeout=2)
            return {
                "state": "timeout",
                "reason": "play_state_idle_timeout",
                "playing_seq": 8,
                "idle_seq": None,
            }
        if self.cleanup_state == "completed":
            return {
                "state": "completed",
                "reason": "",
                "playing_seq": 8,
                "idle_seq": 9,
            }
        return {
            "state": self.cleanup_state,
            "reason": "play_state_idle_timeout",
            "playing_seq": 8,
            "idle_seq": None,
        }


class _InterruptibleFailureCleanupMonitor(_Monitor):
    def __init__(self):
        super().__init__()
        self.cleanup_entered = threading.Event()

    def wait_for_stable_idle(self, **kwargs):
        self.wait_calls.append(kwargs)
        self.cleanup_entered.set()
        cancelled = kwargs.get("cancelled", lambda: False)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if cancelled():
                return {"state": "interrupted", "reason": "interrupted"}
            time.sleep(0.005)
        return {"state": "timeout", "reason": "play_state_idle_timeout"}


class _BlockingBytes:
    def __init__(self, payload):
        self.payload = payload
        self.entered = threading.Event()
        self.release = threading.Event()

    def __bytes__(self):
        self.entered.set()
        self.release.wait(timeout=2)
        return self.payload


class SpeakerCompletionTests(unittest.TestCase):
    def _node(self, monitor=None):
        client = _AudioClient()
        node = DEVICE._SpeakerNode(client, playback_monitor=monitor or _Monitor())
        node.start_play("/test/audio")
        return node, client

    def _publish_utterance(self, node, pcm=b"\x00\x00" * 1600):
        node._on_chunk(_AudioChunk(pcm))
        node._on_chunk(_AudioChunk(node.AUDIO_EOF_MAGIC))

    def test_eof_waits_for_firmware_and_exposes_terminal_record(self):
        monitor = _Monitor()
        node, client = self._node(monitor)
        started = time.monotonic()

        self._publish_utterance(node)
        result = node.wait_for_playback(after_playback_id=0, timeout=2)

        self.assertEqual(result["state"], "completed")
        self.assertEqual(result["completion_basis"], "g1_audio_service_stable_idle")
        self.assertTrue(result["eof_received"])
        self.assertEqual(result["audio_bytes"], 3200)
        self.assertEqual(result["submitted_blocks"], 1)
        self.assertGreaterEqual(time.monotonic() - started, 0.09)
        self.assertEqual(len(client.stream_calls), 1)
        self.assertNotEqual(client.stream_calls[0][1], "0")
        self.assertGreaterEqual(
            monitor.wait_calls[0]["not_before"], result["created_monotonic"],
        )
        node.stop_play()

    def test_each_utterance_has_a_distinct_stream_id(self):
        node, client = self._node(_Monitor(["completed", "completed"]))

        self._publish_utterance(node)
        first = node.wait_for_playback(after_playback_id=0, timeout=2)
        self._publish_utterance(node)
        second = node.wait_for_playback(
            after_playback_id=first["playback_id"], timeout=2,
        )

        self.assertEqual(second["state"], "completed")
        self.assertNotEqual(first["stream_id"], second["stream_id"])
        self.assertEqual(
            {call[1] for call in client.stream_calls},
            {first["stream_id"], second["stream_id"]},
        )
        node.stop_play()

    def test_timeout_closes_stream_but_never_claims_completion(self):
        monitor = _Monitor(["timeout", "completed"])
        node, client = self._node(monitor)

        self._publish_utterance(node)
        result = node.wait_for_playback(after_playback_id=0, timeout=2)

        self.assertEqual(result["state"], "error")
        self.assertEqual(
            result["reason"],
            "playback_not_complete_before_forced_close",
        )
        self.assertTrue(result["forced_stream_close"])
        self.assertEqual(result["completion_basis"], "")
        self.assertEqual(result["cleanup_state"], "completed")
        # Initial stale-session cleanup + EOF stream close.
        self.assertGreaterEqual(len(client.stop_calls), 2)
        node.stop_play()

    def test_wait_after_id_does_not_return_a_stale_completion(self):
        node, _client = self._node()
        self._publish_utterance(node)
        first = node.wait_for_playback(after_playback_id=0, timeout=2)

        result = node.wait_for_playback(
            after_playback_id=first["playback_id"], timeout=0,
        )

        self.assertEqual(result["state"], "timeout")
        self.assertEqual(result["reason"], "playback_not_started")
        node.stop_play()

    def test_wait_requires_an_explicit_exact_or_after_id(self):
        node, _client = self._node()

        self.assertEqual(
            node.wait_for_playback(timeout=0),
            {
                "state": "error",
                "reason": "playback_id_or_after_playback_id_required",
            },
        )
        node.stop_play()

    def test_idle_interrupt_does_not_mute_next_utterance(self):
        node, _client = self._node()

        result = node.interrupt()
        self.assertEqual(result["state"], "ready")
        self.assertFalse(node._muted)

        self._publish_utterance(node)
        completed = node.wait_for_playback(after_playback_id=0, timeout=2)
        self.assertEqual(completed["state"], "completed")
        node.stop_play()

    def test_interrupt_before_eof_mutes_only_until_old_eof(self):
        node, _client = self._node()
        node._on_chunk(_AudioChunk(b"\x00\x00" * 1600))
        first_id = node._playback_sequence

        result = node.interrupt()
        self.assertEqual(result["state"], "ready")
        self.assertTrue(node._muted)
        self.assertEqual(
            node.wait_for_playback(playback_id=first_id, timeout=0)["state"],
            "cancelled",
        )

        node._on_chunk(_AudioChunk(node.AUDIO_EOF_MAGIC))
        self.assertFalse(node._muted)
        self._publish_utterance(node)
        completed = node.wait_for_playback(
            after_playback_id=first_id, timeout=2,
        )
        self.assertEqual(completed["state"], "completed")
        node.stop_play()

    def test_interrupt_after_eof_does_not_mute_next_utterance(self):
        node, _client = self._node(_Monitor(["completed", "completed"]))
        node._on_chunk(_AudioChunk(b"\x00\x00" * 1600))
        node._on_chunk(_AudioChunk(node.AUDIO_EOF_MAGIC))

        node.interrupt()
        self.assertFalse(node._muted)
        first_id = node._playback_sequence
        self._publish_utterance(node)
        completed = node.wait_for_playback(
            after_playback_id=first_id, timeout=2,
        )
        self.assertEqual(completed["state"], "completed")
        node.stop_play()

    def test_stop_cancels_open_playback(self):
        node, _client = self._node()
        node._on_chunk(_AudioChunk(b"\x00\x00" * 1600))
        playback_id = node._playback_sequence

        stopped = node.stop_play()

        self.assertEqual(stopped["state"], "idle")
        result = node.wait_for_playback(
            playback_id=playback_id, timeout=0,
        )
        self.assertEqual(result["state"], "cancelled")
        self.assertEqual(result["reason"], "speaker_stopped")

    def test_missing_eof_never_reports_completed(self):
        node, _client = self._node()
        node._on_chunk(_AudioChunk(b"\x00\x00" * 1600))
        playback_id = node._playback_sequence

        node._check_flush()
        result = node.wait_for_playback(
            playback_id=playback_id, timeout=0.3,
        )

        self.assertEqual(result["state"], "timeout")
        self.assertEqual(result["reason"], "playback_not_finished")
        node.stop_play()

    def test_two_utterances_can_queue_without_mixing_stream_ids(self):
        node, client = self._node(_Monitor(["completed", "completed"]))

        self._publish_utterance(node)
        self._publish_utterance(node)
        first = node.wait_for_playback(playback_id=1, timeout=2)
        second = node.wait_for_playback(playback_id=2, timeout=2)

        self.assertEqual(first["state"], "completed")
        self.assertEqual(second["state"], "completed")
        self.assertNotEqual(first["stream_id"], second["stream_id"])
        self.assertEqual(len(client.stream_calls), 2)
        node.stop_play()

    def test_stuck_worker_blocks_reuse_until_it_really_exits(self):
        node, client = self._node()
        client.block_stream = True
        for _ in range(3):
            node._on_chunk(_AudioChunk(b"\x00\x00" * 1600))
        self.assertTrue(client.stream_entered.wait(timeout=1))

        interrupted = node.interrupt()

        self.assertEqual(interrupted["state"], "error")
        self.assertEqual(interrupted["reason"], "speaker_worker_stop_timeout")
        self.assertTrue(node._interrupt_flag.is_set())
        with self.assertRaisesRegex(RuntimeError, "still stopping"):
            node.start_play("/test/new")

        client.stream_release.set()
        deadline = time.monotonic() + 2
        while (node._drain_thread is not None
               and node._drain_thread.is_alive()
               and time.monotonic() < deadline):
            time.sleep(0.01)
        recovered = node.stop_play()
        self.assertEqual(recovered["state"], "idle")

    def test_mcp_contract_requires_a_correlation_id(self):
        node, _client = self._node()
        plugin = DEVICE.SpeakerPlugin.__new__(DEVICE.SpeakerPlugin)
        plugin._node = node

        schema = plugin.get_tool()["inputSchema"]
        self.assertIn("wait_playback", schema["properties"]["action"]["enum"])
        self.assertIn("playback_id", schema["properties"])
        self.assertIn("after_playback_id", schema["properties"])
        self.assertEqual(
            plugin.dispatch("wait_playback", {"timeout_sec": 0})["reason"],
            "playback_id_or_after_playback_id_required",
        )
        self.assertEqual(
            plugin.dispatch(
                "wait_playback",
                {
                    "playback_id": 1,
                    "after_playback_id": 0,
                    "timeout_sec": 0,
                },
            )["reason"],
            "choose_playback_id_or_after_playback_id",
        )
        node.stop_play()

    def test_native_tts_is_rejected_while_pcm_completion_mode_is_enabled(self):
        node, client = self._node()
        native = DEVICE.NativeTtsPlugin({}, "test", None, client)

        blocked = native.dispatch("speak", {"text": "hello", "voice": 0})
        self.assertEqual(blocked["state"], "error")
        self.assertEqual(
            blocked["reason"],
            "native_tts_disabled_for_pcm_completion",
        )
        self.assertEqual(client.tts_calls, [])

        node.stop_play()
        still_blocked = native.dispatch(
            "speak", {"text": "hello", "voice": 0},
        )
        self.assertEqual(still_blocked["state"], "error")
        self.assertEqual(client.tts_calls, [])

    def test_active_pause_is_rejected_without_stopping_audio(self):
        node, client = self._node()
        node._on_chunk(_AudioChunk(b"\x00\x00" * 1600))
        stop_calls_before = len(client.stop_calls)

        paused = node.pause()

        self.assertEqual(paused["state"], "error")
        self.assertEqual(paused["reason"], "active_pcm_pause_unsupported")
        self.assertEqual(len(client.stop_calls), stop_calls_before)
        node._on_chunk(_AudioChunk(node.AUDIO_EOF_MAGIC))
        completed = node.wait_for_playback(
            playback_id=node._playback_sequence, timeout=2,
        )
        self.assertEqual(completed["state"], "completed")
        node.stop_play()

    def test_play_stop_failure_blocks_reuse_and_never_reports_idle(self):
        node, client = self._node()
        client.stop_code = 37

        failed = node.stop_play()

        self.assertEqual(failed["state"], "error")
        self.assertEqual(failed["reason"], "play_stop_failed:37")
        self.assertTrue(node._interrupt_flag.is_set())
        with self.assertRaisesRegex(RuntimeError, "still stopping"):
            node.start_play("/test/reuse")

        client.stop_code = 0
        recovered = node.stop_play()
        self.assertEqual(recovered["state"], "idle")

    def test_interrupt_during_forced_cleanup_stays_cancelled(self):
        monitor = _ForcedCleanupMonitor()
        node, _client = self._node(monitor)
        self._publish_utterance(node)
        self.assertTrue(monitor.cleanup_entered.wait(timeout=2))
        playback_id = node._playback_sequence

        interrupted = node.interrupt()
        result = node.wait_for_playback(
            playback_id=playback_id, timeout=1,
        )

        self.assertEqual(interrupted["state"], "ready")
        self.assertEqual(result["state"], "cancelled")
        self.assertNotEqual(
            result["reason"],
            "playback_not_complete_before_forced_close",
        )
        node.stop_play()

    def test_eof_interrupt_race_never_mutes_next_generation(self):
        for _ in range(10):
            node, _client = self._node(_Monitor(["completed", "completed"]))
            node._on_chunk(_AudioChunk(b"\x00\x00" * 1600))
            first_id = node._playback_sequence
            barrier = threading.Barrier(3)

            def send_eof():
                barrier.wait()
                node._on_chunk(_AudioChunk(node.AUDIO_EOF_MAGIC))

            def interrupt():
                barrier.wait()
                node.interrupt()

            eof_thread = threading.Thread(target=send_eof)
            interrupt_thread = threading.Thread(target=interrupt)
            eof_thread.start()
            interrupt_thread.start()
            barrier.wait()
            eof_thread.join(timeout=2)
            interrupt_thread.join(timeout=2)

            self.assertFalse(node._muted)
            self._publish_utterance(node)
            result = node.wait_for_playback(
                after_playback_id=first_id, timeout=2,
            )
            self.assertEqual(result["state"], "completed")
            node.stop_play()

    def test_late_callback_cannot_cross_stop_and_restart_generation(self):
        node, _client = self._node()
        old_callback = node._sub.callback
        blocking_data = _BlockingBytes(b"\x00\x00" * 1600)
        message = types.SimpleNamespace(data=blocking_data)
        callback_error = []

        def invoke_old_callback():
            try:
                old_callback(message)
            except Exception as exc:  # pragma: no cover - asserted below
                callback_error.append(exc)

        callback_thread = threading.Thread(target=invoke_old_callback)
        callback_thread.start()
        self.assertTrue(blocking_data.entered.wait(timeout=1))

        self.assertEqual(node.stop_play()["state"], "idle")
        node.start_play("/test/restarted")
        blocking_data.release.set()
        callback_thread.join(timeout=1)

        self.assertFalse(callback_thread.is_alive())
        self.assertEqual(callback_error, [])
        self.assertEqual(node._playback_sequence, 0)
        self.assertIsNone(node._current_input_id)
        self.assertTrue(node._buf.empty())
        self.assertEqual(node.state, "ready")
        node.stop_play()

    def test_stop_and_smart_motion_interrupt_leave_no_false_ready_state(self):
        for _ in range(50):
            node, _client = self._node()
            barrier = threading.Barrier(3)
            results = []

            def stop():
                barrier.wait()
                results.append(node.stop_play())

            def interrupt():
                barrier.wait()
                results.append(node.interrupt())

            stop_thread = threading.Thread(target=stop)
            interrupt_thread = threading.Thread(target=interrupt)
            stop_thread.start()
            interrupt_thread.start()
            barrier.wait()
            stop_thread.join(timeout=1)
            interrupt_thread.join(timeout=1)

            self.assertFalse(stop_thread.is_alive())
            self.assertFalse(interrupt_thread.is_alive())
            self.assertEqual(len(results), 2)
            self.assertEqual(node.state, "idle")
            self.assertIsNone(node._sub)
            self.assertIsNone(node._topic)
            self.assertIsNone(node._active_subscription_generation)

    def test_failed_forced_stop_cancels_queued_audio_and_blocks_reuse(self):
        monitor = _GatedNaturalTimeoutMonitor()
        node, client = self._node(monitor)
        self._publish_utterance(node)
        self.assertTrue(monitor.natural_wait_entered.wait(timeout=1))
        self._publish_utterance(node)
        client.stop_code = 37
        monitor.natural_wait_release.set()

        first = node.wait_for_playback(playback_id=1, timeout=1)
        second = node.wait_for_playback(playback_id=2, timeout=1)

        self.assertEqual(first["state"], "error")
        self.assertEqual(first["reason"], "play_stop_failed:37")
        self.assertEqual(second["state"], "cancelled")
        self.assertEqual(second["reason"], "play_stop_failed:37")
        self.assertEqual(len(client.stream_calls), 1)
        self.assertEqual(node.state, "error")
        self.assertTrue(node._interrupt_flag.is_set())
        sequence = node._playback_sequence
        self._publish_utterance(node)
        self.assertEqual(node._playback_sequence, sequence)
        with self.assertRaisesRegex(RuntimeError, "still stopping"):
            node.start_play("/test/reuse")

        client.stop_code = 0
        self.assertEqual(node.stop_play()["state"], "idle")

    def test_unconfirmed_idle_after_forced_stop_blocks_queued_audio(self):
        monitor = _GatedNaturalTimeoutMonitor(cleanup_state="timeout")
        node, client = self._node(monitor)
        self._publish_utterance(node)
        self.assertTrue(monitor.natural_wait_entered.wait(timeout=1))
        self._publish_utterance(node)
        monitor.natural_wait_release.set()

        first = node.wait_for_playback(playback_id=1, timeout=1)
        second = node.wait_for_playback(playback_id=2, timeout=1)

        self.assertEqual(first["state"], "error")
        self.assertEqual(
            first["reason"],
            "playback_not_complete_before_forced_close",
        )
        self.assertEqual(second["state"], "cancelled")
        self.assertEqual(
            second["reason"],
            "hardware_idle_unconfirmed_after_forced_close",
        )
        self.assertEqual(len(client.stream_calls), 1)
        self.assertEqual(node.state, "error")
        self.assertTrue(node._interrupt_flag.is_set())
        self.assertEqual(node.stop_play()["state"], "idle")

    def test_midstream_rpc_failure_stops_and_blocks_later_audio(self):
        node, client = self._node(_Monitor(["completed"]))
        client.stream_codes = [0, 37]
        client.block_stream_call = 2
        block = b"\x00\x00" * 4800

        # Three full merge blocks start the drain.  Hold the second RPC so an
        # EOF and a later utterance are deterministically queued behind it.
        for _ in range(3):
            node._on_chunk(_AudioChunk(block))
        deadline = time.monotonic() + 2
        while (len(client.stream_calls) < 2
               and time.monotonic() < deadline):
            time.sleep(0.01)
        self.assertEqual(len(client.stream_calls), 2)

        node._on_chunk(_AudioChunk(node.AUDIO_EOF_MAGIC))
        self._publish_utterance(node)
        client.stream_release.set()

        first = node.wait_for_playback(playback_id=1, timeout=2)
        second = node.wait_for_playback(playback_id=2, timeout=2)

        self.assertEqual(first["state"], "error")
        self.assertEqual(first["reason"], "play_stream_failed")
        self.assertEqual(first["stream_error"], "rpc_code:37")
        self.assertEqual(first["play_stop_code"], 0)
        self.assertEqual(second["state"], "cancelled")
        self.assertEqual(second["reason"], "play_stream_failed")
        self.assertEqual(len(client.stream_calls), 2)
        self.assertEqual(node.state, "error")
        self.assertTrue(node._interrupt_flag.is_set())

        interrupted = node.interrupt()
        self.assertEqual(interrupted["state"], "error")
        self.assertEqual(interrupted["reason"], "speaker_stop_required")
        self.assertEqual(
            interrupted["fail_closed_reason"], "play_stream_failed",
        )
        self.assertTrue(node._interrupt_flag.is_set())
        sequence = node._playback_sequence
        self._publish_utterance(node)
        self.assertEqual(node._playback_sequence, sequence)
        self.assertEqual(len(client.stream_calls), 2)

        self.assertEqual(node.stop_play()["state"], "idle")
        node.start_play("/test/recovered")
        self._publish_utterance(node)
        recovered = node.wait_for_playback(
            after_playback_id=sequence, timeout=2,
        )
        self.assertEqual(recovered["state"], "completed")
        self.assertEqual(len(client.stream_calls), 3)
        node.stop_play()

    def test_stream_exception_is_fail_closed_and_cancels_later_audio(self):
        node, client = self._node(_Monitor(["completed"]))
        client.stream_exceptions = [OSError("ack lost")]
        self.assertEqual(node.pause()["state"], "paused")
        self._publish_utterance(node)
        self._publish_utterance(node)

        self.assertEqual(node.resume()["state"], "playing")
        first = node.wait_for_playback(playback_id=1, timeout=2)
        second = node.wait_for_playback(playback_id=2, timeout=2)

        self.assertEqual(first["state"], "error")
        self.assertEqual(first["reason"], "play_stream_failed")
        self.assertEqual(first["stream_error"], "OSError:ack lost")
        self.assertEqual(second["state"], "cancelled")
        self.assertEqual(second["reason"], "play_stream_failed")
        self.assertEqual(len(client.stream_calls), 1)
        self.assertEqual(node.state, "error")
        self.assertTrue(node._interrupt_flag.is_set())
        self.assertEqual(node.stop_play()["state"], "idle")

    def test_interrupt_during_failed_stream_cleanup_cannot_clear_latch(self):
        monitor = _InterruptibleFailureCleanupMonitor()
        node, client = self._node(monitor)
        client.stream_code = 37
        self._publish_utterance(node)
        self.assertTrue(monitor.cleanup_entered.wait(timeout=1))

        interrupted = node.interrupt()

        self.assertEqual(interrupted["state"], "error")
        self.assertEqual(interrupted["reason"], "speaker_stop_required")
        self.assertEqual(
            interrupted["fail_closed_reason"], "play_stream_failed",
        )
        self.assertEqual(node.state, "error")
        self.assertEqual(node._fail_closed_reason, "play_stream_failed")
        self.assertTrue(node._interrupt_flag.is_set())
        sequence = node._playback_sequence
        self._publish_utterance(node)
        self.assertEqual(node._playback_sequence, sequence)
        self.assertEqual(len(client.stream_calls), 1)

        client.stop_code = 0
        self.assertEqual(node.stop_play()["state"], "idle")
        self.assertIsNone(node._fail_closed_reason)

    def test_stop_cancels_startup_without_deadlock(self):
        node, _client = self._node()
        plugin = DEVICE.SpeakerPlugin.__new__(DEVICE.SpeakerPlugin)
        plugin._node = node
        plugin._lifecycle_lock = node._lifecycle_lock
        startup_entered = threading.Event()

        def cancellable_startup():
            startup_entered.set()
            node._startup_cancel.wait(timeout=2)
            return {"state": "cancelled", "reason": "startup_cancelled"}

        plugin._play_startup_sound = cancellable_startup
        start_result = []

        start_thread = threading.Thread(
            target=lambda: start_result.append(
                plugin.dispatch(
                    "start", {"input_topic": "/test/after-startup"},
                )
            ),
        )
        start_thread.start()
        self.assertTrue(startup_entered.wait(timeout=1))

        stopped = plugin.dispatch("stop", {})
        start_thread.join(timeout=2)

        self.assertFalse(start_thread.is_alive())
        self.assertEqual(stopped["state"], "idle")
        self.assertEqual(start_result[0]["state"], "error")
        self.assertEqual(
            start_result[0]["reason"], "startup_audio_not_settled",
        )
        self.assertEqual(node.state, "idle")
        self.assertIsNone(node._sub)

    def test_stop_during_internal_start_cleanup_is_not_cleared(self):
        node, client = self._node()
        plugin = DEVICE.SpeakerPlugin.__new__(DEVICE.SpeakerPlugin)
        plugin._node = node
        plugin._lifecycle_lock = node._lifecycle_lock
        original_stop_play = node.stop_play
        cleanup_finished = threading.Event()
        allow_cleanup_return = threading.Event()
        startup_submissions = []

        def gated_stop_play(*, cancel_startup=True):
            result = original_stop_play(cancel_startup=cancel_startup)
            if not cancel_startup:
                cleanup_finished.set()
                allow_cleanup_return.wait(timeout=2)
            return result

        def startup_probe():
            if node._startup_cancel.is_set():
                return {"state": "cancelled", "reason": "startup_cancelled"}
            startup_submissions.append(True)
            client.PlayStream("startup", "startup", b"\x00\x00")
            return {"state": "completed"}

        node.stop_play = gated_stop_play
        plugin._play_startup_sound = startup_probe
        start_result = []
        stop_result = []
        start_thread = threading.Thread(
            target=lambda: start_result.append(
                plugin.dispatch("start", {"input_topic": "/test/start-race"})
            ),
        )
        start_thread.start()
        self.assertTrue(cleanup_finished.wait(timeout=1))

        stop_thread = threading.Thread(
            target=lambda: stop_result.append(plugin.dispatch("stop", {})),
        )
        stop_thread.start()
        deadline = time.monotonic() + 1
        while (not node._startup_cancel.is_set()
               and time.monotonic() < deadline):
            time.sleep(0.005)
        self.assertTrue(node._startup_cancel.is_set())
        allow_cleanup_return.set()
        start_thread.join(timeout=2)
        stop_thread.join(timeout=2)

        self.assertFalse(start_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(startup_submissions, [])
        self.assertEqual(start_result[0]["state"], "error")
        self.assertEqual(stop_result[0]["state"], "idle")
        self.assertEqual(node.state, "idle")


if __name__ == "__main__":
    unittest.main()
