"""
Tests for the G1/R1 speaker drain: prefill accounting, EOF flush, and block pacing.

The drain logic is pure arithmetic over a byte queue and a monotonic clock, so it
is testable without ROS2, the MCU, or a real AudioClient. Three behaviours are
pinned here, each of which was wrong at some point and is inaudible in code review:

  * Prefill must be re-satisfied for every utterance. _pending_bytes counts bytes
    rather than chunks (chunk size is the TTS's choice), and it must only count
    what accumulates while no drain thread is running — otherwise it climbs to the
    whole utterance's size, and after the drain exits on EXIT_AFTER_IDLE the next
    utterance's first chunk trips the threshold and playback starts starved.
  * EOF flushes the tail immediately without killing the drain thread, so a short
    utterance does not pay the flush-timer + idle-check latency.
  * PlayStream pacing keeps a *bounded* lead over the audio timeline. The lead
    must not grow with utterance length, and must not go unbounded when the call
    itself is slower than realtime.

Run:  python3 -m unittest discover -s unitree/g1/tests -t .
"""

import importlib.util
import queue
import sys
import threading
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DRIVER_ROOT = ROOT.parents[1]
sys.path.insert(0, str(ROOT))


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")


class FakeNode:
    """Node stand-in. create_timer returns a handle stop_play/_start_drain can cancel."""

    def __init__(self, name):
        self._name = name
        self.timers = []

    def create_publisher(self, *a, **kw):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, *a, **kw):
        return object()

    def create_timer(self, period, cb):
        t = types.SimpleNamespace(period=period, callback=cb,
                                  cancel=lambda: None, cancelled=False)
        self.timers.append(t)
        return t

    def destroy_timer(self, timer):
        if timer in self.timers:
            self.timers.remove(timer)

    def destroy_subscription(self, sub):
        pass

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                     error=lambda *a: None, debug=lambda *a: None)


def install_stubs():
    rclpy = types.ModuleType("rclpy")
    sys.modules.setdefault("rclpy", rclpy)
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Header = type("Header", (Message,), {})
    std_msgs.String = type("String", (Message,),
                           {"__init__": lambda self: setattr(self, "data", "")})
    std_msgs.UInt8MultiArray = type("UInt8MultiArray", (Message,), {})
    sys.modules["std_msgs.msg"] = std_msgs

    audio_msgs = types.ModuleType("audio_msgs.msg")
    audio_msgs.AudioChunk = type("AudioChunk", (Message,), {})
    sys.modules["audio_msgs.msg"] = audio_msgs

    for name in ("unitree_sdk2py", "unitree_sdk2py.g1", "unitree_sdk2py.g1.audio"):
        sys.modules.setdefault(name, types.ModuleType(name))
    audio_mod = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    audio_mod.AudioClient = type("AudioClient", (), {})
    sys.modules["unitree_sdk2py.g1.audio.g1_audio_client"] = audio_mod

    pcu = types.ModuleType("pointcloud_utils")
    pcu.gravity_align_inplace = lambda *a, **kw: None
    sys.modules["pointcloud_utils"] = pcu

    cdds = types.ModuleType("cyclonedds")
    idl = types.ModuleType("cyclonedds.idl")

    class _IdlStruct:
        def __init_subclass__(cls, **kw):
            pass

    idl.IdlStruct = _IdlStruct
    ann = types.ModuleType("cyclonedds.idl.annotations")
    ann.final = lambda c: c
    ann.autoid = lambda kind: (lambda c: c)
    tps = types.ModuleType("cyclonedds.idl.types")
    tps.uint32 = int
    tps.float32 = float
    idl.annotations = ann
    idl.types = tps
    cdds.idl = idl
    sys.modules.update({"cyclonedds": cdds, "cyclonedds.idl": idl,
                        "cyclonedds.idl.annotations": ann, "cyclonedds.idl.types": tps})


def load_device(path, mod_name):
    install_stubs()
    spec = importlib.util.spec_from_file_location(mod_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = module
    spec.loader.exec_module(module)
    return module


G1 = load_device(ROOT / "device.py", "g1_device_speaker_test")


# ── Fakes ───────────────────────────────────────────────────────────────────

class FakeClock:
    """Monotonic clock advanced only by sleep(), so pacing is deterministic."""

    def __init__(self):
        self.t = 1000.0
        self.sleeps = []

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.sleeps.append(dt)
        self.t += dt

    def advance(self, dt):
        self.t += dt


class FakeAudioClient:
    """Records PlayStream blocks and charges a configurable wall-clock cost."""

    def __init__(self, clock, cost=0.02):
        self.clock = clock
        self.cost = cost
        self.blocks = []
        self.stops = 0

    def PlayStream(self, app, sid, pcm):
        self.blocks.append(pcm)
        self.clock.advance(self.cost)
        return 0, b""

    def PlayStop(self, app):
        self.stops += 1
        return 0, b""


def chunk(nbytes):
    msg = types.SimpleNamespace()
    msg.data = b"\x11\x22" * (nbytes // 2)
    return msg


def eof_msg(node):
    msg = types.SimpleNamespace()
    msg.data = node.AUDIO_EOF_MAGIC
    return msg


def make_node(module, clock, cost=0.02):
    """A _SpeakerNode with the module's `time` swapped for the fake clock."""
    client = FakeAudioClient(clock, cost)
    node = module._SpeakerNode(client)
    node.state = "playing"
    return node


class SpeakerTestBase(unittest.TestCase):
    MODULE = G1

    def setUp(self):
        self.clock = FakeClock()
        self.real_time = self.MODULE.time
        # device.py calls time.monotonic()/time.sleep() at module scope.
        self.MODULE.time = types.SimpleNamespace(
            monotonic=self.clock.monotonic, sleep=self.clock.sleep)
        self.addCleanup(lambda: setattr(self.MODULE, "time", self.real_time))

    def new_node(self, cost=0.02):
        return make_node(self.MODULE, self.clock, cost)


# ── Prefill accounting ──────────────────────────────────────────────────────

class TestPrefill(SpeakerTestBase):

    def test_drain_does_not_start_before_prefill_is_met(self):
        node = self.new_node()
        started = []
        node._start_drain = lambda: started.append(True)
        # 3200B chunks: two are 6400B, still short of PREFILL_BYTES (9600).
        node._on_chunk(chunk(3200))
        node._on_chunk(chunk(3200))
        self.assertEqual(started, [], "drain started before 300ms was buffered")
        node._on_chunk(chunk(3200))
        self.assertEqual(len(started), 1, "drain did not start once prefill was met")

    def test_prefill_is_bytes_not_chunks(self):
        """1024B chunks are legal upstream; 3 of them are 96ms, not 300ms."""
        node = self.new_node()
        started = []
        node._start_drain = lambda: started.append(True)
        for _ in range(3):
            node._on_chunk(chunk(1024))
        self.assertEqual(started, [], "3 small chunks tripped a chunk-count prefill")

    def test_pending_bytes_not_credited_while_draining(self):
        """The regression: bytes consumed by a live drain must not count toward
        the next utterance's prefill."""
        node = self.new_node()
        node._draining.set()          # pretend the drain thread is running
        for _ in range(10):           # a long utterance: 32000 bytes
            node._on_chunk(chunk(3200))
        self.assertEqual(
            node._pending_bytes, 0,
            "bytes consumed by a running drain were credited to prefill")

    def test_prefill_re_satisfied_on_the_next_utterance(self):
        """End-to-end shape of the bug: utterance 1 drains and exits, utterance 2
        must still buffer a full 300ms before playback starts."""
        node = self.new_node()
        node._draining.set()
        for _ in range(10):
            node._on_chunk(chunk(3200))
        # Drain exits on its own (EXIT_AFTER_IDLE), as at the end of an utterance.
        node._draining.clear()
        node._pending_bytes = 0       # what the fixed _drain does on exit

        started = []
        node._start_drain = lambda: started.append(True)
        node._on_chunk(chunk(3200))
        self.assertEqual(
            started, [],
            "utterance 2 started playback on its first chunk — prefill bypassed")


# ── EOF handling ────────────────────────────────────────────────────────────

class TestEofFlush(SpeakerTestBase):

    def test_eof_enqueues_sentinel_while_draining(self):
        node = self.new_node()
        node._draining.set()
        node._on_chunk(eof_msg(node))
        self.assertIs(node._buf.get_nowait(), node._END_OF_UTTERANCE)

    def test_eof_starts_drain_when_idle_with_buffered_audio(self):
        node = self.new_node()
        started = []
        node._start_drain = lambda: started.append(True)
        node._on_chunk(chunk(3200))          # below prefill, so no drain yet
        self.assertEqual(started, [])
        node._on_chunk(eof_msg(node))
        self.assertEqual(len(started), 1,
                         "EOF did not flush a sub-prefill tail")

    def test_eof_only_unmutes_when_muted(self):
        """After interrupt the first EOF is the un-mute signal and must not
        resurrect the discarded utterance."""
        node = self.new_node()
        node._muted = True
        node._buf.put(b"\x00" * 3200)
        started = []
        node._start_drain = lambda: started.append(True)
        node._on_chunk(eof_msg(node))
        self.assertFalse(node._muted)
        self.assertEqual(started, [], "EOF that un-mutes also started playback")

    def test_sentinel_flushes_tail_without_ending_drain(self):
        node = self.new_node()
        node._buf.put(b"\x33" * 3200)
        node._buf.put(node._END_OF_UTTERANCE)
        node._draining.set()

        # Let the drain run until it exits on idle, then confirm the tail played
        # as one block and the thread survived the sentinel to reach EXIT_AFTER_IDLE.
        t = threading.Thread(target=node._drain, daemon=True)
        t.start()
        t.join(timeout=5)
        self.assertFalse(t.is_alive(), "drain thread hung")
        self.assertEqual(len(node._client.blocks), 1)
        self.assertEqual(node._client.blocks[0], b"\x33" * 3200)

    def test_drain_resets_pending_bytes_on_exit(self):
        node = self.new_node()
        node._pending_bytes = 32000
        node._draining.set()
        t = threading.Thread(target=node._drain, daemon=True)
        t.start()
        t.join(timeout=5)
        self.assertFalse(t.is_alive(), "drain thread hung")
        self.assertEqual(node._pending_bytes, 0,
                         "drain exited leaving a stale prefill credit")


# ── Idle thresholds ─────────────────────────────────────────────────────────

class TestIdleThresholds(SpeakerTestBase):

    def test_exit_tolerates_a_stall_longer_than_the_flush_window(self):
        """The core fix: the drain must outlive a brief upstream stall. A 300ms
        exit threshold was shorter than one text chunk's synthesis."""
        node = self.new_node()
        self.assertGreaterEqual(
            node.EXIT_AFTER_IDLE * node.EMPTY_POLL_S, 1.0,
            "drain exits too eagerly to ride out a TTS stall")
        self.assertLess(node.FLUSH_AFTER_IDLE, node.EXIT_AFTER_IDLE,
                        "partial blocks must flush long before the thread exits")

    def test_flush_window_stays_short(self):
        """Flushing the partial block early is what shortens audible silence."""
        node = self.new_node()
        self.assertLessEqual(node.FLUSH_AFTER_IDLE * node.EMPTY_POLL_S, 0.3)


# ── Pacing ──────────────────────────────────────────────────────────────────

class TestPacing(SpeakerTestBase):

    def _lead_after(self, node, nblocks, block_bytes=9600):
        """Audio queued minus wall clock consumed — how far ahead of the MCU we are."""
        deadline = None
        t_start = self.clock.monotonic()
        queued = 0.0
        leads = []
        for i in range(nblocks):
            deadline = node._play_merged(b"\x00" * block_bytes, i + 1, deadline)
            queued += block_bytes / 32000
            leads.append(queued - (self.clock.monotonic() - t_start))
        return leads

    def test_lead_is_bounded_over_a_long_utterance(self):
        node = self.new_node(cost=0.02)
        leads = self._lead_after(node, 40)
        self.assertLessEqual(
            max(leads), node.MAX_LEAD_S + 1e-6,
            "pacing ran further ahead of the MCU than MAX_LEAD_S allows")

    def test_lead_does_not_grow_with_utterance_length(self):
        """The old `duration - elapsed - 0.08` form drifted +80ms per block."""
        node = self.new_node(cost=0.02)
        leads = self._lead_after(node, 40)
        self.assertAlmostEqual(
            leads[4], leads[-1], delta=0.01,
            msg=f"lead drifted from {leads[4]:.3f}s to {leads[-1]:.3f}s")

    def test_slow_playstream_does_not_accrue_unpayable_debt(self):
        """When PlayStream is slower than realtime the deadline re-anchors on now
        instead of banking a debt that would silently skip all later pacing."""
        node = self.new_node(cost=0.5)   # 500ms call for 300ms of audio
        deadline = None
        for i in range(10):
            before = self.clock.monotonic()
            deadline = node._play_merged(b"\x00" * 9600, i + 1, deadline)
            self.assertLessEqual(deadline, self.clock.monotonic() + 1e-9,
                                 "deadline banked a debt it cannot pay off")
            self.assertGreaterEqual(self.clock.monotonic(), before)

    def test_interrupt_short_circuits_before_playing(self):
        node = self.new_node()
        node._interrupt_flag.set()
        node._play_merged(b"\x00" * 9600, 1, None)
        self.assertEqual(node._client.blocks, [],
                         "played a block after interrupt")


# ── R1 parity ───────────────────────────────────────────────────────────────

class TestR1Parity(unittest.TestCase):
    """R1's speaker is a line-for-line copy of G1's; the tuning must not diverge."""

    def test_constants_match(self):
        r1 = load_device(DRIVER_ROOT / "unitree" / "r1" / "device.py",
                         "r1_device_speaker_test")
        for const in ("PREFILL_BYTES", "MERGE_BYTES", "EMPTY_POLL_S",
                      "FLUSH_AFTER_IDLE", "EXIT_AFTER_IDLE", "MAX_LEAD_S"):
            self.assertEqual(
                getattr(G1._SpeakerNode, const),
                getattr(r1._SpeakerNode, const),
                f"{const} diverged between G1 and R1")


class TestR1Speaker(TestPrefill, TestEofFlush, TestPacing):
    """Run the same behavioural suite against R1's copy."""

    @classmethod
    def setUpClass(cls):
        cls.MODULE = load_device(DRIVER_ROOT / "unitree" / "r1" / "device.py",
                                 "r1_device_speaker_test")


if __name__ == "__main__":
    unittest.main(verbosity=2)
