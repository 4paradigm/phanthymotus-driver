import importlib.util
import unittest
from pathlib import Path


G1_DIR = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "g1_playback_state_under_test", G1_DIR / "playback_state.py",
)
PLAYBACK_STATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLAYBACK_STATE)


class _Clock:
    def __init__(self):
        self.value = 10.0

    def monotonic(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class PlaybackStateMonitorTests(unittest.TestCase):
    def setUp(self):
        self.clock = _Clock()
        self.monitor = PLAYBACK_STATE.G1PlaybackStateMonitor(
            available=True,
            monotonic=self.clock.monotonic,
            idle_settle_sec=0.35,
        )

    def test_waits_for_playing_then_stable_idle_after_pcm_deadline(self):
        checkpoint = self.monitor.checkpoint()
        self.monitor.on_raw_message('{"play_state":1}')
        self.clock.advance(0.5)
        pcm_deadline = self.clock.monotonic()
        self.monitor.on_raw_message('{"play_state":0}')

        self.assertEqual(
            self.monitor.wait_for_stable_idle(
                after_seq=checkpoint,
                not_before=pcm_deadline,
                timeout=0,
            )["state"],
            "timeout",
        )

        self.clock.advance(0.36)
        result = self.monitor.wait_for_stable_idle(
            after_seq=checkpoint,
            not_before=pcm_deadline,
            timeout=0,
        )
        self.assertEqual(result["state"], "completed")
        self.assertEqual(result["playing_seq"], 1)
        self.assertEqual(result["idle_seq"], 2)

    def test_final_zero_one_zero_bounce_uses_last_idle(self):
        checkpoint = self.monitor.checkpoint()
        self.monitor.on_raw_message('{"play_state":1}')
        self.clock.advance(1.0)
        pcm_deadline = self.clock.monotonic()
        self.monitor.on_raw_message('{"play_state":0}')
        self.clock.advance(0.08)
        self.monitor.on_raw_message('{"play_state":1}')
        self.clock.advance(0.18)
        self.monitor.on_raw_message('{"play_state":0}')
        self.clock.advance(0.36)

        result = self.monitor.wait_for_stable_idle(
            after_seq=checkpoint,
            not_before=pcm_deadline,
            timeout=0,
        )

        self.assertEqual(result["state"], "completed")
        self.assertEqual(result["playing_seq"], 3)
        self.assertEqual(result["idle_seq"], 4)

    def test_does_not_gate_valid_idle_on_rpc_ack_order(self):
        checkpoint = self.monitor.checkpoint()
        self.monitor.on_raw_message('{"play_state":1}')
        pcm_deadline = self.clock.monotonic()
        # The final DDS idle can arrive before PlayStream returns.  There is no
        # post-ACK gate here; stable firmware idle remains valid.
        self.monitor.on_raw_message('{"play_state":0}')
        self.clock.advance(0.36)

        self.assertEqual(
            self.monitor.wait_for_stable_idle(
                after_seq=checkpoint,
                not_before=pcm_deadline,
                timeout=0,
            )["state"],
            "completed",
        )

    def test_idle_from_an_earlier_block_cannot_finish_final_block(self):
        stream_start = self.monitor.checkpoint()
        self.monitor.on_raw_message('{"play_state":1}')
        self.monitor.on_raw_message('{"play_state":0}')
        final_submit = self.monitor.checkpoint()
        self.clock.advance(1.0)
        pcm_deadline = self.clock.monotonic()
        self.clock.advance(0.36)

        result = self.monitor.wait_for_stable_idle(
            after_seq=stream_start,
            idle_after_seq=final_submit,
            not_before=pcm_deadline,
            timeout=0,
        )
        self.assertEqual(result["state"], "timeout")
        self.assertEqual(result["reason"], "play_state_idle_timeout")

        # A final idle after the pre-send checkpoint is valid even if the
        # firmware does not repeat play_state=1 for a continuous stream.
        self.monitor.on_raw_message('{"play_state":0}')
        self.clock.advance(0.36)
        self.assertEqual(
            self.monitor.wait_for_stable_idle(
                after_seq=stream_start,
                idle_after_seq=final_submit,
                not_before=pcm_deadline,
                timeout=0,
            )["state"],
            "completed",
        )

    def test_requires_playing_after_checkpoint(self):
        self.monitor.on_raw_message('{"play_state":1}')
        checkpoint = self.monitor.checkpoint()
        self.monitor.on_raw_message('{"play_state":0}')
        self.clock.advance(1.0)

        result = self.monitor.wait_for_stable_idle(
            after_seq=checkpoint,
            not_before=self.clock.monotonic(),
            timeout=0,
        )

        self.assertEqual(result["state"], "timeout")
        self.assertEqual(result["reason"], "play_state_start_timeout")

    def test_rejects_invalid_messages(self):
        for raw in (
            None,
            "",
            "not-json",
            "[]",
            '{"play_state":true}',
            '{"play_state":2}',
            '{"text":"hello"}',
        ):
            self.assertFalse(self.monitor.on_raw_message(raw))
        self.assertEqual(self.monitor.status()["event_seq"], 0)

    def test_unavailable_monitor_fails_closed(self):
        monitor = PLAYBACK_STATE.G1PlaybackStateMonitor(
            connect_error="dds_subscribe_failed:RuntimeError",
        )

        self.assertEqual(
            monitor.wait_for_stable_idle(
                after_seq=0,
                not_before=0,
                timeout=0,
            ),
            {
                "state": "error",
                "reason": "dds_subscribe_failed:RuntimeError",
            },
        )


if __name__ == "__main__":
    unittest.main()
