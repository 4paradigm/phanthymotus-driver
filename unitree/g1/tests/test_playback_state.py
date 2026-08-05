import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


G1_DIR = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'g1_playback_state_under_test', G1_DIR / 'playback_state.py',
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
            idle_settle_sec=0,
        )
        self.key = (7, 'utt:test')

    def test_correlates_playing_then_idle_after_final_submission(self):
        first_checkpoint = self.monitor.checkpoint()
        self.monitor.note_submission(self.key, first_checkpoint)
        self.assertTrue(self.monitor.on_raw_message('{"play_state":1}'))

        final_checkpoint = self.monitor.checkpoint()
        self.monitor.note_submission(self.key, final_checkpoint)
        self.clock.advance(1.2)
        self.assertTrue(self.monitor.on_raw_message('{"play_state":0}'))

        result = self.monitor.wait_for_completion(self.key, timeout=0)

        self.assertEqual(result['state'], 'completed')
        self.assertEqual(result['playing_seq'], 1)
        self.assertEqual(result['idle_seq'], 2)
        self.assertAlmostEqual(
            result['idle_ts'] - result['playing_ts'], 1.2,
        )

    def test_idle_before_final_submission_cannot_complete(self):
        self.monitor.note_submission(self.key, self.monitor.checkpoint())
        self.monitor.on_raw_message('{"play_state":1}')
        self.monitor.on_raw_message('{"play_state":0}')
        self.monitor.note_submission(self.key, self.monitor.checkpoint())

        result = self.monitor.wait_for_completion(self.key, timeout=0)

        self.assertEqual(result, {
            'state': 'timeout',
            'reason': 'play_state_idle_timeout',
        })

        self.monitor.on_raw_message('{"play_state":1}')
        self.monitor.on_raw_message('{"play_state":0}')
        self.assertEqual(
            self.monitor.wait_for_completion(self.key, timeout=0)['state'],
            'completed',
        )

    def test_idle_racing_between_checkpoint_and_submit_ack_cannot_complete(self):
        self.monitor.note_submission(self.key, self.monitor.checkpoint())
        self.monitor.on_raw_message('{"play_state":1}')

        final_checkpoint = self.monitor.checkpoint()
        # The preceding block drains while the next PlayStream RPC is in
        # flight. note_submission happens only after that RPC succeeds.
        self.monitor.on_raw_message('{"play_state":0}')
        self.monitor.note_submission(self.key, final_checkpoint)

        self.assertEqual(
            self.monitor.wait_for_completion(self.key, timeout=0),
            {'state': 'timeout', 'reason': 'play_state_idle_timeout'},
        )

        self.monitor.on_raw_message('{"play_state":1}')
        self.monitor.on_raw_message('{"play_state":0}')
        self.assertEqual(
            self.monitor.wait_for_completion(self.key, timeout=0)['state'],
            'completed',
        )

    def test_later_playing_invalidates_idle_until_a_new_idle_arrives(self):
        self.monitor.note_submission(self.key, self.monitor.checkpoint())
        self.monitor.on_raw_message('{"play_state":1}')
        self.monitor.on_raw_message('{"play_state":0}')
        self.monitor.on_raw_message('{"play_state":1}')

        self.assertEqual(
            self.monitor.wait_for_completion(self.key, timeout=0),
            {'state': 'timeout', 'reason': 'play_state_idle_timeout'},
        )

        self.monitor.on_raw_message('{"play_state":0}')
        self.assertEqual(
            self.monitor.wait_for_completion(self.key, timeout=0)['state'],
            'completed',
        )

    def test_idle_must_remain_stable_for_settle_window(self):
        monitor = PLAYBACK_STATE.G1PlaybackStateMonitor(
            available=True,
            monotonic=self.clock.monotonic,
            idle_settle_sec=0.1,
        )
        monitor.note_submission(self.key, monitor.checkpoint())
        monitor.on_raw_message('{"play_state":1}')
        monitor.on_raw_message('{"play_state":0}')

        self.assertEqual(
            monitor.wait_for_completion(self.key, timeout=0)['state'],
            'timeout',
        )
        self.clock.advance(0.11)
        self.assertEqual(
            monitor.wait_for_completion(self.key, timeout=0)['state'],
            'completed',
        )

    def test_idle_without_playing_fails_start_timeout(self):
        self.monitor.note_submission(self.key, self.monitor.checkpoint())
        self.monitor.on_raw_message('{"play_state":0}')

        self.assertEqual(
            self.monitor.wait_for_completion(self.key, timeout=0),
            {'state': 'timeout', 'reason': 'play_state_start_timeout'},
        )

    def test_unavailable_monitor_fails_closed(self):
        monitor = PLAYBACK_STATE.G1PlaybackStateMonitor(
            connect_error='dds_subscribe_failed:RuntimeError',
        )

        self.assertEqual(
            monitor.wait_for_completion(self.key, timeout=0),
            {
                'state': 'error',
                'reason': 'dds_subscribe_failed:RuntimeError',
            },
        )

    def test_missing_observation_is_error(self):
        self.assertEqual(
            self.monitor.wait_for_completion(self.key, timeout=0),
            {
                'state': 'error',
                'reason': 'play_state_observation_missing',
            },
        )

    def test_cancel_wins_over_a_late_idle(self):
        self.monitor.note_submission(self.key, self.monitor.checkpoint())
        self.monitor.on_raw_message('{"play_state":1}')
        self.monitor.on_raw_message('{"play_state":0}')

        self.assertEqual(
            self.monitor.wait_for_completion(
                self.key, timeout=1, cancelled=lambda: True,
            ),
            {'state': 'interrupted', 'reason': 'interrupted'},
        )

    def test_parses_both_generated_string_field_names(self):
        self.monitor.on_dds_message(types.SimpleNamespace(
            data='{"play_state":1}',
        ))
        self.monitor.on_dds_message(types.SimpleNamespace(
            data_='{"play_state":0}',
        ))

        status = self.monitor.status()
        self.assertEqual(status['event_seq'], 2)
        self.assertEqual(status['current_state'], 0)

    def test_subscription_matching_controls_readiness(self):
        self.monitor.on_subscription_matched(0)
        self.assertEqual(
            self.monitor.wait_until_ready(timeout=0),
            {'state': 'timeout', 'reason': 'play_state_publisher_timeout'},
        )

        self.monitor.on_subscription_matched(1)
        self.assertEqual(
            self.monitor.wait_until_ready(timeout=0),
            {'state': 'ready', 'reason': ''},
        )
        self.assertTrue(self.monitor.status()['ready'])
        self.assertEqual(self.monitor.status()['matched_publishers'], 1)

    def test_connect_dds_uses_direct_callback_and_match_handler(self):
        created = []

        class _Subscriber:
            def __init__(self, topic, message_type):
                self.topic = topic
                self.message_type = message_type
                self.init_args = None
                created.append(self)

            def Init(self, handler, queue_len, match_handler):
                self.init_args = (handler, queue_len, match_handler)
                match_handler(1)

            def Close(self):
                pass

        channel_module = types.ModuleType('unitree_sdk2py.core.channel')
        channel_module.ChannelSubscriber = _Subscriber
        message_module = types.ModuleType(
            'unitree_sdk2py.idl.std_msgs.msg.dds_',
        )
        message_module.String_ = type('String_', (), {})
        modules = {
            'unitree_sdk2py': types.ModuleType('unitree_sdk2py'),
            'unitree_sdk2py.core': types.ModuleType('unitree_sdk2py.core'),
            'unitree_sdk2py.core.channel': channel_module,
            'unitree_sdk2py.idl': types.ModuleType('unitree_sdk2py.idl'),
            'unitree_sdk2py.idl.std_msgs': types.ModuleType(
                'unitree_sdk2py.idl.std_msgs',
            ),
            'unitree_sdk2py.idl.std_msgs.msg': types.ModuleType(
                'unitree_sdk2py.idl.std_msgs.msg',
            ),
            'unitree_sdk2py.idl.std_msgs.msg.dds_': message_module,
        }

        with mock.patch.dict(sys.modules, modules):
            monitor = PLAYBACK_STATE.G1PlaybackStateMonitor.connect_dds()

        self.assertTrue(monitor.available)
        self.assertTrue(monitor.status()['ready'])
        self.assertEqual(created[0].init_args[1], 0)
        self.assertEqual(
            created[0].init_args[2].__self__, monitor,
        )

    def test_ignores_non_state_audio_messages(self):
        invalid = [
            None,
            '',
            'not-json',
            '[]',
            '{"text":"hello"}',
            '{"play_state":true}',
            '{"play_state":2}',
            '{"play_state":"1"}',
        ]

        self.assertEqual(
            [self.monitor.on_raw_message(value) for value in invalid],
            [False] * len(invalid),
        )
        self.assertEqual(self.monitor.status()['event_seq'], 0)

    def test_forget_prevents_stale_session_reuse(self):
        self.monitor.note_submission(self.key, self.monitor.checkpoint())
        self.monitor.on_raw_message('{"play_state":1}')
        self.monitor.on_raw_message('{"play_state":0}')
        self.monitor.forget(self.key)

        self.assertEqual(
            self.monitor.wait_for_completion(self.key, timeout=0)['reason'],
            'play_state_observation_missing',
        )


if __name__ == '__main__':
    unittest.main()
