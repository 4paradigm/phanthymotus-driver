import importlib.util
import argparse
import json
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


G1_DIR = Path(__file__).resolve().parents[1]


class _Header:
    def __init__(self):
        self.frame_id = ''
        self.stamp = None


class _AudioChunk:
    def __init__(self):
        self.header = _Header()
        self.format = ''
        self.data = []


class _String:
    def __init__(self, data=''):
        self.data = data


class _Publisher:
    def __init__(self):
        self.messages = []
        self.subscription_count = 1

    def publish(self, msg):
        self.messages.append(msg)

    def get_subscription_count(self):
        return self.subscription_count


class _Subscription:
    def __init__(self):
        self.publisher_count = 1

    def get_publisher_count(self):
        return self.publisher_count


class _Node:
    def __init__(self, _name):
        self.publisher = None
        self.callback = None

    def create_publisher(self, *_args):
        self.publisher = _Publisher()
        return self.publisher

    def create_subscription(self, _type, _topic, callback, _qos):
        self.callback = callback
        return _Subscription()

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: 'stamp'),
        )

    def destroy_node(self):
        pass


class _QoSProfile:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def _package(name):
    module = types.ModuleType(name)
    module.__path__ = []
    return module


def _load_smoke_module():
    rclpy = _package('rclpy')
    rclpy.init = lambda **_kwargs: None
    rclpy.shutdown = lambda: None
    rclpy.spin_once = lambda *_args, **_kwargs: None
    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = _Node
    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(
        BEST_EFFORT='best_effort', RELIABLE='reliable',
    )
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST='keep_last')
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(
        VOLATILE='volatile', TRANSIENT_LOCAL='transient_local',
    )
    audio_msgs = _package('audio_msgs')
    audio_msg = types.ModuleType('audio_msgs.msg')
    audio_msg.AudioChunk = _AudioChunk
    std_msgs = _package('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    std_msg.String = _String
    modules = {
        'rclpy': rclpy,
        'rclpy.node': rclpy_node,
        'rclpy.qos': rclpy_qos,
        'audio_msgs': audio_msgs,
        'audio_msgs.msg': audio_msg,
        'std_msgs': std_msgs,
        'std_msgs.msg': std_msg,
    }
    spec = importlib.util.spec_from_file_location(
        'g1_hardware_smoke_under_test',
        G1_DIR / 'tools/g1_speaker_receipt_smoke.py',
    )
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class HardwareSmokeContractTests(unittest.TestCase):
    def setUp(self):
        self.smoke = _load_smoke_module()

    def test_publishes_correlated_nonzero_pcm_and_finds_terminal(self):
        node = self.smoke._SmokeNode('/review/smoke/')
        utterance_id = 'utt:review-smoke'

        node.publish(utterance_id, self.smoke.PCM_BLOCK)
        message = node.publisher.messages[-1]
        self.assertEqual(message.header.frame_id, utterance_id)
        self.assertEqual(message.format, 'audio/pcm-16k')
        self.assertEqual(len(message.data), 9600)
        self.assertNotEqual(bytes(message.data), bytes(9600))
        self.assertNotEqual(bytes(message.data), self.smoke.AUDIO_EOF_MAGIC)

        terminal = {
            'session_id': 'speaker:test',
            'utterance_id': utterance_id,
            'state': 'completed',
            'completion_basis': 'g1_play_state_observed',
        }
        node._on_receipt(_String(json.dumps(terminal)))
        self.assertEqual(
            node.wait_for_terminal('speaker:test', utterance_id, 0.01),
            terminal,
        )

    def test_waits_for_both_ros_graph_directions(self):
        node = self.smoke._SmokeNode('/review/smoke/')
        self.assertEqual(node.wait_for_graph(0.01), {
            'speaker_input_subscriptions': 1,
            'receipt_publishers': 1,
        })

        node.publisher.subscription_count = 0
        self.assertIsNone(node.wait_for_graph(0.001))

    def test_mcp_wait_call_sends_correlated_identity_and_timeout(self):
        wait_result = {
            'session_id': 'speaker:test',
            'utterance_id': 'utt:review-smoke',
            'state': 'completed',
            'terminal': True,
        }
        response_body = json.dumps({
            'jsonrpc': '2.0',
            'id': 'test',
            'result': {
                'content': [{
                    'type': 'text',
                    'text': json.dumps(wait_result),
                }],
            },
        }).encode()

        class _Response:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                pass

            def read(self):
                return response_body

        with mock.patch.object(
            self.smoke.urllib.request, 'urlopen', return_value=_Response(),
        ) as urlopen:
            result = self.smoke._speaker_call(
                'http://127.0.0.1:15701/mcp', 'wait_playback', timeout=15,
                session_id='speaker:test',
                utterance_id='utt:review-smoke',
                timeout_sec=10,
            )

        self.assertEqual(result, wait_result)
        request = urlopen.call_args.args[0]
        body = json.loads(request.data.decode())
        self.assertEqual(body['params']['name'], 'speaker')
        self.assertEqual(body['params']['arguments'], {
            'action': 'wait_playback',
            'session_id': 'speaker:test',
            'utterance_id': 'utt:review-smoke',
            'timeout_sec': 10,
        })
        self.assertEqual(urlopen.call_args.kwargs['timeout'], 15)

    def _main_args(self):
        return argparse.Namespace(
            confirm_exclusive_hardware=True,
            mcp_url='http://127.0.0.1:15701/mcp',
            input_topic='/review/smoke',
            timeout=0.01,
            start_timeout=30.0,
            cleanup_timeout=30.0,
            discovery_timeout=0.01,
        )

    def test_start_timeout_still_stops_speaker(self):
        calls = []

        def speaker_call(_url, action, **_kwargs):
            calls.append(action)
            if action == 'start':
                raise TimeoutError('start response timed out')
            return {'state': 'idle'}

        with mock.patch.object(self.smoke, '_parse_args', self._main_args), \
                mock.patch.object(self.smoke, '_speaker_call', speaker_call), \
                mock.patch('sys.stderr'):
            result = self.smoke.main()

        self.assertEqual(result, 1)
        self.assertEqual(calls, ['start', 'stop'])

    def test_success_reports_mcp_and_ros_terminal_evidence(self):
        calls = []
        smoke = self.smoke

        class _HappyNode:
            def __init__(self, input_topic):
                self.input_topic = input_topic

            def spin_for(self, _seconds):
                pass

            def wait_for_graph(self, _timeout):
                return {
                    'speaker_input_subscriptions': 1,
                    'receipt_publishers': 1,
                }

            def publish(self, _utterance_id, _payload):
                pass

            def wait_for_terminal(self, session_id, utterance_id, _timeout):
                return {
                    'session_id': session_id,
                    'utterance_id': utterance_id,
                    'state': 'completed',
                    'completion_basis': 'g1_play_state_observed',
                    'audio_bytes': 5 * len(smoke.PCM_BLOCK),
                }

            def destroy_node(self):
                pass

        def speaker_call(_url, action, **kwargs):
            calls.append((action, kwargs))
            if action == 'start':
                return {
                    'state': 'ready',
                    'completion_mode': 'hardware_state',
                    'play_state': {'ready': True, 'matched_publishers': 1},
                    'session_id': 'speaker:test',
                }
            if action == 'wait_playback':
                return {
                    'session_id': kwargs['session_id'],
                    'utterance_id': kwargs['utterance_id'],
                    'state': 'completed',
                    'terminal': True,
                    'completion_basis': 'g1_play_state_observed',
                    'audio_bytes': 5 * len(smoke.PCM_BLOCK),
                }
            if action == 'stop':
                return {'state': 'idle'}
            self.fail(f'unexpected speaker action: {action}')

        with mock.patch.object(smoke, '_parse_args', self._main_args), \
                mock.patch.object(smoke, '_SmokeNode', _HappyNode), \
                mock.patch.object(smoke, '_speaker_call', speaker_call), \
                mock.patch('builtins.print') as print_output:
            result = smoke.main()

        self.assertEqual(result, 0)
        self.assertEqual([action for action, _kwargs in calls], [
            'start', 'wait_playback', 'stop',
        ])
        evidence = json.loads(print_output.call_args.args[0])
        self.assertEqual(evidence['result'], 'passed')
        self.assertTrue(evidence['mcp_wait_result']['terminal'])
        self.assertEqual(
            evidence['mcp_wait_result']['utterance_id'],
            evidence['terminal_receipt']['utterance_id'],
        )
        self.assertEqual(
            evidence['mcp_wait_result']['completion_basis'],
            evidence['terminal_receipt']['completion_basis'],
        )

    def test_cleanup_failure_turns_success_into_failure(self):
        calls = []

        class _HappyNode:
            def __init__(self, input_topic):
                self.input_topic = input_topic

            def spin_for(self, _seconds):
                pass

            def wait_for_graph(self, _timeout):
                return {
                    'speaker_input_subscriptions': 1,
                    'receipt_publishers': 1,
                }

            def publish(self, _utterance_id, _payload):
                pass

            def wait_for_terminal(self, session_id, utterance_id, _timeout):
                return {
                    'session_id': session_id,
                    'utterance_id': utterance_id,
                    'state': 'completed',
                    'completion_basis': 'g1_play_state_observed',
                    'audio_bytes': 5 * len(self.smoke.PCM_BLOCK),
                }

            def destroy_node(self):
                pass

        # Bind the module through the fake instance without depending on ROS.
        smoke = self.smoke
        _HappyNode.smoke = smoke

        def speaker_call(_url, action, **_kwargs):
            calls.append((action, _kwargs))
            if action == 'start':
                return {
                    'state': 'ready',
                    'completion_mode': 'hardware_state',
                    'play_state': {'ready': True, 'matched_publishers': 1},
                    'session_id': 'speaker:test',
                }
            if action == 'wait_playback':
                return {
                    'session_id': _kwargs['session_id'],
                    'utterance_id': _kwargs['utterance_id'],
                    'state': 'completed',
                    'terminal': True,
                    'completion_basis': 'g1_play_state_observed',
                    'audio_bytes': 5 * len(smoke.PCM_BLOCK),
                }
            raise TimeoutError('stop response timed out')

        with mock.patch.object(smoke, '_parse_args', self._main_args), \
                mock.patch.object(smoke, '_SmokeNode', _HappyNode), \
                mock.patch.object(smoke, '_speaker_call', speaker_call), \
                mock.patch('sys.stderr'):
            result = smoke.main()

        self.assertEqual(result, 1)
        self.assertEqual([action for action, _kwargs in calls], [
            'start', 'wait_playback', 'stop',
        ])
        wait_args = calls[1][1]
        self.assertEqual(wait_args['session_id'], 'speaker:test')
        self.assertTrue(wait_args['utterance_id'].startswith('utt:hardware-smoke-'))
        self.assertEqual(wait_args['timeout_sec'], self._main_args().timeout)
        self.assertEqual(
            wait_args['timeout'],
            self._main_args().timeout
            + smoke.MCP_WAIT_TRANSPORT_MARGIN_SEC,
        )


if __name__ == '__main__':
    unittest.main()
