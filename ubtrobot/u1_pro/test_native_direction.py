"""Native DOA bridge lifecycle, data validity and private worker contracts."""
import importlib.util
import json
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent))
from native_direction import NativeDirectionSource, DOMAIN, RAW_TOPIC, DEVICE_TOPIC, EVENT_TOPIC
import direction_card


class NativeSourceTests(unittest.TestCase):
    def setUp(self):
        self.nodes = mock.Mock()
        self.source = NativeDirectionSource(self.nodes)
        self.source.running = True
        self.source._raw_pub = mock.Mock()
        self.source._device_pub = mock.Mock()
        self.clock = mock.patch('native_direction.time.monotonic', return_value=100).start()
        self.addCleanup(mock.patch.stopall)

    def device(self, id=131, internal=True):
        return types.SimpleNamespace(device_id=id, device_name='Bothlent UAC Dongle (USB Audio)',
            channels=8, is_built_in=internal, sample_rate=16000, sample_format='S16LE')

    def audio(self, data=b'\0\1' * 8):
        return types.SimpleNamespace(data=types.SimpleNamespace(data=data))

    def event(self, angle=80):
        return types.SimpleNamespace(azimuth=angle, confidence=4.31)

    def test_real_device_metadata_and_audio_are_forwarded_unchanged(self):
        device = self.device(); audio = self.audio()
        self.source._on_device(device); self.source._on_audio(audio)
        self.source._device_pub.publish.assert_called_once_with(device)
        self.source._raw_pub.publish.assert_called_once_with(audio)
        self.assertEqual(self.source.audio_frames, 1)
        self.assertEqual(self.source.device_updates, 1)
        self.nodes.set_system_enabled.assert_not_called()
        self.nodes.call_sdk.assert_not_called()

    def test_no_audio_without_device_or_with_empty_samples(self):
        self.source._on_audio(self.audio())
        self.source._on_device(self.device(internal=False))
        self.source._on_audio(self.audio(data=b''))
        self.source._raw_pub.publish.assert_not_called()

    def test_vendor_false_builtin_flag_does_not_reject_verified_array(self):
        self.source._on_device(self.device(internal=False))
        audio = self.audio(); self.source._on_audio(audio)
        self.source._raw_pub.publish.assert_called_once_with(audio)
        self.assertFalse(self.source.device['is_built_in'])

    def test_warmup_and_stale_audio_never_revive_direction(self):
        self.source._on_device(self.device()); self.source._on_audio(self.audio())
        self.source.drain()
        self.source._on_direction(self.event())
        self.assertEqual(self.source.drain(), [])
        self.clock.return_value = 101.1
        self.source._on_audio(self.audio())  # a gap starts a new warmup
        self.clock.return_value = 101.5
        self.source._on_audio(self.audio())
        self.clock.return_value = 101.9
        self.source._on_audio(self.audio())
        self.clock.return_value = 102.2
        self.source._on_audio(self.audio())
        self.source.drain(); self.source._on_direction(self.event())
        events = self.source.drain()
        self.assertEqual(events[0][0], {'azimuth': 80, 'confidence': 4.31})
        self.assertEqual(events[0][1], 102.2)
        self.clock.return_value = 103
        self.source._on_direction(self.event(-30))
        self.assertEqual(self.source.drain(), [])

    def test_device_change_invalidates_direction_and_requires_new_audio(self):
        self.source._on_device(self.device()); self.source._on_audio(self.audio())
        self.source._events.append(({'azimuth': 45}, 100, 123))
        self.source._on_device(self.device(id=132))
        self.assertEqual([e[0] for e in self.source.drain()], [None])
        self.assertIsNone(self.source._last_audio)
        self.source._on_direction(self.event())
        self.assertEqual(self.source.drain(), [])

    def fake_ros(self):
        ros = types.ModuleType('rclpy')
        ros.init = mock.Mock(); ros.ok = mock.Mock(return_value=True); ros.shutdown = mock.Mock()
        node = mock.Mock(); executor = mock.Mock(); context = object()
        modules = {'rclpy': ros,
            'rclpy.context': types.SimpleNamespace(Context=lambda: context),
            'rclpy.node': types.SimpleNamespace(Node=mock.Mock(return_value=node)),
            'rclpy.executors': types.SimpleNamespace(SingleThreadedExecutor=mock.Mock(return_value=executor)),
            'rclpy.qos': types.SimpleNamespace(QoSProfile=lambda **kw: kw,
                ReliabilityPolicy=types.SimpleNamespace(RELIABLE='reliable'),
                DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL='latched')),
            'audio_msgs.msg': types.SimpleNamespace(AudioInData=type('Audio', (), {}),
                CurrentAudioDeviceInfo=type('Device', (), {}), DoaEvent=type('Doa', (), {}))}
        return ros, node, executor, context, modules

    def test_start_stop_owns_only_private_domain_and_its_subscriptions(self):
        ros, node, executor, context, modules = self.fake_ros()
        source = NativeDirectionSource(self.nodes)
        with mock.patch.dict(sys.modules, modules), mock.patch('native_direction.threading.Thread') as thread:
            thread.return_value.is_alive.return_value = False
            source.start()
            ros.init.assert_called_once_with(context=context, domain_id=43)
            self.assertEqual([c.args[1] for c in self.nodes.audio_device.create_subscription.call_args_list],
                             [DEVICE_TOPIC, '/sys/device/audio_in/raw'])
            self.assertEqual([c.args[1] for c in node.create_publisher.call_args_list], [RAW_TOPIC, DEVICE_TOPIC])
            self.assertEqual(node.create_subscription.call_args.args[1], EVENT_TOPIC)
            self.assertEqual(self.nodes.audio_device.create_subscription.call_args_list[0].args[3]['durability'], 'latched')
            source.stop()
            self.assertEqual(self.nodes.audio_device.destroy_subscription.call_count, 2)
            node.destroy_node.assert_called_once()
            ros.shutdown.assert_called_once_with(context=context)
            self.nodes.audio_device.destroy_node.assert_not_called()
            self.nodes.robot.create_publisher.assert_not_called()
            self.nodes.core.create_publisher.assert_not_called()

    def test_partial_start_cleans_private_resources(self):
        ros, node, executor, context, modules = self.fake_ros()
        self.nodes.audio_device.create_subscription.side_effect = RuntimeError('cannot subscribe')
        source = NativeDirectionSource(self.nodes)
        with mock.patch.dict(sys.modules, modules), self.assertRaisesRegex(RuntimeError, 'cannot subscribe'):
            source.start()
        node.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once_with(context=context)
        self.assertFalse(source.running)

    def test_card_consumes_typed_source_without_refreshing_receipt_time(self):
        nodes = mock.Mock(namespace='test', String=type('String', (), {}))
        nodes.config = {'sound_direction': {'source': 'native'}}
        native = mock.Mock(); native.error = ''; native.info.return_value = {'state': 'receiving'}
        native.drain.return_value = [({'azimuth': 80, 'confidence': 4.31}, 99.9, 1700000000000)]
        with mock.patch.object(direction_card, 'NativeDirectionSource', return_value=native):
            card = direction_card.SoundDirectionPlugin(nodes); card.start(); card._tick()
            data = json.loads(nodes.core.create_publisher.return_value.publish.call_args.args[0].data)
            self.assertEqual(data['angle_deg'], -80)
            self.assertEqual(data['confidence'], 4.31)
            self.assertEqual(data['timestamp_ms'], 1700000000000)
            self.assertEqual(data['audio_window_start_us'], 0)
            nodes.robot.create_subscription.assert_not_called()
            native.drain.return_value = []
            self.clock.return_value = 103; card._tick()
            self.assertEqual(card._observation()['state'], 'stale')
            card.stop(); native.stop.assert_called_once()
            card.start(); self.assertIsNone(card._observation()['angle_deg'])

    def test_private_configuration_has_no_wakeup_or_actuator_nodes(self):
        root = Path(__file__).with_name('native_doa')
        allowed = {'root', 'BehaviorTree', 'Sequence', 'Fallback', 'RunOnce', 'Script',
                   'ScriptCondition', 'AudioSubscriptionNode', 'AecNode', 'IsInternalMicNode', 'DoaNode'}
        for path in root.glob('*.xml'):
            tree = ET.parse(path)
            self.assertLessEqual({n.tag for n in tree.iter()}, allowed)
        direction = ET.parse(root / 'direction.xml').find('.//DoaNode')
        self.assertEqual(direction.get('doa_event_topic_in'), EVENT_TOPIC)
        self.assertEqual(direction.get('wake_flag_in'), '{trigger}')
        sub = ET.parse(root / 'input.xml').find('.//AudioSubscriptionNode')
        self.assertEqual(sub.get('audio_topic_in'), RAW_TOPIC)
        config = (root / 'controller.yaml').read_text()
        self.assertIn('inter_pipeline_channels:', config)
        self.assertIn('audio_after_aec:', config)
        self.assertFalse(json.loads((root / 'doa-plugin.json').read_text())['plugin_config']['debug_mode'])

class NativeDeploymentTests(unittest.TestCase):
    def manager(self):
        path = Path(__file__).parent / 'deploy' / 'native_doa_manage.py'
        spec = importlib.util.spec_from_file_location('native_doa_manage_test', path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        return module

    def test_worker_is_pinned_to_running_vendor_image_and_only_config_mount(self):
        m = self.manager()
        native = {'State': {'Running': True}, 'Image': 'sha256:actual-vendor-image',
                  'Config': {'Image': 'vendor/audio:tag'},
                  'Mounts': [{'Type': 'volume', 'Name': 'vendor-config', 'Destination': '/opt/walker/config_csh2'},
                             {'Type': 'bind', 'Source': '/home/user/.ssh', 'Destination': '/home/ubt/.ssh'}]}
        calls = []
        def docker(*args, **kwargs):
            calls.append(args)
            if args[:2] == ('image', 'inspect'):
                return types.SimpleNamespace(stdout='sha256:' + 'a' * 64)
            if args[0] == 'create':
                return types.SimpleNamespace(stdout='extract-only-container')
            if args[0] == 'cp':
                (Path(args[-1]) / 'controller.yaml').write_text('test configuration')
            return types.SimpleNamespace(stdout='', returncode=0)
        with tempfile.TemporaryDirectory() as temp, mock.patch.object(m, 'ROOT', Path(temp)), \
             mock.patch.object(m, 'inspect', side_effect=lambda name: native if name == 'native-audio' else None), \
             mock.patch.object(m, 'docker', side_effect=docker):
            m.start('reviewed-driver:commit', 'native-audio')
        run = next(c for c in calls if c[0] == 'run')
        self.assertIn('ROSA_DOMAIN_ID=43', run)
        self.assertIn('vendor-config:/opt/walker/config_csh2:ro', run)
        self.assertIn('sha256:actual-vendor-image', run)
        self.assertNotIn('vendor/audio:tag', run)
        self.assertIn('--read-only', run)
        self.assertNotIn('--privileged', run)
        self.assertFalse(any('.ssh' in v or 'docker.sock' in v or '/dev/snd' in v for v in run))
        self.assertFalse(any(c[0] in ('restart', 'exec') for c in calls))

    def test_stop_rejects_unrelated_container(self):
        m = self.manager()
        with mock.patch.object(m, 'inspect', return_value={'Config': {'Labels': {}}}), \
             mock.patch.object(m, 'docker') as docker, self.assertRaisesRegex(RuntimeError, 'unrelated'):
            m.stop()
        docker.assert_not_called()


if __name__ == '__main__':
    unittest.main()
