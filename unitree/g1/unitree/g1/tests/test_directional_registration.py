"""Bundle integration with hardware dependencies replaced, without ROS or DDS."""
import ast
from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))
from directional_pointing import DirectionalPointing
from common import lifecycle


def bundle_class():
    tree = ast.parse((DRIVER / 'main.py').read_text(encoding='utf8'))
    bundle = next(node for node in tree.body
                  if isinstance(node, ast.ClassDef) and node.name == 'G1DeviceBundle')
    future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
    scope = {'_lifecycle': lifecycle}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[future, bundle], type_ignores=[])),
                 str(DRIVER / 'main.py'), 'exec'), scope)
    return scope['G1DeviceBundle']


class Registration(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.modules = {}
        events = self.events

        def module(name, **values):
            value = ModuleType(name)
            value.__dict__.update(values)
            self.modules[name] = value

        class Plugin:
            def __init__(self, *args, **kwargs):
                self.node = SimpleNamespace()

            def get_tool(self):
                return {'name': self.PREFIX, 'type': 'actuator', 'inputSchema': {}}

            def start(self):
                events.append(self.PREFIX + '.start')

            def stop(self):
                events.append(self.PREFIX + '.stop')

            def dispatch(self, action, args):
                return {'plugin': self.PREFIX, 'action': action}

        def plugin(name):
            return type(name, (Plugin,), {'PREFIX': name})

        self.notify = lambda *args, **kwargs: events.append('notify')
        module('device', LocoStatePlugin=plugin('loco_state'), LocoPlugin=plugin('loco'),
               StatePlugin=plugin('state'), ArmActionPlugin=plugin('arm'),
               _loco_acp_notify=self.notify)
        module('low_state', LowStateSource=plugin('low_state'))
        module('arm_stream', ArmStreamExecutor=plugin('executor'))
        module('motion_control', MotionControl=lambda cfg, stream: SimpleNamespace(executor=stream))
        module('teleop_control', TeleopControl=plugin('teleop_control'))
        module('teleop_bus', TeleopBus=lambda *args: SimpleNamespace())
        module('teaching', teaching_plugin=lambda *args: plugin('g1_teaching')())

        class Backend:
            def __init__(self, motion, *, stationary, config):
                self.motion, self.config = motion, config
                self.kinematics = SimpleNamespace(angular_tolerance_deg=5., solver_mode='bounded_ik',
                    yaw_limits_deg=(-180., 180.), pitch_limits_deg=(-90., 90.))

        class Guard:
            def __init__(self, *args):
                pass

            def check(self):
                raise AssertionError('registration must not acquire hardware or admit motion')

            def close(self):
                events.append('guard.close')

        self.node = SimpleNamespace(destroy_node=lambda: events.append('monitor.destroy'))
        module('directional_backend', DirectionalBackend=Backend)
        module('directional_stationary', DirectionalStationaryGuard=Guard,
               create_stationary_node=lambda *args: self.node)
        self.executor = SimpleNamespace(add_node=lambda n: events.append('monitor.add'),
                                       remove_node=lambda n: events.append('monitor.remove'))
        self.config = {'teleop_control': {'enabled': True}, 'plugins': {
            'arm': {'enabled': True}, 'loco': {'enabled': True},
            'smart_motion': {'enabled': False},
            'g1_directional_pointing': {'enabled': True, 'solver_mode': 'bounded_ik'}}}
        self.patcher = patch.dict(sys.modules, self.modules)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)

    def make_bundle(self):
        with redirect_stdout(io.StringIO()):
            return bundle_class()(self.config, 'g1', self.executor, None, None, None, None)

    def test_catalog_start_uses_shared_owner_without_output(self):
        instance = self.make_bundle()
        card = next(p for p in instance._plugins if isinstance(p, DirectionalPointing))
        arm = next(p for p in instance._plugins if p.PREFIX == 'arm')
        self.assertIs(card.backend.motion.executor, arm._stream)
        self.assertIs(card.notify, self.notify)
        schema = card.get_tool()['inputSchema']
        self.assertEqual(schema['x-resource'], ['arm_l', 'arm_r'])
        self.assertEqual(schema['x-completion']['actions'], ['point'])
        with redirect_stdout(io.StringIO()):
            instance.start_all()
        self.assertEqual(self.events.count('monitor.add'), 1)
        self.assertNotIn('notify', self.events)

    def test_stop_by_id_does_not_call_base_or_convert_errors_to_success(self):
        instance = self.make_bundle()
        instance._point_card = SimpleNamespace(dispatch_base=lambda *a: self.fail('base used'))
        result = instance.dispatch('g1_directional_pointing',
                                   {'action': 'stop', 'action_id': 'missing'})
        self.assertEqual(result, {'error': 'unknown action_id'})
        self.assertEqual(instance.dispatch('g1_directional_pointing', {'action': 'stop'}),
                         {'state': 'idle'})
        self.assertEqual(instance.dispatch('g1_directional_pointing', {'action': 'start'}),
                         {'state': 'ready'})

    def test_disabled_point_leaves_other_catalog_unchanged(self):
        enabled = self.make_bundle()
        self.config['plugins']['g1_directional_pointing']['enabled'] = False
        disabled = self.make_bundle()
        enabled_tools = {t['name']: t for t in enabled.get_all_tools()
                         if t['name'] != 'g1_directional_pointing'}
        self.assertEqual(enabled_tools, {t['name']: t for t in disabled.get_all_tools()})
        self.assertIsNone(disabled._directional_monitor)
        self.assertEqual(disabled.dispatch('arm', {'action': 'list'}),
                         {'plugin': 'arm', 'action': 'list'})

    def test_point_stops_before_shared_owner_and_feedback(self):
        instance = self.make_bundle()
        card = next(p for p in instance._plugins if isinstance(p, DirectionalPointing))
        original_stop = card.stop
        card.stop = lambda: (self.events.append('point.stop'), original_stop())
        with redirect_stdout(io.StringIO()):
            instance.stop_all()
        for name in ('arm.stop', 'teleop_control.stop', 'guard.close', 'low_state.stop'):
            self.assertLess(self.events.index('point.stop'), self.events.index(name))
        self.assertLess(self.events.index('guard.close'), self.events.index('low_state.stop'))

    def test_dependencies_are_checked_before_hardware_construction(self):
        for name in ('arm', 'loco'):
            with self.subTest(name=name):
                self.config['plugins'][name]['enabled'] = False
                with self.assertRaisesRegex(ValueError, 'requires arm, loco'):
                    self.make_bundle()
                self.config['plugins'][name]['enabled'] = True
        self.config['teleop_control']['enabled'] = False
        with self.assertRaisesRegex(ValueError, 'requires arm, loco'):
            self.make_bundle()
        self.config['teleop_control']['enabled'] = True
        for name in ('servo', 'servo_eef'):
            self.config['plugins'][name] = {'enabled': True}
            with self.assertRaisesRegex(ValueError, 'cannot coexist'):
                self.make_bundle()
            self.config['plugins'][name]['enabled'] = False
        self.assertEqual(self.events, [])

    def test_monitor_registration_failure_closes_only_new_monitor(self):
        def fail(node):
            raise RuntimeError('executor_unavailable')
        self.executor.add_node = fail
        with self.assertRaisesRegex(RuntimeError, 'executor_unavailable'):
            self.make_bundle()
        self.assertEqual(self.events, ['guard.close', 'monitor.destroy'])


if __name__ == '__main__':
    unittest.main()
