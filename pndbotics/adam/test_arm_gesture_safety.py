"""Offline trajectory lifecycle and shared hardware ownership regressions."""
import threading
import time
import types
import unittest
from unittest import mock

import device
from test_arm_controls import _FakePublisher, _fake_lowcmd, _prime_arm_plugin


class Feedback:
    def __init__(self):
        self.live = True
        self._next = 0

    def Read(self, timeout=0):
        if timeout == 0:
            raise AssertionError("CycloneDDS zero timeout means an infinite wait")
        # Model new DDS samples, not an infinite queue of old samples.
        if self.live and time.monotonic() >= self._next:
            self._next = time.monotonic() + 0.005
            return types.SimpleNamespace(motor_state=[
                types.SimpleNamespace(q=0.1, dq=0.0, state=0) for _ in range(31)])
        return None


class GestureSafetyTests(unittest.TestCase):
    def setUp(self):
        self.factory = mock.patch.object(device, 'pnd_adam_msg_dds__LowCmd_',
                                         _fake_lowcmd, create=True)
        self.factory.start()
        self.calls = []
        self.callback = mock.patch.object(device, '_notify_action_completion',
            side_effect=lambda *call: self.calls.append(call))
        self.callback.start()
        self.feedback = Feedback()
        self.control = device.ArmControlPlugin({'feedback_timeout_s': 0.1}, '', None,
            dds_lowcmd_pub=_FakePublisher(), dds_arm_lowstate_sub=self.feedback,
            grpc_client=types.SimpleNamespace(get_control_state=lambda **kw: {'success':True,'domain_id':0}))
        self.control._RELEASE_SECONDS = 0.04
        self.control.start()
        self.assertTrue(self.control._state_ready.wait(1))
        self.gesture = device.ArmGesturePlugin(self.control)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.gesture._cancel_sequence()
        self.control.stop()
        time.sleep(0.04)
        self.callback.stop()
        self.factory.stop()

    def wait_for(self, action_id):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            found = [c for c in self.calls if c[0] == action_id]
            if found:
                self.assertEqual(1, len(found))
                return found[0]
            time.sleep(0.01)
        self.fail('No terminal ACP callback')

    def fast(self):
        self.control._MAX_VELOCITY_RAD_S = 1000
        original = self.control._set_targets
        def targets(values, **kwargs):
            kwargs.update(preferred_span=0.02, velocity_limit=1000)
            return original(values, **kwargs)
        self.control._set_targets = targets
        self.gesture._WAVE_SEQUENCE = (('wave_out', 0.02),)
        self.gesture._WAVE_LOWER_SECONDS = 0.02

    def test_schema_and_runtime_reject_unsupported_parameters(self):
        schema = self.gesture.get_tool()['inputSchema']
        self.assertEqual({'wave', 'salute', 'reset', 'stop', 'info'},
                         set(schema['properties']['action']['enum']))
        self.assertEqual({'wave', 'salute', 'reset'}, set(schema['x-completion']['actions']))
        for action in ('wave', 'salute'):
            self.assertEqual('INVALID_ARGUMENT', self.gesture.dispatch(action, {'side':'both'})['code'])
        for value in (True, float('nan'), -1, 61, 'bad'):
            self.assertEqual('INVALID_ARGUMENT', self.gesture.dispatch('reset', {'duration_s':value})['code'])
        self.assertIsNone(self.gesture.dispatch('handshake', {}))

    def test_static_and_wave_complete_once_without_measured_arrival_claim(self):
        self.fast()
        for action in ('salute', 'reset', 'wave'):
            accepted = self.gesture.dispatch(action, {'side':'left'})
            call = self.wait_for(accepted['action_id'])
            self.assertEqual('completed', call[1])
            self.assertEqual('commanded trajectory', call[2]['completion_basis'])
            self.assertIsNone(call[2]['measured_arrival'])

    def test_stop_freezes_and_restart_reacquires_feedback(self):
        first = self.gesture.dispatch('wave', {})
        result = self.gesture.dispatch('stop', {})
        self.assertTrue(result['success'])
        self.assertEqual('cancelled', self.wait_for(first['action_id'])[1])
        self.assertFalse(self.control._thread and self.control._thread.is_alive())
        self.fast()
        second = self.gesture.dispatch('reset', {})
        self.assertTrue(second['success'], second)
        self.assertEqual('completed', self.wait_for(second['action_id'])[1])
        self.assertEqual([0.1]*31, self.control._hold_q)

    def test_stale_feedback_fails_and_never_resumes_old_trajectory(self):
        accepted = self.gesture.dispatch('wave', {})
        self.feedback.live = False
        call = self.wait_for(accepted['action_id'])
        self.assertEqual('error', call[1])
        self.assertIn('feedback timed out', call[2]['error'])
        frozen = list(self.control._last_command_q)
        time.sleep(0.1)
        self.assertEqual(frozen, self.control._last_command_q)
        self.assertEqual('LOWSTATE_UNAVAILABLE', self.gesture.dispatch('salute', {})['code'])

    def test_arm_waist_and_head_commands_cancel_the_gesture(self):
        for kind, action, args in (
            (lambda c: c, 'set_elbow', {'side':'left', 'bend_deg':-10}),
            (device.WaistControlPlugin, 'set_angles', {'yaw_deg':5}),
            (device.HeadControlPlugin, 'set_angles', {'yaw_deg':5}),
        ):
            accepted = self.gesture.dispatch('wave', {})
            result = kind(self.control).dispatch(action, args)
            self.assertIsNotNone(result)
            self.assertNotEqual(False, result.get('success'), result)
            self.assertEqual('cancelled', self.wait_for(accepted['action_id'])[1])

    def test_timer_cannot_report_completion_without_final_written_command(self):
        accepted = self.gesture.dispatch('salute', {})
        self.control._write_command = lambda dt: None
        with self.gesture._sequence_lock:
            self.gesture._sequence_deadline = time.monotonic() + 0.05
        call = self.wait_for(accepted['action_id'])
        self.assertEqual('error', call[1])
        self.assertEqual('TIMEOUT', call[2]['code'])
        # Allow cleanup to finish its gain-release write.
        self.control._write_command = types.MethodType(device.ArmControlPlugin._write_command, self.control)

    def test_invalid_feedback_latches_a_fault(self):
        frames = iter([types.SimpleNamespace(motor_state=[])])
        self.control._lowstate_sub = types.SimpleNamespace(Read=lambda **kwargs: next(frames, None))
        self.control._poll_feedback()
        self.assertIn('fewer than 31 motors', self.control._feedback_error)
        self.assertEqual('LOWSTATE_UNAVAILABLE', self.control._ready_error()['code'])

    def test_new_gesture_cancels_prior_static_pose(self):
        first = self.gesture.dispatch('salute', {})
        second = self.gesture.dispatch('reset', {})
        self.assertEqual('cancelled', self.wait_for(first['action_id'])[1])
        self.gesture.dispatch('stop', {})
        self.assertEqual('cancelled', self.wait_for(second['action_id'])[1])

    def test_false_writer_is_failure_and_target_is_discarded(self):
        self.control._publisher.Write = lambda *a, **k: False
        result = self.gesture.dispatch('wave', {})
        self.assertEqual('DDS_WRITE_FAILED', result['code'])
        self.assertFalse(self.control._active)
        self.assertEqual({}, self.control._target_q)


class BundleRegistrationTests(unittest.TestCase):
    def bundle(self, **changes):
        names = ('state','estop','loco','motion','tracking_motion','camera',
                 'vision_capture','hand','hand_gesture','hand_state','model','waist','head')
        plugins = {n:{'enabled':False} for n in names}
        plugins.update(arm={'enabled':True}, arm_gesture={'enabled':True})
        plugins.update(changes)
        return device.AdamDeviceBundle({'variant':'pro','plugins':plugins}, '', None, None,
                                       ros2_enabled=False)

    def test_registration_respects_both_switches(self):
        bundle = self.bundle()
        self.assertIs(bundle._tool_map['arm_control'], bundle._tool_map['arm_gesture']._control)
        self.assertNotIn('arm_gesture', self.bundle(arm_gesture={'enabled':False})._tool_map)
        self.assertNotIn('arm_gesture', self.bundle(arm={'enabled':False})._tool_map)

    def test_hand_binding_works_even_when_hand_gesture_card_is_hidden(self):
        bundle = self.bundle(hand={'enabled':True}, hand_gesture={'enabled':False})
        self.assertIs(bundle._tool_map['hand'], bundle._tool_map['arm_gesture']._hand._control)
        self.assertNotIn('hand_gesture', bundle._tool_map)

    def test_vendor_and_lowcmd_ownership_exclude_each_other(self):
        bundle = self.bundle()
        vendor = mock.Mock()
        vendor.dispatch.return_value = {'success':True}
        bundle._tool_map['motion'] = vendor
        control = bundle._upper_body
        control._active = True
        self.assertEqual('CONTROL_BUSY', bundle.dispatch('motion', {'action':'play'})['code'])
        vendor.dispatch.assert_not_called()
        control._active = False
        self.assertTrue(bundle.dispatch('motion', {'action':'play'})['success'])
        self.assertEqual('CONTROL_BUSY', bundle.dispatch('arm_gesture', {'action':'wave'})['code'])
        bundle._tool_map['loco'] = mock.Mock()
        self.assertEqual('CONTROL_BUSY', bundle.dispatch('loco', {'action':'move'})['code'])
        bundle._tool_map['loco'].dispatch.assert_not_called()
        vendor.dispatch.return_value = {'success':False}
        bundle.dispatch('motion', {'action':'stop'})
        self.assertEqual('motion', bundle._vendor_owner)
        vendor.dispatch.return_value = {'success':True}
        bundle.dispatch('motion', {'action':'stop'})
        self.assertIsNone(bundle._vendor_owner)


if __name__ == '__main__':
    unittest.main()
