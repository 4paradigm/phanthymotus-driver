"""Regressions for the 2026-10-22 actuator fault; no hardware writes."""
import json
import math
from pathlib import Path
import time
import types
import unittest
from unittest import mock

import device
from test_arm_controls import _FakePublisher, _fake_lowcmd, _prime_arm_plugin
from test_arm_gesture_safety import Feedback


class FirmwareEnvelopeTests(unittest.TestCase):
    def test_slots_and_all_public_limits_match_installed_service_envelope(self):
        rows = json.loads((Path(__file__).parent / 'testdata/adam_pro_service_limits.json').read_text())['joints']
        bounds = {}
        for slot, name, low, high in rows:
            self.assertEqual(name, device.ADAM_PRO_JOINTS[slot])
            bounds[name] = (low, high)
        for table in (device.ARM_JOINT_CONTROLS, device.WAIST_JOINT_CONTROLS, device.HEAD_JOINT_CONTROLS):
            for _, name, low, high in table.values():
                self.assertGreaterEqual(math.radians(low), bounds[name][0], name)
                self.assertLessEqual(math.radians(high), bounds[name][1], name)
        gesture = device.ArmGesturePlugin(_prime_arm_plugin(_FakePublisher()))
        for pose, _, _ in gesture._GESTURES.values():
            if pose == 'neutral':
                continue
            for side in ('left', 'right'):
                for joint, q in gesture._targets_for(pose, side).items():
                    self.assertGreaterEqual(q, bounds[joint][0], joint)
                    self.assertLessEqual(q, bounds[joint][1], joint)

    def test_original_minus_85_degree_wave_target_is_rejected(self):
        control = _prime_arm_plugin(_FakePublisher())
        for side in ('Left', 'Right'):
            result = control._set_targets({f'shoulderPitch_{side}': math.radians(-85)})
            self.assertEqual('INVALID_ARGUMENT', result['code'])
        self.assertEqual([], control._publisher.commands)


class ControlTakeoverTests(unittest.TestCase):
    def setUp(self):
        self.factory = mock.patch.object(device, 'pnd_adam_msg_dds__LowCmd_', _fake_lowcmd, create=True)
        self.factory.start()
        self.addCleanup(self.factory.stop)
        self.control = _prime_arm_plugin(_FakePublisher())
        self.control._confirm_target_write = lambda generation: None

    def test_rl_or_unknown_control_mode_rejects_before_first_packet(self):
        for response in ({'success':True,'domain_id':1}, {'success':False}, None):
            self.control._grpc.get_control_state = lambda **kw: response
            result = self.control._set_targets({'shoulderPitch_Left': -0.3})
            self.assertEqual('CONTROL_CONFLICT', result['code'])
            self.assertFalse(self.control._active)
            self.assertEqual([], self.control._publisher.commands)

    def test_fresh_measured_pose_replaces_idle_startup_snapshot_on_takeover(self):
        old = list(self.control._hold_q)
        fresh = [q + 0.04 for q in old]
        self.control._current_q = fresh
        self.assertIsNone(self.control._set_targets({'shoulderPitch_Left': -0.3}))
        self.control._write_command(0.02)
        command = self.control._publisher.commands[-1]
        for i in range(31):
            if i != 17:
                self.assertEqual(fresh[i], command.motor_cmd[i].q)
        self.assertLessEqual(abs(command.motor_cmd[17].q - fresh[17]), 0.5 / 50)
        self.assertNotEqual(old[0], command.motor_cmd[0].q)

    def test_late_writer_cannot_jump_to_wall_clock_trajectory_endpoint(self):
        self.assertIsNone(self.control._set_targets({'shoulderPitch_Left': -0.8}, velocity_limit=0.3))
        previous = self.control._last_command_q[17]
        self.control._seg_started_at -= 30
        self.control._write_command(3.0)
        q = self.control._publisher.commands[-1].motor_cmd[17].q
        self.assertLessEqual(abs(q - previous), 0.3 / 50 + 1e-12)
        self.assertNotEqual(-0.8, q)

    def test_expired_ownership_ceases_publishing_without_sending_release_into_new_owner(self):
        self.assertIsNone(self.control._set_targets({'shoulderPitch_Left': -0.3}))
        self.control._owner_checked_at = time.monotonic() - 1
        self.control._write_command(0.02)
        self.assertFalse(self.control._active)
        self.assertFalse(self.control._streaming)
        self.assertEqual([], self.control._publisher.commands)
        self.assertEqual('CONTROL_CONFLICT', self.control._ready_error()['code'])

    def test_feedback_fault_records_exact_joint_and_vendor_status(self):
        motors = [types.SimpleNamespace(q=0.0,dq=0.0,state=0) for _ in range(31)]
        motors[17].state = 64
        frames = iter([types.SimpleNamespace(motor_state=motors)])
        self.control._lowstate_sub = types.SimpleNamespace(Read=lambda **kw:next(frames,None))
        with self.assertLogs(device.__name__, level='ERROR') as logs:
            self.control._poll_feedback()
        message = '\n'.join(logs.output)
        self.assertIn('shoulderPitch_Left', message)
        self.assertIn('64', message)
        self.assertIsNotNone(self.control._feedback_error)

    def test_transient_fault_followed_by_good_sample_still_latches(self):
        good = [types.SimpleNamespace(q=0.0,dq=0.0,state=0) for _ in range(31)]
        bad = list(good)
        bad[17] = types.SimpleNamespace(q=0.0,dq=0.0,state=64)
        frames = iter([types.SimpleNamespace(motor_state=bad), types.SimpleNamespace(motor_state=good)])
        self.control._lowstate_sub = types.SimpleNamespace(Read=lambda **kw:next(frames,None))
        self.control._poll_feedback()
        self.assertIn('64', self.control._feedback_error)

    def test_owner_loss_after_feedback_fault_still_stops_release_packets(self):
        self.assertIsNone(self.control._set_targets({'shoulderPitch_Left': -0.3}))
        self.control._fail_feedback('motor fault')
        self.assertIsNotNone(self.control._release_started_at)
        self.control._lose_control_ownership('RL resumed')
        self.control._write_command(0.02)
        self.assertEqual([], self.control._publisher.commands)
        self.assertFalse(self.control._streaming)

    def test_guardian_detects_control_mode_change_during_active_stream(self):
        state = {'success':True,'domain_id':0}
        self.control._grpc.get_control_state = lambda **kw:dict(state)
        self.control.start()
        self.addCleanup(self.control.stop)
        self.assertIsNone(self.control._set_targets({'shoulderPitch_Left': -0.3}))
        state['domain_id'] = 1
        deadline = time.monotonic() + 1
        while self.control._feedback_error is None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual('CONTROL_CONFLICT', self.control._ready_error()['code'])
        count = len(self.control._publisher.commands)
        time.sleep(0.05)
        self.assertEqual(count, len(self.control._publisher.commands))


if __name__ == '__main__':
    unittest.main()
