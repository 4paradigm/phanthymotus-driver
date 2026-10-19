"""Exercise record/end-record through fake SDK only; never contact hardware."""
import tempfile
import unittest

from agilex.piper.runtime import PiperRuntime
from agilex.piper.teaching_control import teaching_action, add_teaching_schema
from test_piper_controls import Driver


class TeachingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.runtime=PiperRuntime({'data_dir':self.temp.name,'async_actions':False,
            'mode':{'verified_teaching_firmwares':['S-V1.9-0']}},driver_factory=Driver)
        self.runtime.motion_enabled=True
        self.driver=self.runtime.connected();self.sdk=self.driver.arm
        self.flags={'teaching_supported':True,'teaching_space_clear':True,'other_controllers_stopped':True}
        self.addCleanup(self.runtime.stop)

    def call(self,action,**args):
        return teaching_action(self.runtime,action,{**self.flags,**args})

    def execute(self,action):
        preview=self.call(action)
        self.assertTrue(preview['allowed'])
        self.assertEqual(self.sdk.calls,[])
        return self.call(action,execute=True,preview_id=preview['preview_id'])

    def test_schema_has_two_guarded_actions_without_another_canvas_card(self):
        d={'inputSchema':{'properties':{'action':{'enum':['halt'],'oneOf':[]}},'x-action-params':{}}}
        add_teaching_schema(d)
        schema=d['inputSchema']
        for name in ('enter_teaching','exit_teaching'):
            self.assertIn(name,schema['properties']['action']['enum'])
            self.assertEqual(schema['x-confirmation'][name]['preview'],{'action':name,'execute':False})

    def test_enter_and_exit_recording_send_only_the_requested_command(self):
        entered=self.execute('enter_teaching')
        self.assertTrue(entered['confirmed']);self.assertTrue(entered['executed'])
        self.assertEqual(self.sdk.calls,[('teach',{'grag_teach_ctrl':1})])
        self.sdk.calls.clear()
        exited=self.execute('exit_teaching')
        self.assertTrue(exited['confirmed'])
        self.assertEqual(self.sdk.calls,[('teach',{'grag_teach_ctrl':2})])
        self.assertFalse(self.runtime.gate.locked())
        self.assertIsNone(self.runtime.worker_context.operation_object)

    def test_exit_accepts_standby_feedback_without_taking_over_can(self):
        self.sdk.mode='TEACHING_MODE(0x2)';self.sdk.teach='START_RECORDING(0x1)'
        original=self.sdk.MotionCtrl_1
        def end(**args):
            original(**args);self.sdk.mode='STANDBY(0x0)'
        self.sdk.MotionCtrl_1=end
        result=self.execute('exit_teaching')
        self.assertTrue(result['confirmed'])
        self.assertTrue(result['state']['control_mode'].startswith('STANDBY'))
        self.assertEqual(self.sdk.calls,[('teach',{'grag_teach_ctrl':2})])

    def test_unverified_firmware_guidance_never_sends_even_when_execute_is_requested(self):
        self.runtime.config['mode']['verified_teaching_firmwares']=[]
        for action in ('enter_teaching','exit_teaching'):
            if action=='exit_teaching':
                self.sdk.mode='TEACHING_MODE(0x2)';self.sdk.teach='START_RECORDING(0x1)'
            result=self.call(action,execute=True)
            self.assertFalse(result['allowed']);self.assertFalse(result['executed'])
            self.assertTrue(result['button_guidance'])
            self.assertNotIn('preview_id',result)
        self.assertEqual(self.sdk.calls,[])

    def test_invalid_flags_and_unknown_params_are_rejected_before_connection(self):
        def forbidden():raise AssertionError('hardware reached')
        self.runtime.connected=forbidden
        for args in ({'execute':'true'},{'teaching_supported':1},{'target_mode':'can'},{'preview_id':8}):
            with self.subTest(args=args),self.assertRaises(ValueError):
                self.call('enter_teaching',**args)

    def test_permission_stale_feedback_and_changed_preview_block_all_commands(self):
        preview=self.call('enter_teaching')
        self.runtime.motion_enabled=False
        with self.assertRaises(RuntimeError):self.call('enter_teaching',execute=True,preview_id=preview['preview_id'])
        self.runtime.motion_enabled=True
        self.sdk.frozen=True
        with self.assertRaises(RuntimeError):self.call('enter_teaching')
        self.sdk.frozen=False
        preview=self.call('enter_teaching')
        self.sdk.raw=(self.sdk.raw[0]+1000,*self.sdk.raw[1:])
        with self.assertRaises(RuntimeError):self.call('enter_teaching',execute=True,preview_id=preview['preview_id'])
        self.assertEqual(self.sdk.calls,[])

    def test_active_playback_and_missing_support_do_not_exit_or_release_torque(self):
        self.sdk.mode='TEACHING_MODE(0x2)';self.sdk.teach='START_EXECUTION(0x3)'
        result=self.call('exit_teaching',execute=True)
        self.assertFalse(result['allowed'])
        self.assertTrue(any('回放' in s for s in result['missing_conditions']))
        self.sdk.teach='START_RECORDING(0x1)'
        self.assertFalse(self.call('exit_teaching',teaching_supported=False)['allowed'])
        self.assertEqual(self.sdk.calls,[])

    def test_already_entered_or_exited_is_an_idempotent_read_only_result(self):
        self.assertFalse(self.call('exit_teaching')['executed'])
        self.sdk.mode='TEACHING_MODE(0x2)';self.sdk.teach='START_RECORDING(0x1)'
        self.assertFalse(self.call('enter_teaching')['executed'])
        self.assertEqual(self.sdk.calls,[])


if __name__=='__main__':unittest.main()
