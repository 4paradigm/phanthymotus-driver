from copy import deepcopy
from dataclasses import replace
import threading
import time
import unittest
from types import SimpleNamespace

from agilex.piper.arm import ArmState
from agilex.piper.control import Previews, ready_to_move
from agilex.piper.direct_control import direct_schema, install_direct_control
from agilex.piper.monitor import monitor_frame


class FakeControl:
    def __init__(self):
        self.worker_context=threading.local();self.closed=False;self.stopped_cards=set()
        self.cancel=threading.Event();self.motion_enabled=False;self.previews=Previews()
        self.calls=[];self.invocations=[];self.between=None;self.corrective=False
        self.state=ArmState((0,10,-10,0,20,0),(True,)*6,'CAN_CTRL(0x1)','NORMAL(0x0)','DISABLED(0x0)',0,time.time())
    def require_motion_permission(self):
        raise RuntimeError('separate permission required')
    def read_monitor_snapshot(self):return {'motion_enabled':False}
    def dispatch(self,action,args):
        self.invocations.append(deepcopy(args))
        result=self.dispatch_sync(action,args)
        if args.get('execute') is False and self.between:self.between()
        return result
    def dispatch_sync(self,action,args):
        if 'execute' in args and type(args['execute']) is not bool:raise ValueError('bad execute')
        if action=='disable':
            if args.get('supported') is not True:raise ValueError('support required')
            self.require_motion_permission();self.calls.append(action);return {'executed':True}
        self.require_motion_permission();ready_to_move(self.state)
        if args.get('execute') is not True:return self.previews.put(action,self.state)
        self.previews.consume(args['preview_id'],action,self.state)
        if self.corrective and args.get('allow_limit_adjustment') is not True:raise ValueError('correction not approved')
        self.calls.append(action);return {'executed':True}


class DirectControlTests(unittest.TestCase):
    def setUp(self):
        self.r=FakeControl();install_direct_control(self.r)
    def call(self,action='move_joint',**args):return self.r.dispatch(action,{'_tool_name':'arm_control',**args})

    def test_schema_has_no_permission_menu_or_extra_confirmation(self):
        d={'inputSchema':{'properties':{'action':{'enum':['allow_motion','block_motion','move_joint'],
            'oneOf':[{'const':a} for a in ['allow_motion','block_motion','move_joint']]}},
            'x-action-params':{'move_joint':{'params':['joint','target_deg','execute','preview_id','take_control']},
                              'allow_motion':{'params':[]}},'x-confirmation':{'move_joint':{}}}}
        direct_schema(d);s=d['inputSchema']
        self.assertEqual(s['properties']['action']['enum'],['move_joint'])
        self.assertEqual(s['x-action-params']['move_joint']['params'],['joint','target_deg'])
        self.assertNotIn('x-confirmation',s)
        for action in ('allow_motion','block_motion'):
            with self.assertRaises(ValueError):self.call(action)
        self.assertEqual(self.r.calls,[])

    def test_one_click_checks_then_executes_without_a_global_grant(self):
        self.assertTrue(self.call()['executed'])
        self.assertEqual(self.r.calls,['move_joint'])
        self.assertEqual([v['execute'] for v in self.r.invocations],[False,True])
        self.assertFalse(self.r.motion_enabled)
        with self.assertRaises(RuntimeError):self.r.require_motion_permission()

    def test_explicit_preview_does_not_execute_and_preview_is_one_use(self):
        preview=self.call(execute=False);self.assertEqual(self.r.calls,[])
        self.call(execute=True,preview_id=preview['preview_id'])
        with self.assertRaises(RuntimeError):self.call(execute=True,preview_id=preview['preview_id'])
        self.assertEqual(self.r.calls,['move_joint'])

    def test_invalid_mode_or_a_state_change_between_check_and_execute_blocks(self):
        self.r.state=replace(self.r.state,control_mode='TEACHING_MODE(0x2)')
        with self.assertRaises(RuntimeError):self.call()
        self.r.state=replace(self.r.state,control_mode='CAN_CTRL(0x1)')
        self.r.between=lambda:setattr(self.r,'state',replace(self.r.state,joints_deg=(2,10,-10,0,20,0)))
        with self.assertRaises(RuntimeError):self.call()
        self.assertEqual(self.r.calls,[])

    def test_stop_or_cancel_prevents_execution_and_clears_scoped_permission(self):
        self.r.between=self.r.cancel.set
        with self.assertRaises(RuntimeError):self.call()
        self.r.cancel.clear();self.r.closed=True
        with self.assertRaises(RuntimeError):self.call()
        self.assertEqual(self.r.calls,[])
        self.assertFalse(self.r.worker_context.direct_operator_action)

    def test_an_async_worker_has_its_own_grant_and_does_not_grant_another_thread(self):
        started=threading.Event();release=threading.Event();errors=[]
        original=self.r.dispatch_sync
        def paused(action,args):
            if args.get('execute') is True:
                # This hook runs outside the scoped sync wrapper; no permission
                # is available here even though another operation is pending.
                started.set();release.wait(1)
            return original(action,args)
        self.r.dispatch_sync=paused
        def work():
            try:self.call()
            except Exception as e:errors.append(e)
        worker=threading.Thread(target=work);worker.start()
        self.assertTrue(started.wait(1))
        with self.assertRaises(RuntimeError):self.r.require_motion_permission()
        release.set();worker.join(1)
        self.assertFalse(worker.is_alive());self.assertEqual(errors,[])
        self.assertEqual(self.r.calls,['move_joint'])

    def test_disable_requires_support_and_enable_does_not_auto_approve_correction(self):
        with self.assertRaises(ValueError):self.call('disable',supported=False)
        self.assertTrue(self.call('disable',supported=True)['executed'])
        self.r.calls.clear();self.r.corrective=True
        with self.assertRaises(ValueError):self.call('enable')
        self.assertEqual(self.r.calls,[])
        self.assertNotIn('allow_limit_adjustment',self.r.invocations[-1])

    def test_monitor_reports_direct_policy_without_falsely_latching_global_permission(self):
        data=self.r.read_monitor_snapshot();original=deepcopy(data)
        _,params,_=monitor_frame(data,now=time.time(),device_name='机械臂 1')
        self.assertEqual(params['动作许可'],'直接控制（逐次校验）')
        self.assertFalse(data['motion_enabled']);self.assertEqual(data,original)


if __name__=='__main__':unittest.main()
