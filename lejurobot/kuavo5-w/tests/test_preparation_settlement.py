"""A resolved preparation can settle passively; uncertain writes stay fenced."""
import copy
import tempfile
import unittest
from unittest.mock import patch
from test_controlled_spatial import Bundle, Client
from controlled_spatial import ControlledSpatialPlugin


class PreparationSettlementTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.bundle, self.client = Bundle(), Client()
        self.client.feedback['state'].update(nodeStates=[], edgeStates=[], actionStates=[])
        self.plugin = ControlledSpatialPlugin(self.bundle, {'db_path':self.temp.name+'/state.db'},self.client)
        self.bundle.spatial = self.plugin
        self.addCleanup(self.plugin.close)
        self.plugin._preparation = {'delivery_uncertain':False}
        self.plugin._persist()
        original = self.client.native
        def native(operation, **args):
            if operation == 'held_stationary':
                self.client.calls.append((operation,args))
                return {'success':True,'stationary':True,'software_estop':True}
            return original(operation,**args)
        self.client.native = native

    def test_info_clears_only_resolved_preparation_without_motion(self):
        result = self.plugin.dispatch('info',{})
        self.assertIsNone(result['chassis_preparation'])
        self.assertIsNone(self.plugin.store.state('pending')['chassis_preparation'])
        self.assertEqual([c[0] for c in self.client.calls],['held_stationary'])
        self.assertTrue(self.plugin.base_preparation_required)
        self.assertFalse(self.bundle.pauses)
        self.assertFalse(self.client.commands)

    def test_unknown_release_and_other_owners_cannot_settle(self):
        for attr,value in [('_preparation',{'delivery_uncertain':True}),('_task',{'task_id':'old'}),
                           ('_uncertain',True),('_map_pending',{'operation':'load_map'}),
                           ('_keyboard_pending',True)]:
            with self.subTest(attr=attr):
                old=getattr(self.plugin,attr);setattr(self.plugin,attr,value)
                self.plugin._settle_held_preparation()
                self.assertIsNotNone(self.plugin._preparation)
                setattr(self.plugin,attr,old)
        self.assertFalse(self.client.calls)

    def test_live_or_missing_factory_task_state_cannot_settle(self):
        variants=[('agvState','isMapping',True),('state','driving',True),
                  ('state','nodeStates',[{}]),('state','edgeStates',None),('state','actionStates',[{}])]
        for group,key,value in variants:
            with self.subTest(key=key):
                old=copy.deepcopy(self.client.feedback)
                self.client.feedback[group][key]=value
                self.plugin._preparation_check_at=0.
                self.plugin._settle_held_preparation()
                self.assertIsNotNone(self.plugin._preparation)
                self.client.feedback=old
        self.assertFalse(self.client.calls)

    def test_stop_or_reconnect_during_passive_check_keeps_fence(self):
        original=self.client.native
        def stop(operation,**args):
            self.bundle._stop_epoch+=1
            return original(operation,**args)
        with patch.object(self.client,'native',side_effect=stop):
            self.plugin._settle_held_preparation()
        self.assertIsNotNone(self.plugin._preparation)

    def test_failed_held_feedback_keeps_record_and_throttles_repeated_checks(self):
        with patch.object(self.client,'native',side_effect=TimeoutError()) as native:
            self.plugin._settle_held_preparation()
            self.plugin._settle_held_preparation()
        self.assertEqual(native.call_count,1)
        self.assertIsNotNone(self.plugin._preparation)

    def test_preparation_has_its_own_block_reason(self):
        self.assertIn('chassis preparation',self.plugin.motion_block_reason())
        self.assertNotIn('native navigation is active',self.plugin.motion_block_reason())


if __name__=='__main__':
    unittest.main()
