"""Joint-angle card uses real feedback and the existing JSON renderer contract."""
import asyncio
import json
import math
import time
import unittest
from unittest.mock import AsyncMock, patch
from test_sensors import settings, info, joints
from device import Adapter, TOPICS
from state import joint_state
from joint_angles import ANGLE_FIELDS, angle_sample, angles_stream
from skeleton import JOINT_NAMES

class AngleTests(unittest.TestCase):
    def sample(self, **data):
        return {'connected':True, 'available':True, 'sample_status':'recently_received',
                'received_at':123, 'reception_age_seconds':.1, 'measurement_time_verified':False,
                'data':joint_state(joints(**data))}

    def test_all_sixteen_actual_angles_have_group_index_name_and_degree_unit(self):
        q=[i/100 for i in range(16)]
        source=self.sample(q=q)
        result=angle_sample(source)
        self.assertEqual(len(result['data']),16)
        self.assertEqual(list(result['data']),list(ANGLE_FIELDS))
        self.assertTrue(ANGLE_FIELDS[0].startswith('左臂_J0_'))
        self.assertTrue(ANGLE_FIELDS[7].startswith('右臂_J7_'))
        self.assertTrue(ANGLE_FIELDS[14].startswith('头部_J14_'))
        self.assertIn('俯仰',ANGLE_FIELDS[14])
        self.assertIn('偏航',ANGLE_FIELDS[15])
        self.assertTrue(all(k.endswith('_度') for k in ANGLE_FIELDS))
        for i,k in enumerate(ANGLE_FIELDS): self.assertEqual(result['data'][k],math.degrees(q[i]))
        self.assertNotEqual(result['data'][ANGLE_FIELDS[3]],round(math.degrees(.03),3))
        self.assertEqual(source['data']['q'],q)
        self.assertEqual(result['received_at'],source['received_at'])

    def test_vendor_named_reordered_feedback_maps_to_correct_motor(self):
        source=self.sample(names=list(reversed(JOINT_NAMES)),q=[i/100 for i in range(16)])
        packet=angles_stream(angle_sample(source))
        self.assertEqual(packet[ANGLE_FIELDS[0]],math.degrees(.15))
        self.assertEqual(packet[ANGLE_FIELDS[15]],0)

    def test_unavailable_or_unsupported_data_clears_every_tile_instead_of_fake_zero(self):
        for source in (self.sample(q=[.1]*14,dq=[],tau=[]),
                       self.sample(names=['unknown_'+str(i) for i in range(16)]),
                       {**self.sample(), 'available':False,'data':None,'sample_status':'stale'},
                       {**self.sample(), 'available':False,'data':None,'connected':False,'sample_status':'offline'}):
            sample=angle_sample(source)
            self.assertFalse(sample['available'])
            self.assertIsNone(sample['data'])
            packet=angles_stream(sample)
            self.assertEqual(set(packet),set(ANGLE_FIELDS))
            self.assertTrue(all(v is None for v in packet.values()))
            json.dumps(packet,allow_nan=False)

    def test_card_can_be_enabled_alone_without_model_or_extra_robot_query(self):
        adapter=Adapter(lambda *args:None, settings(enabled_cards=['joint_angles']))
        self.assertEqual({t['name'] for t in adapter.tools()},{'joint_angles'})
        tool=adapter.tools()[0]
        self.assertEqual(tool['topic_out'],[{'topic':TOPICS['joint_angles'],'format':'data/json'}])
        adapter.reader.accept_info(info())
        adapter.reader.joints=joint_state(joints())
        adapter.reader.joints_at=time.monotonic(); adapter.reader.joints_received=time.time()
        with patch.object(adapter.reader,'read_joints',new_callable=AsyncMock) as query, patch.object(adapter.reader,'send_movej',new_callable=AsyncMock) as move:
            for action in ('start','info','stop'):
                result=adapter.call('joint_angles',{'action':action,'_trace_id':'test'})
                self.assertTrue(result['read_only'])
            query.assert_not_called(); move.assert_not_called()
        self.assertEqual(len(adapter.call('joint_angles',{})['sample']['data']),16)

    def test_skeleton_and_angles_share_same_receipt_and_one_cache_read(self):
        adapter=Adapter(lambda *args:None,settings(enabled_cards=['joint_state','joint_angles']))
        source=self.sample()
        with patch.object(adapter.reader,'sample',return_value=source) as read:
            result=adapter.snapshot()
        read.assert_called_once_with('joint_state')
        self.assertIs(result['joint_state'],source)
        self.assertEqual(result['joint_angles']['received_at'],source['received_at'])

class AnglePublicationTests(unittest.IsolatedAsyncioTestCase):
    async def test_feedback_changes_and_disconnect_clear_native_json_tiles(self):
        adapter=Adapter(lambda *args:None,settings(enabled_cards=['joint_angles']))
        adapter.reader.accept_info(info())
        adapter.reader.joints_at=time.monotonic(); adapter.reader.joints_received=time.time()
        async def packet():
            ready=asyncio.Event(); packets=[]
            def capture(topic,data):
                self.assertEqual(topic,TOPICS['joint_angles']); packets.append(data); ready.set()
            adapter.publish=capture
            task=asyncio.create_task(adapter.publish_loop())
            try: await asyncio.wait_for(ready.wait(),.5)
            finally:
                task.cancel(); await asyncio.gather(task,return_exceptions=True)
            return packets[0]
        captured=[]
        for elbow in (-.0032,-.3522657):
            q=[0.0]*16; q[3]=elbow; q[9]=-1.4785
            adapter.reader.joints=joint_state(joints(q=q))
            result=await packet(); captured.append(result)
            self.assertEqual(result[ANGLE_FIELDS[3]],math.degrees(elbow))
            self.assertEqual(result[ANGLE_FIELDS[9]],math.degrees(-1.4785))
        self.assertGreater(abs(captured[1][ANGLE_FIELDS[3]]-captured[0][ANGLE_FIELDS[3]]),19)
        adapter.reader.joints_at-=10
        self.assertTrue(all(v is None for v in (await packet()).values()))
        adapter.reader.disconnect('test')
        self.assertTrue(all(v is None for v in (await packet()).values()))

if __name__=='__main__': unittest.main()
