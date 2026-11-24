import copy
import importlib.util
import json
from pathlib import Path
import time
import types
import unittest
from collections import deque
from unittest.mock import patch

from test_driver import FakeNode, FakeRos1, ROOT, sensors_module
KuavoSensors = sensors_module.KuavoSensors
from spatial_telemetry import imu_packet, odometry_pose


class SpatialTelemetryTests(unittest.TestCase):
    def packet(self):
        return dict(schema='motus.imu/1', epoch='epoch1', stamp_ns=1000000003,
                    accel_stamp_ns=1000000004, angular_velocity=[1.,2.,3.],
                    linear_acceleration=[0.,0.,9.81],
                    camera_from_gyro=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
                    camera_from_accel=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

    def test_imu_preserves_raw_axes_timestamps_and_missing_lever_arms(self):
        p = self.packet()
        self.assertEqual(imu_packet(p), p)
        self.assertNotIn('camera_from_gyro_translation', imu_packet(p))
        for key, value in [('stamp_ns', 1.2), ('accel_stamp_ns', 0),
                           ('angular_velocity', [float('nan')]*3),
                           ('camera_from_gyro', [[-1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])]:
            p = self.packet(); p[key] = value
            with self.assertRaises(ValueError):imu_packet(p)

    def test_camera_imu_computation_port_is_independent_of_body_display(self):
        ros, node = FakeRos1(), FakeNode()
        hub = KuavoSensors({}, 'test', ros, node)
        try:
            r = hub.dispatch('imu', 'start')
            self.assertEqual([t['format'] for t in r['topic_out']], ['data/json','sensor/imu'])
            p = self.packet(); hub._on_camera_imu({'data':json.dumps(p)})
            self.assertEqual(json.loads(hub._camera_imu_pub.messages[-1].data), p)
            at = hub._camera_imu_at
            hub._on_camera_imu({'data':json.dumps(p)})
            self.assertEqual(hub._camera_imu_at, at)
            self.assertEqual(hub.camera_imu_status()['state'], 'available')
            self.assertEqual(hub.camera_imu_status()['fusion']['state'], 'unavailable')
            hub.dispatch('imu', 'stop'); before=len(hub._camera_imu_pub.messages)
            p['stamp_ns']+=100;hub._on_camera_imu({'data':json.dumps(p)})
            self.assertEqual(len(hub._camera_imu_pub.messages), before)
            ros.connection=(True,2);hub._session()
            self.assertEqual(hub.camera_imu_status()['state'],'unavailable')
        finally:hub.close()

    def odom(self):
        return {'header':{'stamp':{'secs':1,'nsecs':234567891},'frame_id':'odom'},
                'child_frame_id':'base_footprint',
                'pose':{'pose':{'position':dict(x=1.,y=2.,z=0.),
                                'orientation':dict(x=0.,y=0.,z=0.,w=1.)},'covariance':[.01]*36},
                'twist':{'twist':{'linear':dict(x=.1,y=.2,z=0.),'angular':dict(x=0.,y=0.,z=.3)}}}

    def test_source_pose_ns_survives_navigation_receipt_fallback_and_window_mean(self):
        ros,node=FakeRos1(),FakeNode();hub=KuavoSensors({},'test',ros,node)
        try:
            hub._on_odom(self.odom());s=hub.snapshot('loco_state')
            self.assertEqual(s.get('motion_model'), 'planar')
            self.assertEqual(s['pose']['stamp_ms'],1234.567891)
            self.assertNotEqual(s['stamp_ms'],s['pose']['stamp_ms'])
            self.assertEqual(s['vendor']['motus_slam']['stamp_ns'],1234567891)
            self.assertEqual(s['pose']['position'],[1.,2.,0.])
            self.assertEqual(s['pose']['quaternion'],[0.,0.,0.,1.])
            self.assertEqual(s['twist'],[.1,.2,None,None,None,.3])
            m=self.odom();del m['pose']['pose']['orientation'];hub._on_odom(m)
            self.assertTrue(hub.snapshot('loco_state')['valid'])
            self.assertIsNone(hub.snapshot('loco_state')['pose'])
            self.assertEqual(hub.snapshot('loco_state')['vendor']['motus_slam']['status'],'unavailable')
        finally:hub.close()

    def test_body_gravity_requires_fixed_imu_mount_and_nearby_capture(self):
        hub = KuavoSensors({}, 'test', FakeRos1(), FakeNode())
        odom = self.odom()
        stamp = odom['header']['stamp'].copy()
        axes = dict(x=0., y=0., z=0.)
        imu = dict(header=dict(stamp=stamp, frame_id='imu_link'),
                   orientation=dict(w=1., **axes), angular_velocity=axes, linear_acceleration=axes)
        try:
            hub._on_odom(odom)
            hub._on_standard_imu(imu)
            self.assertNotIn('body_gravity', hub.snapshot('loco_state')['vendor'])
            turn = .5 ** .5
            hub._on_static_tf(dict(transforms=[dict(header=dict(frame_id='base_link'),
                child_frame_id='imu_link', transform=dict(rotation=dict(x=turn,y=0.,z=0.,w=turn)))]))
            hub._on_standard_imu(imu)
            gravity = hub.snapshot('loco_state')['vendor']['body_gravity']
            self.assertEqual(gravity['frame_id'], 'base_link')
            self.assertEqual(gravity['stamp_ns'], 1234567891)
            for actual, expected in zip(gravity['up'], [0., -1., 0.]):
                self.assertAlmostEqual(actual, expected)
            # Equivalent mounting through a shared fixed parent.
            hub._on_static_tf(dict(transforms=[
                dict(header=dict(frame_id='mount'), child_frame_id='base_link',
                     transform=dict(rotation=dict(x=-turn,y=0.,z=0.,w=turn))),
                dict(header=dict(frame_id='mount'), child_frame_id='imu_link',
                     transform=dict(rotation=dict(x=0.,y=0.,z=0.,w=1.)))]))
            hub._on_standard_imu(imu)
            self.assertEqual(hub.snapshot('loco_state')['vendor']['body_gravity'], gravity)
            imu['orientation_covariance'] = [.01,0.,0.,0.,.02,0.,0.,0.,.03]
            hub._on_standard_imu(imu)
            self.assertEqual(hub.snapshot('loco_state')['vendor']['body_gravity']['variance_bound_rad2'], .03)
            stamp['secs'] += 1
            hub._on_standard_imu(imu)
            self.assertNotIn('body_gravity', hub.snapshot('loco_state')['vendor'])
            imu['orientation_covariance'] = [-1.] + [0.] * 8
            stamp['secs'] -= 1
            hub._on_standard_imu(imu)
            self.assertNotIn('body_gravity', hub.snapshot('loco_state')['vendor'])
        finally:
            hub.close()

    def test_missing_source_timestamp_never_becomes_a_fresh_slam_pose(self):
        m=self.odom();m['header']['stamp']={'secs':0,'nsecs':0}
        with self.assertRaises(ValueError):odometry_pose(m,'epoch')

    def test_sdk_imu_packet_preserves_physical_sensor_origins(self):
        spec=importlib.util.spec_from_file_location('head_imu',ROOT/'host/head_imu.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        stamp=lambda n:types.SimpleNamespace(to_nsec=lambda:n)
        gyro=types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(1000000003),frame_id='gyro'),angular_velocity=types.SimpleNamespace(x=1.,y=2.,z=3.))
        accel=types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(1000000004),frame_id='accel'),linear_acceleration=types.SimpleNamespace(x=0.,y=0.,z=9.8))
        tr=types.SimpleNamespace(transform=types.SimpleNamespace(rotation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.),translation=types.SimpleNamespace(x=.01,y=.02,z=.03)))
        p=mod.make_packet(gyro,accel,[tr,tr],'e')
        self.assertEqual(p['camera_from_accel_translation'],[.01,.02,.03])
        self.assertEqual(p['stamp_ns'],1000000003)
        self.assertEqual(p['angular_velocity'],[1.,2.,3.])
        self.assertEqual(imu_packet(p),p)
        accel.header.stamp=stamp(1100000000)
        with self.assertRaises(ValueError):mod.make_packet(gyro,accel,[tr,tr],'e')

    def test_camera_imu_status_expires_when_native_stream_stops(self):
        spec=importlib.util.spec_from_file_location('head_imu',ROOT/'host/head_imu.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        self.assertEqual(mod.availability(None,10.,10.1),('available',None))
        self.assertEqual(mod.availability(None,10.,10.6)[0],'unavailable')
        self.assertEqual(mod.availability('TF missing',10.,10.1),('unavailable','TF missing'))
        self.assertEqual(mod.availability('waiting',None,100.),('unavailable','waiting'))

    def test_camera_imu_drains_delivery_batches_and_waits_for_accel_watermark(self):
        spec=importlib.util.spec_from_file_location('head_imu',ROOT/'host/head_imu.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        frame=lambda n:types.SimpleNamespace(header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(to_nsec=lambda:n)))
        gyros=deque(frame(n)for n in range(1,51))
        accels=deque(frame(n)for n in range(1,42))
        first=mod.take_samples(gyros,accels)
        self.assertEqual(len(first),32)
        self.assertEqual(len(mod.take_samples(gyros,accels)),9)
        self.assertEqual(len(gyros),9)
        self.assertEqual(mod.take_samples(gyros,accels),[])
        accels.extend(frame(n)for n in range(42,51))
        self.assertEqual(len(mod.take_samples(gyros,accels)),9)
        self.assertFalse(gyros)


if __name__=='__main__':unittest.main()
