"""Numeric geometry/time regressions for Panther's PR519-compatible inputs."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spatial_inputs import make_geometry, metric_depth, imu_packet, odom_pose, transform_matrix


def header(stamp=1234567890123, frame="optical"):
    return NS(stamp=NS(sec=stamp//10**9, nanosec=stamp % 10**9), frame_id=frame)


def info(width=640, height=480):
    return NS(header=header(), width=width, height=height,
              k=[500., 0., width/2, 0., 500., height/2, 0., 0., 1.],
              d=[0.]*8, distortion_model="plumb_bob")


class GeometryTest(unittest.TestCase):
    def test_shared_rays_preserve_pixel_geometry(self):
        g = make_geometry(info(), info(), registered=True)
        u, v = np.meshgrid(np.arange(640), np.arange(480))
        np.testing.assert_allclose(g['color_x'], u, atol=1e-4)
        np.testing.assert_allclose(g['color_y'], v, atol=1e-4)
        np.testing.assert_array_equal(g['color_x'], g['depth_x'])

    def test_crop_resize_updates_principal_point_with_pixel_centres(self):
        g = make_geometry(info(1280, 720), info(1280, 720), registered=True)
        self.assertAlmostEqual(g['K'][0, 0], 500*2/3)
        self.assertAlmostEqual(g['K'][0, 2], (640-160+.5)*2/3-.5)
        self.assertAlmostEqual(g['K'][1, 2], (360+.5)*2/3-.5)
        self.assertEqual(g['depth_x'].shape, (480, 640))

    def test_missing_or_contradictory_calibration_is_rejected(self):
        for defect in ('registration', 'frame', 'intrinsics', 'distortion', 'nonfinite'):
            color, depth = info(), info()
            if defect == 'frame': depth.header.frame_id = 'depth_optical'
            if defect == 'intrinsics': depth.k[0] = 0
            if defect == 'distortion': depth.d[0] = .1
            if defect == 'nonfinite': color.d[0] = float('nan')
            with self.subTest(defect=defect), self.assertRaises(ValueError):
                make_geometry(color, depth, registered=defect != 'registration')


class MotionTest(unittest.TestCase):
    def imu(self):
        return NS(header=header(frame='imu'), angular_velocity=NS(x=1., y=2., z=3.),
                  linear_acceleration=NS(x=0., y=0., z=9.8),
                  angular_velocity_covariance=[0.]*9, linear_acceleration_covariance=[0.]*9)

    def odom(self):
        return NS(header=header(frame='odom'), child_frame_id='base_link',
                  pose=NS(pose=NS(position=NS(x=1., y=2., z=0.),
                                  orientation=NS(x=0., y=0., z=0., w=2.)), covariance=[0.]*36))

    def test_imu_keeps_raw_vectors_and_declares_rotation_and_origin(self):
        tf = NS(transform=NS(translation=NS(x=.1, y=.2, z=.3),
                             rotation=NS(x=0., y=0., z=2**-.5, w=2**-.5)))
        matrix = transform_matrix(tf)
        packet = imu_packet(self.imu(), matrix, 'optical', 'epoch')
        self.assertEqual(packet['stamp_ns'], 1234567890123)
        self.assertEqual(packet['accel_stamp_ns'], packet['stamp_ns'])
        self.assertEqual(packet['angular_velocity'], [1., 2., 3.])
        np.testing.assert_allclose(np.array(packet['camera_from_gyro']) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        self.assertEqual(packet['camera_from_accel_translation'], [.1, .2, .3])
        self.assertFalse(packet['clock_mapping']['verified'])

    def test_bad_imu_data_cannot_be_substituted_by_zeroes_or_receipt_time(self):
        for defect in ('stamp', 'covariance', 'vector'):
            m = self.imu()
            if defect == 'stamp': m.header = header(0)
            if defect == 'covariance': m.angular_velocity_covariance[0] = -1
            if defect == 'vector': m.linear_acceleration.x = float('nan')
            with self.subTest(defect=defect), self.assertRaises(ValueError):
                imu_packet(m, np.eye(4), 'optical', 'epoch')

    def test_odom_has_independent_precise_source_pose_time(self):
        pose, quality = odom_pose(self.odom(), 'segment')
        self.assertEqual(pose['position'], [1., 2., 0.])
        self.assertEqual(pose['quaternion'], [0., 0., 0., 1.])
        self.assertEqual(pose['stamp_ms'], 1234567)
        self.assertEqual(quality['stamp_ns'], 1234567890123)
        self.assertEqual(quality['status'], 'tracking')
        self.assertEqual(quality['segment_id'], 'segment')

    def test_invalid_odom_pose_clears_computation_pose(self):
        for defect in ('stamp', 'quaternion', 'position', 'frame'):
            m = self.odom()
            if defect == 'stamp': m.header = header(0)
            if defect == 'quaternion': m.pose.pose.orientation.w = 0
            if defect == 'position': m.pose.pose.position.x = float('nan')
            if defect == 'frame': m.child_frame_id = ''
            with self.subTest(defect=defect):
                pose, quality = odom_pose(m, 'segment')
                self.assertIsNone(pose)
                self.assertNotEqual(quality['status'], 'tracking')


class DepthTest(unittest.TestCase):
    def test_big_endian_depth_ignores_row_padding_and_scales_units(self):
        message = NS(encoding='16UC1', width=2, height=2, step=6, is_bigendian=True,
                     data=bytes.fromhex('000a0014ffff001e0028ffff'))
        np.testing.assert_array_equal(metric_depth(message, .5), [[5, 10], [15, 20]])
        self.assertEqual(metric_depth(message).dtype.str, '<u2')

    def test_float_metres_normalize_to_mm_and_invalid_to_zero(self):
        message = NS(encoding='32FC1', width=6, height=1, step=24, is_bigendian=False,
                     data=np.array([1.25, float('nan'), float('inf'), -1., 80., 0.], '<f4').tobytes())
        np.testing.assert_array_equal(metric_depth(message), [[1250, 0, 0, 0, 0, 0]])

    def test_truncated_and_unknown_units_are_rejected(self):
        message = NS(encoding='16UC1', width=2, height=1, step=4, is_bigendian=False, data=b'\0\0')
        with self.assertRaisesRegex(ValueError, 'truncated'):
            metric_depth(message)
        message.data = b'\0'*4
        for scale in (0, -1, float('nan')):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                metric_depth(message, scale)


class RuntimeContractTest(unittest.TestCase):
    def test_estimated_mount_composes_both_capture_time_chains_and_native_wins(self):
        from camera import PantherCameraRuntime
        runtime = object.__new__(PantherCameraRuntime)
        runtime._config = {'estimated_body_mount': {'enabled': True, 'model_frame': 'link_rgbd',
                           'source_frame': 'camera_link', 'translation_m': [.1, 0., 0.]}}
        runtime._geometry = {'frame_id': 'optical'}
        runtime._epoch = 'epoch'; runtime._mount_error = ''
        packets, calls = [], []
        runtime._bridge = NS(_namespace='test', publish=lambda card, packet: packets.append(packet))
        def lookup(target, source, message):
            calls.append((target, source, message.header.stamp))
            if target == 'base_link' and source == 'optical':
                raise LookupError('disconnected')
            matrix = np.eye(4)
            if source == 'link_rgbd': matrix[:3, 3] = [1., 2., 3.]
            if target == 'camera_link': matrix[:3, 3] = [0., .2, 0.]
            return matrix
        runtime._lookup_matrix = lookup
        msg = NS(header=header())
        runtime._publish_mount(msg)
        packet = packets[-1]
        np.testing.assert_allclose(np.array(packet['matrix'])[:3, 3], [1.1, 2.2, 3.])
        self.assertEqual(packet['source'], 'configured_cad_estimate')
        self.assertFalse(packet['accuracy_validated'])
        self.assertTrue(all(c[2] is msg.header.stamp for c in calls))
        runtime._lookup_matrix = lambda *args: np.eye(4)
        runtime._publish_mount(msg)
        self.assertEqual(packets[-1]['source'], 'tf')
        np.testing.assert_array_equal(packets[-1]['matrix'], np.eye(4))

    def test_missing_optional_imu_does_not_restart_camera_service(self):
        import threading
        from unittest.mock import Mock, patch
        from camera import PantherCameraRuntime
        runtime = object.__new__(PantherCameraRuntime)
        runtime._config = {}
        runtime._closed = False
        runtime._lock = threading.RLock()
        runtime._active = {'imu'}
        runtime._managed_unit = 'owned-camera.service'
        runtime._armed_at = 1.
        runtime._last_frame_at = {'imu': 0., 'rgb': 0., 'depth': 0.}
        runtime._watchdog_restarts = 0
        runtime._run_host = Mock()
        def finish_loop(*args):
            runtime._closed = True
        with patch('camera.time.sleep', side_effect=finish_loop):
            runtime._watchdog_loop()
        runtime._run_host.assert_not_called()

    def test_failed_tf_clears_previous_mount_matrix_without_blocking_images(self):
        from camera import PantherCameraRuntime
        runtime = object.__new__(PantherCameraRuntime)
        runtime._config = {}
        runtime._geometry = {'frame_id': 'optical'}
        runtime._epoch = 'epoch'
        runtime._mount_error = ''
        packets = []
        runtime._bridge = NS(_namespace='test', publish=lambda card, packet: packets.append((card, packet)))
        runtime._lookup_matrix = lambda *args: np.eye(4)
        message = NS(header=header())
        runtime._publish_mount(message)
        self.assertIn('matrix', packets[-1][1])
        def missing(*args):
            raise LookupError('disconnected TF trees')
        runtime._lookup_matrix = missing
        runtime._publish_mount(message)
        self.assertNotIn('matrix', packets[-1][1])
        self.assertEqual(packets[-1][1]['status'], 'unavailable')

    def test_existing_image_ports_remain_single_and_imu_appends_named_port(self):
        from plugins import CameraPlugin, ImuPlugin, OdomPlugin
        from core_bridge import output_topics, encode_bridge_payload
        from common.camera_info import build
        topics = output_topics('test')
        d = NS(topics=topics, camera_imu=lambda action: {'state': 'idle'},
               camera_info=lambda kind: build(topic=topics['camera_color' if kind == 'rgb' else 'camera_depth']['topic'],
                                              id='test/calibration', K=[500, 0, 320, 0, 500, 240, 0, 0, 1],
                                              width=640, height=480, source='derived-from-K'))
        for kind, name in [('rgb', 'camera_color'), ('depth', 'camera_depth')]:
            tool = CameraPlugin(d, name, kind, 'test').get_tool()
            self.assertEqual(tool['topic_out'], [topics[name]])
            self.assertIsNotNone(tool['camera_info'][0]['K'])
        imu = ImuPlugin(d, 'imu', 'test', ['imu']).get_tool()
        self.assertEqual(imu['topic_out'][0], topics['imu'])
        self.assertEqual(imu['topic_out'][1]['name'], 'camera_imu')
        self.assertEqual(imu['topic_out'][1]['format'], 'sensor/imu')
        self.assertEqual(encode_bridge_payload('camera_imu', {'schema': 'motus.imu/1'})['kind'], 'string')
        self.assertEqual(OdomPlugin(d, 'odom', 'test', ['odom']).get_tool()['odom_interface']['pose_drift'], 'unbounded')

    def test_odom_advertises_one_navi_compatible_port(self):
        from core_bridge import output_topics
        from plugins import OdomPlugin

        topics = output_topics('test')
        device = NS(topics=topics, sensor_info=lambda _: {'available': True, 'fresh': True})
        odom = OdomPlugin(device, 'odom', 'test', ['odom'])
        expected = [{'topic': '/test/panther/state/odom', 'format': 'state/odom'}]
        self.assertEqual(odom.get_tool()['topic_out'], expected)
        for action in ('start', 'info'):
            with self.subTest(action=action):
                self.assertEqual(odom.dispatch(action, {})['topic_out'], expected)


if __name__ == '__main__':
    unittest.main()
