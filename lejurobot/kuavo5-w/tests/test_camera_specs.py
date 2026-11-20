import copy
import math
import struct
import unittest

import test_driver  # installs ROS stubs and the driver/common import paths
from camera_specs import calibration, declare, jpeg_dimensions, registered_pair
from base_servo import footprint_from_polygon, footprint_from_model, build_descriptor


class CameraSpecsTests(unittest.TestCase):
    def info(self):
        return {'width': 640, 'height': 480, 'K': [370., 0., 320., 0., 371., 240., 0., 0., 1.],
                'D': [0.] * 8, 'distortion_model': 'rational_polynomial',
                'header': {'frame_id': 'camera_head_color_optical_frame'}}

    def test_k_and_resolution_come_from_runtime_not_a_saved_unit_calibration(self):
        c = calibration(self.info())
        d = declare('camera_head', '/head', 'image/jpeg', c)[0]
        self.assertEqual(d['source'], 'derived-from-K')
        self.assertEqual(d['K'], c['K'])
        self.assertEqual((d['width'], d['height']), (640, 480))
        self.assertIsNone(d['half_fov_rad'])  # consumer derives from K

    def test_registration_requires_parameter_frame_intrinsics_and_dimensions(self):
        c = calibration(self.info())
        self.assertTrue(registered_pair(c, c, True))
        self.assertFalse(registered_pair(c, c, False))
        for field, value in [('frame_id', 'native_depth'), ('width', 320),
                             ('K', [300., 0., 320., 0., 371., 240., 0., 0., 1.]), ('D', [.01]*8)]:
            d = copy.deepcopy(c); d[field] = value
            self.assertFalse(registered_pair(c, d, True))

    def test_native_depth_and_another_camera_never_claim_rgb_identity(self):
        head = declare('camera_head', '/rgb', 'image/jpeg')[0]
        native = declare('camera_depth', '/depth', 'image/depth-zlib')[0]
        aligned = declare('camera_depth', '/depth', 'image/depth-zlib', aligned=True)[0]
        self.assertNotEqual(head['id'], native['id'])
        self.assertEqual(head['id'], aligned['id'])
        self.assertNotEqual(head['id'], declare('camera_left', '/left', 'image/jpeg')[0]['id'])

    def test_invalid_or_different_resolution_intrinsics_are_not_exported(self):
        for key, value in [('K', [math.nan]*9), ('width', 0), ('D', [True])]:
            m = self.info(); m[key] = value
            with self.assertRaises(ValueError):calibration(m)
        c = calibration(self.info())
        d = declare('camera_head', '/rgb', 'image/jpeg', c, (1280, 720))[0]
        self.assertIsNone(d['K'])
        self.assertEqual((d['width'], d['height']), (1280, 720))

    def test_jpeg_dimensions_are_read_without_decoding_pixels(self):
        sof = b'\xff\xd8\xff\xc0'+struct.pack('>HBHHB', 8, 8, 480, 640, 0)+b'\xff\xd9'
        self.assertEqual(jpeg_dimensions(sof), (640, 480))
        self.assertIsNone(jpeg_dimensions(b'\xff\xd8bad\xff\xd9'))
        self.assertIsNone(jpeg_dimensions(sof[:8]))

    def test_factory_costmap_envelope_is_an_estimate_not_a_hardware_measurement(self):
        f = footprint_from_polygon('[[0.34,0.34],[0.34,-0.34],[-0.34,-0.34],[-0.34,0.34]]', .01)
        self.assertAlmostEqual(f['half_width'], .35)
        self.assertEqual(f['source'], 'estimate')
        self.assertIsNone(f['height'])
        self.assertEqual(build_descriptor(f)['footprint'], f)

    def test_runtime_vendor_chassis_collision_is_nominal_geometry_without_padding(self):
        xml = '<robot><link name="base_link"><collision><origin xyz="0 0 0.18"/>' \
              '<geometry><cylinder radius="0.37" length="0.32"/></geometry></collision></link></robot>'
        f = footprint_from_model(xml)
        self.assertEqual([f[k] for k in ('half_width', 'front', 'rear')], [.37]*3)
        self.assertEqual(f['source'], 'vendor-spec')
        self.assertEqual(f['scope'], 'chassis')
        self.assertIsNone(f['height'])  # Cylinder height is not the robot height.
        self.assertEqual(len(f['runtime_urdf_sha256']), 64)
        shifted = footprint_from_model(xml.replace('0 0 0.18', '0.02 -0.01 0.18'))
        self.assertAlmostEqual(shifted['half_width'], .38)
        self.assertAlmostEqual(shifted['front'], .39)
        self.assertAlmostEqual(shifted['rear'], .35)
        for invalid in (xml.replace('0.37', 'nan'), xml.replace('0.37', '-1'),
                        xml.replace('xyz="0 0 0.18"', 'rpy="0.18 0 0"'), xml.replace('name="base_link"', 'name="other"'),
                        '<robot><link name="base_link"/></robot>',
                        '<!DOCTYPE robot [<!ENTITY x "bad">]>' + xml):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    footprint_from_model(invalid)
        for p in ([], [[1, 1]]*3, [[math.nan, 0]]*4):
            with self.assertRaises(ValueError):footprint_from_polygon(p)


if __name__ == '__main__':unittest.main()
