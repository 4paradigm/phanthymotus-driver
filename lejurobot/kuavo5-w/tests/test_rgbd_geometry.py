import copy
import importlib.util
from pathlib import Path
import types
import unittest

ROOT=Path(__file__).resolve().parents[1]
def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'host'/(name+'.py'))
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod
geometry,relay=load('rgbd_geometry'),load('head_rgbd')
try:
    import cv2
    import numpy as np
except ImportError:
    cv2=np=None


class RgbdGeometryTests(unittest.TestCase):
    def info(self):
        return types.SimpleNamespace(width=640,height=480,K=[367.468933,0.,320.404572,0.,367.310486,245.939484,0.,0.,1.],
            D=[0.]*8,distortion_model='rational_polynomial',header=types.SimpleNamespace(frame_id='camera_head_color_optical_frame'))

    def frame(self,ns):
        return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_nsec=lambda:ns)))

    def test_capture_pairing_is_bounded_drops_wrong_times_and_preserves_headers(self):
        pairs=relay.Pairs();c,d=self.frame(1_000_000_000),self.frame(1_012_000_000)
        pairs.receive('color',c);pairs.receive('depth',self.frame(800_000_000))
        self.assertIsNone(pairs.take())
        pairs.receive('depth',d);self.assertEqual(pairs.take(),(c,d));self.assertIsNone(pairs.take())
        pairs.receive('color',c);pairs.receive('depth',d);self.assertIsNone(pairs.take())
        for n in range(100):pairs.receive('color',self.frame(2_000_000_000+n))
        self.assertLessEqual(len(pairs.frames['color']),16)

    def test_registration_transport_delay_does_not_discard_capture_partners(self):
        pairs=relay.Pairs()
        colors=[self.frame(1_000_000_000+i*66_666_667) for i in range(30)]
        count=0
        for i,c in enumerate(colors):
            pairs.receive('color',c)
            if i>=5:
                d=self.frame(1_014_000_000+(i-5)*66_666_667)
                pairs.receive('depth',d)
                pair=pairs.take()
                self.assertIsNotNone(pair)
                self.assertIs(pair[0],colors[i-5])
                self.assertIs(pair[1],d)
                count+=1
        self.assertEqual(count,25)

    def test_geometry_refuses_different_optical_frames_or_distorted_depth(self):
        color,depth=self.info(),self.info();depth.header.frame_id='native_depth'
        with self.assertRaises(ValueError):geometry.make_geometry(color,depth,None,None)
        depth=self.info();depth.D=[.01]*8
        with self.assertRaises(ValueError):geometry.make_geometry(color,depth,None,None)

    @unittest.skipIf(cv2 is None,'OpenCV is a host runtime dependency; live host runs this test')
    def test_rectification_maps_rgb_to_depth_rays_and_declares_effective_k(self):
        color,depth=self.info(),self.info()
        color.D=[-.0295765,.0325162,-.0005005,.0005817,-.0112032,0.,0.,0.]
        g=geometry.make_geometry(color,depth,cv2,np)
        self.assertEqual(g['info'].D,[0.]*5)
        self.assertEqual(g['info'].distortion_model,'plumb_bob')
        k=np.asarray(g['info'].K).reshape(3,3)
        points=np.array([[[0.,0.]],[[320.,240.]],[[639.,479.]]],dtype=np.float64)
        source=cv2.undistortPoints(points,np.asarray(color.K).reshape(3,3),np.asarray(color.D),P=k)
        # A distorted lens must not be copied unchanged into the rectified stream.
        self.assertGreater(abs(float(source[0,0,0])),.01)
        self.assertTrue(np.isfinite(g['color_x']).all())
        self.assertGreaterEqual(float(g['color_x'].min()),0.)
        self.assertLessEqual(float(g['color_x'].max()),639.)
        header=types.SimpleNamespace(frame_id='actual',stamp='original_capture')
        out=geometry.camera_info(g,header)
        self.assertIs(out.header,header);self.assertEqual(out.K,g['info'].K)
        self.assertIsNot(out,g['info'])


if __name__=='__main__':unittest.main()
