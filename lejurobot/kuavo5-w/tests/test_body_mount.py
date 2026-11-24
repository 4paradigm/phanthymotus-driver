import importlib.util
from pathlib import Path
import unittest
import math
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('body_mount', ROOT/'host/body_mount.py')
mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)


class MountTests(unittest.TestCase):
    def setUp(self):
        self.urdf = (ROOT/'resource/kuavo5w_model.urdf').read_text()
        self.model = mod.Kinematics(self.urdf)

    def test_zero_pose_and_head_yaw_follow_model_not_identity(self):
        q = [0.]*20
        matrix = self.model.forward(q)
        # Independent CAD chain sum at zero joint angles (metres).
        c,s = math.cos(.3406),math.sin(.3406)
        expected = [.093-.46521+.48296+.099619+.011715*c+.014311*s,
                    .0475, .30085+.1055+.18325+.12941+.25+.43935+.032844-.011715*s+.014311*c]
        np.testing.assert_allclose(matrix[:3,3],expected,atol=1e-9)
        np.testing.assert_allclose(matrix[3], [0,0,0,1])
        self.assertGreater(np.linalg.norm(matrix[:3,3]), 1.)
        # Rotating yaw changes both optical orientation and offset camera origin.
        q[18] = math.pi/2
        turned = self.model.forward(q)
        self.assertGreater(np.linalg.norm(turned[:3,3]-matrix[:3,3]), .05)
        np.testing.assert_allclose(turned[:3,:3].T@turned[:3,:3], np.eye(3),atol=1e-12)

    def test_lower_body_and_pitch_affect_camera_pose(self):
        initial = self.model.forward([0.]*20)
        for index in (0,1,2,3,18,19):
            q=[0.]*20; q[index]=.1
            self.assertGreater(np.linalg.norm(self.model.forward(q)-initial), .01)
        q=[0.]*20;q[4]=.5
        np.testing.assert_allclose(self.model.forward(q), initial)

    def test_capture_interpolation_refuses_latest_and_large_gap(self):
        samples=[(1_000_000_000,[0.]*20),(1_020_000_000,[.2]*20)]
        q,bracket=mod.interpolate(samples,1_010_000_000)
        np.testing.assert_allclose(q,[.1]*20)
        self.assertEqual(bracket,(1_000_000_000,1_020_000_000))
        for ns in (999_000_000,1_021_000_000):
            with self.assertRaises(ValueError):mod.interpolate(samples,ns)
        with self.assertRaises(ValueError):mod.interpolate([(1,[0.]*20),(100_000_001,[1.]*20)],50_000_001)

    def test_reject_changed_cad_or_unknown_joint_chain(self):
        root=ET.fromstring(self.urdf)
        j=next(j for j in root.findall('joint') if j.find('child').get('link')=='head_camera_depth')
        j.find('origin').set('xyz','0 0 0')
        with self.assertRaises(ValueError):mod.Kinematics(ET.tostring(root,encoding='unicode'))
        for q in ([0.]*19,[float('nan')]*20):
            with self.assertRaises(ValueError):self.model.forward(q)
