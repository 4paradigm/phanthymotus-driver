"""Prove rigid bends preserve true FK and also work in the existing renderer."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np

from dobot.cra.model import JOINT_NAMES, skeleton_urdf


def transform(xyz, rpy, platform=False):
    x, y, z = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(x), -math.sin(x)], [0, math.sin(x), math.cos(x)]])
    ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    rz = np.array([[math.cos(z), -math.sin(z), 0], [math.sin(z), math.cos(z), 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rx @ ry @ rz if platform else rz @ ry @ rx
    result[:3, 3] = xyz
    return result


def fk(root, values, platform=False):
    joints = list(root.findall('joint'))
    children = {j.find('child').get('link') for j in joints}
    root_name = next(l.get('name') for l in root.findall('link') if l.get('name') not in children)
    poses = {root_name: np.eye(4)}
    while joints:
        progress = False
        for joint in list(joints):
            parent = joint.find('parent').get('link')
            if parent not in poses:
                continue
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0] * 3
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0] * 3
            local = transform(xyz, rpy, platform)
            if joint.get('type') == 'revolute':
                assert joint.find('axis').get('xyz') == '0 0 1'
                local = local @ transform([0] * 3, [0, 0, values[joint.get('name')]])
            poses[joint.find('child').get('link')] = poses[parent] @ local
            joints.remove(joint)
            progress = True
        assert progress, 'cyclic or disconnected URDF'
    return poses


def test_all_physical_link_poses_preserved_for_standard_urdf_and_platform():
    source = ET.parse(Path(__file__).resolve().parents[1] / 'dobot/cra/resource/cr3_robot.urdf').getroot()
    display = ET.fromstring(skeleton_urdf())
    physical_links = [node.get('name') for node in source.findall('link')]
    samples = [np.zeros(6), np.radians([172, -.005, -150, -35, 0, 0])]
    samples.extend(np.random.default_rng(42).uniform(-math.pi, math.pi, (30, 6)))
    for angles in samples:
        values = dict(zip(JOINT_NAMES, angles))
        expected = fk(source, values)
        for platform in (False, True):
            actual = fk(display, values, platform)
            for name in physical_links:
                np.testing.assert_allclose(actual[name], expected[name], atol=1e-12)


def test_right_angle_has_two_rigid_legs_and_only_six_actuated_axes():
    root = ET.fromstring(skeleton_urdf())
    actuated = [j for j in root.findall('joint') if j.get('type') != 'fixed']
    assert [j.get('name') for j in actuated] == list(JOINT_NAMES)
    bend = root.find("joint[@name='joint4_right_angle']")
    assert bend.get('type') == 'fixed'
    assert float(bend.find('origin').get('rpy').split()[1]) == math.pi / 2
    poses = fk(root, {name: 0 for name in JOINT_NAMES}, platform=True)
    parent = poses['Link3'][:3, 3]
    corner = poses['joint4_right_angle_link'][:3, 3]
    tip = poses['joint4_right_angle_restore_link'][:3, 3]
    np.testing.assert_allclose(np.linalg.norm(corner - parent), .23, atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(tip - corner), .1283, atol=1e-12)
    np.testing.assert_allclose(np.dot(corner - parent, tip - corner), 0, atol=1e-12)
