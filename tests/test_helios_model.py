"""Runtime model discovery uses bounded reads and never bundles factory CAD."""

import json
import threading
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from rokae.helios.feedback import JOINT_NAMES
from rokae.helios.model import fetch_model


def synthetic_model():
    # Original test geometry, deliberately unrelated to physical robot dimensions.
    links = ['base_link'] + [f'test_link_{i}' for i in range(20)]
    xml = '<robot name="test_robot">' + ''.join(f'<link name="{name}"/>' for name in links)
    for i, name in enumerate(JOINT_NAMES):
        xml += (f'<joint name="{name}" type="revolute"><parent link="{links[i]}"/>'
                f'<child link="{links[i+1]}"/><origin xyz="0 0 0.1" rpy="0.1 0.2 0.3"/>'
                '<axis xyz="0 0 1"/></joint>')
    return xml + '</robot>'


@contextmanager
def model_server():
    state = {'reference': '/model.urdf', 'xml': synthetic_model(), 'requests': []}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            state['requests'].append(self.path)
            if self.path == '/api/robot_3d_model':
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps({'urdf_url': state['reference']}).encode())
            elif self.path == '/model.urdf':
                self.send_response(200)
                self.end_headers()
                self.wfile.write(state['xml'].encode())
            elif self.path == '/redirect':
                self.send_response(302)
                self.send_header('Location', '/model.urdf')
                self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    state['state_url'] = f'ws://127.0.0.1:{server.server_port}/ws'
    try:
        yield state
    finally:
        server.shutdown()
        server.server_close()
        worker.join()


def test_discover_model_from_state_endpoint():
    with model_server() as server:
        assert fetch_model(server['state_url']) == server['xml']
        assert server['requests'] == ['/api/robot_3d_model', '/model.urdf']


@pytest.mark.parametrize('reference', [None, '', 'file:///etc/passwd', '//example.com/model',
                                        'http://user:password@127.0.0.1/model'])
def test_reject_missing_or_external_model_reference(reference):
    with model_server() as server:
        server['reference'] = reference
        with pytest.raises(ValueError):
            fetch_model(server['state_url'])
        assert server['requests'] == ['/api/robot_3d_model']


def test_reject_redirect_and_mismatched_or_oversized_model():
    with model_server() as server:
        server['reference'] = '/redirect'
        with pytest.raises(Exception, match='302'):
            fetch_model(server['state_url'])
        assert '/model.urdf' not in server['requests']
        server['reference'] = '/model.urdf'
        server['xml'] = '<robot name="wrong"><link name="base"/></robot>'
        with pytest.raises(ValueError, match='movable joints'):
            fetch_model(server['state_url'])
        server['xml'] = 'x' * (1024 * 1024 + 1)
        with pytest.raises(ValueError, match='size limit'):
            fetch_model(server['state_url'])


def _head_model(rpy="0 0 0", axis="0 1 0"):
    # Original test geometry: a head frame offset sideways with mirrored child mounts.
    return ('<robot name="head_test"><link name="head_link_1"/>'
            '<link name="head_link_2"><visual><origin xyz="0 0 0" rpy="0 0 0"/></visual></link>'
            '<link name="cam_l"/><link name="cam_r"/><link name="cam_front"/>'
            '<joint name="head_joint_2" type="revolute"><parent link="head_link_1"/>'
            f'<child link="head_link_2"/><origin xyz="0.02 0.04 0.2" rpy="{rpy}"/><axis xyz="{axis}"/></joint>'
            '<joint name="cl" type="fixed"><parent link="head_link_2"/><child link="cam_l"/>'
            '<origin xyz="-0.05 0.03 0.03" rpy="0 0 2.1"/></joint>'
            '<joint name="cr" type="fixed"><parent link="head_link_2"/><child link="cam_r"/>'
            '<origin xyz="-0.05 -0.11 0.03" rpy="0 0 -2.1"/></joint>'
            '<joint name="cf" type="fixed"><parent link="head_link_2"/><child link="cam_front"/>'
            '<origin xyz="0.1 -0.04 0.05" rpy="0 0.3 0"/></joint></robot>')


def _positions(xml):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(xml)
    joints = {j.get("name"): [float(v) for v in j.find("origin").get("xyz").split()]
              for j in root.findall("joint")}
    head = joints["head_joint_2"]
    return head, {n: [h + c for h, c in zip(head, joints[n])] for n in ("cl", "cr", "cf")}


def test_head_frame_recentered_without_moving_children():
    from rokae.helios.model import recenter_head
    original = _head_model()
    head_before, before = _positions(original)
    recentred = recenter_head(original)
    head_after, after = _positions(recentred)
    assert head_after[1] == 0 and head_after[0] == head_before[0] and head_after[2] == head_before[2]
    for name in before:  # Same position in the parent frame: kinematics unchanged.
        assert after[name] == pytest.approx(before[name])
    rel = {n: [a - h for a, h in zip(after[n], head_after)] for n in after}
    assert rel["cl"][1] == pytest.approx(-rel["cr"][1])  # Mirrored mounts now look mirrored.
    assert rel["cf"][1] == pytest.approx(0)
    assert 'rpy="0 0 2.1"' in recentred  # Mount orientations are untouched.


@pytest.mark.parametrize("rpy, axis", [("0 0.1 0", "0 1 0"), ("0 0 0", "0 0 1")])
def test_head_frame_left_untouched_when_shift_would_change_kinematics(rpy, axis):
    from rokae.helios.model import recenter_head
    model = _head_model(rpy, axis)
    assert recenter_head(model) == model
