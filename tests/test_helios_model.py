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
