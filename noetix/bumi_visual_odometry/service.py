"""Standalone MCP sensor service. Lifecycle requests never send motion commands."""
from __future__ import annotations
import json
import os
from pathlib import Path
import signal
import ssl
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

NAME = 'rgbd_visual_odometry'


def tool():
    return dict(name=NAME, type='sensor', multiInstance=False,
                description='Bumi RGB-D 视觉里程计（RTAB-Map rgbd_odometry），只读；'
                            '无 IMU/EKF 融合，未接入距离控制，精度未验收。',
                inputSchema={'type': 'object', 'properties': {'action': {'type': 'string',
                             'enum': ['start', 'stop', 'info', 'reset', 'config']}}},
                configSchema={'type': 'object', 'properties': {
                    'source_namespace': {'type': 'string',
                                         'description': 'Bumi namespace; auto-discover only when unique'},
                    'calibration': {'type': 'object',
                                    'description': 'Optional measured camera_to_body mount; '
                                                   'without it the pose stays in the optical frame'}}},
                topic_out=[{'topic': '/bumi_visual_odometry/state/odom', 'format': 'state/odom',
                            'message_type': 'std_msgs/msg/String'}])


class SensorService:
    def __init__(self, runtime, config_path):
        self.runtime = runtime
        self.config_path = Path(config_path)

    def dispatch(self, args):
        with self.runtime.lock:
            action = args.get('action', 'info')
            if action == 'start':
                return self.runtime.start()
            if action == 'stop':
                return self.runtime.stop()
            if action == 'reset':
                return self.runtime.reset()
            if action == 'info':
                return self.runtime.info()
            if action == 'config':
                result = self.runtime.configure(args)
                self.config_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.config_path.with_suffix('.tmp')
                temporary.write_text(json.dumps(self.runtime.config, allow_nan=False))
                temporary.replace(self.config_path)
                return result
            raise ValueError('unsupported sensor action')


def handler(service):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            if self.path != '/mcp':
                self.send_error(404)
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 1024*1024:
                    self.send_error(413)
                    return
                req = json.loads(self.rfile.read(length))
                if 'id' not in req:
                    self.send_response(202)
                    self.end_headers()
                    return
                method = req.get('method')
                if method == 'initialize':
                    result = {'protocolVersion': '2024-11-05',
                              'capabilities': {'tools': {}},
                              'serverInfo': {'name': NAME, 'version': '1.0.0'}}
                elif method == 'tools/list':
                    result = {'tools': [tool()]}
                elif method == 'tools/call':
                    params = req.get('params') or {}
                    if params.get('name') != NAME:
                        raise ValueError('unknown tool')
                    try:
                        payload = service.dispatch(params.get('arguments') or {})
                    except Exception as exc:
                        payload = {'state': 'error', 'error': str(exc)}
                    result = {'content': [{'type': 'text',
                                           'text': json.dumps(payload, allow_nan=False)}]}
                else:
                    raise ValueError('unsupported method')
                response = {'jsonrpc': '2.0', 'id': req['id'], 'result': result}
            except Exception as exc:
                response = {'jsonrpc': '2.0', 'id': locals().get('req', {}).get('id'),
                            'error': {'code': -32602, 'message': str(exc)}}
            data = json.dumps(response, allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
    return Handler


def main():
    from common import logsafe
    logsafe.install()
    import rclpy
    try:
        from .runtime import VisualRuntime
    except ImportError:
        from runtime import VisualRuntime
    path = Path(os.environ.get('CONFIG_PATH',
                               '/opt/phanthy-motus/data/bumi_visual_odometry/config.json'))
    config = json.loads(path.read_text()) if path.exists() else {}
    rclpy.init()
    runtime = VisualRuntime(config)
    service = SensorService(runtime, path)
    port = int(os.environ.get('MCP_PORT', '15720'))
    server = ThreadingHTTPServer(('0.0.0.0', port), handler(service))
    stopped = threading.Event()

    def register():
        # Same local registration path as the production Bumi bundle.
        context = ssl._create_unverified_context()
        url = os.environ.get('AGENT_CORE_URL', 'https://localhost:15678')+'/api/mcp'
        data = json.dumps({'name': 'Bumi RGB-D Visual Odometry',
                           'url': f'http://localhost:{port}/mcp',
                           'category': 'driver'}).encode()
        while not stopped.is_set():
            try:
                req = urllib.request.Request(url, data=data,
                                             headers={'Content-Type': 'application/json'},
                                             method='POST')
                with urllib.request.urlopen(req, timeout=3, context=context):
                    pass
                stopped.wait(30)
            except Exception:
                stopped.wait(5)

    def shutdown(*args):
        stopped.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    threading.Thread(target=register, daemon=True).start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        while not stopped.is_set():
            rclpy.spin_once(runtime.node, timeout_sec=.05)
    finally:
        stopped.set()
        server.shutdown()
        server.server_close()
        runtime.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
