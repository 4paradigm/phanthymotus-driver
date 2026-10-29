"""Independent loopback MCP card. Startup and project start never move the robot."""
import json
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from engine import Engine, VERSION, tool


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def send(self, code, value):
        raw = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        self.send(200, self.server.engine.info()) if self.path == '/health' else self.send(404, {'error': 'Not found'})

    def do_POST(self):
        if self.path != '/mcp':
            self.send(404, {'error': 'Not found'})
            return
        rid = None
        try:
            if self.headers.get('Transfer-Encoding'):
                raise ValueError('不接受分块请求')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 8192:
                raise ValueError('请求大小无效')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict) or body.get('jsonrpc') != '2.0':
                raise ValueError('无效JSON-RPC')
            rid, method, params = body.get('id'), body.get('method'), body.get('params', {})
            if not isinstance(params, dict):
                raise ValueError('无效参数')
            if rid is None:
                if method != 'notifications/initialized':
                    raise ValueError('不接受无ID的控制请求')
                self.send_response(204)
                self.end_headers()
                return
            if method == 'initialize':
                result = {'protocolVersion': '2024-11-05', 'capabilities': {'tools': {}},
                          'serverInfo': {'name': 'TRON2 Head Visual Follow', 'version': VERSION}}
            elif method == 'tools/list':
                result = {'tools': [tool()]}
            elif method == 'resources/list':
                result = {'resources': []}
            elif method == 'ping':
                result = {}
            elif method == 'tools/call':
                if params.get('name') != 'head_visual_follow':
                    raise ValueError('未知卡片')
                try:
                    data = self.server.engine.call(params.get('arguments', {}))
                    error = False
                except Exception as exc:
                    data, error = {'error': str(exc)[:300], 'motion_sent_by_this_call': False}, True
                result = {'content': [{'type': 'text', 'text': json.dumps(data, ensure_ascii=False)}],
                          'structuredContent': data, 'isError': error}
            else:
                self.send(200, {'jsonrpc': '2.0', 'id': rid, 'error': {'code': -32601, 'message': 'Method not found'}})
                return
            self.send(200, {'jsonrpc': '2.0', 'id': rid, 'result': result})
        except Exception as exc:
            self.send(400, {'jsonrpc': '2.0', 'id': rid, 'error': {'code': -32602, 'message': str(exc)[:300]}})


def main():
    engine = Engine()
    server = ThreadingHTTPServer(('127.0.0.1', 15793), Handler)
    server.daemon_threads = True
    server.engine = engine
    stopped = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *args: stopped.set())
    engine.launch()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print('Head-follow card ready on loopback15793; movement DISABLED until follow.', flush=True)
    try:
        stopped.wait()
    finally:
        engine.close()
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
