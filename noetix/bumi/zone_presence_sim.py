"""WSL Gazebo vision -> VOP-compatible ROS -> production occupancy Card preview."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
import urllib.request


PAGE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>Bumi 区域占用仿真</title><style>body{max-width:1000px;margin:28px auto;background:#111820;color:#e7edf5;font:17px system-ui}img{width:640px;max-width:100%}button{padding:12px;margin:6px}pre{white-space:pre-wrap}a{color:#a8e0d0}</style>
<h1>Bumi 服务区域占用检测</h1><p>Gazebo RGB-D → YOLO → VOP 格式桥接 → ROS → zone_presence</p>
<img id="frame" src="/frame.jpg" alt="相机画面及检测区域"><p id="state">连接中</p>
<button onclick="scene('outside')">人员在区域外</button><button onclick="scene('single')">单人进入</button><button onclick="scene('pair')">双人进入</button>
<button onclick="send('/pause',{})">暂停输入</button><button onclick="send('/resume',{})">恢复输入</button>
<p>绿框为服务区域，按人物检测框中心计数。仿真角色固定站立，场景按钮切换其位置。</p><pre id="events"></pre>
<a href="http://127.0.0.1:15713/">Gazebo 场景</a>
<script>
async function send(url,data){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const d=await r.json();if(d.error)alert(d.error);}
function scene(name){send('/scene',{name});}
async function poll(){try{const d=await(await fetch('/status')).json();document.querySelector('#state').textContent=JSON.stringify(d.state,null,2);document.querySelector('#events').textContent=JSON.stringify(d.events.slice(-12),null,2);const im=document.querySelector('#frame');if(im.complete)im.src='/frame.jpg?t='+Date.now();}catch(e){document.querySelector('#state').textContent=e.message;}setTimeout(poll,300);}poll();
</script></html>'''


def request(path, data=None):
    body = None if data is None else json.dumps(data).encode()
    req = urllib.request.Request('http://127.0.0.1:15713'+path, data=body,
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=5) as response:
        return response.read()


def main():
    import io
    from PIL import Image, ImageDraw
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String
    from zone_presence_card import ZonePresencePlugin
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=2)
    node = Node('zone_gazebo_bridge')
    executor.add_node(node)
    card = ZonePresencePlugin({}, 'zone_sim', executor)
    card.dispatch('start', {'input_topic': '/zone_sim/vop'})
    publisher = node.create_publisher(String, '/zone_sim/vop', 10)
    events = []
    lock = threading.RLock()
    paused = threading.Event()
    closed = threading.Event()
    latest = {'stamp': None, 'error': '', 'frames_forwarded': 0}

    def receive(message):
        with lock:
            events.append({**json.loads(message.data), 'wall_time_s': time.time()})
            del events[:-200]

    node.create_subscription(String, '/zone_sim/zone_presence/events', receive, 10)

    def bridge():
        while not closed.wait(.08):
            if paused.is_set():
                continue
            try:
                rpc = {'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
                       'params': {'name': 'bumi_gazebo', 'arguments': {'action': 'read'}}}
                response = json.loads(request('/mcp', rpc))
                observation = json.loads(response['result']['content'][0]['text'])
                stamp = observation.get('source_stamp_ns')
                if not observation.get('fresh') or stamp == latest['stamp']:
                    continue
                objects = []
                for person in observation['people']:
                    x1, y1, x2, y2 = person['box_xyxy_px']
                    objects.append({'name': 'person', 'confidence': person['confidence'],
                                    'position': [(x1+x2)/640-1, (y1+y2)/480-1]})
                message = String()
                message.data = json.dumps({'objects': objects})
                publisher.publish(message)
                with lock:
                    latest.update(stamp=stamp, error='', frames_forwarded=latest['frames_forwarded']+1)
            except Exception as exc:
                with lock:
                    latest['error'] = str(exc)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, data, content_type='application/json'):
            if isinstance(data, dict):
                data = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/':
                self.send(PAGE.encode(), 'text/html; charset=utf-8')
            elif self.path == '/status':
                with lock:
                    self.send({'state': card.dispatch('read', {}), 'events': list(events),
                               'bridge': dict(latest), 'paused': paused.is_set()})
            elif self.path.startswith('/frame.jpg'):
                try:
                    picture = Image.open(io.BytesIO(request('/frame.jpg'))).convert('RGB')
                    draw = ImageDraw.Draw(picture)
                    left, top, right, bottom = card.engine.region
                    draw.rectangle(((left+1)*320, (top+1)*240, (right+1)*320, (bottom+1)*240), outline='lime', width=3)
                    output = io.BytesIO()
                    picture.save(output, format='JPEG')
                    self.send(output.getvalue(), 'image/jpeg')
                except Exception:
                    self.send_error(503)
            else:
                self.send_error(404)

        def do_POST(self):
            if self.headers.get('Origin') not in (None, 'http://127.0.0.1:15715'):
                self.send_error(403)
                return
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 1000:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(size))
                if self.path == '/pause':
                    paused.set()
                elif self.path == '/resume':
                    paused.clear()
                elif self.path == '/scene':
                    positions = {'outside': [[3, 2.5]], 'single': [[3, 0]],
                                 'pair': [[3, .65], [3, -.65]]}[data['name']]
                    result = json.loads(request('/scene', {'positions': positions}))
                    if result.get('error'):
                        raise ValueError(str(result['error']))
                else:
                    raise ValueError('Unknown operation')
                self.send({'ok': True})
            except Exception as exc:
                self.send({'error': str(exc)})

    threads = [threading.Thread(target=executor.spin, daemon=True),
               threading.Thread(target=bridge, daemon=True)]
    for thread in threads:
        thread.start()
    server = ThreadingHTTPServer(('127.0.0.1', 15715), Handler)
    try:
        server.serve_forever()
    finally:
        closed.set()
        card.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
