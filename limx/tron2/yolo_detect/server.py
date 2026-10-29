"""Loopback MCP and ROS image I/O only. No vendor/robot API imports."""
import json,os,re,signal,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from engine import Engine,ModelClient,tool,VERSION,IMAGE_TOPIC,RESULT_TOPIC,INPUTS,MAX_JPEG

class RosBus:
    def __init__(self):
        domain=os.environ.get('ROS_DOMAIN_ID','');expected=os.environ.get('YOLO_EXPECTED_DOMAIN','42')
        if domain!=expected or not re.fullmatch(r'0|[1-9][0-9]{0,2}',domain) or int(domain)>232:raise ValueError('DDS domain differs from reviewed Core')
        profile=os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE','')
        if not profile or not Path(profile).is_file() or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':raise ValueError('Missing existing Core FastDDS configuration')
        os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS',None)
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
        self.rclpy=rclpy;self.Image=CompressedImage;self.String=String;self.lock=threading.RLock();self.sub=None
        rclpy.init();self.node=rclpy.create_node('tron2_yolo_card')
        self.qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
        self.image=self.node.create_publisher(CompressedImage,IMAGE_TOPIC,self.qos)
        self.results=self.node.create_publisher(String,RESULT_TOPIC,self.qos)
        self._closing=False
        def spin():
            try:rclpy.spin(self.node)
            except Exception:
                if not self._closing:raise
        self.thread=threading.Thread(target=spin,daemon=True,name='yolo_ros');self.thread.start()
    def bind(self,topic,callback):
        if topic not in INPUTS:raise ValueError('Unsupported camera input')
        def receive(msg):
            if str(msg.format).lower().split(';')[0].strip() not in ('jpeg','jpg','image/jpeg'):return
            if len(msg.data)>MAX_JPEG:return
            stamp={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec,'frame_id':msg.header.frame_id}
            callback(bytes(msg.data),stamp)
        with self.lock:
            self.unbind();self.sub=self.node.create_subscription(self.Image,topic,receive,self.qos)
    def unbind(self):
        with self.lock:
            if self.sub is not None:self.node.destroy_subscription(self.sub);self.sub=None
    def publish(self,image,data,stamp):
        with self.lock:
            msg=self.Image();msg.format='jpeg';msg.data=image
            if stamp:
                msg.header.stamp.sec=stamp['sec'];msg.header.stamp.nanosec=stamp['nanosec'];msg.header.frame_id=stamp['frame_id']
            else:msg.header.stamp=self.node.get_clock().now().to_msg();msg.header.frame_id='yolo_status'
            result=self.String();result.data=json.dumps(data,ensure_ascii=False,allow_nan=False)
            self.image.publish(msg);self.results.publish(result)
    def close(self):
        self._closing=True;self.unbind();self.rclpy.shutdown();self.thread.join(timeout=3);self.node.destroy_node()

class Handler(BaseHTTPRequestHandler):
    protocol_version='HTTP/1.1'
    def log_message(self,*args):pass
    def setup(self):super().setup();self.connection.settimeout(8)
    def send(self,status,value,ctype='application/json'):
        data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode() if ctype=='application/json' else value
        self.send_response(status);self.send_header('Content-Type',ctype);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers()
        if data:self.wfile.write(data)
    def do_GET(self):
        if self.path=='/health':self.send(200,dict(self.server.engine.info(),service='tron2-yolo-card'));return
        if self.path=='/latest.jpg':
            with self.server.engine.lock:image=self.server.engine.last_image
            self.send(200,image,'image/jpeg');return
        self.send(404,{'error':'Not found'})
    def do_POST(self):
        if self.path!='/mcp':self.send(404,{'error':'Not found'});return
        rid=None
        try:
            if self.headers.get('Transfer-Encoding'):raise ValueError('Chunked MCP requests not accepted')
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=16384:raise ValueError('MCP body size invalid')
            body=json.loads(self.rfile.read(length))
            if not isinstance(body,dict) or body.get('jsonrpc')!='2.0':raise ValueError('Invalid JSON-RPC request')
            rid=body.get('id');method=body.get('method');params=body.get('params',{})
            if rid is None:
                if method!='notifications/initialized':raise ValueError('Tool notifications not accepted')
                self.send(204,b'','application/octet-stream');return
            if not isinstance(params,dict):raise ValueError('Invalid parameters')
            if method=='initialize':result={'protocolVersion':'2024-11-05','capabilities':{'tools':{}},'serverInfo':{'name':'TRON2 YOLO-World Vision','version':VERSION}}
            elif method=='tools/list':result={'tools':[tool()]}
            elif method=='resources/list':result={'resources':[]}
            elif method=='ping':result={}
            elif method=='tools/call':
                if params.get('name')!='yolo_detect':raise ValueError('Unknown tool')
                try:data=self.server.engine.call(params.get('arguments',{}));is_error=False
                except Exception as exc:data={'error':str(exc)[:300],'robot_motion_enabled':False};is_error=True
                result={'content':[{'type':'text','text':json.dumps(data,ensure_ascii=False,allow_nan=False)}],'isError':is_error}
            else:
                self.send(200,{'jsonrpc':'2.0','id':rid,'error':{'code':-32601,'message':'Method not found'}});return
            self.send(200,{'jsonrpc':'2.0','id':rid,'result':result})
        except Exception as exc:self.send(400,{'jsonrpc':'2.0','id':rid,'error':{'code':-32602,'message':str(exc)[:300]}})


def main():
    bus=RosBus();engine=Engine(bus);engine.launch()
    server=ThreadingHTTPServer(('127.0.0.1',15792),Handler);server.daemon_threads=True;server.engine=engine
    stopped=threading.Event()
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *args:stopped.set())
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    print('YOLO card ready on loopback15792; model inference remains on GPU server.',flush=True)
    try:stopped.wait()
    finally:engine.close();server.shutdown();server.server_close();thread.join(timeout=2);bus.close()

if __name__=='__main__':main()
