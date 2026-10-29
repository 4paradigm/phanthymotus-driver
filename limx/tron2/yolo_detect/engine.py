"""Camera-to-existing-YOLOWorld adapter. No robot control transport."""
import hashlib,io,json,math,threading,time,urllib.request,urllib.error
from collections import deque
from dataclasses import dataclass
from PIL import Image,ImageDraw,ImageFont

VERSION='0.1.2'
MODEL='yolov8l-worldv2.pt'
INPUTS=('/limx_tron2/camera','/limx_tron2/camera_left','/limx_tron2/camera_right')
IMAGE_TOPIC='/limx_tron2/yolo/annotated'
RESULT_TOPIC='/limx_tron2/yolo/detections'
MAX_JPEG=4*1024*1024
MAX_PIXELS=4*1024*1024
MAX_AGE=3.


def require(condition,message):
    if not condition:raise ValueError(message)

def number(x,lo,hi):
    return type(x) in (int,float) and math.isfinite(x) and lo<=x<=hi

@dataclass(frozen=True)
class Settings:
    input_topic:str=INPUTS[0]
    def validate(self):
        require(self.input_topic in INPUTS,'请选择已接入的 TRON2 相机')
        return self
    @property
    def fps(self):return 2.0  # Internal workload limit; no user-facing tuning.


def tool():
    return {'name':'yolo_detect','type':'processor','multiInstance':False,
        'description':'连接摄像头后持续识别画面中的物体，直接查看识别结果。',
        'topic_in':[{'format':'image/jpeg','desc':'摄像头图像'}],
        'topic_out':[{'topic':IMAGE_TOPIC,'format':'image/jpeg'},{'topic':RESULT_TOPIC,'format':'data/json'}],
        'inputSchema':{'type':'object','additionalProperties':False,'properties':{
            'action':{'type':'string','enum':['start','stop','info'],'title':'操作','default':'info'},
            'input_topic':{'type':'string','enum':list(INPUTS),'default':INPUTS[0],'title':'摄像头输入'},
            },
            'required':['action'],'x-action-params':{
                'start':{'params':[],'description':'开始检测'},
                'stop':{'params':[],'description':'停止视觉检测'},
                'info':{'params':[],'description':'查看检测状态、帧率和最近结果'}}}}


def decode_jpeg(raw):
    require(isinstance(raw,bytes) and 4<=len(raw)<=MAX_JPEG and raw[:2]==b'\xff\xd8' and raw[-2:]==b'\xff\xd9','输入不是受支持的JPEG')
    im=Image.open(io.BytesIO(raw))
    require(im.format=='JPEG' and im.width<=4096 and im.height<=4096 and im.width*im.height<=MAX_PIXELS,'图像尺寸超限')
    im.load();return im.convert('RGB')


def validate_detections(data,width,height,settings):
    require(isinstance(data,dict) and data.get('image_width')==width and data.get('image_height')==height,'GPU结果与本次图像尺寸不一致')
    require(data.get('box_coordinate_system')=='original_image_pixels','GPU检测框坐标格式不兼容')
    values=data.get('detections');require(isinstance(values,list) and len(values)<=1000,'GPU检测结果数量无效')
    out=[]
    for item in values:
        require(isinstance(item,dict),'GPU检测项格式无效')
        label=item.get('label');conf=item.get('confidence');box=item.get('box_xyxy')
        require(isinstance(label,str) and 0<len(label)<=160 and all(ord(x)>=32 for x in label),'GPU类别名称无效')
        require(number(conf,0,1) and isinstance(box,list) and len(box)==4 and all(number(x,0,max(width,height)) for x in box),'GPU检测框数值无效')
        x1,y1,x2,y2=box
        require(0<=x1<x2<=width and 0<=y1<y2<=height,'GPU检测框越界')
        out.append({'label':label,'confidence':float(conf),'box_xyxy':[float(x) for x in box]})
    return out


def annotate(image,detections,latency):
    draw=ImageDraw.Draw(image)
    for d in detections:
        box=d['box_xyxy'];draw.rectangle(box,outline=(40,255,100),width=3)
        text=(d['label'][:60]+' %.2f'%d['confidence']).encode('ascii','replace').decode()
        x,y=box[:2];y=max(18,y)-18
        draw.rectangle((x,y,min(image.width,x+len(text)*7+6),y+17),fill=(0,35,10))
        draw.text((x+2,y+1),text,fill=(255,255,255))
    text='YOLOWorld | %d detections | %.0f ms'%(len(detections),latency*1000)
    draw.rectangle((0,0,min(image.width,len(text)*7+8),18),fill=(0,0,0));draw.text((4,2),text,fill='white')
    out=io.BytesIO();image.save(out,format='JPEG',quality=85);return out.getvalue()


def slate(message):
    im=Image.new('RGB',(640,480),(35,35,35));d=ImageDraw.Draw(im)
    d.text((24,190),'YOLO DETECTION',fill='white')
    d.text((24,220),message[:90].encode('ascii','replace').decode(),fill=(255,200,90))
    out=io.BytesIO();im.save(out,format='JPEG',quality=80);return out.getvalue()

class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self,*a,**kw):raise ValueError('GPU服务重定向被拒绝')

class ModelClient:
    def __init__(self,base='http://172.28.4.23:18080'):
        require(base=='http://172.28.4.23:18080','本版本只接入已部署的GPU推理端点')
        self.base=base;self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect())
    def request(self,path,raw=None):
        req=urllib.request.Request(self.base+path,data=raw,headers={'Content-Type':'application/octet-stream' if raw is not None else 'application/json'})
        with self.opener.open(req,timeout=6) as r:data=r.read(512*1024+1)
        require(len(data)<=512*1024,'GPU响应超限');return json.loads(data)
    def health(self):
        data=self.request('/health')
        require(data.get('status')=='ok' and data.get('model_loaded') is True and data.get('model')==MODEL,'GPU未加载预期的YOLO-World模型')
        return data
    def detect(self,raw):return self.request('/detect',raw)

class Engine:
    def __init__(self,bus,model=None,clock=time.monotonic,wall=time.time):
        self.bus=bus;self.model=model or ModelClient();self.clock=clock;self.wall=wall
        self.lock=threading.RLock();self.lifecycle=threading.RLock();self.wake=threading.Event()
        self.closed=False;self.active=False;self.generation=0;self.settings=Settings();self.pending=None
        self.sequence=0;self.received=0;self.overwritten=0;self.inferred=0;self.published=0
        self.last_input=None;self.last_publish=None;self.last_status=0.;self.last_error=None;self.last_model_check=None
        self.last_result=None;self.last_image=slate('IDLE');self.times=deque(maxlen=20);self.thread=None
    def launch(self):
        if self.thread is None:
            self.thread=threading.Thread(target=self.run,daemon=True,name='yolo_latest_frame');self.thread.start()
    def info(self):
        with self.lock:
            now=self.clock();age=None if self.last_publish is None else max(0,now-self.last_publish)
            alive=self.thread.is_alive() if self.thread is not None else None
            error=('YOLO处理线程已退出，请检查日志' if self.active and alive is False else self.last_error)
            available=self.active and age is not None and age<MAX_AGE and error is None
            hz=(len(self.times)-1)/(self.times[-1]-self.times[0]) if len(self.times)>1 and self.times[-1]>self.times[0] else None
            return {'version':VERSION,'model':MODEL,'state':'running' if available else 'error' if self.active and error else 'starting' if self.active else 'idle',
                'active':self.active,'available':available,'read_only':True,'robot_motion_enabled':False,
                'input_topic':self.settings.input_topic if self.active else None,
                'received_frames':self.received,'overwritten_frames':self.overwritten,'inference_requests':self.inferred,'published_frames':self.published,
                'observed_output_fps':hz,'last_result_age_seconds':age,'last_error':error,'worker_alive':alive,
                'measurement_time_verified':False,'latest_result':self.last_result,
                'topic_in':[{'topic':self.settings.input_topic,'format':'image/jpeg'}] if self.active else [],'topic_out':tool()['topic_out']}
    def call(self,args):
        require(isinstance(args,dict) and not set(args)-{'action','input_topic','input_topics','confidence','fps','labels','instance_id','_trace_id'},'不支持的检测参数')
        action=args.get('action','info')
        if action=='info':return self.info()
        if action=='stop':self.stop();return self.info()
        require(action=='start','仅支持start/stop/info')
        topics=args.get('input_topics')
        if topics is not None:require(isinstance(topics,list) and len(topics)==1 and topics[0]==args.get('input_topic',topics[0]),'每张YOLO卡片只接一个摄像头')
        # Legacy form values are deliberately ignored, not retained as hidden filters.
        settings=Settings(args.get('input_topic',topics[0] if topics else INPUTS[0])).validate()
        self.start(settings);return self.info()
    def start(self,settings):
        settings.validate()
        with self.lifecycle:
            with self.lock:
                require(not self.closed,'视觉服务正在关闭')
                if self.active and self.settings==settings:return
            self.model.health()
            with self.lock:
                self.generation+=1;generation=self.generation;self.active=False;self.pending=None
            self.bus.unbind()
            with self.lock:
                self.settings=settings;self.last_error=None;self.last_publish=None;self.last_result=None;self.last_input=None
                self.times.clear();self.last_model_check=self.clock();self.active=True
            try:
                self.publish_status('STARTING: waiting for camera',force=True)
                self.bus.bind(settings.input_topic,lambda raw,stamp:self.receive(raw,stamp,generation))
            except Exception:
                with self.lock:self.active=False;self.generation+=1
                raise
            self.wake.set()
    def stop(self):
        with self.lifecycle:
            with self.lock:self.active=False;self.generation+=1;self.pending=None;self.last_result=None;self.last_publish=None;self.last_error=None;self.times.clear()
            self.bus.unbind();self.publish_status('IDLE',force=True)
    def receive(self,raw,stamp,generation):
        if not isinstance(raw,bytes) or not 4<=len(raw)<=MAX_JPEG:return
        with self.lock:
            if not self.active or generation!=self.generation:return
            self.sequence+=1;self.received+=1
            if self.pending is not None:self.overwritten+=1
            self.last_input=self.clock();self.pending=(raw,stamp,generation,self.sequence,self.last_input,self.wall(),self.settings)
        self.wake.set()
    def publish_status(self,message,force=False):
        with self.lock:
            now=self.clock()
            if not force and now-self.last_status<1:return
            self.last_status=now;image=slate(message);self.last_image=image
            data={'model':MODEL,'available':False,'state':message,'detections':[],'published_at':self.wall(),'measurement_time_verified':False,'robot_motion_enabled':False}
            self.bus.publish(image,data,None)
    def step(self):
        with self.lock:
            if not self.active:return False
            frame=self.pending;self.pending=None;generation=self.generation
        if frame is None:
            with self.lock:
                if not self.active or generation!=self.generation:return False
                old=self.last_input is None or self.clock()-self.last_input>MAX_AGE
                if old:self.last_error='等待新鲜相机帧';self.last_result=None;self.last_publish=None
                if old:self.publish_status('NO FRESH CAMERA FRAME')
            return False
        raw,stamp,generation,seq,received,wall,settings=frame
        if self.clock()-received>MAX_AGE:
            with self.lock:
                if not self.active or generation!=self.generation:return False
                self.last_error='输入帧已过期';self.last_result=None;self.last_publish=None
                self.publish_status('NO FRESH CAMERA FRAME',force=True)
            return False
        try:
            image=decode_jpeg(raw)
            if self.last_model_check is None or self.clock()-self.last_model_check>30:
                self.model.health();self.last_model_check=self.clock()
            with self.lock:self.inferred+=1
            response=self.model.detect(raw)
            det=validate_detections(response,image.width,image.height,settings)
            latency=self.clock()-received
            if latency>MAX_AGE:raise ValueError('GPU结果已过期，丢弃本帧')
            annotated=annotate(image,det,latency)
            result={'model':MODEL,'available':True,'input_topic':settings.input_topic,'frame_sequence':seq,'source_header_stamp':stamp,
                'source_jpeg_sha256':hashlib.sha256(raw).hexdigest(),'annotated_jpeg_sha256':hashlib.sha256(annotated).hexdigest(),'source_received_at':wall,'published_at':self.wall(),
                'latency_from_receipt_ms':latency*1000,'measurement_time_verified':False,'image_width':image.width,'image_height':image.height,
                'box_coordinate_system':'original_image_pixels','detections':det,'robot_motion_enabled':False}
            with self.lock:
                if not self.active or generation!=self.generation:return False
                self.bus.publish(annotated,result,stamp);self.last_image=annotated;self.last_result=result
                self.last_publish=self.clock();self.times.append(self.last_publish);self.published+=1;self.last_error=None
            return True
        except Exception as exc:
            with self.lock:
                if not self.active or generation!=self.generation:return False
                self.last_error=str(exc)[:300];self.last_result=None;self.last_publish=None
                self.publish_status('DETECTION ERROR: '+type(exc).__name__,force=True)
            return False
    def run(self):
        next_due=0.
        while not self.closed:
            self.wake.wait(.25);self.wake.clear()
            with self.lock:active=self.active;interval=1/self.settings.fps
            if not active:continue
            # Sample once: a second clock read can cross next_due and produce
            # a negative sleep, killing the daemon worker while HTTP stays alive.
            remaining=next_due-self.clock()
            if remaining>0:time.sleep(min(.1,remaining));self.wake.set();continue
            started=self.clock()
            try:self.step()
            except Exception as exc:
                with self.lock:self.last_error='视觉处理失败: '+str(exc)[:250];self.last_result=None;self.last_publish=None
                time.sleep(.5)
            next_due=started+interval
    def close(self):
        self.stop();self.closed=True;self.wake.set()
        if self.thread:self.thread.join(timeout=8)
