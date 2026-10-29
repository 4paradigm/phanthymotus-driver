"""Read-only DDS receiver proving two correlated annotated image/result pairs."""
import hashlib,json,time
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from engine import IMAGE_TOPIC,RESULT_TOPIC
rclpy.init();node=rclpy.create_node('tron2_yolo_verify');images={};results={};seen={}
qos=QoSProfile(depth=3,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
def image(msg):
    raw=bytes(msg.data);images[hashlib.sha256(raw).hexdigest()]=len(raw)
    while len(images)>8:images.pop(next(iter(images)))
def result(msg):
    try:d=json.loads(msg.data)
    except ValueError:return
    if d.get('available') is True and d.get('annotated_jpeg_sha256'):results[d['annotated_jpeg_sha256']]=d
    while len(results)>8:results.pop(next(iter(results)))
a=node.create_subscription(CompressedImage,IMAGE_TOPIC,image,qos);b=node.create_subscription(String,RESULT_TOPIC,result,qos)
try:
    deadline=time.monotonic()+25
    while time.monotonic()<deadline and len(seen)<2:
        rclpy.spin_once(node,timeout_sec=.2)
        for digest in images.keys() & results.keys():
            d=results[digest];seen[d['frame_sequence']]={'frame_sequence':d['frame_sequence'],'input_topic':d['input_topic'],'source_jpeg_sha256':d['source_jpeg_sha256'],'annotated_jpeg_sha256':digest,'jpeg_bytes':images[digest],'detections':len(d['detections']),'latency_from_receipt_ms':d['latency_from_receipt_ms']}
    if len(seen)<2:raise RuntimeError('Two fresh matched DDS image/result frames were not received')
    print(json.dumps({'result':'LIVE_DDS_YOLO_PASS','frames':list(seen.values())[-2:]},ensure_ascii=False))
finally:node.destroy_node();rclpy.shutdown()
