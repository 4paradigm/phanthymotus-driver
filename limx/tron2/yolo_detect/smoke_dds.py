"""Actual ROS image/JSON loopback proof; run ONLY in network-none test container."""
import json,threading,time
from pathlib import Path
from engine import IMAGE_TOPIC,RESULT_TOPIC,slate
from server import RosBus
bus=RosBus();received={}
try:
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    si=bus.node.create_subscription(CompressedImage,IMAGE_TOPIC,lambda m:received.update(image=bytes(m.data)),bus.qos)
    sj=bus.node.create_subscription(String,RESULT_TOPIC,lambda m:received.update(result=json.loads(m.data)),bus.qos)
    raw=slate('ISOLATED DDS TEST');deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        bus.publish(raw,{'isolated_test':True},None);time.sleep(.2)
        if received.get('image')==raw and received.get('result')=={'isolated_test':True}:break
    assert received.get('image')==raw and received.get('result')=={'isolated_test':True},'No real DDS image/JSON loopback'
    print('ISOLATED_DDS_IMAGE_JSON_PASS',flush=True)
finally:bus.close()
