"""Read-only domain-0 scheduler comparison; source the installed ROS environments first.

Subscribes to five existing status streams for five seconds per mode. No hardware
commands or control rights. Callback-age metrics are not hardware timestamp proof.
"""
import time,threading,json
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor,SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from bodyctrl_msgs.msg import MotorStatusMsg,PowerBoardKeyStatus
for kind in ('single','bounded'):
 c=Context();rclpy.init(context=c,domain_id=0);n=Node('teleop_readonly_scheduler_probe',context=c)
 ex=MultiThreadedExecutor(num_threads=8,context=c) if kind=='multi' else SingleThreadedExecutor(context=c)
 ex.add_node(n);counts={};gaps=[];latest={};ages={};closed=threading.Event()
 def callback(msg,key):
  counts[key]=counts.get(key,0)+1;latest[key]=time.monotonic()
 qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
 for key in ('arm','head','waist','leg'):
  n.create_subscription(MotorStatusMsg,'/'+key+'/status',lambda m,k=key:callback(m,k),qos)
 n.create_subscription(PowerBoardKeyStatus,'/power/board/key_status',lambda m:callback(m,'power'),qos)
 def spin():
  while not closed.is_set():
   if kind=='bounded':
    for _ in range(10):ex.spin_once(timeout_sec=0.)
    closed.wait(.01)
   else:ex.spin_once(timeout_sec=.01)
 t=threading.Thread(target=spin);t.start();start=time.monotonic();cpu=time.process_time();last=start
 while time.monotonic()-start<5:
  time.sleep(.02);now=time.monotonic();gaps.append((now-last)*1000);last=now
  if now-start>1:
   for k,v in list(latest.items()):ages.setdefault(k,[]).append((now-v)*1000)
 closed.set();t.join();ex.shutdown();n.destroy_node();c.shutdown()
 gaps.sort();print(json.dumps({'kind':kind,'counts':counts,'cpu_seconds':time.process_time()-cpu,'watchdog_gap_p95_ms':gaps[int(.95*(len(gaps)-1))],'max_ms':max(gaps),'hardware_writes':0,'ages_ms':{k:{'p95':sorted(v)[int(.95*(len(v)-1))],'max':max(v),'over100':sum(x>100 for x in v)} for k,v in ages.items()}}),flush=True)
