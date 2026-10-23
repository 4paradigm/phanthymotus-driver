"""Read-only DDS feedback under synthetic lidar load; Docker --network none only.

Publishes only MotionInfo and PointCloud2 fixtures. It neither opens the command
worker nor proves real-machine control, stop behavior or latency bounds.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path("/work")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "deep_robotics/s10_pro"))


def main():
    from s10_network_isolation import require_loopback_network
    require_loopback_network()
    from bridge import ReaderBridge, reader_environment
    from main import build_runtime_plugins, validate_platform_profile
    import yaml

    profile = os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]
    validate_platform_profile(profile)
    config = {"register": False, "reader": {"domain_id": 0, "vendor_profile": profile},
              "plugins": {"lidar": {"refresh_hz": 10, "max_points": 40000}}}
    publisher_code = '''import array,struct,sys,time,rclpy
from drdds.msg import MotionInfo
from sensor_msgs.msg import PointCloud2,PointField
from rclpy.signals import SignalHandlerOptions
sys.path.insert(0,'/work/tests')
from s10_network_isolation import require_loopback_network
require_loopback_network()
rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
node=rclpy.create_node('synthetic_readonly_feedback_load',enable_rosout=False,start_parameter_services=False)
motion=node.create_publisher(MotionInfo,'/MOTION_INFO',1)
lidar=node.create_publisher(PointCloud2,'/LIDAR/POINTS',1)
cloud=PointCloud2();cloud.height=1;cloud.width=52642;cloud.point_step=26;cloud.row_step=cloud.width*26;cloud.is_dense=True
cloud.fields=[PointField(name=name,offset=i*4,datatype=7,count=1) for i,name in enumerate(['x','y','z','intensity'])]
cloud.data=array.array('B',struct.pack('<ffffHd',1.,2.,3.,4.,0,0.)*cloud.width)
last_cloud=0.
while rclpy.ok():
 now=time.monotonic()
 info=MotionInfo();info.header.stamp=node.get_clock().now().to_msg();info.data.motion_state.state=0;info.data.gait_state.gait=0x3002
 info.data.vel_x=0.;info.data.vel_y=0.;info.data.vel_yaw=0.;motion.publish(info)
 if now-last_cloud>=.1:
  cloud.header.stamp=node.get_clock().now().to_msg();lidar.publish(cloud);last_cloud=now
 rclpy.spin_once(node,timeout_sec=.05)
'''
    result = {"synthetic_isolated_test": True, "physical_robot": False, "passed": False,
              "synthetic_source_points": 52642, "display_points": 40000}
    with tempfile.TemporaryDirectory(prefix="s10-readonly-feedback-") as temporary:
        config_path = Path(temporary) / "config.yaml"
        config_path.write_text(yaml.safe_dump(config))
        publisher = subprocess.Popen([sys.executable, "-u", "-c", publisher_code],
                                     env=reader_environment(config), stdout=subprocess.DEVNULL)
        sensors = ReaderBridge(config, config_path)
        motion = ReaderBridge(config, config_path)
        plugins = build_runtime_plugins(config, "isolated", sensors, lambda *args: None, motion_bridge=motion)
        cards = {plugin.name: plugin for plugin in plugins}

        def until(predicate, timeout=15):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if predicate():
                    return
                time.sleep(.02)
            raise AssertionError("isolated feedback condition timed out")

        try:
            cards["state"].start()
            cards["lidar"].start()
            cards["motion"].start()
            until(lambda: sensors.snapshot("lidar")["fresh"] and
                  sensors.snapshot("motion_info")["fresh"] and
                  cards["motion"]._backend.feedback.observation()["fresh"])
            assert motion.status()["active_sources"] == ["motion_info"]
            assert sensors.status()["active_sources"] == ["battery", "lidar", "motion_info"]
            assert motion.status()["reader_pid"] != sensors.status()["reader_pid"]
            assert not cards["motion"]._backend.diagnostics()["command_worker_running"]
            motion_sequences, lidar_sequences, ages = set(), set(), []
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                observed = cards["motion"]._backend.feedback.observation()
                assert observed["fresh"] is True, observed
                ages.append(observed["age_s"])
                motion_sequences.add(observed["sequence"])
                lidar_sequences.add(sensors.snapshot("lidar")["sequence"])
                time.sleep(.02)
            assert len(motion_sequences) >= 30 and len(lidar_sequences) >= 10
            assert max(ages) < .5
            result.update(motion_sample_count=len(motion_sequences), lidar_sample_count=len(lidar_sequences),
                          motion_max_age_s=round(max(ages), 6), independent_reader_pids=True,
                          command_worker_running=False, execution_authorized=False)
            cards["motion"].stop()
            until(lambda: not motion.status()["reader_running"])
            assert sensors.status()["reader_running"] and sensors.snapshot("lidar")["fresh"]
            cards["motion"].start()
            until(lambda: cards["motion"]._backend.feedback.observation()["fresh"])
            cards["state"].stop()
            cards["lidar"].stop()
            until(lambda: not sensors.status()["reader_running"])
            assert motion.status()["reader_running"]
            assert cards["motion"]._backend.feedback.observation()["fresh"]
            cards["motion"].stop()
            until(lambda: not motion.status()["reader_running"])
            result.update(independent_stop_and_reader_exit=True, passed=True)
        finally:
            for plugin in plugins:
                plugin.stop()
                if hasattr(plugin, "close"):
                    plugin.close()
            sensors.close()
            motion.close()
            publisher.terminate()
            try:
                publisher.wait(timeout=3)
            except subprocess.TimeoutExpired:
                publisher.kill()
                publisher.wait(timeout=2)
            print(json.dumps(result, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
