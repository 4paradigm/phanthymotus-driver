"""Isolated ROS/DDS integration test. Synthetic inputs, never a hardware test.

Run only in a Docker --network none container with a prepared loopback profile.
The guard below refuses to publish even state fixtures on a real LAN.
"""
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path("/work")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "deep_robotics/s10_pro"))


def run():
    from s10_network_isolation import require_loopback_network
    require_loopback_network()
    from bridge import ReaderBridge, reader_environment
    from device import build_plugins
    from main import PlatformBus, validate_platform_profile
    from std_msgs.msg import String, UInt8MultiArray
    from pointcloud import convert_pointcloud
    import yaml

    validate_platform_profile(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"])
    config = {"reader": {"domain_id": 0}, "register": False, "plugins": {
        name: {"refresh_hz": 5, "stale_after_s": 0.6} for name in ("imu", "lidar")}}
    tmp = tempfile.TemporaryDirectory(prefix="s10-isolated-test-")
    path = Path(tmp.name) / "config.yaml"
    path.write_text(yaml.safe_dump(config))
    vendor_code = '''import struct,time,rclpy
from sensor_msgs.msg import Imu,PointCloud2,PointField
from rclpy.signals import SignalHandlerOptions
rclpy.init(domain_id=0,signal_handler_options=SignalHandlerOptions.NO)
n=rclpy.create_node('synthetic_state_source',enable_rosout=False,start_parameter_services=False)
imu=n.create_publisher(Imu,'/IMU',1)
lidar=n.create_publisher(PointCloud2,'/LIDAR/POINTS',1)
while rclpy.ok():
 m=Imu();m.header.frame_id='synthetic_test_only';m.header.stamp=n.get_clock().now().to_msg()
 m.orientation.w=1.;m.angular_velocity.z=.125;m.linear_acceleration.z=9.81;imu.publish(m)
 p=PointCloud2();p.header=m.header;p.height=1;p.width=3;p.point_step=16;p.row_step=48
 p.fields=[PointField(name=s,offset=i*4,datatype=7,count=1) for i,s in enumerate(['x','y','z','intensity'])]
 p.data=struct.pack('<12f',1,2,3,4,2,3,4,5,3,4,5,6);p.is_dense=True;lidar.publish(p)
 rclpy.spin_once(n,timeout_sec=.05)
'''
    vendor = subprocess.Popen([sys.executable, "-u", "-c", vendor_code],
                               env=reader_environment(config), stdout=subprocess.DEVNULL)
    bus = PlatformBus(42)
    bridge = ReaderBridge(config, path)
    plugins = {p.name: p for p in build_plugins(config, "isolated", bridge, bus.publish)}
    messages = {"imu": [], "lidar": [], "lidar_info": []}
    lock = threading.Lock()

    def callback(key):
        def receive(msg):
            with lock:
                messages[key].append(bytes(msg.data) if key == "lidar" else json.loads(msg.data))
        return receive

    subscriptions = [bus.node.create_subscription(cls, f"/isolated/s10_pro/{key}", callback(key), 10)
                     for key, cls in (("imu", String), ("lidar", UInt8MultiArray), ("lidar_info", String))]

    def wait_until(predicate, seconds=12):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            with lock:
                if predicate():
                    return
            time.sleep(0.05)
        raise AssertionError("isolated ROS data condition timed out")

    try:
        plugins["imu"].start()
        plugins["lidar"].start()
        wait_until(lambda: messages["imu"] and messages["imu"][-1]["fresh"] and messages["lidar"])
        with lock:
            assert messages["imu"][-1]["data"]["angular_velocity"]["z"] == .125
            binary = messages["lidar"][-1]
            assert struct.unpack_from("<II", binary) == (16, 3)
            assert struct.unpack_from("<ffff", binary, 8) == (1, 2, 3, 4)
        vendor.send_signal(signal.SIGSTOP)
        wait_until(lambda: messages["imu"][-1]["stale"], seconds=5)
        vendor.send_signal(signal.SIGCONT)
        wait_until(lambda: messages["imu"][-1]["fresh"], seconds=5)
        plugins["imu"].stop()
        assert bridge.status()["active_sources"] == ["lidar"]
        wait_until(lambda: messages["lidar_info"] and messages["lidar_info"][-1]["fresh"])
        plugins["lidar"].stop()
        wait_until(lambda: not bridge.status()["reader_running"], seconds=5)
        time.sleep(0.3)  # Drain any previously queued platform callback.
        with lock:
            before = {key: len(value) for key, value in messages.items()}
        time.sleep(0.4)
        with lock:
            assert before == {key: len(value) for key, value in messages.items()}
        # A source-size parsing check, still explicitly synthetic and isolated.
        source_count = 52642
        large_cloud = {"header": {"frame_id": "synthetic_test_only"}, "width": source_count,
                       "height": 1, "point_step": 26, "row_step": source_count * 26,
                       "is_bigendian": False,
                       "fields": [{"name": name, "offset": index * 4, "datatype": 7, "count": 1}
                                  for index, name in enumerate(("x", "y", "z", "intensity"))],
                       "data": struct.pack("<ffffHd", 1, 2, 3, 4, 0, 0) * source_count}
        begin = time.monotonic()
        large_binary, large_meta = convert_pointcloud(large_cloud, 6000)
        parsing_ms = round((time.monotonic() - begin) * 1000, 2)
        assert large_meta["output_count"] == 6000 and len(large_binary) == 96008
        print(json.dumps({"synthetic_isolated_test": True, "hardware_test": False,
                          "architecture": os.uname().machine, "ros_distro": os.environ.get("ROS_DISTRO"),
                          "vendor_reader_process": "passed", "platform_dds_json": "passed",
                          "platform_pointcloud_binary": "passed", "stale_and_recovery": "passed",
                          "independent_stop": "passed", "reader_exit": "passed",
                          "received_frames": before,
                          "synthetic_source_points": source_count, "display_points": 6000,
                          "pointcloud_conversion_ms": parsing_ms}, sort_keys=True))
    finally:
        vendor.send_signal(signal.SIGCONT)
        vendor.terminate()
        try:
            vendor.wait(timeout=3)
        except subprocess.TimeoutExpired:
            vendor.kill(); vendor.wait(timeout=2)
        for plugin in plugins.values():
            plugin.stop()
        bridge.close()
        for subscription in subscriptions:
            bus.node.destroy_subscription(subscription)
        bus.close()
        tmp.cleanup()


if __name__ == "__main__":
    run()
