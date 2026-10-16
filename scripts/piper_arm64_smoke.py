"""Native ARM64 dependency/DDS smoke test; fake CAN, no Core registration.

Run in a --network=none container without host devices, with tests on
PYTHONPATH. This script is development tooling, not a production entrypoint.
"""
import json
import math
import os
import platform
import tempfile
import time


def main():
    assert platform.machine() in ('aarch64', 'arm64'), 'Native ARM64 execution required'
    import cv2
    import numpy as np
    import piper_sdk
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from common.vendor_runtime import DualDomainROS2, DriverBundle
    from agilex.piper.device import build_plugins
    from test_piper_intent import IntentDriver

    os.environ['PIPER_MOTION_ENABLED'] = '0'
    os.environ['PIPER_CAMERA_ENABLED'] = '0'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    ros = DualDomainROS2(210, 211)
    frames = {'skeleton': [], 'parameters': []}
    received = Node('piper_arm64_development_receiver', context=ros.ctx_core)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    subscriptions = []
    for kind, topic in (('skeleton', '/piper_arm64_dev/piper/state/joints'),
                        ('parameters', '/piper_arm64_dev/piper/state/parameters')):
        subscriptions.append(received.create_subscription(String, topic,
            lambda msg, key=kind: frames[key].append(json.loads(msg.data)), qos))
    ros.executor_core.add_node(received)
    with tempfile.TemporaryDirectory() as directory:
        plugins = build_plugins({'data_dir': directory, 'can_interface': 'fake_can',
                                 'async_actions': False}, 'piper_arm64_dev', ros)
        runtime, background = plugins[:2]
        runtime.factory = IntentDriver
        sdk = runtime.connected().arm
        bundle = DriverBundle(plugins)
        try:
            bundle.start_all()
            ros.start_spin()
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                if min(map(len, frames.values())) >= 8:break
                time.sleep(.05)
            assert min(map(len, frames.values())) >= 8, {k:len(v) for k,v in frames.items()}
            skeleton = frames['skeleton'][-1]
            parameters = frames['parameters'][-1]
            assert skeleton['position_unit'] == 'rad'
            assert [j['name'] for j in skeleton['joints']] == ['joint'+str(i) for i in range(1,7)]
            expected = runtime.read_monitor_snapshot()['joints_deg']
            assert all(abs(j['q']-math.radians(q)) < 1e-9 for j,q in zip(skeleton['joints'], expected))
            assert parameters['Feedback'] == 'fresh' and parameters['Task']['state'] == 'idle'
            assert sdk.calls == [], sdk.calls
            assert set(t['name'] for t in bundle.get_all_tools()) == {'arm_control','model','piper_photo','piper_camera_live'}
            assert '<robot' in bundle.dispatch('model', {})['urdf']
            image = np.zeros((32, 48, 3), dtype=np.uint8)
            encoded, jpeg = cv2.imencode('.jpg', image)
            assert encoded and cv2.imdecode(jpeg, cv2.IMREAD_COLOR).shape == image.shape
            print(json.dumps({'architecture':platform.machine(), 'piper_sdk_import':True,
                'opencv':cv2.__version__, 'numpy':np.__version__, 'jpeg_roundtrip':True,
                'dds_domains':[210,211], 'namespace':'piper_arm64_dev',
                'frames':{k:len(v) for k,v in frames.items()}, 'six_joint_radians':True,
                'hardware_control_commands':len(sdk.calls), 'hardware_used':False,
                'core_registration':False, 'passed':True}), flush=True)
        finally:
            bundle.stop_all()
            ros.executor_core.remove_node(received)
            received.destroy_node()
            ros.shutdown()


if __name__ == '__main__':main()
