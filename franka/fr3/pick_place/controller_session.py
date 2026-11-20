"""Own one isolated official MoveIt instance for a visual card action."""
from contextlib import contextmanager
import ipaddress
import random
import subprocess
import tempfile
import threading

from . import controller_plan as rpc
from .controller_ssh import command_for, planning_worker


_LAUNCH = """
import os
import signal
import socket
import sys
import threading
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

address, domain, fake = sys.argv[1:]
# A socket owned by this process serializes launches for this arm without
# creating a persistent service or modifying the installed workspace.
lock = socket.socket()
lock.bind(("127.0.0.1", 41000 + int(address.split(".")[-1])))
os.environ["ROS_DOMAIN_ID"] = domain
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ.pop("ROS_LOCALHOST_ONLY", None)
os.environ["ROS_LOG_DIR"] = "/tmp"
path = get_package_share_directory("franka_fr3_moveit_config") + "/launch/moveit.launch.py"
args = ["robot_ip:=" + ("127.0.0.1" if fake == "true" else address),
        "use_fake_hardware:=" + fake]
# Use the installed launch graph, omitting only RViz and the separate gripper
# launch. The hand model remains loaded and Orin retains the hand connection.
source = PythonLaunchDescriptionSource(path)
service = LaunchService(argv=args)
description = source.get_launch_description(service.context)
entities = [entity for entity in description.entities
            if not isinstance(entity, IncludeLaunchDescription)
            and (not isinstance(entity, ExecuteProcess) or isinstance(entity, Node))
            and not (isinstance(entity, Node) and entity.node_executable == "rviz2")]
settings = [SetLaunchConfiguration(*arg.split(":=", 1)) for arg in args]
# Use the controller interface declared by the official MoveIt configuration.
spawn = ExecuteProcess(cmd=["ros2", "run", "controller_manager", "spawner",
    "joint_state_broadcaster", "fr3_arm_controller", "--controller-manager-timeout", "20"])
service.include_launch_description(LaunchDescription(settings + entities + [spawn]))

def disconnected():
    sys.stdin.buffer.read()
    os.kill(os.getpid(), signal.SIGINT)

threading.Thread(target=disconnected, daemon=True).start()
sys.exit(service.run())


"""


class ControllerSession:
    def __init__(self, endpoint, username, password, cancel, *, local):
        self.connection = endpoint, username, password
        self.cancel = cancel
        self.local = local
        self.domain = random.randrange(130, 190)

    def call(self, source, payload, timeout):
        self.check()
        value = self._planning_call(source, payload, timeout)
        self.check()
        return value

    def check(self):
        if self.cancel.is_set():
            raise RuntimeError('Pick cancelled')
        if self.process.poll() is not None:
            raise RuntimeError('Controller MoveIt launch exited')

    @contextmanager
    def start(self, address):
        import shlex
        address = str(ipaddress.IPv4Address(address))
        endpoint, username, password = self.connection
        command = command_for(endpoint, username)
        command[-1] = command[-1].removesuffix('python3 -') + shlex.join([
            'python3', '-u', '-c', _LAUNCH, address, str(self.domain), "true",
        ])
        with tempfile.TemporaryFile() as secret:
            secret.write(password.encode() + b'\n')
            secret.seek(0)
            self.process = subprocess.Popen(
                ['sshpass', '-d', str(secret.fileno()), *command], stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, pass_fds=(secret.fileno(),),
            )
            finished = threading.Event()

            def watch_cancel():
                while not finished.wait(.1):
                    if self.cancel.is_set():
                        self.process.stdin.close()
                        return

            monitor = threading.Thread(target=watch_cancel, daemon=True)
            monitor.start()
            try:
                with planning_worker(*self.connection, self.domain, self.cancel) as self._planning_call:
                    self.call(_READY, {}, 35)
                    yield self
            finally:
                finished.set()
                monitor.join()
                self.process.stdin.close()
                try:
                    self.process.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    self.process.wait(timeout=5)

    def plan_joint(self, joints, start=None):
        payload = dict(action='/move_action', joints=list(joints), joint_bounds=rpc.JOINT_PATH_BOUNDS)
        if start is not None:
            payload['start'] = start
        return self.call(rpc._PLAN, payload, 40)

    def snapshot(self):
        return self.local.snapshot()

    def motion(self, command):
        return self.local.motion(command)

    def wait(self, command, timeout):
        return self.local.wait(command, timeout)

    def stop_motion(self, **kwargs):
        return self.local.stop_motion(**kwargs)

    def flange_at(self, joints):
        return self.call(rpc._FK, dict(service='/compute_fk', base_frame='fr3_link0',
                                      link='fr3_link8', joints=list(joints)), 15)['pose']

    def solve_ik(self, pose, start):
        result = self.call(rpc._IK, dict(pose=pose, start=start), 12)
        if result['code'] != 1:
            raise RuntimeError(f'MoveIt approach IK failed (code={result["code"]})')
        joints = [result['joints'][f'fr3_joint{i}'] for i in range(1, 8)]
        if any(not lo <= value <= hi for value, (lo, hi) in zip(joints, rpc.JOINT_PATH_BOUNDS)):
            raise ValueError('MoveIt approach IK solution violates joint margin')
        return joints

    def plan_cartesian(self, poses, speed_scale, start):
        return self.call(rpc._CARTESIAN, dict(service='/compute_cartesian_path', frame='fr3_link0',
                                             link='fr3_link8', poses=poses,
                                             speed_scale=speed_scale, start=start, joint_bounds=rpc.JOINT_PATH_BOUNDS), 22)

_READY = """
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
rclpy.init()
node = rclpy.create_node("fr3_pick_ready")
try:
    for action, name in ((MoveGroup, "/move_action"),
                         (ExecuteTrajectory, "/execute_trajectory"),
                         (FollowJointTrajectory, "/fr3_arm_controller/follow_joint_trajectory")):
        client = ActionClient(node, action, name)
        if not client.wait_for_server(timeout_sec=9):
            raise RuntimeError("MoveIt controller is not ready: " + name)
    print('FR3_RESULT {"ready": true}', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
"""
