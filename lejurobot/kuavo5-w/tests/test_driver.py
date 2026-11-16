import importlib.util
import json
import math
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parents[1]))
spec = importlib.util.spec_from_file_location("kuavo_device", ROOT / "device.py")
driver = importlib.util.module_from_spec(spec)
std = types.ModuleType("std_msgs.msg")
std.String = type("String", (), {})
with patch.dict(sys.modules, {"std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": std}):
    spec.loader.exec_module(driver)
    from kuavo_sensors import LINK_ORDER, model_names
    JOINT_NAMES = model_names((ROOT / 'resource' / 'kuavo5w_model.urdf').read_text())


class FakeRos1:
    def __init__(self):
        self.published, self.services, self.subscriptions, self.advertised = [], [], {}, []
        self.connection, self.motion_ready = (True, 1), True
        self.service_accepts, self.confirm = True, True
        self.params = {'robot_type': 1, 'robot_version': 400062, 'armRealDof': 14,
                       'headRealDof': 2, 'legRealDof': 4, 'waistRealDof': 0,
                       'use_shm_communication': False,
                       'humanoid_description': '<robot name="fixture">' + ''.join(
                           f'<joint name="{JOINT_NAMES[i]}" type="revolute"><child link="{link}"/></joint>'
                           for i, link in enumerate(LINK_ORDER)) + '</robot>'}

    def get_param(self, name, default=None):
        return self.params.get(name.lstrip('/'), default)

    def unsubscribe(self, topic):
        self.subscriptions.pop(topic, None)

    def advertise(self, topic, message_type):
        self.advertised.append((topic, message_type))

    def publish(self, topic, message_type, message):
        if not self.connection[0]:
            raise ConnectionError("disconnected")
        self.published.append((topic, message_type, message))

    def call_service(self, service, service_type, args, timeout):
        self.services.append((service, service_type, args, timeout))
        if not self.connection[0]:
            raise ConnectionError("disconnected")
        if self.service_accepts and self.confirm:
            if service == "/enable_control":
                self.emit("/enable_control_state", args["data"])
            elif service == "/mobile_manipulator_mpc_control":
                self.emit("/mobile_manipulator/lb_mpc_control_mode", args["control_mode"])
        return {"success": self.service_accepts, "result": self.service_accepts, "message": "test service"}

    def subscribe(self, topic, message_type, callback):
        self.subscriptions[topic] = (message_type, callback)

    def emit(self, topic, value):
        self.subscriptions[topic][1]({"data": value})

    def raw(self, q=None, **overrides):
        msg = {'sensor_time': {'secs': 0, 'nsecs': 0},
               'joint_data': {'joint_q': q if q is not None else [0.] * 20,
                              'joint_v': [1.] * 20, 'joint_vd': [2.] * 20, 'joint_torque': [3.] * 20},
               'imu_data': {'quat': dict(x=0., y=0., z=0., w=1.),
                            'gyro': dict(x=1., y=2., z=3.), 'acc': dict(x=0., y=0., z=9.81),
                            'free_acc': dict(x=0., y=0., z=0.)}}
        msg.update(overrides)
        topic = '/sensors_data_raw_shm' if self.params['use_shm_communication'] else '/sensors_data_raw'
        self.subscriptions[topic][1](msg)


class FakePublisher:
    def __init__(self, topic):
        self.topic_name, self.messages = topic, []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def create_publisher(self, _type, topic, _depth):
        self.publisher = FakePublisher(topic)
        return self.publisher

    def create_timer(self, _period, callback):
        return callback


def wait_until(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


class DriverContractTests(unittest.TestCase):
    def setUp(self):
        self.callbacks = []
        self.callback_patch = patch.object(driver, "_acp_complete", side_effect=lambda *a: self.callbacks.append(a))
        self.callback_patch.start()
        self.ros, self.node = FakeRos1(), FakeNode()
        self.bundle = driver.KuavoBundle({}, "test_robot", self.ros, self.node)
        self.bundle.sensors.discover()
        self.ros.raw()
        self.ros.emit("/enable_control_state", True)
        self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 3)

    def tearDown(self):
        self.bundle.close()
        self.callback_patch.stop()

    def call(self, card, action, **args):
        return self.bundle.dispatch(card, {"action": action, **args})

    def test_metadata_and_schema_actions_agree(self):
        import yaml
        tools = {tool["name"]: tool for tool in self.bundle.get_all_tools()}
        metadata = yaml.safe_load((ROOT / "driver.yaml").read_text())
        self.assertEqual(set(tools), {c["name"] for c in metadata["cards"]})
        for name, tool in tools.items():
            if tool['type'] != 'actuator':
                continue
            schema = tool["inputSchema"]
            self.assertEqual(set(schema["properties"]["action"]["enum"]), set(schema["x-action-params"]))
            self.assertTrue(all(set(a["params"]) <= set(schema["properties"])
                                for a in schema["x-action-params"].values()))
            if name != "switch_mode":
                self.assertIn("x-completion", schema)
                self.assertIn("x-resource", schema)
            else:
                self.assertNotIn("x-resource", schema)  # Whole-body exclusion.
        self.assertEqual(tools["leg"]["inputSchema"]["x-resource"], tools["waist"]["inputSchema"]["x-resource"])

    def test_state_start_stop_and_ros2_forwarding(self):
        self.bundle.state._publish()
        self.assertEqual(self.node.publisher.messages, [])
        self.assertEqual(self.call("state", "start")["state"], "running")
        self.bundle.state._publish()
        state = json.loads(self.node.publisher.messages[-1].data)
        self.assertTrue(state["connected"])
        self.assertIs(state["fields"]["control_enabled"]["value"], True)
        self.assertEqual(self.bundle.state.tool()["topic_out"], [{"topic": "/test_robot/state/control", "format": "data/json"}])
        self.call("state", "stop")
        before = len(self.node.publisher.messages)
        self.bundle.state._publish()
        self.assertEqual(before, len(self.node.publisher.messages))
        self.assertEqual(self.call("state", "start")["state"], "running")
        self.assertIn("topic_out", self.bundle.dispatch("state", {"action": "info"}))

    def test_reconnection_invalidates_old_feedback_even_if_disconnect_missed(self):
        self.ros.connection = (True, 2)
        self.assertEqual(self.bundle.state.snapshot()["fields"], {})
        result = self.call("arm", "move_joints", positions=[0] * 14)
        self.assertEqual(result["state"], "error")
        self.assertEqual(self.ros.published, [])

    def test_disconnected_state_not_reported_ready(self):
        self.call('state', 'start')
        self.ros.connection = (False, 1)
        self.bundle.state._publish()
        self.assertFalse(json.loads(self.node.publisher.messages[-1].data)['connected'])
        self.assertFalse(self.call("state", "info")["feedback"]["connected"])
        self.assertEqual(self.call("loco", "start")["state"], "disconnected")

    def test_motion_requires_enabled_fresh_supported_mode_and_warmup(self):
        for setup in (
            lambda: self.ros.emit("/enable_control_state", False),
            lambda: self.bundle.state._seen.update(mpc_mode=time.monotonic() - 3),
            lambda: self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 1),
            lambda: setattr(self.ros, "motion_ready", False),
        ):
            self.ros.emit("/enable_control_state", True)
            self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 3)
            setup()
            self.assertEqual(self.call("loco", "move", duration=0.1)["state"], "error")
        self.assertEqual(self.ros.published, [])

    def test_rejects_invalid_values_before_hardware_calls(self):
        for args in ({"vx": 1, "duration": 1}, {"vx": True, "duration": 1},
                     {"vx": float("nan"), "duration": 1}, {"duration": 0}, {"duration": "1"}):
            self.assertEqual(self.call("loco", "move", **args)["state"], "error")
        self.assertEqual(self.call("arm", "move_joints", positions=[0] * 13)["state"], "error")
        self.assertEqual(self.call("waist", "set_pose", x=0)["state"], "error")
        self.assertEqual(self.ros.published, [])
        self.assertEqual(self.ros.services, [])

    def test_arm_does_not_enable_quick_mode_and_requires_fresh_estimate(self):
        self.ros.emit("/lb_arm_joint_reach_time", 0)
        result = self.call("arm", "move_joints", positions=list(range(14)))
        self.assertIn("action_id", result)
        self.assertEqual(self.ros.services, [])
        self.assertEqual(self.ros.published[-1], ("/kuavo_arm_traj", "sensor_msgs/JointState", {
            "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
            "name": [f"joint{i}" for i in range(1, 15)], "position": list(range(14)), "velocity": [], "effort": []}))
        time.sleep(0.06)
        self.assertEqual(self.callbacks, [])  # Cached estimate cannot complete new motion.
        self.ros.emit("/lb_arm_joint_reach_time", 0)
        self.ros.raw([0] * 4 + [math.radians(v) for v in range(14)] + [0] * 2)
        time.sleep(0.06)
        self.assertEqual(self.callbacks, [])  # A single in-tolerance sample is insufficient.
        time.sleep(0.17)
        self.ros.raw([0] * 4 + [math.radians(v) for v in range(14)] + [0] * 2)
        self.assertTrue(wait_until(lambda: len(self.callbacks) == 1))
        self.assertEqual(self.callbacks[0][0:3], (result["action_id"], "arm", "completed"))
        self.assertIs(self.callbacks[0][3]["measured_convergence"], True)

    def test_leg_and_waist_share_busy_channel(self):
        first = self.call("leg", "move_joints", positions=[1, 2, 3, 4])
        self.assertIn("action_id", first)
        second = self.call("waist", "set_pose", x=0, z=0.8, pitch=0, yaw=0)
        self.assertIn("busy", second["error"])
        self.assertEqual(len(self.ros.published), 1)

    def test_cartesian_wire_type_frame_quaternion(self):
        args = dict(frame="world", left_position=[0, 0.4, 0], right_position=[0, -0.4, 0],
                    left_quaternion=[0, 0, 0, 1], right_quaternion=[0, 0, 0, 1])
        bad = self.call("arm", "set_pose", **{**args, "left_quaternion": [0] * 4})
        self.assertIn("normalized", bad["error"])
        result = self.call("arm", "set_pose", **args)
        self.assertIn("action_id", result)
        topic, msg_type, msg = self.ros.published[-1]
        self.assertEqual((topic, msg_type, msg["frame"]), ("/mm/two_arm_hand_pose_cmd", "kuavo_msgs/twoArmHandPoseCmd", 1))
        self.assertFalse(msg["use_custom_ik_param"])
        self.ros.emit("/lb_arm_ee_reach_time", 0)
        self.assertTrue(wait_until(lambda: self.callbacks))

    def test_loco_velocity_acp_waits_for_vendor_stop_window(self):
        result = self.call("loco", "move", vx=0.1, duration=0.1)
        self.assertIn("action_id", result)
        self.assertGreater(self.ros.published[0][2]["linear"]["x"], 0)
        self.assertTrue(wait_until(lambda: self.ros.published[-1][2]["linear"]["x"] == 0))
        self.assertEqual(self.callbacks, [])
        self.assertTrue(wait_until(lambda: self.callbacks, 1.3))
        self.assertEqual(self.callbacks[0][2], "completed")
        self.assertEqual(self.callbacks[0][3]["stop_delay_s"], 1.0)

    def test_base_busy_or_invalid_command_preserves_active_motion(self):
        velocity = self.call("loco", "move", vx=0.1, duration=1)
        bad = self.call("loco", "move_relative", x=0.2)
        self.assertEqual(bad['state'], 'error')
        pose = self.call("loco", "move_relative", x=0.2, y=0, yaw=0)
        self.assertIn('busy', pose['error'])
        self.assertTrue(self.bundle.tracker.current('base', velocity['action_id']))
        self.assertFalse(any(p[0] == '/cmd_pose' for p in self.ros.published))
        self.assertEqual(self.callbacks, [])

    def test_split_action_schemas_are_action_specific(self):
        tool = next(t for t in self.bundle.get_all_tools() if t["name"] == "switch_mode")
        specs = tool["inputSchema"]["x-action-params"]
        self.assertIn("mode", tool["inputSchema"]["properties"])
        self.assertEqual(specs["set_mode"]["required"], ["mode"])
        self.assertEqual(specs["resume"]["required"], [])
        tools = {t['name']: t for t in self.bundle.get_all_tools()}
        self.assertEqual(tools['loco']['inputSchema']['x-action-params']['move']['required'], ['duration'])
        self.assertEqual(tools['arm']['inputSchema']['x-action-params']['move_joints']['required'], ['positions'])

    def test_joint_command_rejected_without_fresh_measurement(self):
        self.bundle.sensors._seen['joints'] = time.monotonic() - 1
        result = self.call('arm', 'move_joints', positions=[0.] * 14)
        self.assertIn('stale', result['error'])
        self.assertEqual(self.ros.published, [])

    def test_reach_time_alone_cannot_complete_unconverged_joints(self):
        result = self.call('leg', 'move_joints', positions=[20.] * 4)
        self.ros.emit('/lb_leg_joint_reach_time', 0.)
        for _ in range(6):
            self.ros.raw()  # Fresh measurements show that the leg has not moved.
            time.sleep(0.05)
        self.assertEqual(self.callbacks, [])
        self.assertTrue(self.bundle.tracker.current('lower_body', result['action_id']))
        self.bundle.sensors._seen['joints'] = time.monotonic() - 1
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][2], 'error')
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_leg_measured_convergence_degrees_to_radians(self):
        result = self.call('leg', 'move_joints', positions=[10.] * 4)
        self.ros.emit('/lb_leg_joint_reach_time', 0.)
        self.ros.raw([math.radians(10)] * 4 + [0.] * 16)
        time.sleep(0.04)
        time.sleep(0.18)
        self.ros.raw([math.radians(10)] * 4 + [0.] * 16)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'leg', 'completed'))
        self.assertTrue(self.callbacks[0][3]['measured_convergence'])

    def test_stop_blocks_old_velocity_thread_and_cancels_acp_once(self):
        result = self.call("loco", "move", vx=0.1, duration=1)
        stopped = self.call("loco", "stop_move")
        self.assertEqual(stopped["state"], "paused")
        boundary = len(self.ros.published)
        time.sleep(0.1)
        self.assertEqual(len(self.ros.published), boundary)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual([c[:3] for c in self.callbacks], [(result["action_id"], "loco", "cancelled")])
        self.assertEqual(self.call("loco", "move", duration=0.1)["state"], "error")

    def test_pause_service_acceptance_without_feedback_not_success(self):
        self.ros.confirm = False
        with patch.object(self.bundle, "wait_state", side_effect=TimeoutError("not confirmed")):
            result = self.call("switch_mode", "pause")
        self.assertEqual(result["state"], "error")
        self.assertFalse(result["physical_stop_confirmed"])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_pause_rejection_and_resume_feedback(self):
        self.ros.service_accepts = False
        self.assertEqual(self.call("switch_mode", "pause")["state"], "error")
        self.ros.service_accepts = True
        self.assertEqual(self.call("switch_mode", "pause")["state"], "paused")
        self.assertEqual(self.call("switch_mode", "resume")["state"], "ready")
        self.assertFalse(self.bundle.state.motion_blocked)
        self.assertEqual(self.ros.services[-1][:3], ("/enable_control", "std_srvs/SetBool", {"data": True}))

    def test_mode_switch_can_leave_no_control_and_is_confirmed(self):
        self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 0)
        result = self.call("switch_mode", "set_mode", mode=3)
        self.assertEqual(result["state"], "completed")
        self.assertEqual(self.ros.services[-1][:3], ("/mobile_manipulator_mpc_control", "kuavo_msgs/changeTorsoCtrlMode", {"control_mode": 3}))

    def test_feedback_loss_causes_error_not_false_completed(self):
        result = self.call("arm", "move_joints", positions=[0] * 14)
        self.ros.connection = (False, 1)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result["action_id"], "arm", "error"))
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_head_uses_degrees_per_second_and_has_acp(self):
        result = self.call("head", "move", yaw=12, pitch=4, duration=0.1)
        self.assertIn("action_id", result)
        self.assertEqual(self.ros.published[0], ("/cmd_head_vel", "kuavo_msgs/robotHeadMotionData", {"joint_data": [12, 4]}))
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.ros.published[-1][2], {"joint_data": [0, 0]})
        self.assertEqual(self.callbacks[0][3]["stop_delay_s"], 0.3)

    def test_slow_acp_delivery_does_not_block_stop(self):
        entered, release = threading.Event(), threading.Event()
        def slow_callback(*args):
            entered.set()
            release.wait(1)
        with patch.object(driver, "_acp_complete", side_effect=slow_callback):
            self.call("arm", "move_joints", positions=[0] * 14)
            self.ros.emit("/lb_arm_joint_reach_time", 0)
            self.ros.raw()
            time.sleep(0.05)
            time.sleep(0.18)
            self.ros.raw()
            self.assertTrue(entered.wait(1))
            try:
                result = self.call("loco", "stop_move")
                self.assertEqual(result["state"], "paused")
                self.assertFalse(release.is_set())
            finally:
                release.set()
                self.bundle.tracker._callbacks.join()

    def test_excessive_reach_time_aborts_and_pauses(self):
        result = self.call("arm", "move_joints", positions=[0] * 14)
        self.ros.emit("/lb_arm_joint_reach_time", 111)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result["action_id"], "arm", "error"))
        self.assertIn("110s", self.callbacks[0][3]["reason"])
        self.assertFalse(self.bundle.state.read("control_enabled")[0])

    def test_bad_end_effector_state_is_dropped(self):
        self.ros.emit("/humanoid_wheel/eePoses", [0] * 11)
        self.assertIsNone(self.bundle.state.read("ee_world_poses")[0])
        self.ros.emit("/humanoid_wheel/eePoses", list(range(12)))
        self.assertEqual(self.bundle.state.read("ee_world_poses")[0], list(range(12)))

    def test_dispatch_does_not_mutate_caller_arguments(self):
        args = {"action": "info"}
        self.bundle.dispatch("state", args)
        self.assertEqual(args, {"action": "info"})

    def test_shutdown_pauses_and_rejects_new_motion(self):
        self.call("loco", "move", duration=1)
        self.bundle.close()
        self.assertEqual(self.ros.services[-1][0], "/enable_control")
        self.assertEqual(self.call("loco", "move", duration=1)["state"], "error")
        # close is idempotent, including the callback worker.
        self.assertTrue(self.bundle._closing)


if __name__ == "__main__":
    unittest.main()
