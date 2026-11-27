"""Head following through the bundle, with ROS transport replaced by stubs."""

import importlib.util
import json
import math
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from test_tianyi_loco_servo import Clock, DRIVER, Executor, Node, load_bundle


class HeadNode(Node):
    def count_publishers(self, topic):
        return 0

    def create_publisher(self, kind, topic, qos):
        publisher = NS(messages=[])
        publisher.publish = publisher.messages.append
        self.publishers.append(publisher)
        return publisher


class HeadExecutor(Executor):
    def add_node(self, node):
        super().add_node(node)
        return True


@pytest.fixture
def setup(monkeypatch):
    enum = NS(BEST_EFFORT=1, RELIABLE=2, KEEP_LAST=1, VOLATILE=1)
    modules = {
        "rclpy": {},
        "rclpy.node": {"Node": HeadNode},
        "rclpy.qos": dict(QoSProfile=lambda **kw: kw, HistoryPolicy=enum,
                          ReliabilityPolicy=enum, DurabilityPolicy=enum),
        "rclpy.exceptions": {"InvalidTopicNameException": TypeError},
        "rclpy.validate_full_topic_name": {"validate_full_topic_name": lambda topic: None},
        "sensor_msgs.msg": dict(CompressedImage=NS, CameraInfo=NS, Image=NS),
        "std_msgs.msg": dict(String=NS, Bool=NS, UInt32MultiArray=NS),
        "bodyctrl_msgs.msg": dict(CmdSetMotorPosition=NS, SetMotorPosition=NS,
                                  MotorStatusMsg=NS, PowerBoardKeyStatus=NS),
    }
    for name, attrs in modules.items():
        module = ModuleType(name)
        vars(module).update(attrs)
        monkeypatch.setitem(sys.modules, name, module)

    clock = Clock()
    for name in ("head_face_follow", "device"):
        spec = importlib.util.spec_from_file_location(name, DRIVER / f"{name}.py")
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        monkeypatch.setattr(module, "time", clock)

    ros = NS(ctx_core=object(), ctx_tianyi=object(),
             executor_core=HeadExecutor(), executor_tianyi=HeadExecutor())
    config = {"plugins": {name: {"enabled": True}
                          for name in ("head", "head_gesture", "head_face_follow")}}
    bundle = load_bundle()(config, "tianyi", ros, None)
    bundle.start_all()
    follow = bundle._head_follow
    head = follow._head

    def feedback(yaw=0, pitch=0, estop=False):
        head._on_head_status(NS(status=[
            NS(name=joint, pos=math.radians(angle), speed=0, current=0, temperature=0, error=0)
            for joint, angle in enumerate([0, pitch, yaw], 1)
        ]))
        head._on_power_status(NS(is_estop=NS(data=estop), is_remote_estop=NS(data=False),
                                 is_power_on=NS(data=True)))

    def call(action, **kwargs):
        return bundle.dispatch("head_face_follow", dict(action=action, **kwargs))

    class DecodeError(Exception):
        pass

    def detect(frame):
        offset, delay = frame
        clock.advance(delay)
        if offset is None:
            return [], (200, 200)
        x, y = (round((1 + value) * 100 - 10) for value in offset)
        return [(x, y, 20, 20)], (200, 200)

    monkeypatch.setattr(sys.modules["head_face_follow"], "_Detector",
                        lambda: NS(detect=detect, cv2=NS(error=DecodeError)))
    started = call("start")
    assert started["state"] == "ready"
    assert started["armed"] is False
    assert started["topic_in"][0]["topic"] == "/tianyi/camera/head"
    callback = follow._node.subscriptions[0][2]

    def send(offset=(0.8, 0.8), delay=0, step=0.125):
        clock.advance(step)
        callback(NS(data=(offset, delay)))


    yield NS(bundle=bundle, clock=clock, head=head, call=call, send=send,
             feedback=feedback, follow=follow, commands=head._publisher.messages)
    bundle.stop_all()
    assert not ros.executor_core.nodes


def test_explicit_arm_motion_limits_and_manual_interlock(setup):
    s = setup
    s.feedback(yaw=-29, pitch=11)
    s.send()
    assert not s.commands
    assert s.call("arm")["armed"]
    s.send()
    assert not s.commands
    s.send()
    roll, pitch, yaw = s.commands[-1].cmds
    assert [motor.name for motor in (roll, pitch, yaw)] == [1, 2, 3]
    assert [math.degrees(motor.pos) for motor in (roll, pitch, yaw)] == pytest.approx([0, 12, -30])
    assert math.degrees(yaw.spd) == pytest.approx(10)
    s.send()
    assert len(s.commands) == 1
    assert s.bundle.dispatch("head", {"action": "move_pos"})["state"] == "error"
    assert s.bundle.dispatch("head_gesture", {"action": "reset"})["state"] == "error"
    assert s.call("stop")["state"] == "idle"
    s.send()
    assert len(s.commands) == 1


def test_loss_stale_frames_and_fault_require_safe_reacquisition(setup):
    s = setup
    assert s.call("arm")["state"] == "error"
    s.feedback()
    assert s.call("arm")["armed"]
    s.send()
    s.send(offset=None)
    s.send()
    s.send(delay=1)
    s.send()
    assert not s.commands
    s.feedback(estop=True)
    s.send()
    assert not s.commands
    assert s.call("info")["reason"] == "emergency_stop_active"
    assert not s.call("info")["armed"]
    s.feedback()
    s.send()
    s.send()
    assert not s.commands
    assert s.call("arm")["armed"]
    s.send(offset=(0.1, -0.1))
    s.send(offset=(0.1, -0.1))
    assert not s.commands
    s.clock.advance(1)
    assert s.call("info")["reason"] == "camera_timeout"


def test_nearest_uses_registered_depth_and_rejects_stale_or_wrong_frame(setup):
    np = pytest.importorskip("numpy")
    follow = sys.modules["head_face_follow"]

    def header(frame="color", ns=0):
        return NS(frame_id=frame, stamp=NS(sec=20, nanosec=ns))

    pixels = np.full((80, 100), 2000, dtype="<u2")
    pixels[35:65, 65:95] = 1200
    depth = NS(width=100, height=80, step=200, is_bigendian=False,
               encoding="16UC1", header=header(), data=pixels.tobytes())
    rgb = NS(header=header())
    large, nearer = (0, 0, 50, 60), (65, 35, 30, 30)
    assert follow._nearest_face([large, nearer], (100, 80), rgb, depth, np) == nearer
    depth.header = header(ns=200_000_000)
    assert follow._nearest_face([large, nearer], (100, 80), rgb, depth, np) is None
    depth.header = header(frame="depth")
    assert follow._nearest_face([large, nearer], (100, 80), rgb, depth, np) is None


def test_speaker_keeps_lock_until_speech_ends(setup):
    s = setup
    s.feedback()
    assert s.call("set_mode", mode="speaker")["mode"] == "speaker"
    assert s.call("arm")["armed"]
    s.follow._intrinsics = ((200, 200), 100, 100)
    callback = s.follow._node.subscriptions[2][2]

    def speech(active, bearing=None):
        s.clock.advance(0.01)
        callback(NS(data=json.dumps(dict(ts=s.clock.time(), active=active,
                                         camera_bearing_deg=bearing))))

    speech(True, -39)
    s.send()
    s.send()
    assert len(s.commands) == 1

    speech(True, 39)
    s.send(offset=(-0.8, 0.8))
    assert len(s.commands) == 1
    speech(False)
    speech(True, 39)
    s.send(offset=(-0.8, 0.8))
    s.feedback()
    s.send(offset=(-0.8, 0.8))
    assert len(s.commands) == 2
    assert s.commands[-1].cmds[2].pos > 0


def test_object_mode_starts_vop_on_connected_camera(setup, monkeypatch):
    calls = []
    monkeypatch.setattr(sys.modules["head_face_follow"], "_start_vop", calls.append)
    assert setup.call("set_target", object_name=" Cup ")["object_name"] == "cup"
    assert calls == ["/tianyi/camera/head"]
    assert setup.call("set_target", object_name="")["mode"] == "face"


def test_native_vop_positions_drive_only_on_live_distinct_results(setup, monkeypatch):
    s = setup
    monkeypatch.setattr(sys.modules["head_face_follow"], "_start_vop", lambda topic: None)
    assert s.call("set_target", object_name="cup")["mode"] == "object"
    s.feedback()
    assert s.call("arm")["armed"]
    objects = s.follow._node.subscriptions[1][2]

    def publish(position, age=0):
        objects(NS(data=json.dumps({"timestamp": s.clock.time() - age,
                                    "objects": [{"name": "cup", "position": position,
                                                 "confidence": 0.9}]})))

    publish([0.8, 0.2], age=1)
    s.send()
    publish(["bad", 0.2])
    s.send()
    assert not s.commands
    publish([0.8, 0.2])
    s.send()
    s.feedback()
    publish([0.81, 0.2])
    s.send()
    assert len(s.commands) == 1
    s.send(step=0.25)
    assert len(s.commands) == 1
