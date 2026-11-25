"""Head following through the bundle, with ROS transport replaced by stubs."""

import importlib.util
import json
import math
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from test_tianyi_loco_servo import Clock, DRIVER, Executor, Node, load_bundle


class HeadNode(Node):
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

    assert call("start", input_topic="/test/face_tracking/i_test")["armed"] is False
    callback = follow._node.subscriptions[0][2]

    def send(state="tracking", track=1, offset=(0.8, 0.8), delay=0, step=0.125):
        clock.advance(step)
        callback(NS(data=json.dumps(dict(state=state, frame_ts=clock.time() - delay,
                                         target=dict(track_id=track, offset=list(offset))))))

    yield NS(bundle=bundle, clock=clock, head=head, call=call, send=send,
             feedback=feedback, commands=head._publisher.messages)
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
    s.send(state="lost")
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
    assert s.call("info")["reason"] == "tracking_timeout"
