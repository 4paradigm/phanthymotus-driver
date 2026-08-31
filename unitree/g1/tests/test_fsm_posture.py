"""
Tests for G1 FSM mode handling and joint-derived posture estimation.

Covers the two things the FSM code got wrong and that hardware logs proved:

  * FSM target ids. StandUp2Squat()/Squat2StandUp() both SetFsmId(706); Lie2StandUp()
    sets 702. The old code polled for 2 and 500 respectively, so every squat and
    every lie-to-stand timed out even when the robot physically completed it.
  * mode != posture. In 零力矩/阻尼 (FSM 0/1) the robot is limp and lying flat is the
    same fsm_id as folded into a squat, so the guard has to consult joint angles.

Reference for the ID table and fsm_mode semantics:
https://support.unitree.com/home/zh/G1_developer/sport_services_interface

Run:  python3 -m unittest discover -s unitree/g1/tests -t .
"""

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")


class FakeNode:
    def __init__(self, name):
        self._name = name

    def create_publisher(self, *a, **kw):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, *a, **kw):
        return object()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                     error=lambda *a: None, debug=lambda *a: None)


def install_stubs():
    rclpy = types.ModuleType("rclpy")
    sys.modules.setdefault("rclpy", rclpy)
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Header = type("Header", (Message,), {})
    std_msgs.String = type("String", (Message,),
                           {"__init__": lambda self: setattr(self, "data", "")})
    std_msgs.UInt8MultiArray = type("UInt8MultiArray", (Message,), {})
    sys.modules["std_msgs.msg"] = std_msgs

    audio_msgs = types.ModuleType("audio_msgs.msg")
    audio_msgs.AudioChunk = type("AudioChunk", (Message,), {})
    sys.modules["audio_msgs.msg"] = audio_msgs

    # unitree_sdk2py: device.py only needs AudioClient to exist at import time.
    for name in ("unitree_sdk2py", "unitree_sdk2py.g1", "unitree_sdk2py.g1.audio"):
        sys.modules.setdefault(name, types.ModuleType(name))
    audio_mod = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    audio_mod.AudioClient = type("AudioClient", (), {})
    sys.modules["unitree_sdk2py.g1.audio.g1_audio_client"] = audio_mod

    pcu = types.ModuleType("pointcloud_utils")
    pcu.gravity_align_inplace = lambda *a, **kw: None
    sys.modules["pointcloud_utils"] = pcu

    # cyclonedds is only needed for the IDL dataclass declaration.
    cdds = types.ModuleType("cyclonedds")
    idl = types.ModuleType("cyclonedds.idl")

    class _IdlStruct:
        def __init_subclass__(cls, **kw):
            pass

    idl.IdlStruct = _IdlStruct
    ann = types.ModuleType("cyclonedds.idl.annotations")
    ann.final = lambda c: c
    ann.autoid = lambda kind: (lambda c: c)
    tps = types.ModuleType("cyclonedds.idl.types")
    tps.uint32 = int
    tps.float32 = float
    idl.annotations = ann
    idl.types = tps
    cdds.idl = idl
    sys.modules.update({"cyclonedds": cdds, "cyclonedds.idl": idl,
                        "cyclonedds.idl.annotations": ann, "cyclonedds.idl.types": tps})


def load_device():
    install_stubs()
    spec = importlib.util.spec_from_file_location("g1_device_under_test", ROOT / "device.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["g1_device_under_test"] = module
    spec.loader.exec_module(module)
    return module


DEV = load_device()
import sport_mode_state as SMS  # noqa: E402  (needs the stubs installed first)


# ── Fakes ───────────────────────────────────────────────────────────────────

def make_lowstate(knee, hip_pitch, roll_deg=0.0, pitch_deg=0.0, tau=0.3, n=29):
    """Minimal rt/lowstate stand-in with the leg joints the estimator reads."""
    motors = [types.SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0, temperature=[30, 30])
              for _ in range(n)]
    for i in (3, 9):        # knees
        motors[i].q = knee
        motors[i].tau_est = tau
    for i in (0, 6):        # hip pitch
        motors[i].q = hip_pitch
    imu = types.SimpleNamespace(
        quaternion=[1.0, 0.0, 0.0, 0.0], gyroscope=[0.0] * 3,
        accelerometer=[0.0] * 3, temperature=30.0,
        rpy=[math.radians(roll_deg), math.radians(pitch_deg), 0.0],
    )
    return types.SimpleNamespace(motor_state=motors, imu_state=imu)


class FakeLocoClient:
    """Records what the plugin actually commanded."""

    def __init__(self, fsm_id=1):
        self.fsm_id = fsm_id
        self.calls = []

    def _rec(self, name):
        self.calls.append(name)
        return 0

    def GetFsmId(self):
        return 0, self.fsm_id

    def StopMove(self):
        return self._rec("StopMove")

    def Damp(self):
        return self._rec("Damp")

    def ZeroTorque(self):
        return self._rec("ZeroTorque")

    def RunFsmSequence(self, steps, **kw):
        self.calls.append(("RunFsmSequence", steps))
        return {"ret": 0, "steps": [s[2] for s in steps], "fsm_id": self.fsm_id}


class FakeStateNode:
    def __init__(self, fsm_id, fsm_mode=0, stale=False):
        import time
        self._fsm = {"fsm_id": fsm_id, "fsm_mode": fsm_mode,
                     "ts": 0 if stale else time.time()}

    def get_fsm(self):
        return dict(self._fsm)


class FakePostureNode:
    def __init__(self, posture):
        self._p = posture

    def get_posture(self):
        return dict(self._p) if self._p else {}


def make_plugin(fsm_id, fsm_mode=0, posture=None, use_topic=True):
    client = FakeLocoClient(fsm_id)
    plugin = DEV.LocoPlugin(
        {}, "ubuntu", None, client,
        state_node=FakeStateNode(fsm_id, fsm_mode) if use_topic else None,
        posture_node=FakePostureNode(posture) if posture is not None else None,
    )
    return plugin, client


def switch(plugin, mode):
    """Invoke switch_mode without the ACP background thread."""
    captured = {}

    def fake_async(m, steps):
        captured["mode"] = m
        captured["steps"] = steps
        return {"status": "executing", "mode": m, "action_id": "test"}

    plugin._async_fsm = fake_async
    result = plugin.dispatch(mode, {})
    result = dict(result or {})
    if captured:
        result["_steps"] = captured["steps"]
    return result


def step_targets(steps, name):
    for s in steps:
        if s[2] == name:
            targets = s[1]
            return {targets} if isinstance(targets, int) else set(targets)
    raise AssertionError(f"no step named {name!r} in {steps}")


# ── Official FSM table ──────────────────────────────────────────────────────

class TestFsmTable(unittest.TestCase):
    def test_unbalanced_modes_flagged(self):
        for fsm_id in (0, 1, 2, 3, 4):
            self.assertFalse(SMS.FSM_MODES[fsm_id]["balanced"],
                             f"FSM {fsm_id} has no balance control per the vendor doc")

    def test_balanced_modes_flagged(self):
        for fsm_id in (500, 501, 702, 706, 801, 802):
            self.assertTrue(SMS.FSM_MODES[fsm_id]["balanced"])

    def test_802_is_a_loco_state(self):
        # 29-DoF renumbers 走跑运控 801 -> 802 from ai_sport 8.6.x.x; the old
        # _STANDING_STATES omitted it, so a walking robot read as "not standing".
        self.assertIn(802, SMS.LOCO_STATES)
        self.assertIn(801, SMS.LOCO_STATES)

    def test_squat_and_lie_ids(self):
        self.assertEqual(SMS.BALANCED_SQUAT, 706)
        self.assertEqual(SMS.LIE_TO_STAND, 702)

    def test_706_is_not_treated_as_a_loco_state(self):
        self.assertNotIn(706, SMS.LOCO_STATES)


# ── Posture estimation ─────────────────────────────────────────────────────

class TestPosture(unittest.TestCase):
    def setUp(self):
        self.node = DEV._LowStateNode.__new__(DEV._LowStateNode)

    def estimate(self, **kw):
        return self.node._estimate_posture(make_lowstate(**kw))

    def test_measured_squat_from_hardware(self):
        # Values read off G1 10.100.129.168 while collapsed in a squat.
        p = self.estimate(knee=2.899, hip_pitch=-2.527, pitch_deg=29.2, roll_deg=1.1, tau=0.26)
        self.assertEqual(p["posture"], "squat")
        self.assertFalse(p["loaded"], "0.26 N·m is limp, not actively holding")

    def test_standing(self):
        p = self.estimate(knee=0.15, hip_pitch=-0.1, pitch_deg=1.0, tau=40.0)
        self.assertEqual(p["posture"], "standing")
        self.assertTrue(p["loaded"])

    def test_lying_detected_by_torso_tilt(self):
        # Legs extended like standing, but the torso is horizontal.
        p = self.estimate(knee=0.1, hip_pitch=0.0, pitch_deg=88.0)
        self.assertEqual(p["posture"], "lying")

    def test_intermediate_is_crouched_not_guessed(self):
        p = self.estimate(knee=1.0, hip_pitch=-0.8)
        self.assertEqual(p["posture"], "crouched")

    def test_malformed_message_does_not_raise(self):
        p = self.node._estimate_posture(types.SimpleNamespace())
        self.assertEqual(p["posture"], "unknown")

    def test_reported_values_round_trip(self):
        p = self.estimate(knee=2.9, hip_pitch=-2.5, pitch_deg=29.0)
        self.assertAlmostEqual(p["knee_rad"], 2.9, places=3)
        self.assertAlmostEqual(p["hip_pitch_rad"], -2.5, places=3)
        self.assertAlmostEqual(p["torso_pitch_deg"], 29.0, places=1)


# ── switch_mode: FSM targets ───────────────────────────────────────────────

class TestSquatTargets(unittest.TestCase):
    def test_standup2squat_polls_706_not_2(self):
        plugin, _ = make_plugin(500)
        res = switch(plugin, "standup2squat")
        self.assertEqual(step_targets(res["_steps"], "standup2squat"), {706})

    def test_standup2squat_stops_motion_first(self):
        plugin, client = make_plugin(500)
        switch(plugin, "standup2squat")
        self.assertIn("StopMove", client.calls)

    def test_standup2squat_rejected_when_not_in_loco(self):
        # 706 is only reachable from 主运控; from 阻尼 it is not.
        plugin, _ = make_plugin(1, posture={"posture": "squat"})
        res = switch(plugin, "standup2squat")
        self.assertIn("error", res)
        self.assertNotIn("_steps", res)

    def test_standup2squat_idempotent(self):
        plugin, _ = make_plugin(706)
        res = switch(plugin, "standup2squat")
        self.assertIn("info", res)

    def test_squat2standup_from_706_targets_loco(self):
        plugin, _ = make_plugin(706)
        res = switch(plugin, "squat2standup")
        self.assertEqual(step_targets(res["_steps"], "squat2standup"), set(SMS.LOCO_STATES))

    def test_squat2standup_from_damp_is_refused_with_posture_hint(self):
        # The exact hardware failure: limp in 阻尼, physically squatting.
        plugin, _ = make_plugin(1, posture={"posture": "squat"})
        res = switch(plugin, "squat2standup")
        self.assertIn("error", res)
        self.assertIn("706", res["error"])
        self.assertNotIn("_steps", res, "must not fire a doomed sequence")

    def test_squat2standup_from_damp_while_lying_points_at_lie2standup(self):
        plugin, _ = make_plugin(1, posture={"posture": "lying"})
        res = switch(plugin, "squat2standup")
        self.assertIn("lie2standup", res["error"])

    def test_squat2standup_idempotent_when_standing(self):
        for fsm in SMS.LOCO_STATES:
            plugin, _ = make_plugin(fsm)
            self.assertIn("info", switch(plugin, "squat2standup"))


class TestLieToStand(unittest.TestCase):
    def test_waits_for_702_then_loco(self):
        plugin, _ = make_plugin(1, posture={"posture": "lying"})
        steps = switch(plugin, "lie2standup")["_steps"]
        self.assertEqual(step_targets(steps, "lie2standup"), {702})
        self.assertEqual(step_targets(steps, "start"), set(SMS.LOCO_STATES))

    def test_damps_first_from_zero_torque(self):
        plugin, _ = make_plugin(0, posture={"posture": "lying"})
        steps = switch(plugin, "lie2standup")["_steps"]
        self.assertEqual([s[2] for s in steps], ["damp", "lie2standup", "start"])

    def test_refused_when_posture_is_a_squat(self):
        plugin, _ = make_plugin(1, posture={"posture": "squat"})
        res = switch(plugin, "lie2standup")
        self.assertIn("error", res)
        self.assertNotIn("_steps", res)

    def test_refused_from_706(self):
        # 706 was previously in _UNSAFE_STATES, which routed the caller to
        # emergency_stop -> Damp() and destroyed the balanced squat.
        plugin, _ = make_plugin(706)
        res = switch(plugin, "lie2standup")
        self.assertIn("error", res)
        self.assertNotIn("emergency_stop", res["error"])

    def test_timeouts_exceed_old_15s_budget(self):
        plugin, _ = make_plugin(1, posture={"posture": "lying"})
        steps = switch(plugin, "lie2standup")["_steps"]
        for s in steps:
            if s[2] in ("lie2standup", "start"):
                self.assertGreater(s[3], 15.0, f"step {s[2]} still has the old budget")


class TestStandUp2Lie(unittest.TestCase):
    def test_from_loco_goes_via_706(self):
        plugin, _ = make_plugin(500)
        steps = switch(plugin, "standup2lie")["_steps"]
        self.assertEqual(step_targets(steps, "standup2squat"), {706})
        self.assertEqual(step_targets(steps, "damp"), {1})

    def test_from_706_just_damps(self):
        plugin, _ = make_plugin(706)
        steps = switch(plugin, "standup2lie")["_steps"]
        self.assertEqual([s[2] for s in steps], ["damp"])

    def test_idempotent_when_limp(self):
        plugin, _ = make_plugin(1)
        self.assertIn("info", switch(plugin, "standup2lie"))


# ── switch_mode: fsm_mode gating and reporting ─────────────────────────────

class TestDynamicStateGate(unittest.TestCase):
    def test_dynamic_state_refuses_switches(self):
        plugin, client = make_plugin(500, fsm_mode=1)
        res = switch(plugin, "standup2squat")
        self.assertIn("error", res)
        self.assertIn("fsm_mode=1", res["error"])
        self.assertNotIn("_steps", res)

    def test_emergency_stop_ignores_dynamic_state(self):
        # 阻尼 is documented as always available; it must never be gated.
        plugin, client = make_plugin(801, fsm_mode=1)
        res = switch(plugin, "emergency_stop")
        self.assertNotIn("error", res)
        self.assertIn("Damp", client.calls)

    def test_emergency_stop_does_not_need_the_topic(self):
        plugin, client = make_plugin(500, use_topic=False)
        switch(plugin, "emergency_stop")
        self.assertIn("Damp", client.calls)


class TestStateReporting(unittest.TestCase):
    def test_get_current_mode_includes_posture(self):
        plugin, _ = make_plugin(1, posture={"posture": "squat", "knee_rad": 2.9})
        res = switch(plugin, "get_current_mode")
        self.assertEqual(res["fsm_id"], 1)
        self.assertEqual(res["posture"], "squat")
        self.assertEqual(res["source"], "sportmodestate")
        self.assertIn("NO balance control", res["description"])

    def test_falls_back_to_rpc_when_topic_stale(self):
        client = FakeLocoClient(706)
        plugin = DEV.LocoPlugin({}, "ubuntu", None, client,
                                state_node=FakeStateNode(706, stale=True))
        res = switch(plugin, "get_current_mode")
        self.assertEqual(res["source"], "rpc")
        self.assertIsNone(res["fsm_mode"], "RPC cannot report switchability")

    def test_rpc_failure_aborts(self):
        client = FakeLocoClient(1)
        client.GetFsmId = lambda: (-1, 0)
        plugin = DEV.LocoPlugin({}, "ubuntu", None, client)
        res = switch(plugin, "get_current_mode")
        self.assertIn("error", res)

    def test_damp_refused_from_upright(self):
        for fsm in (500, 706, 2):
            plugin, client = make_plugin(fsm)
            res = switch(plugin, "damp")
            self.assertIn("error", res, f"damp from {fsm} should be refused")
            self.assertNotIn("Damp", client.calls)


if __name__ == "__main__":
    unittest.main()
