"""天轶 weigh 卡：去皮、称重、校准、拒绝条件。

计算核心 WeighEngine 不依赖 ROS，这里用假时钟按 100 Hz 喂合成数据。
数值取自 2026-11-23 右手可行性测试：托举姿态下零点约 (−4.743, 26.164, −10.002) N，
物体重力在传感器系约沿 +y（单位向量约 (0.03, 0.996, 0.07)），原始读数偏高约 DEFAULT_SCALE 倍。

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_tianyi_weigh.py -q
"""

from __future__ import annotations

import json
import math
import random
import re
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "x-humanoid" / "tianyi2.0"
sys.path.insert(0, str(BUNDLE))

import weigh  # noqa: E402

G = weigh.G
F0 = (-4.743, 26.164, -10.002, 0.10, -0.20, 0.05)
UNIT = (0.03, 0.996, 0.07)
POSE_DEG = (5.0, 5.6, 0.0, -90.0, -90.0, 9.0, 0.1)
RATED = [weigh.RATED_CURRENT_A[i] for i in weigh.ARM_JOINT_IDS["right"]]


class Rig:
    """假机器人：假时钟 + 100 Hz 力数据 + 100 Hz 关节状态。sleep 推进时钟并产生数据。"""

    def __init__(self, tmp_path, **cfg):
        self.t = 1000.0
        self.wall = 1_763_000_000.0
        self.mass = 0.0              # 右手掌上物体真实质量（kg）
        self.mass_left = 0.0         # 左手掌上物体真实质量（kg）
        self.left_target = None
        self.left_error = 0
        self.gain = weigh.DEFAULT_SCALE   # 传感器比例误差
        self.lever = 0.12            # 物体到传感器的力臂（m，沿 x）
        self.noise = 0.02            # 力噪声 sd（N）
        self.pose = list(POSE_DEG)
        self.left = [1.0, 2.0, 0.0, -5.0, 0.0, 0.0, 0.0]
        self.speed = 0.0
        self.arm_target = None       # 假 arm 卡：move_pos 后右臂以 0.3 rad/s 逼近目标
        self.arm_stuck = False
        self.arm_calls = []
        self.arm_reply = None
        self.hand_calls = []
        self.hand_reply = None
        self.order = []              # ("arm"|"hand", action) 调用顺序
        self.current_ratio = 0.1
        self.error = 0
        self.force_on = True
        self.rng = random.Random(7)
        self.events = []             # (t, fn)：在假时钟到达 t 时执行
        cfg.setdefault("state_file", str(tmp_path / "weigh.json"))
        self.logs = []
        self.engine = weigh.WeighEngine(cfg, clock=lambda: self.t, wall=lambda: self.wall,
                                        sleep=self.advance, log=self.logs.append,
                                        arm_call=self.fake_arm, hand_call=self.fake_hand)

    def fake_hand(self, args):
        self.hand_calls.append(dict(args))
        self.order.append(("hand", args["action"]))
        if self.hand_reply is not None:
            return self.hand_reply
        if args["action"] == "start":
            return {"state": "ready"}
        return {"state": "moving", "side": args.get("side"), "mode": "set_fingers_raw"}

    def fake_arm(self, args):
        self.arm_calls.append(dict(args))
        self.order.append(("arm", args["action"]))
        if self.arm_reply is not None:
            return self.arm_reply
        if args["action"] == "start":
            return {"state": "ready"}
        if args["action"] == "move_pos":
            self.arm_target = list(args["right_positions"])
            self.left_target = list(args["left_positions"])
            return {"state": "moving", "feedback_verified": True}
        return {"error": "unknown action"}

    def moves(self):
        return [c for c in self.arm_calls if c["action"] == "move_pos"]

    def advance(self, dt: float) -> None:
        steps = max(1, int(round(dt / 0.01)))
        for _ in range(steps):
            self.t += 0.01
            self.wall += 0.01
            for ev in [e for e in self.events if e[0] <= self.t]:
                self.events.remove(ev)
                ev[1]()
            if self.force_on:
                for side, mass in (("right", self.mass), ("left", self.mass_left)):
                    w = mass * G * self.gain
                    f = [F0[i] + w * UNIT[i] + self.rng.gauss(0, self.noise) for i in range(3)]
                    # τ = r × F，r 沿传感器 x 轴
                    df = [w * u for u in UNIT]
                    r = (self.lever, 0.0, 0.0)
                    tq = [F0[3] + r[1] * df[2] - r[2] * df[1],
                          F0[4] + r[2] * df[0] - r[0] * df[2],
                          F0[5] + r[0] * df[1] - r[1] * df[0]]
                    self.engine.add_force(side, f + tq)
            step = math.degrees(0.3) * 0.01

            def approach(cur, target):
                moved = False
                for i in range(7):
                    d = target[i] - cur[i]
                    if abs(d) > 1e-9:
                        moved = True
                        cur[i] += max(-step, min(step, d))
                return moved
            moving = left_moving = False
            if self.arm_target is not None and not self.arm_stuck:
                moving = approach(self.pose, self.arm_target)
            if self.left_target is not None and not self.arm_stuck:
                left_moving = approach(self.left, self.left_target)
            spd = 0.3 if moving else self.speed
            ids = weigh.ARM_JOINT_IDS["right"]
            self.engine.add_motor_status(
                [(mid, math.radians(self.pose[i]), spd, self.current_ratio * RATED[i], self.error)
                 for i, mid in enumerate(ids)]
                + [(mid, math.radians(self.left[i]), 0.3 if left_moving else 0.0, 0.5, self.left_error)
                   for i, mid in enumerate(weigh.ARM_JOINT_IDS["left"])])

    def warm(self, seconds=4.0):
        self.advance(seconds)


@pytest.fixture
def rig(tmp_path):
    r = Rig(tmp_path)
    r.warm()
    return r


def test_tare_then_weigh_matches_known_masses(rig):
    assert rig.engine.tare({"side": "right"})["state"] == "tared"
    for mass in (0.21, 0.59, 1.0):
        rig.mass = mass
        rig.warm()
        res = rig.engine.weigh({"side": "right"})
        assert res["state"] == "weighed" and res["valid"], res
        # 默认 scale = 模拟增益，因此应还原真实质量
        assert res["mass_kg"] == pytest.approx(mass, abs=0.005)
        assert res["raw_kg"] == pytest.approx(mass * weigh.DEFAULT_SCALE, abs=0.006)
        assert res["calibrated"] is False and "uncalibrated_default_scale" in res["warnings"]
        assert res["lever_arm_m"] == pytest.approx(0.12, abs=0.01)
        assert res["experimental"] is True


def test_weigh_immediately_after_placing_uses_fresh_data(rig):
    """回归：不能拿调用之前缓冲区里的空载窗口当结果。"""
    rig.engine.tare({})
    rig.warm()
    rig.mass = 0.59           # 刚放上就调用
    res = rig.engine.weigh({})
    assert res["valid"] and res["mass_kg"] == pytest.approx(0.59, abs=0.005)


def test_object_placed_during_weigh_waits_for_new_stable_window(rig):
    rig.engine.tare({})
    rig.warm()
    t0 = rig.t
    rig.events.append((t0 + 1.2, lambda: setattr(rig, "mass", 0.59)))
    res = rig.engine.weigh({})
    assert res["valid"] and res["mass_kg"] == pytest.approx(0.59, abs=0.005)
    assert rig.t - t0 >= 1.2 + 3.0 - 0.3


def test_weigh_requires_tare(rig):
    res = rig.engine.weigh({})
    assert res["state"] == "error" and res["code"] == "not_tared"


def test_tare_expires(tmp_path):
    rig = Rig(tmp_path, tare_ttl_s=10)
    rig.warm()
    rig.engine.tare({})
    rig.advance(11)
    res = rig.engine.weigh({})
    assert res["code"] == "tare_expired"


def test_empty_hand_reads_below_resolution(rig):
    rig.engine.tare({})
    rig.warm()
    res = rig.engine.weigh({})
    assert res["valid"] and res["mass_kg"] < 0.02
    assert "below_resolution" in res["warnings"]


def test_unstable_force_times_out(rig):
    rig.engine.tare({})
    rig.noise = 1.0           # 物体在晃
    res = rig.engine.weigh({"timeout_s": 5})
    assert res["state"] == "error" and res["code"] == "not_stable"
    assert res["timed_out"] and "force_unstable" in res["reasons"]
    assert res["valid"] is False


def test_pose_change_invalidates(rig):
    rig.engine.tare({})
    rig.pose[6] += 2.0        # 腕 roll 被压下 2°（阈值 1.5°）
    rig.mass = 0.5
    rig.warm()
    res = rig.engine.weigh({})
    assert res["state"] == "weighed" and res["valid"] is False
    assert "pose_changed" in res["reasons"]
    assert res["max_joint_dev_deg"] == pytest.approx(2.0, abs=0.01)


def test_small_pose_change_is_tolerated(rig):
    rig.engine.tare({})
    rig.pose[6] -= 1.0        # 1.28 kg 时实测压下 0.76°，必须仍然有效
    rig.mass = 0.5
    rig.warm()
    res = rig.engine.weigh({})
    assert res["valid"], res


def test_joint_error_fails_fast(rig):
    rig.engine.tare({})
    rig.error = 3
    rig.warm(1)
    t0 = rig.t
    res = rig.engine.weigh({"timeout_s": 15})
    assert res["state"] == "error" and "joint_error" in res["reasons"]
    assert rig.t - t0 < 1.0


def test_moving_arm_is_not_stable(rig):
    rig.engine.tare({})
    rig.speed = 0.2
    res = rig.engine.weigh({"timeout_s": 4})
    assert res["code"] == "not_stable" and "joint_moving" in res["reasons"]


def test_high_current_invalidates(rig):
    rig.engine.tare({})
    rig.current_ratio = 0.65
    rig.mass = 0.5
    rig.warm()
    res = rig.engine.weigh({})
    assert res["valid"] is False and "current_high" in res["reasons"]


def test_current_up_to_60_percent_is_valid(rig):
    rig.engine.tare({})
    rig.current_ratio = 0.55
    rig.mass = 0.5
    rig.warm()
    assert rig.engine.weigh({})["valid"]


def test_no_fixed_weight_limit_by_default(rig):
    rig.engine.tare({})
    rig.mass = 2.0
    rig.warm()
    res = rig.engine.weigh({})
    assert res["valid"] and res["mass_kg"] == pytest.approx(2.0, abs=0.01)


def test_optional_fixed_load_limit(tmp_path):
    rig = Rig(tmp_path, load_limit_kg=1.0)
    rig.warm()
    rig.engine.tare({})
    rig.mass = 1.3
    rig.warm()
    res = rig.engine.weigh({})
    assert res["valid"] is False and "over_load_limit" in res["reasons"]
    assert res["mass_kg"] == pytest.approx(1.3, abs=0.01)


def test_force_dropout_is_detected(rig):
    rig.engine.tare({})
    rig.force_on = False
    res = rig.engine.weigh({"timeout_s": 3})
    assert res["state"] == "error"
    assert "force_stream_stale" in res["reasons"]


def test_no_data_at_all_fails_after_grace(tmp_path):
    rig = Rig(tmp_path)
    rig.engine._tare["right"] = {"F": list(F0), "q": [0] * 7, "t": rig.t, "wall": rig.wall}
    rig.force_on = False
    res = rig.engine.weigh({"timeout_s": 15})
    assert res["state"] == "error" and "no_force_data" in res["reasons"]
    assert res["timed_out"] is False


def test_calibrate_with_known_mass_persists(rig, tmp_path):
    rig.gain = 1.05
    rig.engine.tare({})
    rig.mass = 0.59
    rig.warm()
    res = rig.engine.calibrate({"known_mass_kg": 0.59})
    assert res["state"] == "calibrated", res
    assert res["scale"] == pytest.approx(1.05, abs=0.01)
    assert res["old_scale"] == pytest.approx(weigh.DEFAULT_SCALE)
    saved = json.loads((tmp_path / "weigh.json").read_text())
    assert saved["sides"]["right"]["source"] == "measured"
    assert saved["sides"]["right"]["reference_mass_kg"] == 0.59

    w = rig.engine.weigh({})
    assert w["calibrated"] is True and w["scale_source"] == "measured"
    assert w["mass_kg"] == pytest.approx(0.59, abs=0.005)

    # 重启后仍然有效
    again = weigh.WeighEngine({"state_file": str(tmp_path / "weigh.json")}, log=lambda m: None)
    assert again.scale_info("right")["scale"] == pytest.approx(1.05, abs=0.01)
    assert again.scale_info("right")["calibrated"] is True


def test_calibrate_manual_scale(rig, tmp_path):
    res = rig.engine.calibrate({"scale": 1.02})
    assert res["state"] == "calibrated" and res["scale_source"] == "manual"
    assert rig.engine.scale_info("right")["scale"] == 1.02
    # 手动设定不需要去皮
    assert rig.engine._tare["right"] is None


@pytest.mark.parametrize("args,code", [
    ({}, "bad_arguments"),
    ({"scale": 1.0, "known_mass_kg": 0.5}, "bad_arguments"),
    ({"scale": 1.3}, "scale_out_of_range"),
    ({"scale": "abc"}, "bad_arguments"),
    ({"known_mass_kg": 0.1}, "bad_arguments"),
    ({"known_mass_kg": 5.0}, "bad_arguments"),
    ({"side": "middle", "scale": 1.0}, "bad_side"),
    ({"side": "both", "known_mass_kg": 0.5}, "bad_arguments"),
])
def test_calibrate_rejects_bad_input(rig, tmp_path, args, code):
    res = rig.engine.calibrate(args)
    assert res["state"] == "error" and res["code"] == code
    assert not (tmp_path / "weigh.json").exists()


def test_calibrate_rejects_implausible_scale(rig, tmp_path):
    rig.engine.tare({})
    rig.mass = 0.59
    rig.warm()
    res = rig.engine.calibrate({"known_mass_kg": 0.40})   # 填错重量 → scale≈1.6
    assert res["code"] == "scale_out_of_range"
    assert rig.engine.scale_info("right")["calibrated"] is False
    assert not (tmp_path / "weigh.json").exists()


def test_corrupt_state_file_falls_back_to_default(tmp_path):
    p = tmp_path / "weigh.json"
    p.write_text('{"version":1,"sides":{"right":{"scale":5}}}')
    e = weigh.WeighEngine({"state_file": str(p)}, log=lambda m: None)
    assert e.scale_info("right")["scale"] == pytest.approx(weigh.DEFAULT_SCALE)
    p.write_text("not json")
    e = weigh.WeighEngine({"state_file": str(p)}, log=lambda m: None)
    assert e.scale_info("right")["calibrated"] is False


def test_retare_with_object_on_hand_warns(rig):
    rig.engine.tare({})
    rig.mass = 0.3
    rig.warm()
    res = rig.engine.tare({})
    assert "differs_from_previous_tare" in res["warnings"]


def test_read_is_non_blocking_and_reports_provisional(rig):
    r0 = rig.engine.read({})
    assert r0["tared"] is False and "provisional" not in r0
    rig.engine.tare({})
    rig.mass = 0.5
    rig.warm(1)
    t0 = rig.t
    r = rig.engine.read({})
    assert rig.t == t0
    assert r["tare_valid"] is True
    assert r["provisional"]["mass_kg"] == pytest.approx(0.5, abs=0.02)


def test_disabled_side_is_rejected(tmp_path):
    rig = Rig(tmp_path, sides=["right"])
    rig.warm()
    with pytest.raises(ValueError):
        rig.engine.weigh({"side": "left"})
    with pytest.raises(ValueError):
        rig.engine.weigh({"side": "both"})


# ── 卡片定义与接入 ──────────────────────────────────────────────────────────

class _FakeRos2:
    ctx_tianyi = None
    executor_tianyi = None


def test_tool_schema(tmp_path):
    p = weigh.WeighPlugin({"state_file": str(tmp_path / "w.json")}, "ns", _FakeRos2())
    tool = p.get_tool()
    assert tool["name"] == "weigh"
    schema = tool["inputSchema"]
    actions = schema["properties"]["action"]["enum"]
    assert set(schema["x-action-params"]) == set(actions) == {
        "pose", "tare", "weigh", "calibrate", "read", "retract"}
    for spec in schema["x-action-params"].values():
        assert set(spec["params"]) <= set(schema["properties"])
    assert schema["x-resource"] == ["arm_l", "arm_r", "hand_l", "hand_r"]
    assert schema["x-completion"]["actions"] == ["pose", "retract", "tare", "weigh", "calibrate"]
    assert 60 < schema["x-completion"]["timeout"] <= 180
    assert schema["properties"]["side"]["enum"] == ["right", "left", "both"]
    assert "side" in schema["x-action-params"]["pose"]["params"]
    assert "side" in schema["x-action-params"]["retract"]["params"]
    only_right = weigh.WeighPlugin({"state_file": str(tmp_path / "w.json"), "sides": ["right"]},
                                   "ns", _FakeRos2()).get_tool()["inputSchema"]
    assert only_right["properties"]["side"]["enum"] == ["right"]


def test_plugin_lifecycle_and_unknown_action(tmp_path):
    p = weigh.WeighPlugin({"state_file": str(tmp_path / "w.json")}, "ns", _FakeRos2())
    assert p.dispatch("stop", {}) == {"state": "idle"}
    assert p.dispatch("info", {})["state"] == "idle"
    assert p.dispatch("bogus", {}) == {"error": "unknown action: bogus"}


def test_plugin_never_publishes_commands():
    src = (BUNDLE / "weigh.py").read_text(encoding="utf-8")
    assert "create_publisher" not in src
    assert "cmd_pos" not in src and "CmdSetMotorPosition" not in src


def test_plugin_uses_its_own_executor():
    """共用的 executor_tianyi 已饱和（force_sensor 约 0.2 Hz），本卡不能挂上去。"""
    src = (BUNDLE / "weigh.py").read_text(encoding="utf-8")
    assert "executor_tianyi.add_node" not in src
    assert "SingleThreadedExecutor(context=ctx)" in src
    assert "raw=True" in src


def test_rated_currents_match_device():
    src = (BUNDLE / "device.py").read_text(encoding="utf-8")
    block = re.search(r"_RATED_MOTOR_CURRENT_A = \{(.*?)\}", src, re.S).group(1)
    table = {int(k): float(v) for k, v in re.findall(r"(\d+):\s*([\d.]+)", block)}
    for mid, amps in weigh.RATED_CURRENT_A.items():
        assert table[mid] == amps


def test_bundle_registers_weigh_enabled_by_default():
    main = (BUNDLE / "main.py").read_text(encoding="utf-8")
    assert "from weigh import WeighPlugin" in main
    cfg = (BUNDLE / "config.yaml").read_text(encoding="utf-8")
    block = re.search(r"^  weigh:\n((?:    .*\n|\s*\n)+)", cfg, re.M).group(1)
    assert re.search(r"^    enabled: true", block, re.M)
    docker = (BUNDLE / "Dockerfile").read_text(encoding="utf-8")
    assert re.search(r"^COPY .*\bweigh\.py\b.* /work/$", docker, re.M)


# ── pose / retract ──────────────────────────────────────────────────────────

REST = [0.0] * 7
MID = [5.0, 5.6, 0.0, -90.0, 0.0, 9.0, 0.1]


@pytest.fixture
def hanging(tmp_path):
    r = Rig(tmp_path)
    r.pose = list(REST)
    r.warm()
    return r


def test_pose_from_hanging_raises_forearm_then_turns_palm_up(hanging):
    left_before = list(hanging.left)
    res = hanging.engine.pose({})
    assert res["state"] == "posed", res
    moves = hanging.moves()
    assert [m["right_positions"] for m in moves] == [MID, list(POSE_DEG)]
    # 左臂始终保持当前角度（arm 卡缺省会把左臂送回 0°）
    assert all(m["left_positions"] == left_before for m in moves)
    assert all(m["speed"] == 0.3 for m in moves)
    assert hanging.arm_calls[0] == {"action": "start"}
    assert res["tare"]["state"] == "tared"
    assert max(abs(a - b) for a, b in zip(hanging.pose, POSE_DEG)) < 0.01


def test_full_cycle_pose_weigh_retract(hanging):
    assert hanging.engine.pose({})["state"] == "posed"
    hanging.mass = 0.59
    hanging.warm()
    w = hanging.engine.weigh({})
    assert w["valid"] and w["mass_kg"] == pytest.approx(0.59, abs=0.005)
    r = hanging.engine.retract({})
    assert r["code"] == "object_on_hand"
    hanging.mass = 0.0
    hanging.warm(1)
    n = len(hanging.moves())
    r = hanging.engine.retract({})
    assert r["state"] == "retracted", r
    assert [m["right_positions"] for m in hanging.moves()[n:]] == [MID, REST]
    assert hanging.engine._tare["right"] is None


def test_pose_when_already_posed_does_not_move(rig):
    res = rig.engine.pose({})
    assert res["state"] == "posed" and res["already_in_pose"]
    assert rig.arm_calls == []
    assert res["tare"]["state"] == "tared"


def test_pose_fails_when_automatic_tare_fails(hanging):
    hanging.noise = 1.0                            # 掌心上有东西在晃：去皮不稳定
    res = hanging.engine.pose({"timeout_s": 5})
    assert res["state"] == "error" and res["code"] == "tare_failed", res
    assert res["arm_posed"] is True and res["tare"]["state"] == "error"
    assert hanging.engine._tare["right"] is None
    assert hanging.engine.weigh({})["code"] == "not_tared"


def test_pose_has_no_tare_switch(tmp_path):
    p = weigh.WeighPlugin({"state_file": str(tmp_path / "w.json")}, "ns", _FakeRos2())
    schema = p.get_tool()["inputSchema"]
    assert "tare" not in schema["properties"]
    assert "tare" not in schema["x-action-params"]["pose"]["params"]


def test_pose_refuses_unknown_start_pose(hanging):
    hanging.pose = [40.0, 0, 0, -45.0, 0, 0, 0]
    hanging.warm(1)
    res = hanging.engine.pose({})
    assert res["code"] == "unknown_start_pose"
    assert hanging.arm_calls == []


def test_pose_stops_on_motor_fault_without_retry(hanging):
    hanging.events.append((hanging.t + 2.5, lambda: setattr(hanging, "error", 5)))
    res = hanging.engine.pose({})
    assert res["state"] == "error" and res["code"] == "joint_error"
    assert len(hanging.moves()) == 1          # 不下发第二段，也不发冻结指令


def test_pose_timeout_freezes_at_current_position(hanging):
    hanging.arm_stuck = True
    res = hanging.engine.pose({})
    assert res["code"] == "arrive_timeout"
    moves = hanging.moves()
    assert len(moves) == 2                     # 第一段 + 冻结
    assert moves[1]["right_positions"] == [round(x, 2) for x in REST]


def test_pose_reports_arm_rejection(hanging):
    hanging.arm_reply = {"state": "error", "code": "emergency_stop_active", "error": "estop"}
    res = hanging.engine.pose({})
    assert res["state"] == "error" and res["code"] in ("arm_start_failed", "arm_rejected")


def test_pose_without_arm_card(tmp_path):
    e = weigh.WeighEngine({"state_file": str(tmp_path / "w.json")}, log=lambda m: None)
    e.add_motor_status([(i, 0, 0, 0, 0) for i in (*range(11, 18), *range(21, 28))])
    assert e.pose({})["code"] in ("arm_start_failed", "arm_card_unavailable")


# ── 左手 / both ─────────────────────────────────────────────────────────────

LEFT_POSE = [5.0, -5.6, 0.0, -90.0, 90.0, 9.0, -0.1]
LEFT_MID = [5.0, -5.6, 0.0, -90.0, 0.0, 9.0, -0.1]


def test_left_pose_is_mirror_of_right(hanging):
    right_before = list(hanging.pose)
    res = hanging.engine.pose({"side": "left"})
    assert res["state"] == "posed", res
    moves = hanging.moves()
    assert [m["left_positions"] for m in moves] == [LEFT_MID, LEFT_POSE]
    assert all(m["right_positions"] == right_before for m in moves)   # 右臂不动
    raws = [c for c in hanging.hand_calls if c["action"] == "set_fingers_raw"]
    assert raws[-1]["side"] == "left"
    assert res["tare"]["state"] == "tared" and res["tare"]["side"] == "left"
    assert "warnings" not in res


def test_left_weigh_and_retract(hanging):
    hanging.engine.pose({"side": "left"})
    hanging.mass_left = 0.3
    hanging.warm()
    w = hanging.engine.weigh({"side": "left"})
    assert w["valid"] and w["side"] == "left" and w["mass_kg"] == pytest.approx(0.3, abs=0.005)
    assert hanging.engine.retract({"side": "left"})["code"] == "object_on_hand"
    hanging.mass_left = 0.0
    hanging.warm(1)
    n = len(hanging.moves())
    r = hanging.engine.retract({"side": "left"})
    assert r["state"] == "retracted", r
    assert [m["left_positions"] for m in hanging.moves()[n:]] == [LEFT_MID, [1.0, 2.0, 0.0, -5.0, 0.0, 0.0, 0.0]]


def test_both_pose_moves_arms_together(hanging):
    res = hanging.engine.pose({"side": "both"})
    assert res["state"] == "posed", res
    moves = hanging.moves()
    assert len(moves) == 2                                            # 两段，每段两臂一起
    assert moves[0]["right_positions"] == MID and moves[0]["left_positions"] == LEFT_MID
    assert moves[1]["right_positions"] == list(POSE_DEG) and moves[1]["left_positions"] == LEFT_POSE
    raws = [c for c in hanging.hand_calls if c["action"] == "set_fingers_raw"]
    assert raws[-1]["side"] == "both"
    t = res["tare"]
    assert t["state"] == "tared" and set(t["per_side"]) == {"left", "right"}


def test_both_weigh_sums_hands(hanging):
    hanging.engine.pose({"side": "both"})
    hanging.mass, hanging.mass_left = 0.6, 0.25
    hanging.warm()
    w = hanging.engine.weigh({"side": "both"})
    assert w["state"] == "weighed" and w["valid"], w
    assert w["mass_kg"] == pytest.approx(0.85, abs=0.008)
    assert w["per_side"]["right"]["mass_kg"] == pytest.approx(0.6, abs=0.005)
    assert w["per_side"]["left"]["mass_kg"] == pytest.approx(0.25, abs=0.005)
    assert hanging.engine.read({"side": "both"})["last_result"]["mass_kg"] == w["mass_kg"]


def test_both_invalid_if_one_side_invalid(hanging):
    hanging.engine.pose({"side": "both"})
    hanging.mass, hanging.mass_left = 0.5, 0.5
    hanging.arm_stuck = True          # 假 arm 卡不再把关节拉回目标
    hanging.pose[6] += 2.0            # 右腕被压下 2°
    hanging.warm()
    w = hanging.engine.weigh({"side": "both"})
    assert w["valid"] is False and "right:pose_changed" in w["reasons"]
    assert w["per_side"]["left"]["valid"] is True


def test_both_weigh_requires_both_tared(hanging):
    hanging.engine.pose({"side": "right"})
    res = hanging.engine.weigh({"side": "both"})
    assert res["code"] == "not_tared" and "left" in res["error"]


def test_both_retract_refuses_if_either_hand_loaded(hanging):
    hanging.engine.pose({"side": "both"})
    hanging.mass_left = 0.3
    hanging.warm()
    n = len(hanging.moves())
    res = hanging.engine.retract({"side": "both"})
    assert res["code"] == "object_on_hand" and "left" in res["error"]
    assert len(hanging.moves()) == n


def test_both_calibrate_manual_scale_sets_each_hand(rig):
    res = rig.engine.calibrate({"side": "both", "scale": 1.05})
    assert res["state"] == "calibrated" and set(res["per_side"]) == {"left", "right"}
    assert rig.engine.scale_info("left")["scale"] == 1.05
    assert rig.engine.scale_info("right")["scale"] == 1.05


def test_left_scale_is_independent(rig):
    rig.engine.calibrate({"side": "left", "scale": 1.10})
    assert rig.engine.scale_info("left")["scale"] == 1.10
    assert rig.engine.scale_info("right")["calibrated"] is False


def test_both_pose_refused_if_one_arm_in_unknown_pose(hanging):
    hanging.left = [40.0, 0, 0, -45.0, 0, 0, 0]
    hanging.warm(1)
    res = hanging.engine.pose({"side": "both"})
    assert res["code"] == "unknown_start_pose" and "left" in res["error"]
    assert hanging.arm_calls == []


def test_both_fault_on_left_stops(hanging):
    hanging.events.append((hanging.t + 2.5, lambda: setattr(hanging, "left_error", 7)))
    res = hanging.engine.pose({"side": "both"})
    assert res["code"] == "joint_error" and len(hanging.moves()) == 1


def test_retract_without_tare_needs_confirmation(rig):
    res = rig.engine.retract({})
    assert res["code"] == "load_unknown" and rig.arm_calls == []
    res = rig.engine.retract({"confirm_empty": True})
    assert res["state"] == "retracted"
    assert [m["right_positions"] for m in rig.moves()] == [MID, REST]


# ── 日志 ────────────────────────────────────────────────────────────────────

def test_pose_logs_each_stage(hanging):
    hanging.engine.pose({})
    text = "\n".join(hanging.logs)
    assert "stage raise_forearm: right [0.0,0.0,0.0,0.0,0.0,0.0,0.0] -> [5.0,5.6,0.0,-90.0,0.0,9.0,0.1]" in text
    assert "stage palm_up: arrived in" in text
    assert "left held at [1.0,2.0,0.0,-5.0,0.0,0.0,0.0]" in text


def test_stage_failure_and_freeze_are_logged(hanging):
    hanging.arm_stuck = True
    hanging.engine.pose({})
    text = "\n".join(hanging.logs)
    assert "[weigh] freeze: holding right at" in text
    assert "stage raise_forearm: FAILED arrive_timeout" in text


def test_dispatch_logs_one_summary_line_per_call(rig, capsys, tmp_path):
    p = weigh.WeighPlugin({"state_file": str(tmp_path / "w.json")}, "ns", _FakeRos2())
    p.start = lambda: None
    p._engine = rig.engine
    p._sync_grace_s = 60          # 假时钟跑得快：让这些调用都在宽限期内同步返回
    p.dispatch("tare", {"side": "right", "_tool_name": "weigh"})
    rig.mass = 0.59
    rig.warm()
    res = p.dispatch("weigh", {"side": "right"})
    p.dispatch("read", {"side": "right"})
    p.dispatch("calibrate", {"side": "right", "scale": 2})
    out = capsys.readouterr().out
    lines = [l for l in out.splitlines() if l.startswith("[weigh] ")]
    assert any(l.startswith("[weigh] tare args={'side': 'right'} -> state=tared") for l in lines)
    w = [l for l in lines if l.startswith("[weigh] weigh args={'side': 'right'} -> ")]
    assert w and f"mass_g={res['mass_g']}" in w[0] and "valid=True" in w[0]
    assert any(l.startswith("[weigh] read args={'side': 'right'} -> state=ok") for l in lines)
    assert any("calibrate args={'side': 'right', 'scale': 2} -> state=error code=scale_out_of_range" in l
               for l in lines)
    assert "_tool_name" not in out


def test_brief_is_bounded():
    assert len(weigh._brief({"state": "ok", "error": "x" * 5000})) <= 900


# ── 手型 ────────────────────────────────────────────────────────────────────

CUP = {"little": 65, "ring": 65, "middle": 65, "index": 65, "thumb_bend": 0, "thumb_rotation": 0}
OPEN = {k: 0 for k in CUP}


def _fingers(call):
    return {k: call[k] for k in CUP}


def test_pose_cups_fingers_after_arm_then_tares(hanging):
    res = hanging.engine.pose({})
    assert res["state"] == "posed" and res["hand"] == "cup" and "warnings" not in res
    raws = [c for c in hanging.hand_calls if c["action"] == "set_fingers_raw"]
    assert len(raws) == 1 and raws[0]["side"] == "right" and _fingers(raws[0]) == CUP
    # 手臂两段都做完之后才动手指
    last_arm = max(i for i, o in enumerate(hanging.order) if o == ("arm", "move_pos"))
    first_hand = hanging.order.index(("hand", "set_fingers_raw"))
    assert first_hand > last_arm
    assert res["tare"]["state"] == "tared"
    assert any("hand cup" in l for l in hanging.logs)


def test_pose_already_posed_still_sets_hand_and_retares(rig):
    rig.engine.tare({})
    res = rig.engine.pose({})
    assert res["already_in_pose"] and rig.arm_calls == []
    assert any(c["action"] == "set_fingers_raw" for c in rig.hand_calls)
    assert res["tare"]["state"] == "tared"


def test_pose_hand_keep_and_open(hanging):
    hanging.engine.pose({"hand": "keep"})
    assert hanging.hand_calls == []
    hanging.engine.pose({"hand": "open"})
    raws = [c for c in hanging.hand_calls if c["action"] == "set_fingers_raw"]
    assert _fingers(raws[-1]) == OPEN


def test_hand_failure_fails_pose_without_tare(hanging):
    hanging.hand_reply = {"state": "error", "error": "hand fault"}
    res = hanging.engine.pose({"side": "left"})
    assert res["state"] == "error" and res["code"] == "hand_failed", res
    assert res["hand_problem"] == "hand_start_failed" and res["arm_posed"] is True
    assert "tare" not in res and hanging.engine._tare["left"] is None
    assert hanging.engine.weigh({"side": "left"})["code"] == "not_tared"


def test_pose_without_hand_card(tmp_path):
    r = Rig(tmp_path)
    r.engine._hand_call = None
    r.pose = [0.0] * 7
    r.warm()
    res = r.engine.pose({})
    assert res["code"] == "hand_failed" and res["hand_problem"] == "hand_card_unavailable"
    r2 = Rig(tmp_path)                                            # hand=keep 不需要 hand 卡
    r2.engine._hand_call = None
    r2.pose = [0.0] * 7
    r2.warm()
    res = r2.engine.pose({"hand": "keep"})
    assert res["state"] == "posed" and res["tare"]["state"] == "tared"


def test_pose_rejects_unknown_hand_before_moving(hanging):
    res = hanging.engine.pose({"hand": "fist"})
    assert res["code"] == "bad_hand" and hanging.moves() == []


def test_retract_opens_fingers_before_moving_arm(hanging):
    hanging.engine.pose({})
    hanging.warm(1)
    hanging.order.clear()
    res = hanging.engine.retract({})
    assert res["state"] == "retracted", res
    first_move = hanging.order.index(("arm", "move_pos"))
    open_idx = max(i for i, o in enumerate(hanging.order) if o == ("hand", "set_fingers_raw"))
    assert open_idx < first_move
    raws = [c for c in hanging.hand_calls if c["action"] == "set_fingers_raw"]
    assert _fingers(raws[-1]) == OPEN


def test_retract_refused_with_object_does_not_open_fingers(hanging):
    hanging.engine.pose({})
    hanging.mass = 0.5
    hanging.warm()
    n = len(hanging.hand_calls)
    assert hanging.engine.retract({})["code"] == "object_on_hand"
    assert len(hanging.hand_calls) == n        # 物体还在：不张开手指，免得东西滚落


def test_schema_has_hand_param(tmp_path):
    p = weigh.WeighPlugin({"state_file": str(tmp_path / "w.json")}, "ns", _FakeRos2())
    schema = p.get_tool()["inputSchema"]
    assert schema["properties"]["hand"]["enum"] == ["cup", "open", "keep"]
    assert "hand" in schema["x-action-params"]["pose"]["params"]


# ── side 必填：卡片不替调用方记"当前手" ──────────────────────────────────────

@pytest.mark.parametrize("action", ["pose", "tare", "weigh", "calibrate", "read", "retract"])
def test_side_is_required_at_the_mcp_boundary(rig, tmp_path, action):
    p, calls, fired = _async_plugin(rig, tmp_path)
    n = len(rig.arm_calls) + len(rig.hand_calls)
    res = p.dispatch(action, {})
    assert res["state"] == "error" and res["code"] == "side_required"
    assert len(rig.arm_calls) + len(rig.hand_calls) == n and calls == []


def test_one_callers_pose_does_not_select_the_hand_for_another(hanging, tmp_path):
    p, calls, fired = _async_plugin(hanging, tmp_path, grace=60)
    assert p.dispatch("pose", {"side": "left"})["state"] == "posed"      # 调用方 A
    n = len(hanging.arm_calls)
    assert p.dispatch("retract", {})["code"] == "side_required"          # 调用方 B 没写 side
    assert len(hanging.arm_calls) == n
    r = p.dispatch("read", {"side": "right"})
    assert r["side"] == "right" and "active_side" not in r


def test_schema_requires_side(tmp_path):
    p = weigh.WeighPlugin({"state_file": str(tmp_path / "w.json")}, "ns", _FakeRos2())
    schema = p.get_tool()["inputSchema"]
    assert schema["required"] == ["action", "side"]
    assert "default" not in schema["properties"]["side"]


# ── retract：张不开手指就不动手臂 ───────────────────────────────────────────

@pytest.mark.parametrize("break_hand", ["rejected", "unavailable"])
def test_retract_aborts_when_fingers_fail_to_open(hanging, break_hand):
    hanging.engine.pose({})
    hanging.warm(1)
    n = len(hanging.moves())
    if break_hand == "rejected":
        hanging.hand_reply = {"state": "error", "error": "hand fault"}
    else:
        hanging.engine._hand_call = None
    res = hanging.engine.retract({})
    assert res["state"] == "error" and res["code"] == "hand_open_failed", res
    assert len(hanging.moves()) == n            # 没有任何 move_pos
    assert hanging.engine._tare["right"] is not None   # 状态不变，修好手后可以再 retract


# ── ACP：长动作先返回 action_id，做完回调 ───────────────────────────────────

def _async_plugin(rig, tmp_path, grace=0.0):
    calls, fired = [], threading.Event()

    def notify(action_id, status, result, tool):
        calls.append((action_id, status, result, tool))
        fired.set()
    p = weigh.WeighPlugin({"state_file": str(tmp_path / "w.json")}, "ns", _FakeRos2(),
                          acp_notify=notify)
    p.start = lambda: None
    p._engine = rig.engine
    p._sync_grace_s = grace
    return p, calls, fired


def test_long_action_returns_action_id_then_completes(rig, tmp_path):
    p, calls, fired = _async_plugin(rig, tmp_path)
    rig.engine.tare({})
    rig.mass = 0.59
    rig.warm()
    res = p.dispatch("weigh", {"side": "right"})
    assert res["state"] == "weighing" and res["action_id"].startswith("weigh_weigh_"), res
    assert fired.wait(30)
    action_id, status, result, tool = calls[0]
    assert action_id == res["action_id"] and status == "completed" and tool == "weigh"
    assert result["action"] == "weigh" and result["valid"]
    assert result["mass_kg"] == pytest.approx(0.59, abs=0.005)
    assert len(calls) == 1
    assert p._busy.acquire(blocking=False)        # 完成前已释放
    p._busy.release()


def test_async_error_is_reported_with_error_status(rig, tmp_path):
    p, calls, fired = _async_plugin(rig, tmp_path)
    rig.engine.pose({"hand": "keep"})
    rig.engine._hand_call = None
    res = p.dispatch("retract", {"side": "right"})
    if "action_id" in res:                         # 慢机器上可能超过宽限期
        assert fired.wait(30)
        assert calls[0][1] == "error" and calls[0][2]["code"] == "hand_open_failed"
    else:
        assert res["code"] == "hand_open_failed"


def test_quick_result_stays_synchronous_without_callback(rig, tmp_path):
    p, calls, fired = _async_plugin(rig, tmp_path, grace=30)
    res = p.dispatch("weigh", {"side": "right"})                  # 未去皮：立即失败
    assert res["code"] == "not_tared" and "action_id" not in res
    res = p.dispatch("calibrate", {"side": "right", "scale": 1.05})
    assert res["state"] == "calibrated" and "action_id" not in res
    time.sleep(0.05)
    assert calls == []


def test_busy_while_async_action_runs(rig, tmp_path):
    p, calls, fired = _async_plugin(rig, tmp_path)
    gate = threading.Event()
    real = rig.engine.tare
    rig.engine.tare = lambda args: (gate.wait(10), real(args))[1]
    res = p.dispatch("tare", {"side": "right"})
    assert "action_id" in res
    assert p.dispatch("weigh", {"side": "right"})["code"] == "busy"
    assert p.dispatch("read", {"side": "right"})["state"] == "ok"   # read 不受影响
    gate.set()
    assert fired.wait(30) and calls[0][1] == "completed"


def test_engine_exception_becomes_error_completion(rig, tmp_path):
    p, calls, fired = _async_plugin(rig, tmp_path)
    gate = threading.Event()

    def boom(args):
        gate.wait(10)
        raise RuntimeError("kaput")
    rig.engine.weigh = boom
    res = p.dispatch("weigh", {"side": "right"})
    gate.set()
    assert fired.wait(30)
    assert calls[0][0] == res["action_id"] and calls[0][1] == "error"
    assert calls[0][2]["code"] == "internal_error"


# ── retract 从中间姿态出发也要确认空掌 ─────────────────────────────────────

def test_retract_from_mid_with_load_sends_nothing(hanging):
    hanging.pose = list(MID)                       # 例如上一次 retract 停在了 palm_inward 之后
    hanging.warm()
    assert hanging.engine.tare({})["state"] == "tared"
    hanging.mass = 0.3                             # 有人又把东西放回了手里
    hanging.warm()
    n_hand, n_arm = len(hanging.hand_calls), len(hanging.arm_calls)
    res = hanging.engine.retract({})
    assert res["code"] == "object_on_hand", res
    assert len(hanging.hand_calls) == n_hand and len(hanging.arm_calls) == n_arm


def test_retract_from_mid_with_stale_pose_tare_needs_confirmation(hanging):
    hanging.engine.pose({})                        # 在托举姿态去皮
    hanging.warm(1)
    hanging.pose = list(MID)                       # 之后停在中间姿态：零点不再适用
    hanging.arm_target = list(MID)
    hanging.warm()
    n_hand, n_arm = len(hanging.hand_calls), len(hanging.arm_calls)
    res = hanging.engine.retract({})
    assert res["code"] == "load_unknown" and "pose_changed_since_tare" in res["error"], res
    assert len(hanging.hand_calls) == n_hand and len(hanging.arm_calls) == n_arm
    res = hanging.engine.retract({"confirm_empty": True})
    assert res["state"] == "retracted", res
    assert [m["right_positions"] for m in hanging.moves()[-1:]] == [REST]



# ── stop 取消进行中的动作 ───────────────────────────────────────────────────

def test_stop_cancels_pending_pose_and_sends_nothing_more(hanging, tmp_path):
    p, calls, fired = _async_plugin(hanging, tmp_path)
    p.stop = weigh.WeighPlugin.stop.__get__(p)     # 真的 stop（_async_plugin 只替换了 start）
    entered = threading.Event()
    release = threading.Event()
    real_sleep = hanging.engine._raw_sleep

    def slow_sleep(dt):                             # 第一段运动等待时卡住，模拟 pose 进行中
        entered.set()
        release.wait(0.05)
        real_sleep(dt)
    hanging.engine._raw_sleep = slow_sleep
    res = p.dispatch("pose", {"side": "right"})
    assert "action_id" in res
    assert entered.wait(10)
    n_arm, n_hand = len(hanging.arm_calls), len(hanging.hand_calls)
    assert p.dispatch("stop", {}) == {"state": "idle"}
    assert not p._worker.is_alive()                  # stop 等到动作退出
    assert len(hanging.arm_calls) == n_arm and len(hanging.hand_calls) == n_hand
    assert fired.wait(10)
    assert calls[0][1] == "cancelled" and calls[0][2]["code"] == "cancelled"
    assert hanging.engine._tare["right"] is None
    hanging.engine._raw_sleep = real_sleep          # 之后的新动作不受上一次取消影响
    hanging.warm(1)
    p._sync_grace_s = 60
    assert p.dispatch("read", {"side": "right"})["state"] == "ok"
    assert p.dispatch("calibrate", {"side": "right", "scale": 1.05})["state"] == "calibrated"


# ── servo 指令流：显式动作前先暂停它，动作中被恢复就停下 ──────────────────────

class FakeServo:
    def __init__(self, rig, state="running", pause_ok=True):
        self.rig, self.state, self.pause_ok, self.calls = rig, state, pause_ok, []

    def __call__(self, args):
        self.calls.append(args["action"])
        self.rig.order.append(("servo", args["action"]))
        if args["action"] == "info":
            return {"state": self.state}
        if args["action"] == "pause":
            if not self.pause_ok:
                return {"state": "error", "message": "nope"}
            self.state = "paused"
            return {"state": "paused"}
        return None


def test_pose_pauses_running_servo_before_any_command(hanging):
    servo = FakeServo(hanging)
    hanging.engine._servo_call = servo
    res = hanging.engine.pose({})
    assert res["state"] == "posed" and res["servo_paused"] is True, res
    first_cmd = next(i for i, o in enumerate(hanging.order) if o[0] in ("arm", "hand"))
    assert hanging.order.index(("servo", "pause")) < first_cmd
    assert "resume" not in servo.calls                       # 不自动恢复


def test_pose_refuses_when_servo_cannot_be_paused(hanging):
    hanging.engine._servo_call = FakeServo(hanging, pause_ok=False)
    res = hanging.engine.pose({})
    assert res["code"] == "servo_active", res
    assert hanging.arm_calls == [] and hanging.hand_calls == []


def test_servo_resumed_mid_pose_stops_before_next_stage(hanging):
    servo = FakeServo(hanging, state="idle")
    hanging.engine._servo_call = servo
    hanging.events.append((hanging.t + 2.0, lambda: setattr(servo, "state", "running")))
    res = hanging.engine.pose({})
    assert res["state"] == "error" and res["code"] == "servo_active", res
    assert len(hanging.moves()) == 1                           # 第二段没有下发
    assert not [c for c in hanging.hand_calls if c["action"] == "set_fingers_raw"]


def test_retract_pauses_running_servo_before_opening_fingers(hanging):
    hanging.engine.pose({})
    hanging.warm(1)
    servo = FakeServo(hanging)
    hanging.engine._servo_call = servo
    hanging.order.clear()
    res = hanging.engine.retract({})
    assert res["state"] == "retracted" and res["servo_paused"] is True, res
    assert hanging.order.index(("servo", "pause")) < hanging.order.index(("hand", "set_fingers_raw"))


def test_servo_card_absent_is_fine(hanging):
    hanging.engine._servo_call = lambda args: None               # bundle 里没有 servo 卡
    assert hanging.engine.pose({})["state"] == "posed"


# ── stop：动作卡在 arm 卡调用里时不报 idle，退出后才拆 ─────────────────────────

def test_stop_with_stuck_worker_defers_teardown_and_blocks_new_actions(hanging, tmp_path):
    p, calls, fired = _async_plugin(hanging, tmp_path)
    p.stop = weigh.WeighPlugin.stop.__get__(p)
    torn = []
    p._teardown = lambda: torn.append(True)
    inside, release = threading.Event(), threading.Event()
    real_arm = hanging.engine._arm_call

    def stuck_arm(args):
        if args["action"] == "move_pos" and not release.is_set():
            inside.set()
            release.wait(10)                                    # arm 卡调用卡住
        return real_arm(args)
    hanging.engine._arm_call = stuck_arm
    res = p.dispatch("pose", {"side": "right"})
    assert "action_id" in res and inside.wait(10)
    real_join = p._worker.join
    p._worker.join = lambda timeout=None: real_join(0.2 if timeout else None)   # 测试里别真等 5 s
    r = p.dispatch("stop", {})
    assert r["state"] == "stopping" and torn == []
    assert p.dispatch("weigh", {"side": "right"})["code"] in ("stopping", "busy")
    n = len(hanging.arm_calls)
    release.set()
    assert fired.wait(10) and calls[0][1] == "cancelled"
    deadline = time.time() + 10
    while not torn and time.time() < deadline:
        time.sleep(0.01)
    assert torn == [True] and not p._stopping
    # 只有卡住的那一条 move_pos 最终送达；取消之后没有第二段，也没有冻结指令
    assert [c["action"] for c in hanging.arm_calls[n:]] == ["move_pos"]
    assert not [c for c in hanging.hand_calls if c["action"] == "set_fingers_raw"]


def test_plugin_prefix_and_marketplace_listing():
    assert weigh.WeighPlugin.PREFIX == "weigh"
    manifest = (BUNDLE / "driver.yaml").read_text(encoding="utf-8")
    assert re.search(r"^\s*- \{ name: weigh,\s+type: actuator \}\s*$", manifest, re.M)


def test_freeze_is_skipped_when_servo_is_streaming(hanging):
    hanging.engine._servo_call = FakeServo(hanging, state="running")
    assert hanging.engine._freeze() == {"sent": False, "reason": "servo_active"}
    assert hanging.arm_calls == []


def test_servo_resumed_during_a_stage_stops_waiting_without_commands(hanging):
    servo = FakeServo(hanging, state="idle")
    hanging.engine._servo_call = servo
    t0 = hanging.t
    hanging.events.append((t0 + 1.5, lambda: setattr(servo, "state", "running")))   # 第一段运动中
    res = hanging.engine.pose({})
    assert res["code"] == "servo_active", res
    assert len(hanging.moves()) == 1                       # 没有第二段，也没有冻结
    assert hanging.t - t0 < 4.0                            # 不等到位或超时，立刻退出
    assert not [c for c in hanging.hand_calls if c["action"] == "set_fingers_raw"]
