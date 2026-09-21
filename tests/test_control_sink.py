"""Every check between a command and a motor, exercised without either.

`ControlSink` is the safety core of the command path: a VLA policy, a
navigation stack or a teleop pendant publishes into it at tens of hertz, and
what it lets through drives actuators. It is deliberately ROS-free and takes an
injected clock so that the whole chain can be tested here — no robot, no GPU,
no DDS, no sleeping.

The eight cases the design calls for, plus the descriptor validation that has
to happen before any of them:

  descriptor mismatch    rejected, apply never called
  ttl expiry / seq       dropped silently, apply never called
  step over-limit        apply sees the *clamped* value, with a warning
  hard limit exceeded    whole command rejected, apply never called
  force-torque over      abort, ahead of whatever else is queued
  watchdog timing        fires at watchdog_ms, not before
  repeated silence       escalates from hold to abort after N periods
  priority arbitration   a low-priority source cannot move anything while a
                         high-priority one is live

Run: python3 -m pytest tests/test_control_sink.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.control import ControlSink, Verdict, parse_descriptor  # noqa: E402
from common.control.descriptor import DescriptorError  # noqa: E402


# ── fixtures ─────────────────────────────────────────────────────────────────

def descriptor_dict(**overrides):
    base = {
        "control_interface": "motus.control/1",
        "mode": "joint_position",
        "dof": 2,
        "joint_names": ["shoulder", "elbow"],
        "units": {"angle": "rad"},
        "limits": {
            "lower": [-1.0, -1.0],
            "upper": [1.0, 1.0],
            "max_delta_per_step": [0.1, 0.1],
        },
        "rate": {"max_hz": 100, "expected_hz": 30, "watchdog_ms": 200},
        "force_torque": None,
    }
    base.update(overrides)
    return base


class FakeClock:
    def __init__(self, now_ms: int = 1_000_000):
        self.now = now_ms

    def __call__(self) -> int:
        return self.now

    def advance(self, ms: int):
        self.now += ms


class Recorder:
    """Stands in for the driver's `apply`."""

    def __init__(self):
        self.calls: list[tuple] = []

    def __call__(self, values, gripper):
        self.calls.append((values, gripper))

    @property
    def last(self):
        return self.calls[-1][0]


def message(clock: FakeClock, values, *, seq=1, source="vla", priority=50,
            ttl_ms=100, **overrides):
    msg = {
        "schema": "motus.control/1",
        "seq": seq,
        "stamp_ms": clock.now,
        "obs_stamp_ms": clock.now,
        "ttl_ms": ttl_ms,
        "source": source,
        "priority": priority,
        "mode": "joint_position",
        "dof": 2,
        "values": list(values),
    }
    msg.update(overrides)
    return msg


def make_sink(clock, apply, descriptor=None, **kwargs):
    """**两个时钟都用同一个假时钟**，而这是一个显式的选择。

    sink 内部用单调时钟计时，用墙钟和消息里的 `stamp_ms` 比 —— 因为那个字段是另一
    个进程按 Unix 纪元打的。这里的用例自己构造消息、stamp 取自同一个 FakeClock，
    所以两边合一才说得通。

    但它必须**写出来**：默认不合一。「只注入 clock 就自动合一」曾经是这个文件的
    行为，而那正是让「sink 拿单调时钟去减 Unix 时间戳」活到真机测试前一刻的原因
    —— 每条用例都在一个真实链路上不成立的前提下跑。
    """
    kwargs.setdefault("wall_clock", clock)
    return ControlSink(
        descriptor or descriptor_dict(), apply, clock=clock, **kwargs
    )


# ── descriptor validation ────────────────────────────────────────────────────

def test_descriptor_round_trips():
    d = parse_descriptor(descriptor_dict())
    assert d.dof == 2
    assert d.joint_names == ("shoulder", "elbow")
    assert d.watchdog_ms == 200
    assert d.has_force_torque is False


@pytest.mark.parametrize("mutate, fragment", [
    (lambda d: d.pop("mode"), "mode"),
    (lambda d: d.update(mode="waltz"), "waltz"),
    (lambda d: d.update(joint_names=["only_one"]), "joint_names"),
    (lambda d: d["limits"].pop("lower"), "limits.lower"),
    (lambda d: d["limits"].update(upper=[1.0]), "dof"),
    (lambda d: d["rate"].pop("watchdog_ms"), "watchdog_ms"),
    (lambda d: d.update(control_interface="motus.control/2"), "control_interface"),
])
def test_descriptor_rejects_and_names_the_field(mutate, fragment):
    raw = descriptor_dict()
    mutate(raw)
    with pytest.raises(DescriptorError) as excinfo:
        parse_descriptor(raw)
    assert fragment in str(excinfo.value)


def test_force_torque_must_be_declared_even_as_null():
    """Omitting it is how a robot is assumed to have a protection it lacks."""
    raw = descriptor_dict()
    del raw["force_torque"]
    with pytest.raises(DescriptorError) as excinfo:
        parse_descriptor(raw)
    assert "force_torque" in str(excinfo.value)


def test_lower_above_upper_is_rejected():
    raw = descriptor_dict()
    raw["limits"]["lower"] = [2.0, -1.0]
    with pytest.raises(DescriptorError):
        parse_descriptor(raw)


# ── 1. contract reconciliation ───────────────────────────────────────────────

@pytest.mark.parametrize("overrides, fragment", [
    ({"dof": 3}, "dof"),
    ({"mode": "twist"}, "mode"),
    ({"schema": "motus.control/2"}, "schema"),
    ({"values": [0.0]}, "values"),
    ({"source": ""}, "source"),
])
def test_contract_mismatch_is_rejected_and_apply_is_not_called(overrides, fragment):
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    msg = message(clock, [0.0, 0.0])
    msg.update(overrides)                        # updated, not passed through
    outcome = sink.submit(msg)

    assert outcome.verdict is Verdict.REJECTED
    assert fragment in outcome.reason
    assert apply.calls == []


def test_dof_mismatch_is_rejected_not_truncated():
    """A changed action space must refuse, not apply the first `dof` values."""
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    outcome = sink.submit(message(clock, [0.1, 0.2, 0.3], dof=3))

    assert outcome.verdict is Verdict.REJECTED
    assert apply.calls == []


# ── 2. freshness ─────────────────────────────────────────────────────────────

def test_expired_command_is_dropped_silently():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)
    msg = message(clock, [0.1, 0.1], ttl_ms=100)

    clock.advance(101)
    outcome = sink.submit(msg)

    assert outcome.verdict is Verdict.DROPPED
    assert "expired" in outcome.reason
    assert apply.calls == []


def test_command_inside_ttl_is_applied():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)
    msg = message(clock, [0.1, 0.1], ttl_ms=100)

    clock.advance(99)
    assert sink.submit(msg).verdict is Verdict.APPLIED
    assert apply.last == (0.1, 0.1)


def test_stale_observation_is_dropped_even_when_the_command_is_fresh():
    """A command generated just now can still be acting on an old picture."""
    clock, apply = FakeClock(), Recorder()
    raw = descriptor_dict()
    raw["rate"]["max_obs_age_ms"] = 150
    sink = make_sink(clock, apply, raw)

    msg = message(clock, [0.1, 0.1])
    msg["obs_stamp_ms"] = clock.now - 400   # freshly sent, stale input

    outcome = sink.submit(msg)

    assert outcome.verdict is Verdict.DROPPED
    assert "observation" in outcome.reason
    assert apply.calls == []


def test_out_of_order_seq_is_dropped():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    assert sink.submit(message(clock, [0.05, 0.05], seq=7)).applied
    outcome = sink.submit(message(clock, [0.0, 0.0], seq=6))

    assert outcome.verdict is Verdict.DROPPED
    assert "seq" in outcome.reason
    assert len(apply.calls) == 1


# ── 4. step clamping ─────────────────────────────────────────────────────────

def test_oversized_step_is_clamped_not_rejected():
    """A clamped point still goes where the policy meant, just more slowly."""
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    sink.submit(message(clock, [0.0, 0.0], seq=1))
    clock.advance(33)
    outcome = sink.submit(message(clock, [0.9, -0.9], seq=2))

    assert outcome.verdict is Verdict.CLAMPED
    assert apply.last == (0.1, -0.1)          # one max_delta_per_step, both ways
    assert outcome.warnings and "clamped" in outcome.warnings[0]


def test_step_within_limit_is_untouched():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    sink.submit(message(clock, [0.0, 0.0], seq=1))
    clock.advance(33)
    outcome = sink.submit(message(clock, [0.05, 0.05], seq=2))

    assert outcome.verdict is Verdict.APPLIED
    assert apply.last == (0.05, 0.05)


# ── 5. hard limits ───────────────────────────────────────────────────────────

def test_out_of_bounds_command_is_rejected_whole_not_clamped_to_the_bound():
    """Clamping to a joint limit invents a trajectory nobody validated."""
    clock, apply = FakeClock(), Recorder()
    raw = descriptor_dict()
    del raw["limits"]["max_delta_per_step"]     # isolate the hard-limit check
    sink = make_sink(clock, apply, raw)

    outcome = sink.submit(message(clock, [5.0, 0.0]))

    assert outcome.verdict is Verdict.REJECTED
    assert "shoulder" in outcome.reason
    assert apply.calls == []                    # not a single point got through


def test_velocity_limit_rejects():
    clock, apply = FakeClock(), Recorder()
    raw = descriptor_dict()
    del raw["limits"]["max_delta_per_step"]
    raw["limits"]["max_velocity"] = [1.0, 1.0]  # rad/s
    sink = make_sink(clock, apply, raw)

    sink.submit(message(clock, [0.0, 0.0], seq=1))
    clock.advance(10)                            # 0.5 rad in 10 ms = 50 rad/s
    outcome = sink.submit(message(clock, [0.5, 0.0], seq=2))

    assert outcome.verdict is Verdict.REJECTED
    assert "would move at" in outcome.reason
    assert len(apply.calls) == 1


# ── 7. force-torque abort ────────────────────────────────────────────────────

def test_force_torque_over_threshold_aborts_and_blocks_further_commands():
    clock, apply = FakeClock(), Recorder()
    aborts = []
    raw = descriptor_dict(force_torque=[10.0, 10.0])
    sink = make_sink(clock, apply, raw, on_abort=lambda: aborts.append(True))

    sink.submit(message(clock, [0.0, 0.0], seq=1))
    outcome = sink.force_torque([2.0, 25.0])

    assert outcome.verdict is Verdict.ABORTED
    assert aborts == [True]
    assert sink.aborted

    clock.advance(10)
    after = sink.submit(message(clock, [0.05, 0.05], seq=2))
    assert after.verdict is Verdict.ABORTED
    assert len(apply.calls) == 1                 # the pre-abort one only


def test_force_torque_is_a_noop_when_the_robot_declares_none():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)               # force_torque: None

    assert sink.force_torque([999.0, 999.0]) is None
    assert not sink.aborted


def test_abort_only_clears_on_explicit_reset():
    """An abort that healed itself would turn a fault into a stutter."""
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply, descriptor_dict(force_torque=[1.0, 1.0]))

    sink.submit(message(clock, [0.0, 0.0], seq=1))
    sink.force_torque([5.0, 0.0])
    assert sink.aborted

    sink.reset()
    assert not sink.aborted
    clock.advance(10)
    assert sink.submit(message(clock, [0.05, 0.05], seq=1)).applied


# ── 9. watchdog timing ───────────────────────────────────────────────────────

def test_watchdog_fires_at_watchdog_ms_and_not_before():
    clock, apply = FakeClock(), Recorder()
    fired = []
    sink = make_sink(clock, apply, on_watchdog=lambda: fired.append(clock.now))

    sink.submit(message(clock, [0.0, 0.0]))

    clock.advance(199)
    assert sink.tick() is None
    assert fired == []

    clock.advance(1)                             # exactly watchdog_ms
    outcome = sink.tick()
    assert outcome is not None and "watchdog" in outcome.reason
    assert len(fired) == 1
    assert sink.holding


def test_watchdog_is_silent_before_the_first_command():
    """A card that has never been used has not promised anything yet."""
    clock, apply = FakeClock(), Recorder()
    fired = []
    sink = make_sink(clock, apply, on_watchdog=lambda: fired.append(True))

    clock.advance(10_000)

    assert sink.tick() is None
    assert fired == []


def test_a_valid_command_clears_the_hold():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    sink.submit(message(clock, [0.0, 0.0], seq=1))
    clock.advance(250)
    sink.tick()
    assert sink.holding

    assert sink.submit(message(clock, [0.05, 0.05], seq=2)).applied
    assert not sink.holding
    assert sink.stats()["watchdog_strikes"] == 0


# ── 10. escalation ───────────────────────────────────────────────────────────

def test_repeated_silence_escalates_from_hold_to_abort():
    """Otherwise the robot holds an arm up forever while the agent waits."""
    clock, apply = FakeClock(), Recorder()
    holds, aborts = [], []
    sink = make_sink(
        clock, apply,
        on_watchdog=lambda: holds.append(True),
        on_abort=lambda: aborts.append(True),
        escalate_after=3,
    )

    sink.submit(message(clock, [0.0, 0.0]))

    outcomes = []
    for _ in range(3):
        clock.advance(200)
        outcomes.append(sink.tick())

    assert [o.verdict for o in outcomes[:2]] == [Verdict.DROPPED, Verdict.DROPPED]
    assert outcomes[2].verdict is Verdict.ABORTED
    assert len(holds) == 2
    assert aborts == [True]
    # Stood down, not aborted: the escalation raises the *response*, it does not
    # take the sink out of service. Only force-torque does that.
    assert not sink.aborted
    assert sink.stats()["stood_down"]


def test_strikes_count_periods_not_ticks():
    """Escalation must not depend on how often the caller happens to tick."""
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply, escalate_after=3)

    sink.submit(message(clock, [0.0, 0.0]))
    clock.advance(200)
    sink.tick()

    for _ in range(50):                          # a busy caller, same period
        clock.advance(1)
        sink.tick()

    assert not sink.aborted
    assert sink.stats()["watchdog_strikes"] == 1


# ── standing down vs aborting ────────────────────────────────────────────────
#
# These two used to be the same state, and the rule that produced contradicted
# itself: four watchdog periods of silence resumed on their own, five needed an
# operator. Measured on Tianyi, restarting the policy card upstream takes longer
# than five periods, so reconfiguring a policy left the driver refusing every
# command until somebody rebuilt the card by hand.

def test_a_stood_down_sink_resumes_on_the_next_good_command():
    """The command is the evidence the silence is over.

    Safe for the same reason an ordinary hold is: freshness means it was
    computed from a recent observation, and the step clamp still holds against
    the last applied values, which survive.
    """
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply, escalate_after=3)
    sink.submit(message(clock, [0.0, 0.0]))

    for _ in range(3):
        clock.advance(200)
        sink.tick()
    assert sink.stats()["stood_down"]

    applied_before = len(apply.calls)
    outcome = sink.submit(message(clock, [0.01, 0.01], seq=2))

    assert outcome.applied
    assert len(apply.calls) == applied_before + 1
    assert not sink.stats()["stood_down"]
    assert not sink.stats()["holding"]


def test_a_stood_down_sink_stops_re_firing_the_abort_handler():
    """A driver whose abort handler returns to a safe pose would otherwise
    re-issue that move every watchdog period, forever."""
    clock, apply = FakeClock(), Recorder()
    aborts = []
    sink = make_sink(clock, apply, on_abort=lambda: aborts.append(True),
                     escalate_after=2)
    sink.submit(message(clock, [0.0, 0.0]))

    for _ in range(8):
        clock.advance(200)
        sink.tick()

    assert aborts == [True]


def test_only_an_applied_command_ends_a_stand_down():
    """A command that is dropped or rejected is not evidence of anything."""
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply, escalate_after=2)
    sink.submit(message(clock, [0.0, 0.0]))
    for _ in range(2):
        clock.advance(200)
        sink.tick()
    assert sink.stats()["stood_down"]

    stale = message(clock, [0.01, 0.01], seq=2, ttl_ms=1)
    clock.advance(50)                       # past its ttl
    assert sink.submit(stale).verdict is Verdict.DROPPED
    assert sink.stats()["stood_down"]


def test_force_torque_still_latches_and_no_command_talks_it_round():
    """The difference that justifies having two states at all: after an impact
    the next command is precisely the one that must not run."""
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply, descriptor_dict(force_torque=[10.0, 10.0]))
    sink.submit(message(clock, [0.0, 0.0]))

    assert sink.force_torque([2.0, 25.0]).verdict is Verdict.ABORTED
    assert sink.aborted
    assert not sink.stats()["stood_down"]

    applied_before = len(apply.calls)
    clock.advance(10)
    assert sink.submit(message(clock, [0.01, 0.01], seq=2)).verdict is Verdict.ABORTED
    assert len(apply.calls) == applied_before
    assert sink.aborted

    sink.reset()
    assert not sink.aborted
    assert not sink.stats()["stood_down"]


# ── 3. priority arbitration ──────────────────────────────────────────────────

def test_low_priority_source_cannot_move_anything_while_a_high_one_is_live():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    assert sink.submit(message(clock, [0.05, 0.05], seq=1,
                               source="teleop", priority=90)).applied
    clock.advance(10)
    outcome = sink.submit(message(clock, [0.0, 0.0], seq=1,
                                  source="vla", priority=50))

    assert outcome.verdict is Verdict.DROPPED
    assert "outranked" in outcome.reason
    assert len(apply.calls) == 1
    assert apply.last == (0.05, 0.05)


def test_the_channel_is_released_once_the_holder_goes_quiet():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    sink.submit(message(clock, [0.05, 0.05], seq=1, source="teleop", priority=90))
    clock.advance(201)                           # longer than watchdog_ms

    outcome = sink.submit(message(clock, [0.1, 0.1], seq=1,
                                  source="vla", priority=50))
    assert outcome.applied


def test_an_outranked_source_does_not_advance_its_own_seq():
    """Or its first command after winning the channel back looks like a replay."""
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    sink.submit(message(clock, [0.05, 0.05], seq=1, source="teleop", priority=90))
    clock.advance(10)
    assert sink.submit(message(clock, [0.0, 0.0], seq=5,
                               source="vla", priority=50)).verdict is Verdict.DROPPED

    clock.advance(201)                           # teleop goes quiet
    outcome = sink.submit(message(clock, [0.1, 0.1], seq=5,
                                  source="vla", priority=50))
    assert outcome.applied


def test_higher_priority_takes_over_immediately():
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    sink.submit(message(clock, [0.0, 0.0], seq=1, source="vla", priority=50))
    clock.advance(10)
    outcome = sink.submit(message(clock, [0.05, 0.05], seq=1,
                                  source="estop_pendant", priority=99))

    assert outcome.applied
    assert sink.stats()["holder"] == "estop_pendant"


# ── counters ─────────────────────────────────────────────────────────────────

def test_counters_separate_dropped_from_rejected():
    """Dropped is the network being a network; rejected is somebody's mistake."""
    clock, apply = FakeClock(), Recorder()
    sink = make_sink(clock, apply)

    sink.submit(message(clock, [0.0, 0.0], seq=1))
    sink.submit(message(clock, [0.0, 0.0], seq=1))          # replay → dropped
    sink.submit(message(clock, [0.0, 0.0], seq=2, dof=9))   # contract → rejected

    counters = sink.stats()["counters"]
    assert counters["applied"] == 1
    assert counters["dropped"] == 1
    assert counters["rejected"] == 1


# ── 按段声明动作空间：一个向量里可以有两种空间 ───────────────────────────────


def test_groups_inherit_the_descriptor_mode_when_they_do_not_declare_one():
    """每一个已有的驱动都是单一空间，所以不写就是「和整体一样」，不是「未知」。

    这条钉的是向后兼容：仓库里现存的每一份 descriptor 都没有按段的 mode，它们
    必须继续解析出和从前一样的东西。
    """
    parsed = parse_descriptor(descriptor_dict())
    assert parsed.groups
    assert all(group.mode == parsed.mode for group in parsed.groups)


def test_a_vector_can_carry_two_action_spaces_at_once():
    """UnifoLM-VLA 的 G1 输出就是混的，这是这个字段存在的唯一理由。

    23 维 = 2 × [末端 xyz(3) + R6 旋转(6) + 夹爪(1)] + 腰 rpy(3)：前面是笛卡尔
    位姿，腰那三个是关节角。整个 descriptor 只有一个 mode 的话，这种向量声明
    不出来 —— 只能谎报一个，而谎报的后果是位姿被当成关节角发给机械臂。
    """
    # 共享的 descriptor_dict() 只有 2 维，撑不下「位姿 + 腰」这个形状，所以这条
    # 用例自己搭一个 —— 被测的是混合声明，不是那份 fixture。位姿那段是 7 维
    # （xyz + 单位四元数 xyzw），规范化之后的标准布局，不是模型原生的 R6。
    raw = descriptor_dict()
    raw["dof"] = 8
    raw["joint_names"] = ["x", "y", "z", "qx", "qy", "qz", "qw", "waist"]
    raw["limits"] = {"lower": [-1.0] * 8, "upper": [1.0] * 8}
    raw["groups"] = [
        {"name": "eef", "offset": 0, "count": 7,
         "unit": "m", "resource": "arm", "mode": "eef_pose"},
        {"name": "waist", "offset": 7, "count": 1,
         "unit": "rad", "resource": "waist"},          # 不写 → 继承 joint_position
    ]
    parsed = parse_descriptor(raw)
    assert [g.mode for g in parsed.groups] == ["eef_pose", parsed.mode]
    # 混合向量里只有位姿那段带四元数，腰不带 —— sink 的三处逐值检查就是靠这个
    # 索引集合避开四元数的。
    assert parsed.eef_quat_offsets == (3,)


def test_a_group_mode_outside_the_vocabulary_is_refused():
    """词汇表小是有意的：每个 mode 都是一份关于 `values` 含义的契约，
    而一个没人实现的 mode 就是一个没人测过的 mode。"""
    raw = descriptor_dict()
    raw["groups"] = [{"name": "all", "offset": 0, "count": raw["dof"],
                      "mode": "eef_r6_g1"}]
    with pytest.raises(Exception) as caught:
        parse_descriptor(raw)
    assert "eef_r6_g1" in str(caught.value)


# ── eef_pose：位姿段的四元数 ─────────────────────────────────────────────────
#
# 这一组测的是「为什么标准布局用四元数而不是 rpy 或 R6」。三种都是一串浮点数，
# 接反了都不报错；四元数是其中唯一自带校验的那一个，而下面第一条就是那份校验。


def eef_descriptor_dict(**overrides):
    """一个末端位姿 + 一个腰关节：混合向量的最小形状。

    `max_delta_per_step` 与 `max_velocity` 在 `qx` 那一格上是**弧度**，整段姿态
    共用它 —— 另外三格不读。见 sink._clamp_step。
    """
    base = descriptor_dict()
    base.update({
        "dof": 8,
        "joint_names": ["x", "y", "z", "qx", "qy", "qz", "qw", "waist"],
        "limits": {
            "lower": [-2.0] * 8,
            "upper": [2.0] * 8,
            "max_delta_per_step": [0.05, 0.05, 0.05, 0.2, 0.2, 0.2, 0.2, 0.1],
            "max_velocity": [1.0, 1.0, 1.0, 6.0, 6.0, 6.0, 6.0, 1.0],
        },
        "groups": [
            {"name": "eef", "offset": 0, "count": 7, "mode": "eef_pose"},
            {"name": "waist", "offset": 7, "count": 1, "mode": "joint_position"},
        ],
    })
    base.update(overrides)
    return base


def eef_message(clock, values, **overrides):
    return message(clock, values, dof=8, **overrides)


def identity_pose(waist=0.0):
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, waist]


def test_a_rotation_that_is_not_a_unit_quaternion_is_rejected():
    """把 rpy 或者 R6 的前四个数填进四元数那四格，范数不会是 1。

    **这是选四元数的全部理由。** 维度对得上、消息校验通过、延迟正常，而机械臂
    会走到错误的地方 —— 除非有一条检查抓得住「这四个数根本不是一个旋转」。
    """
    clock, recorder = FakeClock(), Recorder()
    sink = make_sink(clock, recorder, eef_descriptor_dict())

    # rpy (0.1, 0.2, 0.3) 被当成 quat 的前三个数，第四格是 R6 漏进来的下一个值。
    outcome = sink.submit(eef_message(clock, [0.0, 0.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.0]))
    assert outcome.verdict is Verdict.REJECTED
    assert "unit quaternion" in outcome.reason
    assert recorder.calls == []


def test_a_unit_quaternion_passes_and_a_slightly_off_one_still_does():
    """容差要能容下 JSON 往返和 float32 的动作头，不能容下另一种表示。"""
    clock, recorder = FakeClock(), Recorder()
    sink = make_sink(clock, recorder, eef_descriptor_dict())

    assert sink.submit(eef_message(clock, identity_pose())).applied
    clock.advance(40)
    nearly = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 - 5e-4, 0.0]
    assert sink.submit(eef_message(clock, nearly, seq=2)).applied


def test_the_orientation_step_clamp_keeps_the_quaternion_unit_length():
    """逐分量钳位会破坏单位范数，然后下一条消息在契约检查处被拒。

    症状是「策略一动快就全被拒」，而那条报错一个字都不提钳位。所以姿态走
    slerp 角度钳位，这条用例断言的正是「钳过之后它仍然是个合法四元数」。
    """
    import math

    clock, recorder = FakeClock(), Recorder()
    sink = make_sink(clock, recorder, eef_descriptor_dict())
    assert sink.submit(eef_message(clock, identity_pose())).applied

    # 绕 z 转 1.2 rad，远超每步 0.2 rad 的上限。
    half = 1.2 / 2
    clock.advance(40)
    outcome = sink.submit(eef_message(
        clock,
        [0.0, 0.0, 0.0, 0.0, 0.0, math.sin(half), math.cos(half), 0.0],
        seq=2,
    ))

    assert outcome.verdict is Verdict.CLAMPED
    applied = recorder.calls[-1][0]
    quaternion = applied[3:7]
    assert abs(math.sqrt(sum(v * v for v in quaternion)) - 1.0) < 1e-9
    # 走了 0.2 rad，不是 1.2。
    travelled = 2 * math.acos(min(1.0, abs(quaternion[3])))
    assert abs(travelled - 0.2) < 1e-6


def test_the_opposite_sign_quaternion_is_the_same_orientation_not_a_half_turn():
    """`q` 和 `-q` 是同一个朝向。策略输出翻个号不要钱，而按分量算就是 180°。

    不处理的话，一个静止不动的末端会被判成每步都在极速翻转，然后被限速拒掉。
    """
    clock, recorder = FakeClock(), Recorder()
    sink = make_sink(clock, recorder, eef_descriptor_dict())
    assert sink.submit(eef_message(clock, identity_pose())).applied

    clock.advance(40)
    flipped = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0]
    outcome = sink.submit(eef_message(clock, flipped, seq=2))
    assert outcome.verdict is Verdict.APPLIED      # 没被钳，也没被限速拒


def test_a_quaternion_component_is_not_checked_against_position_bounds():
    """四元数分量的上下界没有物理意义 —— 任何单位四元数四个分量都在 [-1, 1]，
    而同一个朝向可以把它们全取反。真正的边界是手臂的关节限位，那是 IK 的事。"""
    clock, recorder = FakeClock(), Recorder()
    raw = eef_descriptor_dict()
    # 位置那三维收紧到 ±0.01，姿态四维给一个**荒谬的**窄区间：它应当被忽略。
    raw["limits"]["lower"] = [-0.01, -0.01, -0.01, 0.9, 0.9, 0.9, 0.9, -2.0]
    raw["limits"]["upper"] = [0.01, 0.01, 0.01, 0.95, 0.95, 0.95, 0.95, 2.0]
    sink = make_sink(clock, recorder, raw)

    assert sink.submit(eef_message(clock, identity_pose())).applied

    # 而位置的限位照旧生效 —— 步长钳位先把 0.5 削到 0.05，仍然在界外。
    clock.advance(40)
    outcome = sink.submit(eef_message(
        clock, [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], seq=2))
    assert outcome.verdict is Verdict.REJECTED
    assert outcome.reason.startswith("x at ") and "outside" in outcome.reason


def test_an_eef_pose_group_whose_width_is_not_a_multiple_of_seven_is_refused():
    """6 是 rpy 配 xyz，9 是 R6 配 xyz，8 是把夹爪塞了进来 —— 三种都会被逐位
    读成四元数，不报错。宽度是能在解析期就抓住它们的地方。"""
    for count in (3, 6, 8, 9):
        raw = descriptor_dict()
        raw["dof"] = count
        raw["joint_names"] = [f"j{i}" for i in range(count)]
        raw["limits"] = {"lower": [-1.0] * count, "upper": [1.0] * count}
        raw["groups"] = [{"name": "eef", "offset": 0, "count": count,
                          "mode": "eef_pose"}]
        with pytest.raises(DescriptorError) as caught:
            parse_descriptor(raw)
        assert "eef_pose" in str(caught.value)


def test_two_end_effectors_in_one_group_are_two_quaternions():
    """双臂可以是一段 14 维，也可以是两段各 7 维。两种都要找得出两个朝向。"""
    raw = descriptor_dict()
    raw["dof"] = 14
    raw["joint_names"] = [f"j{i}" for i in range(14)]
    raw["limits"] = {"lower": [-1.0] * 14, "upper": [1.0] * 14}
    raw["groups"] = [{"name": "arms", "offset": 0, "count": 14,
                      "mode": "eef_pose"}]
    assert parse_descriptor(raw).eef_quat_offsets == (3, 10)


# ── 新鲜度比的是两个进程之间的时间，所以必须同一个纪元 ──────────────────────


def test_a_wall_clock_stamp_is_actually_compared_against_wall_clock():
    """**这条是整个文件里最重要的一条，因为它此前不存在。**

    消息里的 `stamp_ms` / `obs_stamp_ms` 由**另一个进程**打上，`motus.control/1`
    把它们定义成 Unix 毫秒（`actucore` 发的就是 `int(time.time() * 1000)`）。而
    sink 原本拿 `time.monotonic()` 去减它们 —— 「本机开机至今」减「1970 至今」，
    约 -1.79e12。

    后果不是报错，是整条新鲜度检查静音：`age` 永远是巨大的负数，永远小于任何
    `ttl`。实测过：一条一小时前生成、基于一小时前观测的指令，verdict 是 `applied`。

    这个 bug 能活到真机测试的前一刻，是因为**这个文件里每一条用例都注入 FakeClock**
    —— 注入之后 stamp 和 now 自然同一个纪元，而那正是真实链路上不成立的前提。所以
    这条用例故意**不注入** wall_clock，用真实的墙钟。
    """
    import time as real_time

    clock, recorder = FakeClock(), Recorder()
    # 只注入内部计时的那个时钟；wall_clock 留默认，也就是真的 time.time()。
    sink = ControlSink(descriptor_dict(), recorder, clock=clock)

    wall_now = int(real_time.time() * 1000)
    stale = message(clock, [0.1, 0.1], ttl_ms=100)
    stale["stamp_ms"] = wall_now - 3_600_000        # 一小时前生成
    stale["obs_stamp_ms"] = wall_now - 3_600_000    # 基于一小时前的观测

    outcome = sink.submit(stale)

    assert outcome.verdict is Verdict.DROPPED, "过期指令必须被丢弃"
    assert "expired" in outcome.reason
    assert recorder.calls == [], "陈旧指令绝不能到达 apply"


def test_a_fresh_wall_clock_command_still_passes():
    """修复不能把正常的指令也挡掉 —— 否则整条流都停了。"""
    import time as real_time

    clock, recorder = FakeClock(), Recorder()
    sink = ControlSink(descriptor_dict(), recorder, clock=clock)

    wall_now = int(real_time.time() * 1000)
    fresh = message(clock, [0.1, 0.1], ttl_ms=500)
    fresh["stamp_ms"] = wall_now
    fresh["obs_stamp_ms"] = wall_now

    assert sink.submit(fresh).applied
    assert recorder.last == (0.1, 0.1)


def test_the_two_clocks_are_separate_on_purpose():
    """内部计时仍然用单调时钟：时长测量不该被 NTP 跳变影响。

    跨进程比较需要共同的纪元，时长测量不需要 —— 两件事两个时钟。
    """
    clock, recorder = FakeClock(), Recorder()
    sink = ControlSink(descriptor_dict(), recorder, clock=clock)
    assert sink._clock is clock
    assert sink._wall_clock is not clock
