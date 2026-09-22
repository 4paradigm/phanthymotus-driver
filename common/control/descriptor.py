"""The action space a driver declares it accepts.

A driver's command card returns this from `info()`. It is the single
authoritative description of the action interface — joint order, units, limits,
rate — and everything upstream negotiates against it before a card is allowed
to start.

Validation is strict and names the offending field. A descriptor is written by
hand once per driver and then drives motors forever; a silent typo in it is not
a bug that shows up as an exception somewhere, it is a robot moving to the
wrong place. `parse_descriptor` therefore rejects rather than defaults.
"""

from __future__ import annotations

from dataclasses import dataclass, field

SCHEMA = "motus.control/1"

# Modes a driver may declare. The set is deliberately small: each one is a
# different contract about what `values` means, and a mode nobody implements is
# a mode nobody has tested.
MODES = (
    "joint_position",    # absolute joint positions, descriptor.units["angle"]
    "joint_velocity",    # joint velocities
    "joint_torque",      # joint torques
    "eef_pose",          # absolute end-effector pose(s), see EEF_POSE_STRIDE
    "twist",             # body twist (vx, vy, vz, wx, wy, wz)
)

# ── `eef_pose` 的布局 ───────────────────────────────────────────────────────
#
# 每个末端占 **7 维**：`[x, y, z, qx, qy, qz, qw]` —— 米，加一个**单位四元数**，
# `descriptor.frame` 下的**绝对**位姿（不是增量）。一段 `eef_pose` 的 count 必须
# 是 7 的整数倍，双臂就是 14 或者两段各 7。
#
# **夹爪不在这里面。** 它是自己的一段 `joint_position`，unit `normalized`。把它
# 塞进位姿段会让这段的宽度变成 8，而 8 % 7 != 0 正是下面那条检查要抓的东西。
#
# **为什么是四元数，而不是 rpy 或者 R6。** 三种都是一串浮点数，接反了、轴序弄错
# 了、旋转矩阵前两列不正交了，都不报错 —— 机械臂走到错误的地方，而日志是干净的。
# 四元数是其中唯一**自带校验**的：单位范数。一个 rpy 三元组或者 R6 的前四个数，
# 没有理由恰好落在单位球面上，于是 `sink._check_contract` 能把它响亮地拒掉。
# 这条检查是选它的全部理由，见 `rotation.UNIT_TOLERANCE`。
#
# 顺序是 **xyzw**，和 `geometry_msgs/Quaternion`、scipy、`motus.vla/1` 一致。
EEF_POSE_STRIDE = 7
EEF_POSE_QUAT = slice(3, 7)   # 一段 7 维里的四元数部分


class DescriptorError(ValueError):
    """A descriptor that cannot be trusted to drive a motor."""


@dataclass(frozen=True)
class Group:
    """A contiguous slice of the action vector with its own unit and resource.

    A single-arm robot needs none of this: one unit, one physical channel, one
    `units` dict covers it. A humanoid does not. Tianyi's action vector is
    fourteen arm joints in radians followed by twelve finger values that are
    normalised closure — one `units` mapping cannot describe both, and the two
    halves are separate physical channels that an ACP barrier should be able to
    tell apart.

    Optional and backward compatible: a descriptor without `groups` is one
    group covering everything, which is what every existing driver means.
    """

    name: str
    offset: int
    count: int
    unit: str = ""
    resource: str = ""
    # 这一段的动作空间。留空表示继承 `descriptor.mode` —— 每一个已有的驱动都是
    # 单一空间，所以不写就是「和整体一样」，不是「未知」。
    #
    # **为什么要按段声明。** 一个末端位姿模型的输出是混的：UnifoLM-VLA 的 G1
    # checkpoint 是 2 × [末端 xyz(3) + R6 旋转(6) + 夹爪(1)] + 腰 rpy(3) = 23 维 ——
    # 前面那些是笛卡尔位姿，腰那三个是关节角。整个 descriptor 只有一个 mode 的话，
    # 这种向量根本声明不出来，于是只能谎报一个，而谎报的后果是位姿被当成关节角。
    mode: str = ""
    # **这一段收下了但不执行。** 默认 false —— 也就是「会执行」，今天每一个驱动的
    # 每一段都是这样。
    #
    # 它是一个**声明**，不是行为开关：`ControlSink` 看见它就跳过这一段的限位、步长
    # 与速度检查（检查一个不会被执行的数没有意义，而卡死的限位会把整条指令拒掉），
    # 驱动自己负责真的不去驱动它。
    #
    # 具体是为 G1 的 1 自由度腰加的：模型输出腰的三个关节角，而那台机器的腰
    # roll/pitch 电机 mode=0。此前卡片把这两维的限位卡死在 ±0.02，于是
    # `unifolm-vla-g1` 的每一条指令都在硬限位处被整条拒掉 —— 25 步全拒，连 IK 都
    # 到不了。
    #
    # **为什么这不是「静默丢两维」**，也就是这个仓库一直拒绝的那件事：丢维要
    # **生产者先给许可**。生产者在 `motus.vla/1` 的
    # `capabilities.control_groups[].optional` 里声明「这一段任务不要求执行」，
    # 两者在 `actucore/plugins/vla/negotiate.py` 相遇，规则只有一句：**驱动标了
    # advisory 而生产者没标 optional → 拒绝协商**。两个名字故意不同，语义不对称，
    # 同名会让一次复制粘贴把「可以不执行」变成「已经没执行」。
    advisory: bool = False

    @property
    def slice(self) -> slice:
        return slice(self.offset, self.offset + self.count)


@dataclass(frozen=True)
class Descriptor:
    """A validated action space declaration.

    Frozen because the sink caches derived values from it; a descriptor that
    changes under a running sink would silently invalidate the limits every
    command has already been checked against.
    """

    mode: str
    dof: int
    joint_names: tuple[str, ...]
    units: dict
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    watchdog_ms: int
    max_hz: float
    expected_hz: float
    max_velocity: tuple[float, ...] | None = None
    max_delta_per_step: tuple[float, ...] | None = None
    max_obs_age_ms: int | None = None
    frame: str = ""
    end_effector: dict | None = None
    # Absolute per-axis force/torque thresholds. `None` means this robot has no
    # force-torque sensing — declared explicitly rather than omitted, so that a
    # missing protection is visible instead of assumed. See sink.ControlSink.
    force_torque: tuple[float, ...] | None = None
    groups: tuple[Group, ...] = ()
    raw: dict = field(default_factory=dict, repr=False)

    @property
    def has_force_torque(self) -> bool:
        return self.force_torque is not None

    @property
    def eef_quat_offsets(self) -> tuple[int, ...]:
        """Index of `qx` for every end-effector pose in the vector.

        Derived from `groups` rather than from `mode`, because a mixed vector
        is the case this exists for: G1's normalised action space is two poses,
        two grippers and three waist joints, and only the pose segments carry
        a quaternion. Callers should compute this once — the sink does, at
        construction — since a descriptor cannot change under a running sink.
        """
        out = []
        for group in self.groups:
            if group.mode != "eef_pose":
                continue
            for start in range(group.offset, group.offset + group.count,
                               EEF_POSE_STRIDE):
                out.append(start + EEF_POSE_QUAT.start)
        return tuple(out)

    @property
    def advisory_indices(self) -> frozenset:
        """Every index the driver has declared it accepts but will not execute.

        Derived once by the sink at construction, the same way
        `eef_quat_offsets` is — a descriptor cannot change under a running
        sink, so recomputing per command would be pure cost.
        """
        out: set = set()
        for group in self.groups:
            if group.advisory:
                out.update(range(group.offset, group.offset + group.count))
        return frozenset(out)

    @property
    def resources(self) -> tuple[str, ...]:
        """Physical channels this action space occupies, for `x-resource`."""
        seen = []
        for group in self.groups:
            if group.resource and group.resource not in seen:
                seen.append(group.resource)
        return tuple(seen)


def _require(source: dict, key: str, where: str):
    if key not in source:
        raise DescriptorError(f"descriptor.{where}{key} is required")
    return source[key]


def _number_list(raw, *, dof: int, where: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)):
        raise DescriptorError(f"descriptor.{where} must be a list of {dof} numbers")
    if len(raw) != dof:
        raise DescriptorError(
            f"descriptor.{where} has {len(raw)} entries but dof is {dof}"
        )
    out = []
    for i, value in enumerate(raw):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DescriptorError(f"descriptor.{where}[{i}] is not a number: {value!r}")
        out.append(float(value))
    return tuple(out)


def parse_descriptor(raw: dict) -> Descriptor:
    """Validate a driver's declaration, or raise `DescriptorError` naming the field."""
    if not isinstance(raw, dict):
        raise DescriptorError("descriptor must be an object")

    interface = raw.get("control_interface")
    if interface != SCHEMA:
        raise DescriptorError(
            f"descriptor.control_interface must be {SCHEMA!r}, got {interface!r}"
        )

    mode = _require(raw, "mode", "")
    if mode not in MODES:
        raise DescriptorError(
            f"descriptor.mode {mode!r} is not one of {', '.join(MODES)}"
        )

    dof = _require(raw, "dof", "")
    if isinstance(dof, bool) or not isinstance(dof, int) or dof <= 0:
        raise DescriptorError(f"descriptor.dof must be a positive integer, got {dof!r}")

    joint_names = _require(raw, "joint_names", "")
    if not isinstance(joint_names, (list, tuple)):
        raise DescriptorError("descriptor.joint_names must be a list")
    if len(joint_names) != dof:
        raise DescriptorError(
            f"descriptor.joint_names has {len(joint_names)} entries but dof is {dof} — "
            "the order of this list is what gives `values` its meaning, so a "
            "mismatch here cannot be worked around downstream"
        )
    if any(not isinstance(n, str) or not n for n in joint_names):
        raise DescriptorError("descriptor.joint_names must be non-empty strings")

    units = _require(raw, "units", "")
    if not isinstance(units, dict) or not units:
        raise DescriptorError("descriptor.units must be a non-empty object")

    limits = _require(raw, "limits", "")
    if not isinstance(limits, dict):
        raise DescriptorError("descriptor.limits must be an object")
    lower = _number_list(_require(limits, "lower", "limits."), dof=dof, where="limits.lower")
    upper = _number_list(_require(limits, "upper", "limits."), dof=dof, where="limits.upper")
    for i, (lo, hi) in enumerate(zip(lower, upper)):
        if lo > hi:
            raise DescriptorError(
                f"descriptor.limits: lower[{i}]={lo} is above upper[{i}]={hi}"
            )

    max_velocity = limits.get("max_velocity")
    if max_velocity is not None:
        max_velocity = _number_list(max_velocity, dof=dof, where="limits.max_velocity")
        if any(v <= 0 for v in max_velocity):
            raise DescriptorError("descriptor.limits.max_velocity must be positive")

    max_delta = limits.get("max_delta_per_step")
    if max_delta is not None:
        max_delta = _number_list(max_delta, dof=dof, where="limits.max_delta_per_step")
        if any(d <= 0 for d in max_delta):
            raise DescriptorError("descriptor.limits.max_delta_per_step must be positive")

    rate = _require(raw, "rate", "")
    if not isinstance(rate, dict):
        raise DescriptorError("descriptor.rate must be an object")
    watchdog_ms = _require(rate, "watchdog_ms", "rate.")
    if isinstance(watchdog_ms, bool) or not isinstance(watchdog_ms, (int, float)) or watchdog_ms <= 0:
        raise DescriptorError("descriptor.rate.watchdog_ms must be a positive number")
    max_hz = float(rate.get("max_hz", 0) or 0)
    expected_hz = float(rate.get("expected_hz", 0) or 0)
    if max_hz and expected_hz and expected_hz > max_hz:
        raise DescriptorError(
            f"descriptor.rate.expected_hz {expected_hz} exceeds max_hz {max_hz}"
        )
    max_obs_age = rate.get("max_obs_age_ms")
    if max_obs_age is not None:
        if isinstance(max_obs_age, bool) or not isinstance(max_obs_age, (int, float)) or max_obs_age <= 0:
            raise DescriptorError("descriptor.rate.max_obs_age_ms must be a positive number")
        max_obs_age = int(max_obs_age)

    # `force_torque` must be present, even as null. Omitting it is how a robot
    # ends up assumed to have a protection it does not have.
    if "force_torque" not in raw:
        raise DescriptorError(
            "descriptor.force_torque is required — declare the per-axis absolute "
            "thresholds, or null if this robot has no force-torque sensing. "
            "Omitting it would let a missing protection pass as an oversight."
        )
    force_torque = raw["force_torque"]
    if force_torque is not None:
        if not isinstance(force_torque, (list, tuple)) or not force_torque:
            raise DescriptorError(
                "descriptor.force_torque must be null or a non-empty list of "
                "per-axis absolute thresholds"
            )
        cleaned = []
        for i, value in enumerate(force_torque):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
                raise DescriptorError(
                    f"descriptor.force_torque[{i}] must be a positive number, got {value!r}"
                )
            cleaned.append(float(value))
        force_torque = tuple(cleaned)

    end_effector = raw.get("end_effector")
    if end_effector is not None and not isinstance(end_effector, dict):
        raise DescriptorError("descriptor.end_effector must be an object or absent")

    groups = _parse_groups(raw.get("groups"), dof=dof, default_mode=mode)

    return Descriptor(
        mode=mode,
        dof=dof,
        joint_names=tuple(joint_names),
        units=dict(units),
        lower=lower,
        upper=upper,
        watchdog_ms=int(watchdog_ms),
        max_hz=max_hz,
        expected_hz=expected_hz,
        max_velocity=max_velocity,
        max_delta_per_step=max_delta,
        max_obs_age_ms=max_obs_age,
        frame=str(raw.get("frame", "") or ""),
        end_effector=end_effector,
        force_torque=force_torque,
        groups=groups,
        raw=dict(raw),
    )


def _parse_groups(raw, *, dof: int, default_mode: str = "") -> tuple:
    """Validate `groups`, or default to one group covering the whole vector.

    Groups must tile `[0, dof)` exactly, in order and without gaps. A gap would
    leave dimensions with no declared unit and no owning channel — and since
    the whole point of a group is to say what a slice *means*, a dimension in
    no group is a dimension nobody has described.

    每一段可以带自己的 `mode`；不带就继承 `default_mode`（即 `descriptor.mode`）。
    见 `Group.mode` —— 混合空间的向量是这个字段存在的唯一理由。
    """
    if raw is None:
        return (Group(name="all", offset=0, count=dof, mode=default_mode),)
    if not isinstance(raw, (list, tuple)) or not raw:
        raise DescriptorError("descriptor.groups must be a non-empty list or absent")

    groups, expected = [], 0
    for i, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise DescriptorError(f"descriptor.groups[{i}] must be an object")
        name = entry.get("name")
        if not isinstance(name, str) or not name:
            raise DescriptorError(f"descriptor.groups[{i}].name is required")
        offset, count = entry.get("offset"), entry.get("count")
        for key, value in (("offset", offset), ("count", count)):
            if isinstance(value, bool) or not isinstance(value, int):
                raise DescriptorError(
                    f"descriptor.groups[{i}].{key} must be an integer")
        if count <= 0:
            raise DescriptorError(f"descriptor.groups[{i}].count must be positive")
        if offset != expected:
            raise DescriptorError(
                f"descriptor.groups[{i}] ({name}) starts at {offset}, expected "
                f"{expected} — groups must tile the vector in order with no gaps"
            )
        expected = offset + count
        mode = str(entry.get("mode") or "") or default_mode
        if mode not in MODES:
            raise DescriptorError(
                f"descriptor.groups[{i}] ({name}) 的 mode {mode!r} 不在 "
                f"{', '.join(MODES)} 里"
            )
        # 一段位姿只能是整数个 7。宽度不对的时候，最可能的解释是旋转用了别的表示
        # —— 6 是 rpy 配 xyz，9 是 R6 配 xyz，8 是把夹爪塞了进来 —— 而这三种都会
        # 在运行时被逐位读成四元数，不报错。
        if mode == "eef_pose" and count % EEF_POSE_STRIDE:
            raise DescriptorError(
                f"descriptor.groups[{i}] ({name}) 声明 eef_pose 但 count 是 "
                f"{count}，不是 {EEF_POSE_STRIDE} 的整数倍 —— 一个末端位姿是 "
                "[x, y, z, qx, qy, qz, qw]。夹爪要单独成一段 joint_position"
            )
        advisory = entry.get("advisory", False)
        if not isinstance(advisory, bool):
            # 字符串 "false" 是真值，而这个字段的方向是**放行**：填错的代价是一段
            # 本该被限位守住的动作变成不检查。所以只收 bool。
            raise DescriptorError(
                f"descriptor.groups[{i}] ({name}).advisory must be true or "
                f"false, got {advisory!r}")
        groups.append(Group(name=name, offset=offset, count=count,
                            unit=str(entry.get("unit") or ""),
                            resource=str(entry.get("resource") or ""),
                            mode=mode, advisory=advisory))

    if expected != dof:
        raise DescriptorError(
            f"descriptor.groups cover {expected} dimensions but dof is {dof}")
    return tuple(groups)
