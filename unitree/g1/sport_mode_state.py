"""
sport_mode_state.py — unitree_hg SportModeState_ IDL for G1.

The bundled unitree_sdk2py only ships `unitree_go.msg.dds_.SportModeState_`
(the quadruped layout: mode/gait_type/body_height/imu_state/...). G1 publishes a
*different*, much smaller type on rt/sportmodestate:

    module unitree_hg { module msg { module dds_ {
        struct SportModeState_ {
            unsigned long fsm_id;
            unsigned long fsm_mode;
            unsigned long task_id;
            float task_time;
        };
    }; }; };

DDS matches readers to writers by IDL type name, so subscribing with the
unitree_go type never matches G1's writer — the callback simply never fires.
Reference: https://support.unitree.com/home/zh/G1_developer/sport_services_interface
(§ SportModeState接口, firmware >= 1.5.1)
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class SportModeState_(idl.IdlStruct, typename="unitree_hg.msg.dds_.SportModeState_"):
    fsm_id: types.uint32
    fsm_mode: types.uint32
    task_id: types.uint32
    task_time: types.float32


# ── Official FSM mode IDs ───────────────────────────────────────────────────
# From the vendor doc's 专家接口 § 模式ID说明. "balanced" marks the states that
# run active balance control; the rest are open-loop/position modes that will
# collapse or hold a fixed pose with no balancing.
FSM_MODES: dict[int, dict] = {
    0:   {"name": "zero_torque",   "zh": "零力矩",              "balanced": False},
    1:   {"name": "damp",          "zh": "阻尼",                "balanced": False},
    2:   {"name": "position_squat", "zh": "位控下蹲",           "balanced": False},
    3:   {"name": "position_sit",  "zh": "位控落座",            "balanced": False},
    4:   {"name": "locked_stand",  "zh": "锁定站立",            "balanced": False},
    702: {"name": "lie_to_stand",  "zh": "躺起",                "balanced": True},
    706: {"name": "balance_squat", "zh": "平衡下蹲、蹲起",       "balanced": True},
    500: {"name": "normal_loco",   "zh": "常规运控",            "balanced": True},
    501: {"name": "normal_loco_3dof_waist", "zh": "常规运控-3Dof-waist", "balanced": True},
    801: {"name": "walk_run_loco", "zh": "走跑运控",            "balanced": True},
    # 29-DoF devices renumber 走跑运控 to 802 from ai_sport 8.6.x.x onwards.
    802: {"name": "walk_run_loco", "zh": "走跑运控",            "balanced": True},
}

# 常规运控 / 走跑运控 — upright with active balance, safe to issue velocity commands.
LOCO_STATES = frozenset({500, 501, 801, 802})

# Balanced low poses that connect *only* to LOCO_STATES.
BALANCED_SQUAT = 706   # 平衡下蹲、蹲起 — squat down and stand up are the same ID
LIE_TO_STAND   = 702   # 躺起

# Open-loop, no balance control. Limp (0/1) or holding a fixed pose (2/3/4).
UNBALANCED_STATES = frozenset({0, 1, 2, 3, 4})

# Limp on the ground — the FSM ID alone cannot tell lying from squatting here,
# because both collapse into damping. Posture has to come from joint angles.
LIMP_STATES = frozenset({0, 1})


def fsm_name(fsm_id: int) -> str:
    m = FSM_MODES.get(fsm_id)
    return f"{m['name']} ({m['zh']})" if m else f"unknown({fsm_id})"


def fsm_describe(fsm_id: int) -> str:
    m = FSM_MODES.get(fsm_id)
    if not m:
        return f"unknown FSM id {fsm_id}"
    balance = "active balance control" if m["balanced"] else "NO balance control"
    return f"{m['name']} / {m['zh']} — {balance}"
