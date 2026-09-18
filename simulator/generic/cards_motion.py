"""Motion cards: `loco` (direct velocity) and `nav` (waypoint navigation, ACP).

## Why the locomotion card is called `loco`

Not cosmetic. A user barge-in reaches `_interrupt_active_outputs`
(`agent-core/src/event/llm.py`), which fires `on_interrupt_all` and, if nothing
is bound to it, falls back to a hardcoded lookup for tools literally named `tts`
and `loco`:

    for short_name, action in (('tts', 'interrupt'), ('loco', 'stop_move')):

Naming the card `loco` with a `stop_move` action puts it on both paths — the hook
and the fallback — so a barge-in stops this robot whichever way the code goes.
`x-humanoid/tianyi2.0` calls its navigation cards `nav` and `controlled_spatial`,
which neither list catches. `tests/test_sim_interrupt_naming.py` pins the
contrast; the plan's PR description carries the defect report.
"""

from __future__ import annotations

from simulator.generic import acp
from simulator.generic.card_base import Card
from simulator.generic.geometry import Pose, normalize_angle


class LocoCard(Card):
    """Direct base velocity. Synchronous — a velocity command has no completion
    to wait for, so it carries no `x-completion` and never enters the barrier."""

    NAME = "loco"
    KIND = "actuator"
    DESCRIPTION = "虚拟底盘直接速度控制 — 前进/转向/急停"
    TOPIC = ""
    RESOURCES = ["base"]
    HOOKS = {
        "on_interrupt_motion": {"action": "stop_move"},
        # Also on_interrupt_all, so a barge-in stops the base via the hook path
        # and not only via llm.py's hardcoded `loco`/`stop_move` fallback.
        "on_interrupt_all": {"action": "stop_move"},
    }
    ACTIONS = {
        "move": (["lin", "ang"], "以给定线速度(m/s)与角速度(rad/s)行进，直到 stop_move 或下一条指令"),
        "stop_move": ([], "立即停止底盘"),
        "read": ([], "读取当前速度与位姿"),
    }
    PROPERTIES = {
        "lin": {"type": "number", "description": "线速度 m/s，正为前进"},
        "ang": {"type": "number", "description": "角速度 rad/s，正为左转"},
    }

    def do_move(self, lin: float = 0.0, ang: float = 0.0, **_):
        self.world.set_velocity(float(lin), float(ang))
        return {"state": "running", "lin": float(lin), "ang": float(ang)}

    def do_stop_move(self, **_):
        self.world.set_velocity(0.0, 0.0)
        self.world.log("loco_stop")
        return {"state": "idle", "stopped": True}

    def do_read(self, **_):
        snapshot = self.world.snapshot()
        return {"state": "running" if self._running else "idle",
                "pose": snapshot["pose"], "lin": snapshot["lin"], "ang": snapshot["ang"]}


class NavCard(Card):
    """Waypoint navigation. Asynchronous: returns an `action_id` immediately and
    posts the terminal transition to `/api/acp/complete` when the leg ends."""

    NAME = "nav"
    KIND = "actuator"
    DESCRIPTION = "虚拟导航 — 按名称或坐标前往目标点，可中途取消"
    TOPIC = ""
    RESOURCES = ["base"]
    HOOKS = {"on_interrupt_motion": {"action": "cancel"}}
    COMPLETION = {"actions": ["navigate_to", "move_to", "rotate", "rotate_to"], "timeout": 180}
    ACTIONS = {
        "navigate_to": (["name"], "前往一个已命名的航点（展区、入口等）"),
        "move_to": (["x", "y", "yaw"], "前往地图坐标 (x, y)，到达后转到 yaw 朝向"),
        "rotate": (["delta"], "原地相对转动 delta 弧度"),
        "rotate_to": (["yaw"], "原地转到绝对朝向 yaw"),
        "cancel": ([], "取消正在进行的导航；已走过的进度会随 cancelled 一并上报"),
        "list_waypoints": ([], "列出当前地图上所有已命名的航点"),
        "read": ([], "读取当前导航状态与进度"),
    }
    PROPERTIES = {
        "name": {"type": "string", "description": "航点名称"},
        "x": {"type": "number"}, "y": {"type": "number"},
        "yaw": {"type": "number", "description": "朝向，弧度"},
        "delta": {"type": "number", "description": "相对转角，弧度"},
    }

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._waypoints_provider = None
        # Only this card's own jobs. `switch_mode` submits jobs too, and the
        # world emits every terminal to every listener — without the filter,
        # a posture change would be reported to agent-core as a navigation.
        self._owned: set[str] = set()
        world.add_nav_listener(self._on_terminal)

    def set_waypoints_provider(self, fn) -> None:
        self._waypoints_provider = fn

    def waypoints(self) -> list[dict]:
        if self._waypoints_provider is None:
            return []
        try:
            return list(self._waypoints_provider() or [])
        except Exception:
            return []

    # ---- ACP ----------------------------------------------------------

    def _on_terminal(self, payload: dict) -> None:
        """Called once per job by the world, with no lock held."""
        action_id = payload.get("action_id")
        if action_id not in self._owned:
            return
        self._owned.discard(action_id)
        acp.notify(action_id, payload["status"], payload, tool=self.NAME)

    # ---- actions ------------------------------------------------------

    def _accept(self, job) -> dict:
        self._owned.add(job.id)
        # Immediately-returned handle. agent-core registers it as pending and
        # the barrier holds the next actuator call until the callback lands.
        return {"state": "running", "action_id": job.id, "target": job.target.as_dict(),
                "label": job.label, "estimated_distance_m": round(job.dist_total, 2)}

    def do_navigate_to(self, name: str = "", **_):
        match = next((w for w in self.waypoints() if w.get("name") == name), None)
        if match is None:
            known = [w.get("name") for w in self.waypoints()]
            return {"error": f"unknown waypoint: {name}", "known_waypoints": known}
        target = Pose(float(match["x"]), float(match["y"]), float(match.get("yaw", 0.0)))
        return self._accept(self.world.submit_job("navigate_to", target, label=name))

    def do_move_to(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, **_):
        return self._accept(self.world.submit_job("move_to", Pose(float(x), float(y), float(yaw))))

    def do_rotate(self, delta: float = 0.0, **_):
        current = self.world.snapshot()["pose"]["yaw"]
        target = Pose(0.0, 0.0, normalize_angle(current + float(delta)))
        return self._accept(self.world.submit_job("rotate", target))

    def do_rotate_to(self, yaw: float = 0.0, **_):
        return self._accept(self.world.submit_job("rotate_to", Pose(0.0, 0.0, normalize_angle(float(yaw)))))

    def do_cancel(self, **_):
        payload = self.world.cancel_job("interrupted by user instruction")
        if payload is None:
            return {"state": "idle", "cancelled": False, "reason": "nothing in progress"}
        return {"state": "idle", "cancelled": True, **payload}

    def do_list_waypoints(self, **_):
        return {"state": "running" if self._running else "idle",
                "waypoints": [{"name": w.get("name", ""), "x": w["x"], "y": w["y"]}
                              for w in self.waypoints()]}

    def do_read(self, **_):
        snapshot = self.world.snapshot()
        return {"state": "running" if self._running else "idle",
                "pose": snapshot["pose"], "job": snapshot["job"]}


class SwitchModeCard(Card):
    """Posture changes. Present because a real quadruped or humanoid has one and
    the peer role classifier has to cope with its name.

    Deliberately declares **no** interrupt hook: aborting a posture change
    partway is how a controlled descent becomes a fall, which is the same reason
    `llm.py`'s fallback skips `switch_mode`.
    """

    NAME = "switch_mode"
    KIND = "actuator"
    DESCRIPTION = "虚拟姿态切换 — 站立/坐下/阻尼/正常"
    TOPIC = ""
    RESOURCES = ["base"]
    COMPLETION = {"actions": ["stand", "sit"], "timeout": 30}
    ACTIONS = {
        "stand": ([], "站起"), "sit": ([], "坐下"),
        "damp": ([], "进入阻尼态"), "normal": ([], "恢复正常模式"),
        "read": ([], "读取当前模式"),
    }

    DURATIONS = {"stand": 3.0, "sit": 3.0}

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._mode = "normal"
        self._owned: set[str] = set()
        world.add_nav_listener(self._on_terminal)

    def _on_terminal(self, payload: dict) -> None:
        action_id = payload.get("action_id")
        if action_id not in self._owned:
            return
        self._owned.discard(action_id)
        acp.notify(action_id, payload["status"], payload, tool=self.NAME)

    def _posture(self, mode: str):
        self._mode = mode
        # Modelled as a rotate-free job so it takes real simulated time and shows
        # up in the barrier exactly like the real one does.
        job = self.world.submit_job("rotate_to", Pose(0.0, 0.0, self.world.snapshot()["pose"]["yaw"]),
                                    label=mode)
        self._owned.add(job.id)
        self.world.log("switch_mode", mode=mode, action_id=job.id)
        return {"state": "running", "action_id": job.id, "mode": mode}

    def do_stand(self, **_):
        return self._posture("stand")

    def do_sit(self, **_):
        return self._posture("sit")

    def do_damp(self, **_):
        self._mode = "damp"
        self.world.set_velocity(0.0, 0.0)
        return {"state": "idle", "mode": "damp"}

    def do_normal(self, **_):
        self._mode = "normal"
        return {"state": "idle", "mode": "normal"}

    def do_read(self, **_):
        return {"state": "running" if self._running else "idle", "mode": self._mode}


class LedCard(Card):
    """Status light. No physics, no completion — it exists because `led` is one of
    the actuator names no keyword list catches, and the peer role classifier has
    to reach it through `type`, not through its name."""

    NAME = "led"
    KIND = "actuator"
    DESCRIPTION = "虚拟状态灯 — 颜色与呼吸效果"
    TOPIC = ""
    ACTIONS = {
        "set_color": (["color"], "设为指定颜色"),
        "blink": (["color"], "指定颜色闪烁"),
        "off": ([], "熄灭"),
        "read": ([], "读取当前状态"),
    }
    PROPERTIES = {"color": {"type": "string", "description": "颜色名或 #RRGGBB"}}
    HOOKS = {
        "on_thinking": {"action": "blink", "params": {"color": "#4D9EE8"}},
        "on_idle": {"action": "set_color", "params": {"color": "#1C1C1E"}},
    }

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._state = {"color": "#000000", "blinking": False}

    def do_set_color(self, color: str = "#FFFFFF", **_):
        self._state = {"color": color, "blinking": False}
        self.world.log("led", **self._state)
        return {"state": "running", **self._state}

    def do_blink(self, color: str = "#FFFFFF", **_):
        self._state = {"color": color, "blinking": True}
        self.world.log("led", **self._state)
        return {"state": "running", **self._state}

    def do_off(self, **_):
        self._state = {"color": "#000000", "blinking": False}
        return {"state": "idle", **self._state}

    def do_read(self, **_):
        return {"state": "running" if self._running else "idle", **self._state}


class ArmCard(Card):
    """Joint-space arm. Drives the same `JointState` the `servo` card and the VLA
    loop write, so a pose command and a policy chunk are visibly the same thing."""

    NAME = "arm"
    KIND = "actuator"
    DESCRIPTION = "虚拟机械臂 — 关节空间运动，与 servo/VLA 写同一份关节状态"
    TOPIC = ""
    RESOURCES = ["arm"]
    COMPLETION = {"actions": ["move_joints", "home"], "timeout": 60}
    ACTIONS = {
        "move_joints": (["positions"], "移动到给定关节角（弧度），长度须等于 dof"),
        "home": ([], "回到零位"),
        "read": ([], "读取当前关节角"),
    }
    PROPERTIES = {"positions": {"type": "array", "items": {"type": "number"},
                                "description": "目标关节角，弧度"}}

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._dof = int((config or {}).get("embodiment", {}).get("dof", 0))

    def do_move_joints(self, positions=None, **_):
        values = [float(v) for v in (positions or [])]
        if len(values) != self._dof:
            return {"error": f"expected {self._dof} joint values, got {len(values)}"}
        self.world._backend.apply({"joints": values})  # noqa: SLF001
        self.world.log("arm_move", positions=[round(v, 4) for v in values])
        return {"state": "running", "positions": values}

    def do_home(self, **_):
        return self.do_move_joints(positions=[0.0] * self._dof)

    def do_read(self, **_):
        return {"state": "running" if self._running else "idle",
                "positions": self.world.snapshot()["joints"],
                "dof": self._dof}
