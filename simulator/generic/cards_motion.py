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
from simulator.generic.geometry import FREE, OCCUPIED, Pose


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


class ControlledSpatialCard(Card):
    """建图与导航 —— 对标 `x-humanoid/tianyi2.0/controlled_spatial.py`。

    仿的是 `controlled_spatial` 而不是 `nav`：真驱动里那份源码自己写着
    「this tool superseded `nav` for actual navigation」，`nav` 是留下来的旧卡。
    对着被取代的那张写，等于把仿真锚在一个没人再用的接口上。

    跟着它一起过来的，是导览场景真正需要的那套词汇：

    * **tag（打点）** 就是航点。场景里的 POI 在载入时变成 tag，`tag_place` 还能在
      跑的过程中就地新增 —— 这正是现场标注展区的做法。
    * **虚拟墙**是 artifact。加一道墙 → 栅格改变 → `laser_scan` 立刻反映，
      障碍物因此有了一个诚实的来源，不必再造一张凭空的「障碍卡」。
    * **`stop_nav`** 是打断的落点，`x-hooks` 绑的也是它。

    受保护操作要密码，`configSchema` 与真卡一致（默认 123456，底盘的固定初始密码，
    不是用户秘密）。仿真照样强制校验 —— 一个不检查密码的仿真，会让人以为真机上
    也不用给。

    没有实现的部分：`start_mapping`/`stop_mapping`（本后端的地图由场景给定，
    没有「边走边建」这回事）、轨道/区域/artifact POI。调用它们会明确报「未实现」，
    而不是假装成功 —— 假装成功的仿真比没有仿真更坏。
    """

    NAME = "controlled_spatial"
    KIND = "actuator"
    DESCRIPTION = (
        "⚠ 受保护操作需要密码：untag_place、add_wall、remove_wall、clear_walls 执行前"
        "必须先向操作者索取密码并通过 password 传入。\n\n"
        "虚拟底盘的建图与导航 —— 打点、列点、按点导航、按坐标导航、停止导航、"
        "虚拟墙增删查、读取位姿与定位质量。"
    )
    TOPIC = ""
    RESOURCES = ["base"]
    HOOKS = {"on_interrupt_motion": {"action": "stop_nav"}}
    COMPLETION = {"actions": ["navigate_to_tag", "navigate_to_pose"], "timeout": 180}
    CONFIG_SCHEMA = {
        "type": "object",
        "properties": {
            "password": {
                "type": "string",
                "description": "受保护操作（打点删除、虚拟墙增删）所需的密码，初始密码为 123456",
                "default": "123456",
                "format": "password",
                # 与真卡同样的判断：底盘的固定初始密码，不是用户秘密。
                "x-sensitive": False,
                "scope": "shared",
            },
        },
    }
    ACTIONS = {
        "navigate_to_tag": (["name"], "前往一个已打点的位置（展区、入口等）"),
        "navigate_to_pose": (["x", "y", "yaw"], "前往地图坐标 (x, y)，到达后转到 yaw 朝向"),
        "stop_nav": ([], "停止当前导航；已走过的进度会随 cancelled 一并上报"),
        "tag_place": (["name", "description"], "把机器人当前位置打成一个点"),
        "untag_place": (["name", "password"], "🔒 删除一个点"),
        "list_tags": ([], "列出当前地图上所有已打的点"),
        "list_maps": ([], "列出可用地图（本仿真中一张地图即一个场景）"),
        "load_map": (["map_name"], "载入一张地图 —— 等价于载入同名场景"),
        "list_walls": ([], "列出虚拟墙"),
        "add_wall": (["x1", "y1", "x2", "y2", "password"], "🔒 加一道虚拟墙；栅格与激光雷达会立刻反映"),
        "remove_wall": (["wall_id", "password"], "🔒 删除一道虚拟墙"),
        "clear_walls": (["password"], "🔒 清空所有虚拟墙"),
        "get_pose": ([], "读取当前位姿"),
        "get_localization_quality": ([], "读取定位质量"),
        "read": ([], "读取导航状态与进度"),
    }
    PROPERTIES = {
        "name": {"type": "string", "description": "点位名称"},
        "description": {"type": "string"},
        "x": {"type": "number"}, "y": {"type": "number"},
        "yaw": {"type": "number", "description": "朝向，弧度"},
        "x1": {"type": "number"}, "y1": {"type": "number"},
        "x2": {"type": "number"}, "y2": {"type": "number"},
        "wall_id": {"type": "string"},
        "map_name": {"type": "string"},
        "password": {"type": "string", "description": "受保护操作的密码"},
    }

    PROTECTED = ("untag_place", "add_wall", "remove_wall", "clear_walls")
    WALL_THICKNESS = 0.2

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._owned: set[str] = set()
        self._walls: dict[str, dict] = {}
        self._map_loader = None
        self._maps_provider = None
        world.add_nav_listener(self._on_terminal)

    # ---- wiring -------------------------------------------------------

    def set_map_hooks(self, loader, maps_provider) -> None:
        """场景卡把「载入地图 = 载入场景」接过来。"""
        self._map_loader = loader
        self._maps_provider = maps_provider

    # ---- ACP ----------------------------------------------------------

    def _on_terminal(self, payload: dict) -> None:
        action_id = payload.get("action_id")
        if action_id not in self._owned:
            return
        self._owned.discard(action_id)
        acp.notify(action_id, payload["status"], payload, tool=self.NAME)

    # ---- 密码 ----------------------------------------------------------

    def _password(self) -> str:
        return str((self.config.get("controlled_spatial") or {}).get("password", "123456"))

    def _denied(self, action: str, password: str) -> dict | None:
        if action not in self.PROTECTED:
            return None
        if str(password) == self._password():
            return None
        # 真卡会拒，仿真也必须拒：一个不检查密码的仿真，会让人以为真机上也不用给。
        return {"error": f"{action} 是受保护操作，需要正确的 password",
                "hint": "向操作者索取密码后通过 password 参数传入"}

    def dispatch(self, action: str, args: dict) -> dict:
        denied = self._denied(action, (args or {}).get("password", ""))
        return denied if denied else super().dispatch(action, args)

    # ---- 导航 ----------------------------------------------------------

    def _accept(self, job) -> dict:
        self._owned.add(job.id)
        return {"state": "running", "action_id": job.id, "target": job.target.as_dict(),
                "label": job.label, "estimated_distance_m": round(job.dist_total, 2)}

    def do_navigate_to_tag(self, name: str = "", **_):
        tag = self.world.tag(name)
        if tag is None:
            return {"error": f"unknown tag: {name}",
                    "known_tags": [t["name"] for t in self.world.tags()]}
        target = Pose(float(tag["x"]), float(tag["y"]), float(tag.get("yaw", 0.0)))
        return self._accept(self.world.submit_job("navigate_to", target, label=name))

    def do_navigate_to_pose(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, **_):
        return self._accept(self.world.submit_job(
            "navigate_to", Pose(float(x), float(y), float(yaw))))

    def do_stop_nav(self, **_):
        payload = self.world.cancel_job("interrupted by user instruction")
        if payload is None:
            return {"state": "idle", "stopped": False, "reason": "nothing in progress"}
        return {"state": "idle", "stopped": True, **payload}

    # ---- 打点 ----------------------------------------------------------

    def do_tag_place(self, name: str = "", description: str = "", **_):
        if not str(name).strip():
            return {"error": "tag_place requires a name"}
        return {"state": "running", "tag": self.world.tag_place(str(name), str(description))}

    def do_untag_place(self, name: str = "", **_):
        if not self.world.untag_place(str(name)):
            return {"error": f"unknown tag: {name}"}
        return {"state": "running", "removed": name}

    def do_list_tags(self, **_):
        return {"state": "running" if self._running else "idle", "tags": self.world.tags()}

    # ---- 地图 ----------------------------------------------------------

    def do_list_maps(self, **_):
        maps = list(self._maps_provider() or []) if self._maps_provider else []
        return {"state": "running" if self._running else "idle", "maps": maps}

    def do_load_map(self, map_name: str = "", **_):
        if self._map_loader is None:
            return {"error": "no map loader wired"}
        return self._map_loader(str(map_name))

    # ---- 虚拟墙 --------------------------------------------------------

    def _grid(self):
        return self.world._backend.state()["grid"]  # noqa: SLF001

    def do_add_wall(self, x1: float = 0.0, y1: float = 0.0, x2: float = 0.0, y2: float = 0.0, **_):
        grid = self._grid()
        half = self.WALL_THICKNESS / 2.0
        wall_id = f"wall-{len(self._walls) + 1}"
        grid.fill_rect(min(x1, x2) - half, min(y1, y2) - half,
                       max(x1, x2) + half, max(y1, y2) + half, OCCUPIED)
        self._walls[wall_id] = {"id": wall_id, "start": {"x": float(x1), "y": float(y1)},
                                "end": {"x": float(x2), "y": float(y2)}}
        self.world.log("add_wall", **self._walls[wall_id])
        # The grid's revision bump is what makes the map card redraw and the
        # laser scan see it — a wall nobody can perceive is not an obstacle.
        return {"state": "running", "wall": self._walls[wall_id], "grid_revision": grid.revision}

    def do_remove_wall(self, wall_id: str = "", **_):
        wall = self._walls.pop(str(wall_id), None)
        if wall is None:
            return {"error": f"unknown wall: {wall_id}"}
        self._erase(wall)
        return {"state": "running", "removed": wall_id}

    def do_clear_walls(self, **_):
        for wall in list(self._walls.values()):
            self._erase(wall)
        count, self._walls = len(self._walls), {}
        return {"state": "running", "cleared": count}

    def _erase(self, wall: dict) -> None:
        grid = self._grid()
        half = self.WALL_THICKNESS / 2.0
        a, b = wall["start"], wall["end"]
        grid.fill_rect(min(a["x"], b["x"]) - half, min(a["y"], b["y"]) - half,
                       max(a["x"], b["x"]) + half, max(a["y"], b["y"]) + half, FREE)
        self.world.log("remove_wall", id=wall["id"])

    def do_list_walls(self, **_):
        return {"state": "running" if self._running else "idle",
                "walls": list(self._walls.values())}

    # ---- 状态 ----------------------------------------------------------

    def do_get_pose(self, **_):
        return {"state": "running" if self._running else "idle",
                "pose": self.world.snapshot()["pose"]}

    def do_get_localization_quality(self, **_):
        # 本后端的位姿是积分出来的，没有配准，所以定位质量恒为满分。诚实地说出
        # 这件事，而不是随机抖一个数字假装有不确定性。
        return {"state": "running" if self._running else "idle",
                "quality": 100, "note": "仿真位姿由积分得到，不存在配准误差"}

    def do_read(self, **_):
        snapshot = self.world.snapshot()
        return {"state": "running" if self._running else "idle",
                "pose": snapshot["pose"], "job": snapshot["job"],
                "tags": [t["name"] for t in self.world.tags()],
                "walls": len(self._walls)}


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
