"""Bumi-style action selector for Q1's documented factory motions.

Hardware RPCs run in a short-lived, isolated vendor-DDS process. Starting the
driver/canvas never sends a motion command. A service acknowledgement is not
reported as physical completion.
"""
import multiprocessing
import threading
import time

from common.vendor_runtime import action_schema, tool


# Q1 SDK v1.1.8.0 and the official Q1 interface manual; not the shared T1 enums.
PRESETS = {
    "wave": (3004, "挥手"),
    "beckon": (3001, "招手"),
    "handshake": (3002, "握手"),
    "bump_fists": (3003, "碰拳"),
    "show_happiness": (3005, "开心"),
    "make_heart": (3007, "双手比心"),
    "cheer": (3008, "加油"),
    "stretch": (3010, "伸懒腰"),
    "speak_a": (3012, "讲话动作 A"),
    "speak_b": (3013, "讲话动作 B"),
    "speak_c": (3014, "讲话动作 C"),
    "hug": (3018, "拥抱"),
    "refuse": (3021, "拒绝"),
    "dance_a": (3046, "上肢舞蹈 A"),
    "dance_b": (3047, "上肢舞蹈 B"),
    "dance_c": (3048, "上肢舞蹈 C"),
}
MOTIONS = {
    "intro_pose7": ("INTRO_POSE7", "亮相动作 7"),
    "boxing": ("BOXING", "拳击"),
    "dance_locking": ("DANCE_LOCKING", "街舞·锁舞"),
    "dance_mj": ("DANCE_MJ", "星际滑步"),
    "dance_go": ("DANCE_GO", "跃动时刻"),
    "happy_pick": ("HAPPY_PICK", "元气蹦蹦"),
    "dance_q": ("DANCE_Q", "奇妙节拍"),
    "show_enter": ("SHOW_ENTER", "闪亮登场"),
    "show_return": ("SHOW_RETURN", "潇洒退场"),
    "run_jump": ("RUN_JUMP", "欢乐巡游"),
    "siuzhuan": ("SIUZHUAN", "进球狂欢"),
    "dream_fly": ("DREAM_FLY", "梦羽轻扬"),
    "dance_babyq": ("DANCE_BABYQ", "星际萌芽"),
    "taiji": ("TAIJI", "行云太极"),
    "explosive_gunfire": ("EXPLOSIVE_GUNFIRE", "爆裂节拍"),
    "xinbaodao": ("XINBAODAO", "新宝岛"),
}
STANDING_MODES = {1002, 1005, 1012}
MIMIC_MODES = {1003, 1018}
MODE_NAMES = {1002: "BIPED_WALK_RUN", 1005: "BIPED_WHOLE_BODY_CTRL"}


class PresetController:
    """One bounded command; transport is injectable for hardware-free tests."""

    def __init__(self, transport, progress=lambda _: None):
        self.transport = transport
        self.progress = progress
        self.sent = {"mode_request_sent": False, "motion_request_sent": False}

    def _issue(self, method, *args):
        self.transport.check()
        field = "mode_request_sent" if method == "set_mode" else "motion_request_sent"
        self.sent[field] = True  # A lost reply must never lead to an automatic retry.
        self.progress(dict(self.sent))
        reply = getattr(self.transport, method)(*args)
        # PresetMotion may acknowledge with RUNNING, and may also reject with
        # RUNNING + nonzero header.code. Both fields must be checked.
        accepted_states = {1} if method == "set_mode" else {1, 400}
        if reply["code"] != 0 or reply["state"] not in accepted_states:
            return {"state": "rejected", "accepted": False, "response": reply,
                    "message": "机器人拒绝请求，请查看返回码和 reason；不自动重试"}
        return None

    @staticmethod
    def _stable(info):
        if info["status"] != 100 or not info["switch_ready"]:
            raise ValueError("运控正在切换或尚未就绪，未发送动作")

    def execute(self, action):
        try:
            return {**self._execute(action), **self.sent}
        except Exception as exc:
            if action == "info":
                return {"state": "unavailable", "current_action": None,
                        "services": self.transport.services(), "message": str(exc), **self.sent}
            return {"state": "unknown" if any(self.sent.values()) else "rejected",
                    "accepted": None if any(self.sent.values()) else False, "execution_confirmed": False,
                    "message": str(exc), **self.sent}

    def _execute(self, action):
        if action not in (*PRESETS, *MOTIONS, "reset", "info"):
            raise ValueError("未知出厂动作")
        t = self.transport
        info = t.query()
        if action == "info":
            standing = t.upright()
            ready = (standing and info["value"] in STANDING_MODES
                     and info["status"] == 100 and info["switch_ready"])
            return {"state": "ready" if ready else "not_ready", "current_action": info,
                    "standing": standing, "services": t.services(),
                    "message": "状态查询，不发送运动指令" if ready else
                               "当前未满足动作前置条件，请先确认机身稳定站立及运控就绪"}
        self._stable(info)
        mode = info["value"]
        if action == "reset":
            if mode == 1005:
                # DEFAULT_POS only resets the upper-body animation; not an estop.
                failure = self._issue("preset", 0, True)
            elif mode in MIMIC_MODES:
                failure = self._issue("set_mode", 1002)
                if failure is None:
                    self._wait_mode(1002, mode)
            elif mode in {1002, 1012}:
                return {"state": "idle", "message": "当前已是走跑模式，无需复位"}
            else:
                raise ValueError("当前模式不支持动作复位；请使用官方控制方式")
            return failure or self._accepted(action, t.last_response)
        if mode not in STANDING_MODES:
            raise ValueError("请先使用官方方式使机器人稳定站立；本卡不自动起身、解锁或退出保护模式")
        if not t.upright():
            raise ValueError("未获得新鲜有效的直立 IMU 数据，未发送动作")
        target = 1005 if action in PRESETS else 1002
        if mode != target:
            failure = self._issue("set_mode", target)
            if failure:
                return failure
            self._wait_mode(target, mode)
        # Recheck immediately before the actual motion, including after a mode change.
        info = t.query()
        self._stable(info)
        if info["value"] != target or not t.upright():
            raise ValueError("动作前模式或机身朝向已变化，取消后续指令")
        if action in PRESETS:
            failure = self._issue("preset", PRESETS[action][0], False)
        else:
            failure = self._issue("motion", MOTIONS[action][0])
        return failure or self._accepted(action, t.last_response)

    def _wait_mode(self, target, original):
        t = self.transport
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            t.pause(.5)
            info = t.query()
            if info["value"] == target and info["status"] == 100 and info["switch_ready"]:
                return
            if info["value"] not in {original, target}:
                raise ValueError("运控进入其他模式，取消后续动作")
        raise TimeoutError("未确认目标模式就绪，未发送后续动作；不自动重试")

    @staticmethod
    def _accepted(action, response):
        return {"state": "accepted", "accepted": True, "action": action,
                "task_id": response["task_id"], "response": response,
                "execution_confirmed": False,
                "message": "机器人已接受请求；该回执不代表物理动作已完成。复位不是急停。"}


class PresetPosturesCard:
    PREFIX = "preset_postures"

    def __init__(self, namespace, ros, robot_domain):
        self.domain = robot_domain
        self._state_lock = threading.Lock()
        self._call_lock = threading.Lock()
        self._enabled = True
        self._active = None
        self._last = None
        self._status = None
        self._status_stamp = None

    def get_tool(self):
        actions = {name: ([], f"{label}。确认稳定站立、双脚着地且肢体活动范围内无人和障碍物。")
                   for name, (_, label) in PRESETS.items()}
        actions.update({name: ([], f"全身动作：{label}。确认稳定站立，并留足平坦防滑的动作空间。")
                        for name, (_, label) in MOTIONS.items()})
        actions.update(reset=([], "复位上肢动画，或请求结束全身动作返回走跑模式；不是急停，可能被固件拒绝。"),
                       status=([], "刷新运控状态和上次请求，不运动。"))
        schema = action_schema(actions, {})
        schema["properties"]["action"]["description"] = "；".join(
            f"{name}={label}" for name, (_, label) in {**PRESETS, **MOTIONS}.items()
        ) + "；reset=动作复位；status=状态查询"
        definition = tool(self.PREFIX, "actuator",
                          "启元 Q1 出厂动作：挥手、握手、比心及全身舞蹈等。自动确认所需运控模式；"
                          "需已稳定站立，开启画布不会运动。执行回执表示请求被接受，不代表动作完成。",
                          schema)
        definition["topic_out"] = []
        return definition

    def start(self):
        with self._state_lock:
            self._enabled = True

    def stop(self):
        with self._state_lock:
            self._enabled = False
            if self._active:
                self._active[0].set()

    def dispatch(self, action, args):
        # Core polls info frequently with a short timeout. It must never wait
        # for DDS discovery or spawn a process; status is an explicit query.
        if action == "info":
            with self._state_lock:
                age = (time.monotonic() - self._status_stamp
                       if self._status_stamp is not None else None)
                return {"state": "ready" if self._enabled else "idle",
                        "status_snapshot": self._status, "status_age_s": age,
                        "status_fresh": age is not None and age <= 5,
                        "last_request": self._last,
                        "message": "选择 status 刷新机身状态；卡片就绪不代表机器人已满足运动条件"}
        if action == "start":
            self.start()
            return {"state": "ready", "message": "等待动作选择；未发送运动指令"}
        if action == "stop":
            self.stop()
            return {"state": "idle", "message": "已取消后续调度；已送达机器人的动作不保证停止，请使用官方急停"}
        if not isinstance(action, str) or action not in (*PRESETS, *MOTIONS, "reset", "status"):
            return None
        action = "info" if action == "status" else action
        if not self._call_lock.acquire(blocking=False):
            return {"state": "busy", "message": "已有请求处理中；本次请求未排队"}
        process = receiver = sender = None
        progress = {"mode_request_sent": False, "motion_request_sent": False}
        try:
            from preset_robot import run_request
            ctx = multiprocessing.get_context("spawn")
            receiver, sender = ctx.Pipe(duplex=False)
            cancelled = ctx.Event()
            process = ctx.Process(target=run_request,
                                  args=(self.domain, action, sender, cancelled, time.monotonic() + 22))
            with self._state_lock:
                if not self._enabled and action != "info":
                    return {"state": "idle", "message": "卡片已停止，请先启动画布"}
                self._active = (cancelled, process)
                process.start()
            sender.close()
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                if receiver.poll(.1):
                    packet = receiver.recv()
                    if packet["event"] == "progress":
                        progress.update(packet["data"])
                    else:
                        result = packet["data"]
                        with self._state_lock:
                            if action == "info":
                                self._status = dict(result)
                                self._status_stamp = time.monotonic()
                                result["last_request"] = self._last
                            else:
                                self._last = dict(result)
                        return result
                if not process.is_alive():
                    raise RuntimeError("动作请求进程退出，结果未确认")
            raise TimeoutError("动作请求超时；不自动重试")
        except (OSError, EOFError, RuntimeError, TimeoutError) as exc:
            result = {"state": "unknown", "execution_confirmed": False,
                      "message": str(exc), **progress}
            with self._state_lock:
                if action != "info":
                    self._last = dict(result)
            return result
        finally:
            with self._state_lock:
                if self._active:
                    self._active[0].set()
                self._active = None
            if process is not None and process.pid:
                process.join(timeout=1)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=2)
            for channel in (receiver, sender):
                if channel is not None:
                    channel.close()
            self._call_lock.release()
