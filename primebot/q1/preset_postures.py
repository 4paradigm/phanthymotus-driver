"""Bumi-style action selector for Q1's documented factory motions.

Hardware RPCs reuse an isolated vendor-DDS process. Starting the
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
BASIC_POSTURES = {
    "lie_face_up": (1010, "仰面躺下"),
    "lie_face_down": (1009, "趴下／俯卧"),
    "stand_up": (1007, "自主站起"),
    "store_default": (1001, "折叠收储"),
}
MODE_NAMES = {1002: "BIPED_WALK_RUN", 1005: "BIPED_WHOLE_BODY_CTRL",
              **{number: name.upper() for name, (number, _) in BASIC_POSTURES.items()}}
BASIC_DESCRIPTIONS = {
    "lie_face_up": "仰面躺下（1010）。从稳定站立姿态执行，确认身后和全身落地范围平坦、无障碍。",
    "lie_face_down": "趴下／俯卧（1009）。从稳定站立姿态执行，确认身前和全身落地范围平坦、无障碍。",
    "stand_up": "自主站起（1007）。从地面姿态执行，确认起身范围无障碍；具体姿态和切换条件由官方运控判断。",
    "store_default": "折叠收储（1001）。会改变全身姿态，确认周围及落地范围无障碍；具体起始姿态由官方运控判断。",
}


class PresetController:
    """One bounded command; a successful RPC never implies physical completion."""

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
        if info.get("state_consistent") is False:
            raise ValueError("查询与实时运控状态不一致，可能正在切换；未发送后续动作")
        if info["status"] != 100 or not info["switch_ready"]:
            raise ValueError("运控正在切换或尚未就绪，未发送动作")
        if info.get("is_fallen") or info.get("is_lifted"):
            raise ValueError("运控报告摔倒或被提起，未发送动作")

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
        if action not in (*PRESETS, *MOTIONS, *BASIC_POSTURES, "reset", "info"):
            raise ValueError("未知出厂动作")
        t = self.transport
        if action in BASIC_POSTURES:
            return self._basic_posture(action)
        info = t.query()
        if action == "info":
            posture = t.posture()
            standing = posture["upright"]
            ready = (standing and info["value"] in STANDING_MODES
                     and info["status"] == 100 and info["switch_ready"]
                     and not any(info.get(k) for k in ("is_fallen", "is_lifted", "is_moving")))
            reasons = []
            if standing is None:
                reasons.append("姿态数据缺失或过期，无法判断直立状态")
            elif not standing:
                reasons.append("姿态数据未满足直立条件")
            if info["value"] not in STANDING_MODES:
                reasons.append("当前模式不支持直接播放动作")
            if info["status"] != 100 or not info["switch_ready"]:
                reasons.append("运控未报告切换就绪")
            if any(info.get(k) for k in ("is_fallen", "is_lifted", "is_moving")):
                reasons.append("运控报告摔倒、被提起或仍在移动")
            basic_readiness = {}
            for name in BASIC_POSTURES:
                try:
                    self._basic_guard(name, info, posture)
                    basic_readiness[name] = {"can_request": True,
                                             "message": "本地前置条件满足，最终由官方运控判定是否接受"}
                except ValueError as exc:
                    basic_readiness[name] = {"can_request": False, "message": str(exc)}
            return {"state": "ready" if ready else "not_ready", "current_action": info,
                    "standing": standing, "posture": posture, "services": t.services(),
                    "basic_action_readiness": basic_readiness,
                    "message": "状态查询，不发送运动指令" if ready else
                               "；".join(reasons) + "；基础姿态动作请查看 basic_action_readiness"}
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
        if info.get("is_moving"):
            raise ValueError("运控报告机器人仍在移动，未发送动作")
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
        if info["value"] != target or info.get("is_moving") or not t.upright():
            raise ValueError("动作前模式或机身朝向已变化，取消后续指令")
        if action in PRESETS:
            failure = self._issue("preset", PRESETS[action][0], False)
        else:
            failure = self._issue("motion", MOTIONS[action][0])
        return failure or self._accepted(action, t.last_response)

    @staticmethod
    def _basic_guard(action, info, posture):
        # Recovery from the ground must not inherit the standing-only guard.
        # A fresh native state is required to distinguish being lifted/moving
        # from lying on the ground. The firmware owns its transition matrix.
        if info.get("state_consistent") is not True:
            raise ValueError("缺少与查询一致的新鲜运控状态，未发送基础姿态动作")
        if info["status"] != 100 or not info["switch_ready"]:
            raise ValueError("运控正在切换或尚未就绪，未发送基础姿态动作")
        if info["value"] == 2:
            raise ValueError("当前处于阻尼保护模式，请先使用官方方式处理；本卡不自动解除保护")
        if info.get("is_lifted") is not False or info.get("is_moving") is not False:
            raise ValueError("运控报告被提起、仍在移动或状态未知，未发送基础姿态动作")
        if (posture["upright"] is None or posture.get("age_s") is None
                or not 0 <= posture["age_s"] <= .5):
            raise ValueError("缺少新鲜有效的机身朝向，未发送基础姿态动作")
        if action in {"lie_face_up", "lie_face_down"}:
            if (info["value"] not in STANDING_MODES or not posture["upright"]
                    or info.get("is_fallen") is not False):
                raise ValueError("躺下／趴下需要先稳定站立并处于走跑、全身控制或越障模式")
        elif action == "stand_up" and posture["upright"]:
            raise ValueError("机身已处于直立朝向，无需发送地面起身动作；请确认实际姿态")
        # For stand_up/store_default, is_fallen may be true on the ground.
        # Submit only the explicitly selected action; never force PASSIVE or
        # DAMPING, and never manufacture a recovery sequence in the driver.

    def _basic_posture(self, action):
        t = self.transport
        # Posture acquisition may wait for telemetry; query again afterwards.
        posture = t.posture()
        posture_checked = time.monotonic()
        info = t.query()
        if posture.get("age_s") is not None:
            posture = {**posture, "age_s": posture["age_s"] + time.monotonic() - posture_checked}
        self._basic_guard(action, info, posture)
        target = BASIC_POSTURES[action][0]
        if info["value"] == target:
            return {"state": "idle", "action": action, "execution_confirmed": False,
                    "current_action": info, "message": "已处于该运控模式，未重复发送；不代表物理动作完成"}
        failure = self._issue("set_mode", target)
        if failure:
            return {**failure, "action": action, "target_action": target}
        return {**self._accepted(action, t.last_response), "target_action": target,
                "previous_action": info, "starting_posture": posture,
                "message": "官方运控已接受基础姿态请求；请选择 status 查看当前模式与姿态，并现场确认动作结果。"
                           "请求不会自动重发，也不会自动切换到被动或阻尼模式。"}

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
        self._worker = None
        self._worker_ready = False
        self._worker_started = None

    def get_tool(self):
        actions = {name: ([], description) for name, description in BASIC_DESCRIPTIONS.items()}
        actions.update({name: ([], f"{label}。确认稳定站立、双脚着地且肢体活动范围内无人和障碍物。")
                        for name, (_, label) in PRESETS.items()})
        actions.update({name: ([], f"全身动作：{label}。确认稳定站立，并留足平坦防滑的动作空间。")
                        for name, (_, label) in MOTIONS.items()})
        actions.update(reset=([], "复位上肢动画，或请求结束全身动作返回走跑模式；不是急停，可能被固件拒绝。"),
                       status=([], "刷新运控状态和上次请求，不运动。"))
        schema = action_schema(actions, {})
        schema["properties"]["action"]["description"] = "；".join(
            f"{name}={label}" for name, (_, label) in {**BASIC_POSTURES, **PRESETS, **MOTIONS}.items()
        ) + "；reset=动作复位；status=状态查询"
        definition = tool(self.PREFIX, "actuator",
                          "启元 Q1 出厂动作：躺下、趴下、自主站起、折叠收储、挥手、比心及舞蹈等。"
                          "各动作有独立姿态要求，开启画布不会运动。执行回执表示请求被接受，不代表动作完成。",
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

    def close(self):
        """Release the read-only idle worker on driver/test shutdown."""
        self.stop()
        with self._call_lock:
            self._dispose_worker()

    def _dispose_worker(self):
        if self._worker is None:
            return
        process, channel, cancelled = self._worker
        self._worker = None
        self._worker_ready = False
        cancelled.set()
        try:
            channel.send(None)
        except (BrokenPipeError, OSError):
            pass
        process.join(timeout=1)
        if process.is_alive():
            process.terminate()
            process.join(timeout=2)
        channel.close()

    def _connection(self):
        if self._worker is not None and not self._worker[0].is_alive():
            self._dispose_worker()
        if self._worker is None:
            from preset_robot import run_worker
            ctx = multiprocessing.get_context("spawn")
            channel, peer = ctx.Pipe()
            cancelled = ctx.Event()
            process = ctx.Process(target=run_worker, args=(self.domain, peer, cancelled), daemon=True)
            try:
                process.start()
            except Exception:
                channel.close()
                peer.close()
                raise
            peer.close()
            self._worker = (process, channel, cancelled)
            self._worker_started = time.monotonic()
        return self._worker

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
                        "connection_state": ("ready" if self._worker_ready else
                                             "initializing" if self._worker else "idle"),
                        "last_request": self._last,
                        "message": "选择 status 刷新机身状态；卡片就绪不代表机器人已满足运动条件"}
        if action == "start":
            self.start()
            return {"state": "ready", "message": "等待动作选择；未发送运动指令"}
        if action == "stop":
            self.stop()
            return {"state": "idle", "message": "已取消后续调度；已送达机器人的动作不保证停止，请使用官方急停"}
        if not isinstance(action, str) or action not in (*PRESETS, *MOTIONS, *BASIC_POSTURES, "reset", "status"):
            return None
        action = "info" if action == "status" else action
        if not self._call_lock.acquire(blocking=False):
            return {"state": "busy", "message": "已有请求处理中；本次请求未排队"}
        progress = {"mode_request_sent": False, "motion_request_sent": False}
        try:
            with self._state_lock:
                if not self._enabled and action != "info":
                    return {"state": "idle", "message": "卡片已停止，请先启动画布"}
                process, receiver, cancelled = self._connection()
                if not self._worker_ready:
                    if receiver.poll():
                        packet = receiver.recv()
                        if packet["event"] != "initialized":
                            raise RuntimeError(packet.get("data", {}).get("message", "DDS 初始化失败"))
                        self._worker_ready = True
                    elif time.monotonic() - self._worker_started > 60:
                        raise TimeoutError("DDS 初始化超时；本次未发送或排队任何动作")
                    else:
                        # ROS participant creation can exceed a request timeout
                        # on the busy robot network. Never queue an action while
                        # it initializes; the user must explicitly submit again.
                        return {"state": "initializing", **progress,
                                "execution_confirmed": False,
                                "message": "正在建立机身连接；本次未发送或排队动作，请稍后重新执行"}
                cancelled.clear()
                self._active = (cancelled, process)
                receiver.send((action, time.monotonic() + 22))
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
            self._dispose_worker()
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
            self._call_lock.release()
