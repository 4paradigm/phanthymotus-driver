"""PiPER cards: passive status, guarded controls and D435 RGB snapshots.

No connection, enable, motion or camera capture happens at construction/start.
Each physical operation owns one gate shared by the arm and mounted camera.
"""

from contextlib import contextmanager
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import ssl
import sys
import threading
import time
from uuid import uuid4
import urllib.request

from common.vendor_runtime import action_schema, tool
from agilex.piper.arm import PiperDriver
from agilex.piper.control import Previews, healthy, mode_name, mode_plan, ready_to_move, signature, summary, teaching_stopped


def number(args, key, default, low, high, integer=False):
    value = args.get(key, default)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high
            or (integer and not isinstance(value, int))):
        raise ValueError(f"{key} must be {'an integer' if integer else 'a number'} in [{low}, {high}]")
    return value


def confirm(args, key):
    if args.get(key) is not True:
        raise ValueError(f"{key}=true requires the operator's current physical confirmation")


def limits(value, count, label):
    if (not isinstance(value, (list,tuple)) or len(value) != count
            or any(not isinstance(pair,(list,tuple)) or len(pair) != 2
                   or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in pair)
                   or pair[0] >= pair[1] for pair in value)):
        raise ValueError(f"{label} 必须是 {count} 组有限的 [下限, 上限]")
    return value


class PiperRuntime:
    def __init__(self, config, namespace="piper", ros2=None, driver_factory=PiperDriver):
        self.config = config
        if config.get("joint_limits_deg") is not None:
            limits(config["joint_limits_deg"], 6, "joint_limits_deg")
        limits([config.get("gripper_limits_mm", [0.,70.])], 1, "gripper_limits_mm")
        if type(config.get("async_actions",True)) is not bool:
            raise ValueError("async_actions must be a boolean")
        verified = config.get("mode",{}).get("verified_teaching_firmwares",[])
        if not isinstance(verified,list) or any(not isinstance(v,str) or len(v)!=8 or not v.startswith("S-V") for v in verified):
            raise ValueError("verified_teaching_firmwares 必须是 SDK 返回的完整固件字符串列表")
        self.factory = driver_factory
        self.driver = None
        self.ros2 = ros2
        self.topic = f"/{namespace}/piper/rgb"
        self.data_dir = Path(config.get("data_dir", "/opt/phanthy-motus/data/piper"))
        self.gate = threading.Lock()
        self.lifecycle = threading.Lock()
        self.connection_lock = threading.RLock()
        self.owner_file = None
        self.cancel = threading.Event()
        self.closed = False
        self.stopped_cards = set()
        self.active_card = None
        self.active_object = None
        self.previews = Previews()
        self.last_signature = None
        self.firmware_version = None
        self.worker_context = threading.local()
        self.last_actions = {}
        self.node = None
        self.publisher = None
        self.last_image = None
        self.motion_enabled = os.environ.get("PIPER_MOTION_ENABLED", "0") == "1"
        self.manual_motion_permission = False
        self.camera_enabled = os.environ.get("PIPER_CAMERA_ENABLED", "0") == "1"

    def get_tools(self):
        actions = {
            "start": ([], "Ready only; never enables motors"),
            "stop": ([], "Cancel current operation; best-effort hold, never release torque"),
            "info": ([], "Driver configuration and busy state"),
            "prepare": (["execute", "preview_id", "workspace_clear", "allow_limit_adjustment", "speed_pct"],
                        "Preview first; explicit execution enables at measured pose"),
            "disable": (["arm_supported"], "Release all six motors only with physical support"),
            "move_joint": (["joint", "target_deg", "speed_deg_s", "speed_pct", "execute", "preview_id", "workspace_clear", "take_control"], "单轴绝对角度（度）；先预览，再执行"),
            "hold": (["workspace_clear", "take_control"], "保持当前反馈姿态；不失能"),
        }
        flags = {key: {"type": "boolean", "enum": [False, True], "default": False} for key in (
            "execute", "workspace_clear", "allow_limit_adjustment", "take_control", "arm_supported", "confirm_save",
            "other_controllers_stopped", "object_supported", "query_firmware")}
        schema = action_schema(actions, {**{key: value for key, value in flags.items() if key != "confirm_save"},
            "target_deg": {"type": "number", "minimum": -180, "maximum": 180},
            "speed_pct": {"type": "integer", "minimum": 1, "maximum": 10, "default": 5},
            "speed_deg_s": {"type": "number", "minimum": .5, "maximum": 5, "default": 1},
            # Canvas serializes enum selects as strings, but numeric inputs as
            # JSON numbers. Keep this an integer input so strict validation works.
            "joint": {"type": "integer", "minimum": 1, "maximum": 6, "default": 1},
            "preview_id": {"type": "string", "description": "预览返回的编号；留空使用最近一次预览"},
        })
        schema["additionalProperties"] = False
        if self.config.get("can_transport", "socketcan") == "socketcan":
            # Canvas reads this enum; Agent Core advertises only x-action-params
            # to the model. Permission changes are manual card operations.
            # This is catalog separation, not authentication of the MCP caller.
            schema["properties"]["action"]["enum"] += ["allow_motion", "block_motion"]
        schema["x-resource"] = "arm"
        schema["x-is-dangerous"] = True
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        schema["x-completion"] = {"actions": ["prepare", "move_joint"], "timeout": 60}
        camera_schema = action_schema({
            "start": ([], "Ready; no capture or arm motion"),
            "stop": ([], "Cancel in-flight operation"),
            "info": ([], "Camera output topic"),
            "capture": (["capture_confirmed"],
                        "拍摄当前相机 RGB 画面一张，保存并显示；确认本次拍照，不移动机械臂"),
            "import_photo": (["filename"], "保存并显示用户提供的照片；只读取数据目录 imports 下的文件，不调用相机"),
        }, {"capture_confirmed": {"type": "boolean", "enum": [False, True], "default": False,
             "description": "确认允许本次拍摄当前画面；不调整机械臂或相机朝向"},
            "forward_view_confirmed": {"type": "boolean", "enum": [False, True], "default": False,
             "description": "旧版可选朝向记录；不能代替本次 capture_confirmed 拍照确认"},
            "filename": {"type": "string", "description": "imports 目录中的 JPEG/PNG 文件名，不含路径"}})
        camera_schema["additionalProperties"] = False
        # Camera is mounted on the wrist: prevent capture during an arm move.
        camera_schema["x-resource"] = ["arm", "camera"]
        camera_schema["x-completion"] = {"actions": ["capture", "import_photo"], "timeout": 20}
        mode_schema = action_schema({
            "start": ([], "就绪；不切换模式"), "stop": ([], "取消本卡片的切换等待；不失能"),
            "info": ([], "模式卡片信息"),
            "read": (["query_firmware"], "读取模式；query_firmware=true 重新查询实际固件（只发送信息查询）"),
            "preview": (["target_mode", "workspace_clear", "arm_supported", "other_controllers_stopped"], "检查切换条件和简短提醒，不发控制命令"),
            "switch": (["target_mode", "preview_id", "workspace_clear", "arm_supported", "other_controllers_stopped"], "执行有效预览；未验证的示教固件只提供按键引导"),
            "stop_teaching": (["arm_supported"], "停止录制或暂停回放；不要求停放位，不接管控制"),
        }, {**flags, "target_mode": {"type": "string", "enum": ["standby", "can", "teaching"]},
            "preview_id": {"type": "string"}})
        gripper_schema = action_schema({
            "start": ([], "就绪；不使能夹爪"), "stop": ([], "取消夹爪操作并尽力保持，不影响机械臂动作"),
            "info": ([], "夹爪卡片信息"), "read": ([], "读取夹爪反馈"),
            **{a: (["width_mm", "torque_nm", "execute", "preview_id", "workspace_clear", "take_control"], d)
               for a,d in (("open", "打开至限位交集上限；先预览"), ("close", "关闭至限位交集下限；先预览"),
                           ("set_width", "指定开度（毫米）；先预览"))},
            "hold": (["workspace_clear", "take_control"], "保持当前开度，不失能"),
            "disable": (["object_supported"], "确认夹持物已支撑后解除夹爪使能"),
        }, {**flags, "width_mm": {"type": "number", "minimum": 0, "maximum": 100},
            "torque_nm": {"type": "number", "minimum": .001, "maximum": 1, "default": .5},
            "preview_id": {"type": "string"}})
        for guarded in (mode_schema, gripper_schema):
            guarded.update({"additionalProperties": False, "x-resource": "arm", "x-is-dangerous": True,
                            "x-hooks": {"on_interrupt_motion": {"action": "stop"}}})
        gripper_schema["x-completion"] = {"actions": ["open", "close", "set_width"], "timeout": 10}
        return [
            tool("piper_status", "resource", "读取关节、夹爪、模式、使能与故障；不发送控制命令",
                 action_schema({
                     "read": ([], "Read fresh joint angles and motor status without moving"),
                     "start": ([], "Ready only; no hardware connection"),
                     "stop": ([], "Status card lifecycle only; no motor commands"),
                     "info": ([], "Status card readiness; no hardware connection"),
                     "reconnect": ([], "重新连接，废弃预览；不使能、不重放动作"),
                 }, {})),
            tool("piper_mode", "actuator", "模式检查与切换；使能、失能、故障分别处理", mode_schema),
            tool("piper_arm", "actuator", "初始化、单轴低速运动与保持；运动权限默认关闭", schema),
            tool("piper_gripper", "actuator", "夹爪开度与保持；默认力矩 0.5 N·m，上限 1 N·m", gripper_schema),
            tool("piper_photo", "actuator", "导入用户照片并显示，或显式拍摄 RGB；不移动机械臂",
                 camera_schema, topic_out=[{"topic": self.topic, "format": "image/jpeg"}]),
        ]

    def start(self, name=None):
        with self.lifecycle:
            if self.gate.locked() and (name is None or name == self.active_card):
                raise RuntimeError("An operation is still stopping; retry start after it completes")
            self.closed = False
            if name is None:
                self.stopped_cards.clear()
            else:
                self.stopped_cards.discard(name)

    def request_stop(self, name=None):
        with self.lifecycle:
            if name is None:
                self.closed = True
            else:
                self.stopped_cards.add(name)
            if name is None or name in {"arm_control", "piper_arm", "piper_joint", "piper_permission"}:
                self.revoke_manual_permission()
            if name not in {"piper_photo", "piper_status"}:
                self.previews.invalidate()
            affected = self.gate.locked() and (name is None or name == self.active_card)
            if affected:
                self.cancel.set()
                self.previews.invalidate()
                if self.driver is not None and self.active_card != "piper_photo":
                    self.driver.cancel_motion()
        return {"state": "idle", "operation_stopping": affected,
                "motor_torque_released": False,
                "note": "Best-effort hold only; not an emergency stop. CAN loss can prevent holding."}

    def stop(self):
        self.request_stop()
        # Do not close the bus until the action's error/hold path is finished.
        if self.gate.acquire(timeout=2):
            try:
                self.disconnect()
                if self.node is not None:
                    self.ros2.executor_core.remove_node(self.node)
                    self.node.destroy_node()
                    self.node = self.publisher = self.last_image = None
            finally:
                self.gate.release()

    @contextmanager
    def operation(self, name="piper_arm", action=None):
        public_call = getattr(self.worker_context, "public_call", None)
        if public_call is not None:
            name, action = public_call
        if (getattr(self.worker_context, "reserved", None) == (name, action)
                or getattr(self.worker_context, "owned", None) == (name, action)):
            if self.cancel.is_set():
                raise RuntimeError("Operation cancelled")
            yield
            return
        with self.lifecycle:
            if self.closed or name in self.stopped_cards:
                raise RuntimeError("卡片已停止；请先启动卡片再操作")
            if not self.gate.acquire(blocking=False):
                raise RuntimeError("机械臂或相机正在执行操作；请等待完成或使用「停止动作」，不会排队执行")
            self.cancel.clear()
            self.active_card, self.active_action = name, action
            self.active_object = getattr(self.worker_context, "operation_object", None)
            self.worker_context.owned = (name, action) if public_call is not None else None
            if self.driver is not None:
                self.driver.cancelled.clear()
        try:
            yield
        finally:
            with self.lifecycle:
                self.active_card = self.active_action = self.active_object = None
                self.worker_context.owned = None
                self.gate.release()

    def connected(self):
        with self.connection_lock:
            if self.driver is None:
                self.data_dir.mkdir(parents=True, exist_ok=True)
                owner = (self.data_dir / "can-owner.lock").open("a+")
                try:
                    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    candidate = self.factory(self.config.get("can_interface", "can0"))
                    candidate.can_transport = self.config.get("can_transport", "socketcan")
                    candidate.usb_index = self.config.get("usb_index", 0)
                    candidate.joint_limits_deg = self.config.get("joint_limits_deg")
                    candidate.gripper_limits_mm = tuple(self.config.get("gripper_limits_mm", (0.,70.)))
                    candidate.connect()
                    if self.closed or self.cancel.is_set():
                        candidate.close()
                        raise RuntimeError("Connection cancelled")
                    self.driver, self.owner_file = candidate, owner
                except BaseException:
                    owner.close()
                    raise
            return self.driver

    def disconnect(self):
        with self.connection_lock:
            self.revoke_manual_permission()
            self.previews.invalidate()
            self.last_signature = self.firmware_version = None
            if self.driver is not None:
                self.driver.close()
                self.driver = None
            if self.owner_file is not None:
                self.owner_file.close()
                self.owner_file = None

    def read_state(self, monitor=False):
        with self.connection_lock:
            try:
                state = self.connected().get_state()
            except Exception:
                self.previews.invalidate()
                if self.driver is not None and self.active_card in {"arm_control", "piper_arm", "piper_joint", "piper_gripper", "piper_enable", "piper_disable"}:
                    self.driver.cancel_motion()
                raise
            observed = signature(state)
            if self.last_signature is not None and self.last_signature != observed:
                self.previews.invalidate()
                if (monitor and self.active_card in {"arm_control", "piper_arm", "piper_joint", "piper_gripper", "piper_enable", "piper_disable"}
                        and self.active_action not in {"prepare", "enable", "disable", "switch_mode"}):
                    self.driver.cancel_motion()
            self.last_signature = observed
            return state

    def status(self):
        state = self.read_state(monitor=True)
        result = {**asdict(state), "summary": summary(state), "feedback_fresh": True,
                  "busy": self.gate.locked(), "active_card": self.active_card,
                  "firmware": self.firmware_version, "motion_enabled": self.motion_enabled}
        try:
            result["gripper"] = asdict(self.driver.get_gripper())
        except (RuntimeError, AttributeError) as exc:
            result["gripper"] = {"available": False, "reason": str(exc)}
        return result

    def read_monitor_snapshot(self):
        """Passive shared-connection snapshot. Never connects or sends commands.

        No operation-gate acquisition and no waiting behind connection changes.
        Missing/stale values remain None with a field-specific reason.
        """
        result = {
            "device_name": self.config.get("name", self.config.get("can_interface", "PiPER")),
            "can_interface": self.config.get("can_interface", "can0"),
            "connected": self.driver is not None and not self.closed,
            "feedback_time": None, "fresh": False, "joints_deg": None,
            "joint_limits_deg": None, "control_mode": None, "enabled": None,
            "gripper": None, "faults": None, "motion_enabled": self.motion_enabled,
            "busy": self.gate.locked(), "active_card": self.active_card,
            "active_object": self.active_object, "active_action": getattr(self, "active_action", None),
            "last_actions": deepcopy(dict(self.last_actions)),
            "reasons": {},
        }
        if not self.connection_lock.acquire(blocking=False):
            result["reasons"]["connection"] = "连接正在变更，本次监控读取不等待"
            return result
        try:
            if self.driver is None or self.closed:
                result["reasons"]["connection"] = "尚未连接或驱动已停止；监控不会另开 CAN"
                return result
            try:
                state = self.driver.get_state()
                stamp = state.feedback_time
                if math.isfinite(stamp):
                    result["feedback_time"] = stamp
                if (not math.isfinite(stamp) or abs(time.time() - stamp) > .5
                        or len(state.joints_deg) != 6
                        or not all(math.isfinite(value) for value in state.joints_deg)):
                    raise RuntimeError("六轴反馈无效或超过 0.5 秒未更新")
                result.update(fresh=True, joints_deg=list(state.joints_deg), enabled=list(state.enabled),
                              control_mode=state.control_mode,
                              faults={"arm_status": state.arm_status, "teach_status": state.teach_status,
                                      "error_code": state.error_code, "motor_faults": list(state.motor_faults)})
            except Exception as exc:
                result["reasons"]["feedback"] = str(exc)
            try:
                result["joint_limits_deg"] = [list(pair) for pair in self.driver.joint_bounds_deg()]
            except Exception as exc:
                result["reasons"]["joint_limits_deg"] = str(exc)
            try:
                grip = self.driver.get_gripper()
                values = (grip.width_mm, grip.effort_nm, grip.feedback_time)
                if not all(math.isfinite(value) for value in values) or abs(time.time() - grip.feedback_time) > .5:
                    raise RuntimeError("夹爪反馈无效或超过 0.5 秒未更新")
                result["gripper"] = asdict(grip)
            except Exception as exc:
                result["reasons"]["gripper"] = str(exc)
            return result
        finally:
            self.connection_lock.release()

    def motion_allowed(self, args):
        self.require_motion_permission()
        confirm(args, "workspace_clear")

    def require_motion_permission(self):
        if not self.motion_enabled:
            raise RuntimeError("运动许可关闭：请到「操作许可」开启本次控制；Mac USB-CAN 仍需主机授权")

    def revoke_manual_permission(self):
        if self.manual_motion_permission:
            self.motion_enabled = False
            self.manual_motion_permission = False
            self.previews.invalidate()

    def motion_permission(self, action):
        if self.config.get("can_transport", "socketcan") != "socketcan":
            raise RuntimeError("本卡片开关仅支持 Ubuntu SocketCAN；Mac USB-CAN 仍需主机授权")
        if action == "block_motion":
            # Closing permission must remain available while a worker owns the
            # operation gate. Cancellation preserves torque via its hold path.
            with self.lifecycle:
                self.motion_enabled = self.manual_motion_permission = False
                self.previews.invalidate()
                affected = self.gate.locked() and self.active_card in {"arm_control", "piper_arm", "piper_joint", "piper_gripper", "piper_mode", "piper_enable", "piper_disable", "piper_standby", "piper_can"}
                if affected:
                    self.cancel.set()
                    if self.driver is not None:
                        self.driver.cancel_motion()
            return {"motion_enabled": False, "operation_stopping": affected,
                    "motor_torque_released": False, "executed": False,
                    "reminder": "运动许可已关闭；不会失能。失能仍需现场支撑确认。"}
        with self.operation("piper_arm", action):
            state = self.read_state()
            if not math.isfinite(state.feedback_time) or abs(time.time() - state.feedback_time) > .5:
                raise RuntimeError("机械臂反馈超过 0.5 秒未更新，请先检查状态")
            healthy(state)
            if mode_name(state) not in {"standby", "can"} or not teaching_stopped(state):
                raise RuntimeError("请先退出示教或其它控制模式，再开启运动许可")
            if any(state.enabled) and (mode_name(state) != "can" or not self.driver.stationary()):
                raise RuntimeError("已使能时须为电脑控制且机械臂静止，才能开启运动许可")
            with self.lifecycle:
                if self.cancel.is_set() or self.closed or (getattr(self.worker_context, "public_call", None) or ("piper_arm",))[0] in self.stopped_cards:
                    raise RuntimeError("开启许可已取消，请重新操作")
                self.previews.invalidate()
                self.motion_enabled = self.manual_motion_permission = True
            return {"motion_enabled": True, "executed": False,
                    "reminder": "本次控制许可已开启；未使能、未移动。停止单轴卡片、重连或重启后许可关闭。"}

    def dispatch(self, action, args):
        name = args.get("_tool_name", "piper_arm")
        long_action = (name == "piper_photo" and action in {"capture", "import_photo"}
                       or name == "piper_arm" and action in
                           {"prepare", "move_joint"} and args.get("execute") is True
                       or name == "piper_gripper" and action in {"open", "close", "set_width"} and args.get("execute") is True)
        if long_action and self.config.get("async_actions", True):
            return self.run_async(name, action, args)
        return self.dispatch_sync(action, args)

    def run_async(self, name, action, args):
        # Reserve before creating a worker: repeated clicks never queue actions.
        reservation = self.operation(name, action)
        reservation.__enter__()
        action_id = f"{name}_{uuid4().hex}"
        finished, published = threading.Event(), threading.Event()
        outcome = {}
        def work():
            self.worker_context.reserved = (name, action)
            try:
                outcome.update(status="completed", result=self.dispatch_sync(action, dict(args)))
            except BaseException as exc:
                outcome.update(status="cancelled" if self.cancel.is_set() else "error", result={"error": str(exc)})
            finally:
                self.last_actions[name] = {"action_id": action_id, "status": outcome["status"], "result": outcome["result"]}
                reservation.__exit__(None, None, None)
                self.worker_context.reserved = None
                finished.set()
            published.wait()
            if outcome.get("notify"):
                # Let the MCP response register its pending action first.
                time.sleep(.1)
                self.acp_complete(action_id, outcome["status"], outcome["result"], name)
        try:
            threading.Thread(target=work, daemon=True, name=action_id).start()
        except BaseException:
            reservation.__exit__(None, None, None)
            raise
        if finished.wait(.05):
            published.set()
            if outcome["status"] != "completed":
                raise RuntimeError(outcome["result"]["error"])
            return outcome["result"]
        outcome["notify"] = True
        published.set()
        return {"state": "running", "action_id": action_id,
                "reminder": "动作已提交；请查看卡片完成结果，也可在机械臂控制卡读取实际反馈。"}

    def acp_complete(self, action_id, status, result, name):
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                              "tool": name, "ts": time.time()}).encode()
        request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                         headers={"Content-Type": "application/json"}, method="POST")
        context = ssl._create_unverified_context() if url.startswith("https://localhost:") else None
        try:
            with urllib.request.urlopen(request, timeout=3, context=context) as response:
                response.read()
        except Exception:
            # Keep the actual result available even if the platform is offline.
            if self.last_actions.get(name, {}).get("action_id") == action_id:
                self.last_actions[name]["completion_delivery_failed"] = True

    def dispatch_sync(self, action, args):
        name = args.get("_tool_name", "piper_arm")
        if name not in {"piper_status", "piper_mode", "piper_arm", "piper_gripper", "piper_photo"}:
            return None
        if name == "piper_status":
            # Resource dispatch preserves action in args; canvas uses lifecycle
            # calls before enabling its execute buttons.
            status_action = args.get("action", "read")
            if status_action == "start":
                self.start(name)
                return {"state": "ready"}
            if status_action in {"info", "stop"}:
                return {"state": "idle" if self.closed or status_action == "stop" else "ready"}
            if status_action == "reconnect":
                with self.operation(name, "reconnect"):
                    self.disconnect()
                    return self.status()
            if self.closed:
                raise RuntimeError("Driver stopped; call start before reading")
            # Feedback reads never wait on the movement gate.
            return self.status()
        if action == "stop":
            return self.request_stop(name)
        if action == "start":
            self.start(name)
            return {"state": "ready"}
        if action == "info":
            return {"state": "idle" if self.closed or name in self.stopped_cards else "ready", "busy": self.gate.locked(),
                    "last_action": self.last_actions.get(name),
                    "active_card": self.active_card,
                    "camera_enabled": self.camera_enabled,
                    "motion_enabled": self.motion_enabled, "max_j1_step_deg": 12,
                    "topic_out": [{"topic": self.topic, "format": "image/jpeg"}] if name == "piper_photo" else []}
        if name == "piper_arm" and action in {"allow_motion", "block_motion"}:
            return self.motion_permission(action)
        if name == "piper_photo":
            if action not in {"capture", "import_photo"}:
                raise ValueError(f"Unknown photo action: {action}")
            with self.operation(name, action):
                return self.import_photo(args) if action == "import_photo" else self.capture(args)
        for key in ("execute", "workspace_clear", "allow_limit_adjustment", "take_control", "arm_supported", "confirm_save",
                    "other_controllers_stopped", "object_supported", "query_firmware"):
            if key in args and type(args[key]) is not bool:
                raise ValueError(f"{key} must be a boolean")
        if name == "piper_mode":
            return self.mode_action(action, args)
        if name == "piper_gripper":
            return self.gripper_action(action, args)
        if action not in {"prepare", "disable", "move_joint", "hold"}:
            raise ValueError(f"Unknown arm action: {action}")
        # Reject strings like 'false' before any hardware access.
        for key in ("execute", "workspace_clear", "allow_limit_adjustment", "take_control", "arm_supported"):
            if key in args and type(args[key]) is not bool:
                raise ValueError(f"{key} must be a boolean")
        speed = number(args, "speed_pct", 5, 1, 10, integer=True)
        rate = number(args, "speed_deg_s", 1, .5, 5)
        if action == "move_joint":
            number(args, "target_deg", None, -180, 180)
            number(args, "joint", 1, 1, 6, integer=True)
        execute = args.get("execute", False)
        if action == "hold" or (action in {"prepare", "move_joint"} and execute):
            self.motion_allowed(args)
        if action == "hold" or (action == "move_joint" and execute):
            confirm(args, "take_control")
        if action == "disable":
            confirm(args, "arm_supported")
            self.require_motion_permission()
        with self.operation(name, action):
            if action in {"hold", "disable"} or execute:
                self.require_motion_permission()
            driver = self.connected()
            state = self.read_state()
            if action == "prepare":
                key = f"prepare:{speed}"
                plan = driver.prepare_control(False, speed_pct=speed)
                if not execute:
                    return {**plan, **self.previews.put(key, state)}
                self.previews.consume(args.get("preview_id") or None, key, self.read_state())
                try:
                    result = driver.prepare_control(True, args.get("workspace_clear", False),
                                                    args.get("allow_limit_adjustment", False), speed)
                    self.last_signature = signature(driver.get_state())
                    return result
                except BaseException:
                    self.last_signature = None
                    raise
            if action == "disable":
                return asdict(driver.disable(supported=True))
            if action == "hold":
                ready_to_move(state)
                return asdict(driver.hold_current(speed_pct=speed))
            if action == "move_joint":
                target = list(state.joints_deg)
                joint = args.get("joint", 1)
                target[joint-1] = args["target_deg"]
                params = (joint, args["target_deg"])
                key = json.dumps([action, params, rate, speed])
                plan = driver.joint_plan(target, rate, speed)
                if not execute:
                    return {**plan, "executed": False, **self.previews.put(key, state)}
                self.previews.consume(args.get("preview_id") or None, key, self.read_state())
                return {"executed": True, "state": asdict(driver._move_joint_target(target, rate, speed))}

    def verified_teaching(self):
        return (self.firmware_version is not None
                and self.firmware_version in self.config.get("mode", {}).get("verified_teaching_firmwares", []))

    def mode_action(self, action, args):
        if action == "read":
            if args.get("query_firmware"):
                with self.operation("piper_mode", action):
                    self.firmware_version = self.connected().firmware(query=True)
                    self.previews.invalidate()
            return {**self.status(), "software_teaching_verified": self.verified_teaching(),
                    "reminder": "使能、失能和故障单独处理；普通切换不回零、不复位。"}
        if action not in {"preview", "switch", "stop_teaching"}:
            raise ValueError(f"Unknown mode action: {action}")
        if action == "stop_teaching":
            confirm(args, "arm_supported")
            with self.operation("piper_mode", action):
                driver = self.connected()
                self.previews.invalidate()
                # Guidance remains available when motion permission is off.
                return driver.stop_teaching(self.verified_teaching() and self.motion_enabled)
        target = args.get("target_mode")
        if target not in {"standby", "can", "teaching"}:
            raise ValueError("target_mode 仅支持 standby / can / teaching")
        with self.operation("piper_mode", action):
            driver = self.connected()
            state = self.read_state()
            still = driver.stationary() if target == "teaching" and mode_name(state) == "can" else False
            plan = mode_plan(state, target, args, self.verified_teaching(), still)
            key = f"mode:{target}:{self.firmware_version}"
            if action == "preview":
                self.previews.invalidate()
                return {**plan, **(self.previews.put(key, state) if plan["allowed"] else {})}
            if not plan["allowed"]:
                self.previews.invalidate()
                if plan["button_guidance"]:
                    return {**plan, "executed": False}
                raise RuntimeError("；".join(plan["missing_conditions"]))
            self.require_motion_permission()
            self.previews.consume(args.get("preview_id") or None, key, self.read_state())
            try:
                return {**plan, **driver.switch_mode(target, state, self.verified_teaching())}
            finally:
                self.previews.invalidate()
                self.last_signature = None

    def gripper_action(self, action, args):
        if action == "read":
            with self.connection_lock:
                return asdict(self.connected().get_gripper())
        if action not in {"open", "close", "set_width", "hold", "disable"}:
            raise ValueError(f"Unknown gripper action: {action}")
        torque = number(args, "torque_nm", .5, .001, 1)
        if action == "set_width":
            number(args, "width_mm", None, 0, 100)
        execute = args.get("execute", False)
        if action == "hold" or execute:
            self.motion_allowed(args)
            confirm(args, "take_control")
        if action == "disable":
            confirm(args, "object_supported")
            self.require_motion_permission()
        with self.operation("piper_gripper", action):
            if action in {"hold", "disable"} or execute:
                self.require_motion_permission()
            driver = self.connected()
            state = self.read_state()
            grip = driver.get_gripper()
            if grip.error_code:
                raise RuntimeError("夹爪有故障，请先检查状态")
            if action == "disable":
                self.previews.invalidate()
                return asdict(driver.disable_gripper())
            ready_to_move(state)
            if action == "hold":
                return asdict(driver.hold_gripper())
            low, high = driver.gripper_range()
            width = high if action == "open" else low if action == "close" else args["width_mm"]
            if not low <= width <= high:
                raise ValueError(f"开度须在 SDK 与部署限位交集 [{low}, {high}] mm 内")
            key = json.dumps(["gripper", action, width, torque])
            extra = (round(grip.width_mm, 1), grip.enabled, grip.error_code)
            if not execute:
                return {"target_mm": width, "torque_nm": torque, "limits_mm": [low,high],
                        "executed": False, **self.previews.put(key, state, extra)}
            self.previews.consume(args.get("preview_id") or None, key, self.read_state(), extra)
            return {"executed": True, "gripper": asdict(driver.move_gripper(width, torque))}

    def import_photo(self, args):
        """Import an operator-staged image without camera or CAN access."""
        filename = args.get("filename")
        if (not isinstance(filename, str) or not filename or len(filename) > 255
                or Path(filename).name != filename or "\\" in filename
                or Path(filename).suffix.lower() not in {".jpg", ".jpeg", ".png"}):
            raise ValueError("filename 必须为 imports 目录中的 JPEG/PNG 文件名，不含路径")
        root = (self.data_dir / "imports").resolve()
        source = root / filename
        if source.is_symlink() or source.resolve().parent != root or not source.is_file():
            raise ValueError("照片不存在或不是 imports 目录中的普通文件")
        with source.open("rb") as handle:
            data = handle.read(20 * 1024 * 1024 + 1)
        if not data or len(data) > 20 * 1024 * 1024:
            raise ValueError("照片须非空且不超过 20 MiB")
        import cv2
        import numpy as np
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("无法解码照片，请提供有效 JPEG/PNG")
        height, width = image.shape[:2]
        if height * width > 20_000_000:
            raise ValueError("照片不超过 2000 万像素")
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise RuntimeError("照片 JPEG 保存失败")
        if self.cancel.is_set():
            raise RuntimeError("Photo import cancelled")
        output = self.data_dir / f"rgb_{uuid4().hex}.jpg"
        info = {"output": str(output), "width": width, "height": height,
                "source": "user_provided", "source_sha256": hashlib.sha256(data).hexdigest(),
                "imported_at": datetime.now(timezone.utc).isoformat(), "captured_at": None,
                "camera_accessed": False, "arm_moved": False,
                "dds_published": self.ros2 is not None,
                "topic_out": [{"topic": self.topic, "format": "image/jpeg"}]}
        try:
            output.write_bytes(encoded.tobytes())
            if self.cancel.is_set():
                raise RuntimeError("Photo import cancelled")
            output.with_suffix(".json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
            self.publish_photo(output)
            return info
        except BaseException:
            output.unlink(missing_ok=True)
            output.with_suffix(".json").unlink(missing_ok=True)
            raise

    def capture(self, args):
        if not self.camera_enabled:
            raise RuntimeError("相机采集默认关闭；取得新的明确授权后在部署端设置 PIPER_CAMERA_ENABLED=1")
        confirm(args, "capture_confirmed")
        if "forward_view_confirmed" in args and type(args["forward_view_confirmed"]) is not bool:
            raise ValueError("forward_view_confirmed must be a boolean")
        cfg = self.config.get("camera", {})
        warmup = number(cfg, "warmup_seconds", 3, 0.1, 10)
        contrast = number(cfg, "min_contrast", 8, 0, 255)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        output = self.data_dir / f"rgb_{uuid4().hex}.jpg"
        cmd = [sys.executable, str(Path(__file__).with_name("camera_capture.py")),
               "--output", str(output), "--warmup", str(warmup), "--min-contrast", str(contrast)]
        if cfg.get("device"):
            cmd.extend(["--device", str(cfg["device"])])
        # Isolate V4L2's blocking read. Stop/timeout terminate the child and
        # release its device handles even when the USB camera has stalled.
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        deadline = time.monotonic() + warmup + 8
        try:
            while True:
                if self.cancel.is_set():
                    raise RuntimeError("Capture cancelled")
                if time.monotonic() > deadline:
                    raise RuntimeError("Camera capture timed out")
                try:
                    stdout, stderr = proc.communicate(timeout=0.1)
                    break
                except subprocess.TimeoutExpired:
                    continue
            if proc.returncode:
                raise RuntimeError(f"Camera capture failed: {stderr[-1000:]}")
            info = json.loads(stdout)
            info.pop("device", None)  # Do not publish the USB serial in card output.
            info.update(captured_at=datetime.now(timezone.utc).isoformat(), dds_published=self.ros2 is not None,
                        capture_confirmed=True, camera_accessed=True, arm_moved=False)
            if "forward_view_confirmed" in args:
                info["forward_view_confirmed"] = args["forward_view_confirmed"]
            if self.driver is not None:
                try:
                    info["arm_state"] = asdict(self.read_state())
                except RuntimeError as exc:
                    info["arm_state"] = {"available": False, "reason": str(exc)}
            else:
                info["arm_state"] = {"available": False, "reason": "未连接 CAN；本次拍照未连接机械臂"}
            if self.cancel.is_set():
                raise RuntimeError("Capture cancelled")
            self.publish_photo(output)
            output.with_suffix(".json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
            return {**info, "topic_out": [{"topic": self.topic, "format": "image/jpeg"}]}
        except BaseException:
            output.unlink(missing_ok=True)
            raise
        finally:
            if proc.poll() is None:
                proc.kill()
            proc.communicate()

    def publish_photo(self, output):
        if self.ros2 is None:
            return  # Offline contract tests; production runtime always supplies ROS.
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage

        if self.node is None:
            self.node = Node("piper_photo", context=self.ros2.ctx_core)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.node.create_publisher(CompressedImage, self.topic, qos)
            # Replay only the most recent still image for late dashboard subscribers.
            self.node.create_timer(1.0, self.republish_photo)
            self.ros2.executor_core.add_node(self.node)
        msg = CompressedImage()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "piper_camera_color_optical_frame"
        msg.format = "jpeg"
        msg.data = output.read_bytes()
        self.last_image = msg
        self.publisher.publish(msg)

    def republish_photo(self):
        if not self.closed and "piper_photo" not in self.stopped_cards and self.last_image is not None:
            self.publisher.publish(self.last_image)

