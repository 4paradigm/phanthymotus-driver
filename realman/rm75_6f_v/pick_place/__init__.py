"""RealMan pick-and-place actuator with externally supplied RGB-D observations."""

import copy
import json
import math
from pathlib import Path
import shutil
import threading
import time
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from .motion import ObservationMotion, pose_close
from .inputs import ObservationInputs
from .geometry import displacement, positions, rotation_degrees
from .transfer import Transfer
from .completion import Completion
from hardware import JOINT_LIMITS_DEG


# Transfer includes six moves and gripper operations, with time for settling.
# Observation has a separate budget; ACP also allows stopping and notification.
ACTION_TIMEOUT_SECONDS = 45
TRANSFER_TIMEOUT_SECONDS = 120
COMPLETION_TIMEOUT_SECONDS = 210
MOTION_ACTIONS = ("observe", "grab_to", "grab_by")


CONFIG_PROPERTIES = {
    "speed_percent": {
        "type": "integer", "minimum": 1, "maximum": 100, "default": 50,
        "description": "全局运行速度（%），用于观察位、水平移动、抓放升降及夹爪 Rz 旋转。",
    },
    "observation_joints_deg": {
        "type": "string", "default": "-90,0,0,90,0,90,0",
        "description": "观察位 J1～J7 关节角（°），用英文逗号分隔；须符合各关节限位。",
    },
    "x_compensation_mm": {
        "type": "number", "default": 30,
        "description": "基坐标 X 补偿（mm），加到照片计算出的绝对水平目标，可为负数。",
    },
    "y_compensation_mm": {
        "type": "number", "default": -75,
        "description": "基坐标 Y 补偿（mm），加到照片计算出的绝对水平目标，可为负数。",
    },
    "pick_grip_force": {
        "type": "integer", "minimum": 0, "maximum": 100, "default": 15,
        "description": "抓取时的夹持力度，设备原始整数 0～100，非牛顿值。",
    },
    "pick_descent_mm": {
        "type": "number", "exclusiveMinimum": 0, "default": 91,
        "description": "抓取时从当前位置向下移动的距离（mm）。",
    },
    "place_descent_mm": {
        "type": "number", "exclusiveMinimum": 0, "default": 60,
        "description": "放置时从当前位置向下移动的距离（mm）。",
    },
    "observe_after_transfer": {
        "type": "boolean", "default": False,
        "description": "搬运后拍照：抓放与回升完成后始终返回观察位；开启时再保存一张新照片并返回物品列表，关闭时不拍照。",
    },
}


def validate_config(config):
    for name, value in config.items():
        if name == "observation_joints_deg":
            if not isinstance(value, str):
                raise ValueError("observation_joints_deg must be seven comma-separated joint angles in degrees")
            joints = [float(part.strip()) for part in value.split(",")]
            if len(joints) != 7:
                raise ValueError("observation_joints_deg must contain exactly seven joint angles")
            for index, (angle, (low, high)) in enumerate(zip(joints, JOINT_LIMITS_DEG), 1):
                if not math.isfinite(angle) or not low <= angle <= high:
                    raise ValueError(f"Observation J{index} must be between {low:g} and {high:g} degrees")
            continue
        prop = CONFIG_PROPERTIES[name]
        if prop["type"] == "boolean":
            if type(value) is not bool:
                raise ValueError(f"{name} must be a boolean")
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        if prop["type"] == "integer" and value != int(value):
            raise ValueError(f"{name} must be an integer")
        if "minimum" in prop and value < prop["minimum"]:
            raise ValueError(f"{name} must be at least {prop['minimum']}")
        if "maximum" in prop and value > prop["maximum"]:
            raise ValueError(f"{name} must be at most {prop['maximum']}")
        if "exclusiveMinimum" in prop and value <= prop["exclusiveMinimum"]:
            raise ValueError(f"{name} must be greater than {prop['exclusiveMinimum']}")
        if prop["type"] == "integer":
            config[name] = int(value)


class PickPlacePlugin:
    PREFIX = "vision_pick_and_drop"

    def __init__(self, client, config, namespace="rm75", ros2=None, inputs=None):
        self.client = client
        self._ros2 = ros2
        self._inputs = inputs if inputs is not None else ObservationInputs(ros2)
        self._output_dir = Path(config.get("vision_pick_and_drop", {}).get(
            "output_dir", "/opt/phanthy-motus/data/pick_place/realman"))
        self._config = {name: prop["default"] for name, prop in CONFIG_PROPERTIES.items()}
        self._config_lock = threading.RLock()
        self._active = None
        self._starting = None
        self._stopping = False
        self._last_result = None
        self._observation = None
        self._motion_blocked = False

    def get_tools(self):
        transfer_guidance = (
            "使用最近一次成功观察的照片和检测坐标：首次先 observe，从 result.objects 选取目标；"
            "若上次搬运返回 observation.ok=true，也可使用 result.observation.objects。"
            "objects 与对应 captured_at 的静止观察窗口绑定，position[0]/position[1] 分别为 X/Y；"
            "目标缺失或位置不明时不猜测。每张照片仅供一次搬运，首次下发设备命令即消费，旧检测坐标不能复用。"
            "每次调用须显式传 confirm_motion=true；返回 state=running 和 action_id 后等待框架 ACP 终态，"
            "无需轮询、补发消息或重复下发动作。卡片按配置完成抓起、水平搬运、放下和回升。"
            "rotation_deg 是可选的夹爪自身 Rz 旋转角（°），正值为俯视顺时针，负值为逆时针；"
            "常规搬运省略此参数，只有用户明确要求旋转时才设置。抓起并回升后旋转，保持该朝向搬运并放下。"
            "配置 observe_after_transfer 控制后续观察：抓放完成后始终按配置返回观察位；关闭时不拍照，observation.skipped=true、"
            "observation_required=true；开启时在返回观察位并停稳后保存一张新照片，返回 observation.ok=true、"
            "observation_id、captured_at、file_path、objects、count，observation_required=false。"
            "新观察同时成为下一次搬运的依据，可用于评估效果；无需为取得物品列表另外调用 VOP。"
            "result.observation_id 是本次抓放使用的照片编号，result.observation.observation_id 是动作后的新编号。"
            "ACP status=completed 且 result.ok=true 表示配置要求的动作链完成；transfer_completed=true 表示抓放与回升完成，return_completed=true 表示已返回观察位。"
            "grasp_checked=false 表示卡片没有自动判断物体是否实际搬运成功，不能仅据动作成功宣称抓取成功。"
            "抓放完成但后续观察失败时仍保留 transfer_completed=true，不能因此认定物体没有移动或直接重做搬运。"
            "短暂反馈波动和轻微偏移会在有时限的等待中重新核验，恢复后继续当前步骤，不重复设备指令。"
            "持续偏移、真实故障、超时或取消会停止；失败后不自动重试，结果包含 stage、last_completed_stage、"
            "holding_object_possible、release_completed。recovery_required=true 时需人工确认并安全处理持物或停止状态，"
            "不要直接 observe 或从头重试。其余情况下 observation_required=true 时先重新 observe，使用新坐标。")
        properties = {name: {"type": "number", "minimum": -1, "maximum": 1, "description": description}
                      for name, description in (
                          ("start_point_x", "抓取中心在最新观察照片中的归一化 X 坐标，范围 [-1,1]：左边缘 -1、中心 0、右边缘 +1，正方向向右。可直接使用该照片的 VOP position[0]，无需换算像素。"),
                          ("start_point_y", "抓取中心在最新观察照片中的归一化 Y 坐标，范围 [-1,1]：上边缘 -1、中心 0、下边缘 +1，正方向向下。可直接使用该照片的 VOP position[1]，无需换算像素。"),
                          ("target_point_x", "grab_to 放置位置在最新观察照片中的归一化 X 坐标，范围 [-1,1]：左边缘 -1、中心 0、右边缘 +1，正方向向右，与 start_point_x 相同。"),
                          ("target_point_y", "grab_to 放置位置在最新观察照片中的归一化 Y 坐标，范围 [-1,1]：上边缘 -1、中心 0、下边缘 +1，正方向向下，与 start_point_y 相同。"))}
        properties.update({
            "delta_x": {"type": "number", "description": "grab_by 从抓取点沿桌面左右平移的有符号距离（mm），可为小数。以最新观察照片为准：X 正方向向右（正值），负方向向左（负值），0 不左右移动；当前安装对应基坐标 ΔX=-delta_x。不是像素或绝对位置。"},
            "delta_y": {"type": "number", "description": "grab_by 从抓取点沿桌面上下平移的有符号距离（mm），可为小数。以最新观察照片为准：Y 正方向向照片下方（正值），负方向向上方（负值），0 不上下平移；当前安装对应基坐标 ΔY=delta_y。不是机械臂 Z 升降、像素或绝对位置。"},
            "rotation_deg": {"type": "number", "minimum": -180, "maximum": 180, "default": 0,
                             "description": "可选夹爪自身 Rz 旋转角（°），范围 [-180,180]：俯视正值顺时针、负值逆时针。抓起并回升后旋转再搬运放下；常规搬运省略，仅在用户明确要求旋转时填写。0 不旋转。"},
            "confirm_motion": {"type": "boolean", "const": True,
                               "description": "每次 observe、grab_to、grab_by 请求必须显式为 true，确认执行本次机械臂动作；取消无需此参数。"},
        })
        schema = action_schema({
            "observe": (["confirm_motion"], (
                "观察桌面并获得一次可用于抓放的 RGB-D 观察。每次须显式传 confirm_motion=true；按配置速度移动到观察关节角，"
                "停稳后保存一张照片，返回物品列表，不持续推送照片。动作不操作夹爪。"
                "返回 state=running 和 action_id 后等待框架 ACP 通知 status=completed、result.ok=true；"
                "结果包含 observation_id、captured_at、file_path、width、height、objects、count 和 observation_required=false。"
                "从 result.objects 选择目标，使用其 name、position、confidence；position[0]/position[1] 可直接填搬运 start_point_x/start_point_y。"
                "无需读取照片文件、换算像素或另调 VOP；count=0 表示没有检测到物体，此时不猜测抓取点。"
                "前提是三路输入已连接并启动：同一 ext_camera 的 RGB、depth，以及该 RGB 经 VOP 得到的物品列表。"
                "卡片完成内参匹配、深度对齐和静止窗口同步；短暂画面变化会重新等待稳定及新的检测结果。"
                "照片仅供一次搬运。搬运后是否自动生成新观察由配置 observe_after_transfer 决定，以完成结果 observation_required 为准。"
                "此前动作若返回 recovery_required=true，先人工处理持物或停止状态，不以重新观察代替恢复。")),
            "grab_to": (["start_point_x", "start_point_y", "target_point_x", "target_point_y", "rotation_deg", "confirm_motion"], (
                "指定目标点：用于把物体放到照片中的指定位置，或另一物体旁的空位。"
                "(start_point_x,start_point_y) 是抓取物体中心，(target_point_x,target_point_y) 是放置点，四个值均为同一张最新观察照片的归一化坐标 [-1,1]。"
                "照片中心为 (0,0)，X 向右、Y 向下为正；VOP position[0] 对应 X、position[1] 对应 Y，"
                "无需换算像素或读取照片文件。放在另一物体旁边时选择其旁的空位，不能把参照物中心直接当作空位；"
                "指定毫米距离的相对移动使用 grab_by。" + transfer_guidance)),
            "grab_by": (["start_point_x", "start_point_y", "delta_x", "delta_y", "rotation_deg", "confirm_motion"], (
                "指定距离（mm）：用于把一个物体向左、右、照片上方或下方移动指定距离。"
                "(start_point_x,start_point_y) 为最新观察照片中物体中心的归一化坐标 [-1,1]，照片中心为 (0,0)，"
                "直接使用 VOP position[0]、position[1]，无需换算像素或读取照片文件。"
                "delta_x、delta_y 是从物体抓取点出发的桌面毫米位移，两项都要填写；若两项均为 0，须指定非零 rotation_deg，表示原地抓起旋转再放下；1 cm=10 mm。"
                "方向以照片为准：X 正方向向右、Y 正方向向下，负值反向；上/下也是桌面平移，不是 Z 升降。"
                "向右 30 mm：delta_x=30、delta_y=0；向左 30 mm：delta_x=-30、delta_y=0；"
                "向照片上方 20 mm：delta_x=0、delta_y=-20；向下 20 mm：delta_x=0、delta_y=20。"
                "放到照片中指定位置使用 grab_to。" + transfer_guidance)),
            "cancel": ([], "中止当前动作，无需 confirm_motion。停止后不继续抓放、回程或观察，不自动松爪；若已抓取，物体可能仍在夹爪中。等待 ACP 终态并检查 holding_object_possible、release_completed、recovery_required，先安全处理再开始新任务。"),
        }, properties)
        # Require confirmation only for motion, leaving interrupt hooks callable.
        schema["allOf"] = [{"if": {"properties": {"action": {"enum": list(MOTION_ACTIONS)}}},
                            "then": {"required": ["confirm_motion"]}}]
        schema["x-completion"] = {"actions": list(MOTION_ACTIONS), "timeout": COMPLETION_TIMEOUT_SECONDS}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "cancel"},
                             "on_interrupt_all": {"action": "cancel"}}
        schema["x-is-dangerous"] = True
        schema["x-resource"] = "arm"
        definition = tool("vision_pick_and_drop", "actuator", (
            "观察和搬运桌面物体。先 observe 获取一张照片对应的物品列表；按照片中的目标位置放置用 grab_to，"
            "按方向移动指定毫米距离用 grab_by。位置采用中心归一化坐标 [-1,1]，可直接使用本卡片返回的 VOP position："
            "X 向右、Y 向下为正；位移参数单独使用 mm，上下也指桌面方向。"
            "常规搬运不设置 rotation_deg；仅用户明确要求时，抓起回升后绕夹爪 Rz 旋转，正值俯视顺时针、负值逆时针。"
            "每次运动传 confirm_motion=true，收到 action_id 后等待框架 ACP 终态，无需补发消息或重复调用。"
            "搬运完成后始终返回观察位。observe_after_transfer 开启时，再拍照并在 result.observation 返回新照片和物品列表；"
            "关闭时返回观察位后结束，不拍照。以 observation_required 判断是否需要重新 observe，每张照片仅供一次搬运。"
            "对短暂反馈波动先等待并核验，再继续剩余步骤；真实故障或取消会停止。"
            "失败时检查 transfer_completed、holding_object_possible、release_completed、recovery_required，"
            "持物或停稳状态待处理时不要从头重复搬运。卡片不自动判断实际搬运效果，由调用方根据新观察决定后续操作。"),
                          schema, topic_in=self._inputs.topics())
        definition["configSchema"] = {
            "type": "object", "properties": copy.deepcopy(CONFIG_PROPERTIES), "additionalProperties": False,
        }
        return [definition]

    def start(self, args=None):
        # Reserve activation without holding the action lock during ROS setup.
        with self._config_lock:
            if self._active is not None or self._starting is not None or self._stopping:
                return {"state": "error", "message": "Card is busy; wait before rebinding inputs"}
            cancel = threading.Event()
            self._starting = cancel
            before = self._inputs.identity()
        try:
            self._inputs.start(args, cancel=cancel)
        except Exception as exc:
            return {"state": "idle"} if cancel.is_set() else {"state": "error", "message": str(exc)}
        finally:
            with self._config_lock:
                if before != self._inputs.identity():
                    self._observation = None
                self._starting = None
        return {"state": "idle"} if cancel.is_set() else self.dispatch("info", {})

    def stop(self):
        with self._config_lock:
            if self._stopping:
                return {"state": "stopping"}
            self._stopping = True
            if self._starting is not None:
                self._starting.set()
            self._observation = None
        try:
            self._cancel()
            with self._config_lock:
                active = self._active
            if active is not None:
                active["done"].wait(timeout=8)
            with self._config_lock:
                if self._active is not None:
                    return {"state": "stopping"}
            self._inputs.stop()
            return {"state": "error" if self._motion_blocked else "idle",
                    "motion_blocked": self._motion_blocked}
        finally:
            with self._config_lock:
                self._stopping = False

    def _cancel(self):
        with self._config_lock:
            active = self._active
            if active is None:
                return {"state": "error" if self._motion_blocked else "idle",
                        "motion_blocked": self._motion_blocked}
            if active.get("finished"):
                return {"state": "completed"}
            active["cancel"].set()
            self._observation = None
            # Same lock as submission: cancellation cannot be followed by another command.
            if active["motion_sent"]:
                try:
                    active["stop_requested"] = True
                    self.client.command("rm_set_arm_slow_stop")
                except Exception as exc:
                    active["stop_error"] = str(exc)
            return {"state": "stopping"}

    def _invalidate_changed_source(self):
        if (self._observation is not None
                and self._observation.get("input_identity") != self._inputs.identity()):
            self._observation = None

    def _execute(self, action, args):
        if args.get("confirm_motion") is not True:
            return {"state": "error", "code": "CONFIRMATION_REQUIRED",
                    "message": "confirm_motion must be true for every movement request"}
        with self._config_lock:
            if self._starting is not None or self._stopping:
                return {"state": "error", "message": "Card inputs are starting or stopping"}
            if not self.client.connected:
                return {"state": "error", "message": "Mechanical arm is not connected"}
            if not self.client.motion_enabled:
                return {"state": "error", "message": "Driver is in read-only mode; deploy with --mode live to execute actions"}
            if self._motion_blocked:
                return {"state": "error", "message": "Stop could not be verified; inspect the arm before restarting the driver"}
            self._invalidate_changed_source()
            if self._active is None and action != "observe" and self._observation is None:
                return {"state": "error", "code": "OBSERVATION_REQUIRED", "observation_required": True,
                        "message": "Run observe and obtain new detections before transferring. "
                                   "No usable photograph is available; each photograph permits only one transfer."}
            if self._active is not None or not self.client.motion_lock.acquire(blocking=False):
                return {"state": "error", "message": "Another device action is active"}
            try:
                active = {"config": dict(self._config), "cancel": threading.Event(),
                          "done": threading.Event(), "motion_sent": False,
                          "action_id": f"vision_pick_and_drop_{action}_{uuid4().hex}", "action": action}
                if action != "observe":
                    active["rotation_deg"] = rotation_degrees(args)
                    if action == "grab_by":
                        active["positions"] = positions(args, ("start_point_x", "start_point_y"))
                        active["displacement_mm"] = displacement(args, active["rotation_deg"])
                    else:
                        active["positions"] = positions(args)
                    active["observation"] = dict(self._observation)
                active["completion"] = Completion(args.get("_tool_name", self.PREFIX))
                self._active = active
                threading.Thread(target=self._run_action, args=(active,),
                                 name=active["action_id"], daemon=True).start()
                if action == "observe":
                    self._observation = None
            except Exception as exc:
                self._active = None
                self.client.motion_lock.release()
                return {"state": "error", "message": str(exc)}
        return {"state": "running", "action_id": active["action_id"]}

    def _run_action(self, active):
        try:
            terminal = (self._run_observe(active) if active["action"] == "observe"
                        else self._run_transfer(active))
        except Exception as exc:
            # Include failures in worker setup in the same terminal contract.
            status, result = self._failure(active, exc)
            terminal = self._finish(active, status, result)
        try:
            callback, error = active["completion"].send(active["action_id"], terminal["state"],
                {**terminal["result"], "observation_required": terminal["observation_required"]})
        except Exception as exc:
            callback, error = "failed", str(exc)
        with self._config_lock:
            # Update this record, not a newer action's completion.
            terminal["callback"] = callback
            if error:
                terminal["callback_error"] = error

    def _failure(self, active, exc):
        if active["motion_sent"]:
            try:
                if not active.get("stop_requested"):
                    self.client.command("rm_set_arm_slow_stop")
                ObservationMotion(self.client, threading.Event()).settled()
            except Exception as stop_exc:
                active["stop_error"] = str(stop_exc)
                self._motion_blocked = True
        result = {"ok": False, "message": str(exc)}
        if active.get("stop_error"):
            result["stop_error"] = active["stop_error"]
        return "cancelled" if active["cancel"].is_set() else "error", result

    def _finish(self, active, status, result):
        with self._config_lock:
            terminal = {"state": status, "result": result, "action_id": active["action_id"],
                        "callback": "pending",
                        "observation_required": self._observation is None}
            self._last_result = terminal
            self._active = None
            if not self._motion_blocked:
                self.client.motion_lock.release()
            active["done"].set()
        return terminal

    def _run_transfer(self, active):
        motion = ObservationMotion(self.client, active["cancel"],
                                   deadline=time.monotonic() + TRANSFER_TIMEOUT_SECONDS)
        def send(method, *args):
            with self._config_lock:
                motion.check_cancel()
                # Moving the scene consumes the photograph; invalid input does not.
                self._observation = None
                active["motion_sent"] = True
                self.client.command(method, *args)

        def stage(value):
            active["stage"] = value

        transfer, transfer_result = None, None
        observe_after = active["config"]["observe_after_transfer"]
        try:
            transfer = Transfer(self.client, motion, active["observation"], active["config"],
                                active["positions"], send, stage, active.get("displacement_mm"),
                                rotation_deg=active["rotation_deg"])
            transfer_result = transfer.run()
            result = {**transfer_result, "transfer_completed": True,
                      "observe_after_transfer": observe_after, "recovery_required": False,
                      "observation": {"skipped": True, "reason": "disabled"}}
            stage("returning")
            observation = self._observe(active, capture=observe_after)
            result.update(final_pose=observation["pose"], return_completed=True)
            if observe_after:
                result["observation"] = observation
            with self._config_lock:
                if not active.get("finished"):
                    motion.check_cancel()
                active["finished"] = True
            status = "completed"
        except Exception as exc:
            status, result = self._failure(active, exc)
            result.update(transfer.progress() if transfer is not None else {})
            result["stage"] = active.get("stage", "checking")
            # Preserve a completed transfer if returning/observing subsequently fails.
            # This describes command completion, not whether the object was grasped.
            result = {**(transfer_result or {}), **result,
                      "transfer_completed": transfer_result is not None,
                      "return_completed": active.get("return_completed", False),
                      "observe_after_transfer": observe_after,
                      "recovery_required": result.get("holding_object_possible", False) or self._motion_blocked,
                      "observation": {"ok": False, "message": str(exc)} if transfer_result is not None and observe_after
                                     else {"skipped": True, "reason": "transfer_incomplete" if observe_after else "disabled"}}
            if result["recovery_required"]:
                result["recovery_message"] = "动作已停止；夹爪可能仍持物或停稳尚未确认。请人工确认并安全处理，勿直接重新观察或从头重复搬运。"
            if transfer_result is not None:
                result.pop("final_pose", None)  # The return may have moved beyond the place pose.
        return self._finish(active, status, result)

    def _save_photo(self, photo, active, feedback, frames):
        observation_id = active["observation_id"]
        directory = self._output_dir / observation_id
        directory.mkdir(parents=True, exist_ok=False)
        metadata = {key: value for key, value in photo.items() if key not in ("jpeg", "depth_zlib", "request_id")}
        metadata.update(observation_id=observation_id, arm_endpoint=self.client.status()["endpoint"],
                        joint_degree=feedback["joints"], pose=feedback["pose"], frames=frames,
                        config=active["config"], depth_encoding="zlib/uint16-le",
                        depth_aligned_to="color", pose_units={"position": "m", "angle": "rad"})
        try:
            (directory / "photo.jpg").write_bytes(photo["jpeg"])
            (directory / "depth.zlib").write_bytes(photo["depth_zlib"])
            (directory / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
        except Exception:
            shutil.rmtree(directory)
            raise
        return {"ok": True, "media_type": "photo", "observation_id": observation_id,
                "file_path": str(directory / "photo.jpg"), "metadata_path": str(directory / "metadata.json"),
                "captured_at": photo["captured_at"], "width": photo["width"], "height": photo["height"],
                "pose": list(feedback["pose"]),
                "objects": photo.get("objects", []), "count": len(photo.get("objects", [])),
                "objects_timestamp": photo.get("objects_timestamp"),
                "input_identity": photo.get("input_identity", self._inputs.identity()),
                "synchronization": photo.get("synchronization", {})}

    def _run_observe(self, active):
        try:
            result = self._observe(active)
            status = "completed"
        except Exception as exc:
            status, result = self._failure(active, exc)
        return self._finish(active, status, result)

    def _observe(self, active, capture=True):
        # Shared by observe and a completed transfer, under the same device lock,
        # cancellation token and action_id. Only the outer action sends completion.
        if capture:
            active["observation_id"] = "vision_pick_and_drop_observe_" + uuid4().hex
        motion = ObservationMotion(self.client, active["cancel"],
                                   deadline=time.monotonic() + ACTION_TIMEOUT_SECONDS)
        result = None
        try:
            config = active["config"]
            target = [float(value) for value in config["observation_joints_deg"].split(",")]
            motion.validate_target(target)
            motion.settled()
            # A completed transfer returns even when image inputs are unavailable.
            if active["action"] == "observe":
                deadline = time.monotonic() + 12
                while True:
                    motion.read()
                    readiness = self._inputs.info()
                    if readiness["state"] == "error":
                        raise RuntimeError(readiness["error"])
                    if readiness["fresh"]:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("RGB, depth, calibration or VOP input is unavailable; observation motion not submitted")
                    active["cancel"].wait(0.1)
            motion.settled()
            frames = motion.frames()
            with self._config_lock:
                motion.check_cancel()
                active["motion_sent"] = True
                self.client.command("rm_movej", target, config["speed_percent"], 0, 0, 0)
            def camera_ready():
                state = self._inputs.info()
                if state["state"] == "error":
                    raise RuntimeError(state.get("error") or "Observation input feedback stopped")
                return state["state"] == "running" and state["fresh"]

            feedback = motion.settled(target, timeout=ACTION_TIMEOUT_SECONDS)
            if motion.frames() != frames:
                raise RuntimeError("Coordinate frames changed during observation")

            if active["action"] != "observe":
                active["return_completed"] = True
            if not capture:
                with self._config_lock:
                    motion.check_cancel()
                    active["finished"] = True
                return {"pose": list(feedback["pose"])}
            if active["action"] != "observe":
                active["stage"] = "observing"

            def still_at_observation():
                nonlocal feedback
                current = motion.read()
                if motion.frames() != frames:
                    raise RuntimeError("Coordinate frames changed during observation")
                if not pose_close(current["pose"], feedback["pose"], distance=0.002, angle=1):
                    raise RuntimeError("Arm moved during photograph capture")
                if (not current["idle"] or not pose_close(current["pose"], feedback["pose"], distance=0.0003)
                        or any(abs(a-b) > 0.2 for a, b in zip(current["joints"], target))):
                    feedback = motion.settled(target)
                    # Tell snapshot to drop RGB-D/VOP from before settling.
                    return False
                return camera_ready()

            while True:
                photo = self._inputs.snapshot(time.time() + 0.15, active["cancel"], still_at_observation)
                after = motion.settled(target, check=camera_ready)
                if motion.frames() != frames or not pose_close(after["pose"], feedback["pose"], distance=0.002, angle=1):
                    raise RuntimeError("Observation pose or coordinate frames changed during capture")
                if pose_close(after["pose"], feedback["pose"], distance=0.0003):
                    break
                # No new arm command: capture a new window at the settled pose.
                feedback = after
            result = self._save_photo(photo, active, feedback, frames)
            with self._config_lock:
                motion.check_cancel()
                self._observation = dict(result)
                active["finished"] = True
            return result
        except Exception as exc:
            cleanup_error = None
            if result and result.get("file_path"):
                try:
                    shutil.rmtree(Path(result["file_path"]).parent)
                except OSError as cleanup_exc:
                    cleanup_error = str(cleanup_exc)
            if cleanup_error:
                raise RuntimeError(f"{exc}; observation cleanup failed: {cleanup_error}") from exc
            raise

    def dispatch(self, action, args):
        if action == "config":
            updates = {key: value for key, value in args.items() if key not in ("_tool_name", "instance_id")}
            if updates.keys() - CONFIG_PROPERTIES.keys():
                return {"ok": False, "code": "INVALID_CONFIG", "message": "Unknown vision_pick_and_drop configuration field"}
            with self._config_lock:
                if self._active is not None:
                    return {"ok": False, "code": "ACTION_IN_PROGRESS", "message": "Wait for the current action to finish before configuring"}
                config = {**self._config, **updates}
                try:
                    validate_config(config)
                except (ValueError, OverflowError) as exc:
                    return {"ok": False, "code": "INVALID_CONFIG", "message": str(exc)}
                if config != self._config:
                    self._observation = None
                self._config = config
                return {"ok": True, **config}
        if action == "start":
            return self.start(args)
        if action == "stop":
            return self.stop()
        if action == "cancel":
            return self._cancel()
        if action == "info":
            with self._config_lock:
                self._invalidate_changed_source()
                return {"state": "running" if self._active else "error" if self._motion_blocked else "ready",
                        "topic_in": self._inputs.topics(), "inputs": self._inputs.info(),
                        "motion_blocked": self._motion_blocked,
                        "observation_required": self._observation is None,
                        "last_result": copy.deepcopy(self._last_result), "observation": copy.deepcopy(self._observation)}
        if action in MOTION_ACTIONS:
            return self._execute(action, args)
        return None
