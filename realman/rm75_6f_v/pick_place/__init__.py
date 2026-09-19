"""Self-contained RealMan pick-and-place actuator and single-photo observation."""

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
from .camera import SnapshotCameras
from .geometry import displacement, positions
from .transfer import Transfer
from hardware import JOINT_LIMITS_DEG


# Core allows 60 seconds per synchronous actuator call; leave time to stop and clean up.
ACTION_TIMEOUT_SECONDS = 45


CONFIG_PROPERTIES = {
    "speed_percent": {
        "type": "integer", "minimum": 1, "maximum": 100, "default": 50,
        "description": "全局运行速度（%），用于观察位、水平移动及抓放升降。",
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
        "type": "integer", "minimum": 0, "maximum": 100, "default": 10,
        "description": "抓取时的夹持力度，设备原始整数 0～100，非牛顿值。",
    },
    "pick_descent_mm": {
        "type": "number", "exclusiveMinimum": 0, "default": 91,
        "description": "抓取时从当前位置向下移动的距离（mm）。",
    },
    "place_descent_mm": {
        "type": "number", "exclusiveMinimum": 0, "default": 87,
        "description": "放置时从当前位置向下移动的距离（mm）。",
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
    PREFIX = "pick_place"

    def __init__(self, client, config, namespace="rm75", ros2=None, cameras=None):
        self.client = client
        self._ros2 = ros2
        self._cameras = cameras if cameras is not None else SnapshotCameras()
        self._topic = f"/{namespace.strip('/') or 'rm75'}/pick_place/photo"
        self._output_dir = Path(config.get("pick_place", {}).get(
            "output_dir", "/opt/phanthy-motus/data/pick_place/realman"))
        self._config = {name: prop["default"] for name, prop in CONFIG_PROPERTIES.items()}
        self._config_lock = threading.RLock()
        self._active = None
        self._last_result = None
        self._observation = None
        self._motion_blocked = False
        self._node = self._publisher = None

    def _outputs(self):
        return [{"topic": self._topic, "format": "image/jpeg", "desc": "观察完成后输出一张照片"}]

    def get_tools(self):
        transfer_guidance = (
            "前置步骤：先调用 observe，成功后读取本次照片的目标检测结果，再选择目标中心。"
            "检测结果有观察编号时须与 observation_id 一致，否则核对结果时间不早于 captured_at；"
            "目标缺失或无法确定时报告或询问用户，不猜测坐标。"
            "每张照片仅供一次搬运：首次下发设备命令即失效，失败或取消也不能复用。"
            "下一次搬运必须重新 observe 并使用新检测坐标，多个物体也须逐次观察和搬运。"
            "调用直接完成抓起、搬运、放下及回升，同步返回，无需轮询或重复下发。"
            "state=completed 且 result.ok=true 表示动作完成；grasp_checked=false 表示未自动核验实际抓到物体。"
            "observation_required=true 表示再次搬运前需要新照片，无有效照片会返回 OBSERVATION_REQUIRED；不要求任务完成后继续拍照。"
            "失败或取消时报告原因，不自动重试；完成用户要求后报告结果并结束任务。")
        properties = {name: {"type": "number", "minimum": -1, "maximum": 1, "description": description}
                      for name, description in (
                          ("x1", "抓取中心在 observe 照片中的归一化 X 坐标，范围 [-1,1]：左边缘 -1、中心 0、右边缘 +1，正方向向右。可直接使用该照片的 VOP position[0]，无需换算像素。"),
                          ("y1", "抓取中心在 observe 照片中的归一化 Y 坐标，范围 [-1,1]：上边缘 -1、中心 0、下边缘 +1，正方向向下。可直接使用该照片的 VOP position[1]，无需换算像素。"),
                          ("x2", "transfer_to 放置位置在 observe 照片中的归一化 X 坐标，范围 [-1,1]：左边缘 -1、中心 0、右边缘 +1，正方向向右，与 x1 相同。"),
                          ("y2", "transfer_to 放置位置在 observe 照片中的归一化 Y 坐标，范围 [-1,1]：上边缘 -1、中心 0、下边缘 +1，正方向向下，与 y1 相同。"))}
        properties.update({
            "dx_mm": {"type": "number", "description": "transfer_by 从抓取点沿桌面左右平移的有符号距离（mm），可为小数。以 observe 照片为准：X 正方向向右（正值），负方向向左（负值），0 不左右移动；当前安装对应基坐标 ΔX=-dx_mm。不是像素或绝对位置。"},
            "dy_mm": {"type": "number", "description": "transfer_by 从抓取点沿桌面上下平移的有符号距离（mm），可为小数。以 observe 照片为准：Y 正方向向照片下方（正值），负方向向上方（负值），0 不上下平移；当前安装对应基坐标 ΔY=dy_mm。不是机械臂 Z 升降、像素或绝对位置。"},
        })
        schema = action_schema({
            "observe": ([], (
                "观察桌面，或为一次搬运获取新照片。无参数；按卡片配置移动到观察位，停稳后拍摄并输出一张照片，"
                "同步完成后返回 state=completed、result.ok=true，以及 observation_id、captured_at、width、height、topic_out。"
                "这是单次拍照，该动作本身不识别物体；通过已连接的视觉/目标检测工具读取本次照片的结构化检测结果。"
                f"VOP 连线时检测主题为 {self._topic}/objects，可通过框架的传感器数据查询工具读取；"
                "核对检测结果对应本次照片，尚未收到时等待新结果，不使用历史检测。"
                "VOP position[0]、position[1] 可直接作为搬运的 x1、y1，无需读取本地照片文件或换算像素。"
                "一张照片只能用于一次搬运；每次搬运后再次移动任何物体，都须重新 observe 并重新检测。"
                "失败或取消时报告原因，不自动重试。")),
            "transfer_to": (["x1", "y1", "x2", "y2"], (
                "指定目标点：用于把物体放到照片中的指定位置，或另一物体旁的空位。"
                "(x1,y1) 是抓取物体中心，(x2,y2) 是放置点，四个值均为同次 observe 照片的归一化坐标 [-1,1]。"
                "照片中心为 (0,0)，X 向右、Y 向下为正；VOP position[0] 对应 X、position[1] 对应 Y，"
                "无需换算像素或读取照片文件。放在另一物体旁边时选择其旁的空位，不能把参照物中心直接当作空位；"
                "指定毫米距离的相对移动使用 transfer_by。" + transfer_guidance)),
            "transfer_by": (["x1", "y1", "dx_mm", "dy_mm"], (
                "指定距离（mm）：用于把一个物体向左、右、照片上方或下方移动指定距离。"
                "(x1,y1) 为 observe 照片中物体中心的归一化坐标 [-1,1]，照片中心为 (0,0)，"
                "直接使用 VOP position[0]、position[1]，无需换算像素或读取照片文件。"
                "dx_mm、dy_mm 是从物体抓取点出发的桌面毫米位移，两项都要填写，至少一项非零；1 cm=10 mm。"
                "方向以照片为准：X 正方向向右、Y 正方向向下，负值反向；上/下也是桌面平移，不是 Z 升降。"
                "向右 30 mm：dx_mm=30、dy_mm=0；向左 30 mm：dx_mm=-30、dy_mm=0；"
                "向照片上方 20 mm：dx_mm=0、dy_mm=-20；向下 20 mm：dx_mm=0、dy_mm=20。"
                "放到照片中指定位置使用 transfer_to。" + transfer_guidance)),
            "cancel": ([], "中止当前动作，停止后终止后续步骤，不自动回程或释放夹爪；再次搬运前重新 observe。"),
        }, properties)
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "cancel"},
                             "on_interrupt_all": {"action": "cancel"}}
        schema["x-is-dangerous"] = True
        schema["x-resource"] = "arm"
        definition = tool("pick_place", "actuator", (
            "观察并搬运桌面物体。先 observe 获取单张照片，再从本次照片的视觉检测结果选择物体；"
            "放到指定位置用 transfer_to，按方向移动指定毫米距离用 transfer_by。"
            "位置为照片中心归一化坐标 [-1,1]，X 向右、Y 向下为正，可直接使用 VOP position；"
            "毫米位移与位置坐标分别填写。每张照片仅供一次搬运，下一次搬运前必须重新观察并检测。"
            "动作同步执行并返回结果，完成用户要求后报告并结束；失败不自动重试。"),
                          schema, topic_out=self._outputs())
        definition["configSchema"] = {
            "type": "object", "properties": copy.deepcopy(CONFIG_PROPERTIES), "additionalProperties": False,
        }
        return [definition]

    def start(self):
        # Canvas activation advertises the output without capturing or moving.
        with self._config_lock:
            self._ensure_publisher()
        return {"state": "ready", "topic_out": self._outputs()}

    def _ensure_publisher(self):
        if self._publisher is not None:
            return
        if self._ros2 is None:
            raise RuntimeError("ROS context is required for photograph output")
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import CompressedImage

        node = Node("realman_pick_place", context=self._ros2.executor_core.context)
        try:
            publisher = node.create_publisher(CompressedImage, self._topic, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE))
            self._ros2.executor_core.add_node(node)
        except Exception:
            node.destroy_node()
            raise
        self._node, self._publisher = node, publisher

    def _publish_photo(self, photo, observation_id):
        from sensor_msgs.msg import CompressedImage

        msg = CompressedImage()
        seconds = photo["captured_at"]
        msg.header.stamp.sec = int(seconds)
        msg.header.stamp.nanosec = int((seconds % 1) * 1e9)
        msg.header.frame_id = observation_id
        msg.format = "jpeg"
        msg.data = photo["jpeg"]
        self._publisher.publish(msg)

    def stop(self):
        self._cancel()
        with self._config_lock:
            active = self._active
        if active is not None:
            active["done"].wait(timeout=8)
        with self._config_lock:
            if self._active is not None:
                return {"state": "stopping"}
            if self._node is not None:
                self._ros2.executor_core.remove_node(self._node)
                self._node.destroy_node()
                self._node = self._publisher = None
            self._observation = None
        return {"state": "error" if self._motion_blocked else "idle",
                "motion_blocked": self._motion_blocked}

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

    def _execute(self, action, args):
        with self._config_lock:
            if not self.client.connected:
                return {"state": "error", "message": "Mechanical arm is not connected"}
            if not self.client.motion_enabled:
                return {"state": "error", "message": "Driver is in read-only mode; deploy with --mode live to execute actions"}
            if self._motion_blocked:
                return {"state": "error", "message": "Stop could not be verified; inspect the arm before restarting the driver"}
            if self._active is None and action != "observe" and self._observation is None:
                return {"state": "error", "code": "OBSERVATION_REQUIRED", "observation_required": True,
                        "message": "Run observe and obtain new detections before transferring. "
                                   "No usable photograph is available; each photograph permits only one transfer."}
            if self._active is not None or not self.client.motion_lock.acquire(blocking=False):
                return {"state": "error", "message": "Another device action is active"}
            try:
                active = {"config": dict(self._config), "cancel": threading.Event(),
                          "done": threading.Event(), "motion_sent": False}
                if action == "observe":
                    self._ensure_publisher()
                    active["observation_id"] = "pick_place_observe_" + uuid4().hex
                    self._observation = None
                else:
                    if action == "transfer_by":
                        active["positions"] = positions(args, ("x1", "y1"))
                        active["displacement_mm"] = displacement(args)
                    else:
                        active["positions"] = positions(args)
                    active["observation"] = dict(self._observation)
                self._active = active
            except Exception as exc:
                self._active = None
                self.client.motion_lock.release()
                return {"state": "error", "message": str(exc)}
        return self._run_observe(active) if action == "observe" else self._run_transfer(active)

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
            terminal = {"state": status, "result": result,
                        "observation_required": self._observation is None}
            self._last_result = terminal
            self._active = None
            if not self._motion_blocked:
                self.client.motion_lock.release()
            active["done"].set()
        return terminal

    def _run_transfer(self, active):
        motion = ObservationMotion(self.client, active["cancel"],
                                   deadline=time.monotonic() + ACTION_TIMEOUT_SECONDS)
        def send(method, *args):
            with self._config_lock:
                motion.check_cancel()
                # Moving the scene consumes the photograph; invalid input does not.
                self._observation = None
                active["motion_sent"] = True
                self.client.command(method, *args)

        def stage(value):
            active["stage"] = value

        try:
            result = Transfer(self.client, motion, active["observation"], active["config"],
                              active["positions"], send, stage, active.get("displacement_mm")).run()
            with self._config_lock:
                motion.check_cancel()
                active["finished"] = True
            status = "completed"
        except Exception as exc:
            status, result = self._failure(active, exc)
            result["stage"] = active.get("stage", "checking")
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
                "topic_out": self._outputs()}

    def _run_observe(self, active):
        motion = ObservationMotion(self.client, active["cancel"],
                                   deadline=time.monotonic() + ACTION_TIMEOUT_SECONDS)
        camera = result = None
        status = "error"
        try:
            config = active["config"]
            target = [float(value) for value in config["observation_joints_deg"].split(",")]
            motion.validate_target(target)
            motion.settled()
            camera = self._cameras.select()
            camera.start()
            deadline = time.monotonic() + 12
            while True:
                motion.read()
                readiness = camera.info()
                if readiness["state"] == "error":
                    raise RuntimeError(readiness["error"])
                if readiness["fresh"]:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("RealSense startup timed out; no motion submitted")
                active["cancel"].wait(0.1)
            motion.settled()
            frames = motion.frames()
            with self._config_lock:
                motion.check_cancel()
                active["motion_sent"] = True
                self.client.command("rm_movej", target, config["speed_percent"], 0, 0, 0)
            def camera_ready():
                state = camera.info()
                if state["state"] != "running" or not state["fresh"]:
                    raise RuntimeError(state.get("error") or "RealSense feedback stopped")

            feedback = motion.settled(target, timeout=ACTION_TIMEOUT_SECONDS, check=camera_ready)
            if motion.frames() != frames:
                raise RuntimeError("Coordinate frames changed during observation")

            def still_at_observation():
                camera_ready()
                current = motion.read()
                if (motion.frames() != frames or not current["idle"] or not pose_close(current["pose"], feedback["pose"], distance=0.0003)
                        or any(abs(a-b) > 0.2 for a, b in zip(current["joints"], target))):
                    raise RuntimeError("Arm moved during photograph capture")

            photo = camera.snapshot(time.time() + 0.15, active["cancel"], still_at_observation)
            after = motion.settled(target, check=camera_ready)
            if not pose_close(after["pose"], feedback["pose"], distance=0.0003) or motion.frames() != frames:
                raise RuntimeError("Observation pose or coordinate frames changed during capture")
            result = self._save_photo(photo, active, feedback, frames)
            with self._config_lock:
                motion.check_cancel()
                self._publish_photo(photo, active["observation_id"])
                self._observation = dict(result)
                status = "completed"
                active["finished"] = True
        except Exception as exc:
            cleanup_error = None
            if result and result.get("file_path"):
                try:
                    shutil.rmtree(Path(result["file_path"]).parent)
                except OSError as cleanup_exc:
                    cleanup_error = str(cleanup_exc)
            status, result = self._failure(active, exc)
            if cleanup_error:
                result["cleanup_error"] = cleanup_error
        finally:
            if camera is not None:
                try:
                    camera.stop()
                except Exception as exc:
                    result["camera_cleanup_error"] = str(exc)
            terminal = self._finish(active, status, result)
        return terminal

    def dispatch(self, action, args):
        if action == "config":
            updates = {key: value for key, value in args.items() if key not in ("_tool_name", "instance_id")}
            if updates.keys() - CONFIG_PROPERTIES.keys():
                return {"ok": False, "code": "INVALID_CONFIG", "message": "Unknown pick_place configuration field"}
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
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "cancel":
            return self._cancel()
        if action == "info":
            with self._config_lock:
                return {"state": "running" if self._active else "error" if self._motion_blocked else "ready", "topic_out": self._outputs(),
                        "motion_blocked": self._motion_blocked,
                        "observation_required": self._observation is None,
                        "last_result": copy.deepcopy(self._last_result), "observation": copy.deepcopy(self._observation)}
        if action in ("observe", "transfer_to", "transfer_by"):
            return self._execute(action, args)
        return {"state": "error", "message": "Action is not implemented"}
