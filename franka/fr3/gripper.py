"""IP-bound Franka Hand card using the existing lifecycle and ACP machinery."""

from __future__ import annotations

import threading

from common.vendor_runtime import action_schema, tool
from device import FrankaPlugin, JointControlPlugin
from franka.fr3.gripper_hardware import GripperSession
from hardware import SessionManager


class GripperPlugin(JointControlPlugin):
    PREFIX = "gripper"
    CONFIG_FIELDS = FrankaPlugin.CONFIG_FIELDS + ("speed_mm_s", "force_n")
    MOTION_ACTIONS = ("set_position", "grasp", "homing")
    ARM_ACTIONS = (*MOTION_ACTIONS, "stopmotion")

    def __init__(self, *args, manager=None, **kwargs):
        # The Hand has its own SDK connection, independently of the arm FCI loop.
        super().__init__(*args, manager=manager if manager is not None else
                         SessionManager(session_factory=GripperSession), **kwargs)

    @staticmethod
    def _validate(cfg):
        cfg = FrankaPlugin._validate(cfg)
        cfg["speed_mm_s"] = JointControlPlugin._number(
            cfg.get("speed_mm_s", 20), "speed_mm_s", 0.1, 100)
        cfg["force_n"] = JointControlPlugin._number(cfg.get("force_n", 30), "force_n", 30, 70)
        return cfg

    def get_tools(self):
        config = FrankaPlugin.get_tools(self)[0]["configSchema"]
        # Match joint_control's per-card binding on older actuator canvases.
        config.pop("required", None)
        config["properties"].pop("publish_hz", None)
        config["properties"].update({
            "speed_mm_s": {"type": "number", "title": "默认开口速度 (mm/s)",
                           "scope": "instance", "minimum": 0.1, "maximum": 100, "default": 20},
            "force_n": {"type": "number", "title": "默认抓取力 (N)",
                        "scope": "instance", "minimum": 30, "maximum": 70, "default": 30},
        })
        properties = {
            "output_topic": {"type": "string",
                             "description": "由画布输出端口自动传入，用于匹配本卡；无需填写。"},
            "width_mm": {"type": "number", "title": "目标开口宽度 (mm)",
                         "minimum": 0, "maximum": 80,
                         "description": "两指之间的总距离；上限取 80 mm 与当前标定最大开口中的较小值。"},
            "speed_mm_s": {"type": "number", "title": "开口速度 (mm/s)",
                           "minimum": 0.1, "maximum": 100,
                           "description": "总开口宽度的变化速度；留空使用卡片默认速度。"},
            "force_n": {"type": "number", "title": "抓取力 (N)",
                        "minimum": 30, "maximum": 70,
                        "description": "留空使用卡片默认抓取力。"},
            "epsilon_inner_mm": {"type": "number", "title": "内侧容差 (mm)",
                                 "minimum": 0, "maximum": 80, "default": 5,
                                 "description": "允许实际物体宽度比目标宽度小的范围。"},
            "epsilon_outer_mm": {"type": "number", "title": "外侧容差 (mm)",
                                 "minimum": 0, "maximum": 80, "default": 5,
                                 "description": "允许实际物体宽度比目标宽度大的范围。"},
        }
        schema = action_schema({
            "set_position": (["width_mm", "speed_mm_s"], "将夹爪移动到指定开口宽度（mm）。"),
            "grasp": (["width_mm", "speed_mm_s", "force_n", "epsilon_inner_mm", "epsilon_outer_mm"],
                      "按目标物体宽度和抓取力夹持；成功后保持夹持力。"),
            "homing": ([], "执行夹爪标定，夹爪会完整打开并闭合，以测量最大开口。"),
            "stopmotion": ([], "停止本机械臂夹爪动作，并解除夹持力；卡片保持可用。"),
            "info": ([], "查看实际开口、最大开口、抓取状态、温度及执行结果。"),
        }, properties)
        schema["properties"]["target_robot_ip"] = {
            "type": "string",
            "description": "填写要执行动作的机械臂 IP，须与已启动卡片的配置一致。",
        }
        for action in schema["x-action-params"].values():
            action["params"].append("target_robot_ip")
        self._arm_schema(config, schema)
        schema["x-completion"] = {"actions": list(self.MOTION_ACTIONS), "timeout": 130}
        schema["x-resource"] = "arm"
        schema["x-is-dangerous"] = True
        schema["x-hooks"] = {key: {"action": "stop"}
                             for key in ("on_interrupt_motion", "on_interrupt_all")}
        return [{**tool(self.PREFIX, "actuator", "Franka 夹爪：开口宽度、抓取、标定与停止。", schema),
                 "multiInstance": True, "configSchema": config}]

    def _topic(self, instance_id):
        return [{"topic": f"/{self._namespace}/{instance_id.replace('-', '_')}/state/gripper",
                 "format": "data/json"}]

    def _info(self, instance_id, instance):
        snapshot = instance.session.snapshot() if instance.session is not None else {}
        motion = instance.session.motion() if instance.session is not None else {}
        active = instance.session is not None
        connected = snapshot.get("connected", False)
        configuration_error = getattr(instance, "configuration_error", None)
        state = ("ready" if connected else "loading") if active else "idle"
        if active and motion.get("status") in ("submitted", "accepted", "running", "stopping"):
            state = "running"
        if configuration_error:
            state = "error"
        width = snapshot.get("width")
        maximum = snapshot.get("max_width")
        target = (motion.get("target") or {}).get("width_m")
        speed = (motion.get("target") or {}).get("speed_m_s")
        native_speed = (motion.get("native_parameters") or {}).get("speed_m_s")
        return {
            "state": state, "robot_ip": instance.config.get("robot_ip", ""),
            "connected": connected, "readiness": snapshot.get("state", "idle"),
            "control_ready": bool(active and connected and not configuration_error
                                  and snapshot.get("control_ready", False)),
            "control_error": snapshot.get("control_error"),
            "data_age_ms": snapshot.get("data_age_ms"), "timestamp_ms": snapshot.get("timestamp_ms"),
            "width_mm": None if width is None else width * 1000,
            "max_width_mm": None if maximum is None else maximum * 1000,
            "target_width_mm": None if target is None else target * 1000,
            "speed_mm_s": None if speed is None else speed * 1000,
            "speed_source": (motion.get("target") or {}).get("speed_source"),
            "native_speed_mm_s": None if native_speed is None else native_speed * 1000,
            "is_grasped": snapshot.get("is_grasped"), "temperature_c": snapshot.get("temperature"),
            "homed": None if maximum is None else maximum > 0,
            "motion": motion, "last_result": getattr(instance, "last_result", None),
            "acp": getattr(instance, "acp", None), "read_only": False,
            "error": configuration_error or snapshot.get("error"),
            "position_unit": "mm", "speed_unit": "mm/s", "force_unit": "N",
            "topic_in": [], "topic_out": [],
        }

    def _dispatch(self, action, args):
        if args.get("_tool_name", action) != self.PREFIX:
            return None
        if action in (*self.MOTION_ACTIONS, "stopmotion") or (
                action in ("info", "stop") and "output_topic" in args):
            args = self._resolve_card(args)
        if action == "stop" and not args.get("instance_id"):
            # Interrupt only in-flight commands. A completed grasp keeps holding.
            with self._lock:
                sessions = {instance.session for instance in self._instances.values()
                            if instance.session is not None}
            outcomes, errors = [], []
            for session in sessions:
                try:
                    outcomes.append({"robot_ip": session.address, "motion": session.stop_motion()})
                except Exception as exc:
                    errors.append(str(exc))
            return {"state": "error" if errors else "stop_requested", "robots": outcomes,
                    **({"error": "; ".join(errors)} if errors else {})}
        if action in ("config", "start", "info", "stop"):
            result = FrankaPlugin.dispatch(self, action, args)
            if result is not None:
                result["read_only"] = False
            return result
        if action not in (*self.MOTION_ACTIONS, "stopmotion"):
            return None
        instance_id = self._instance_id(args)
        instance = self._get_instance(instance_id)
        with instance.lock:
            if "target_robot_ip" in args and instance.config.get("robot_ip") != args["target_robot_ip"]:
                raise ValueError("target_robot_ip does not match this card")
            session = instance.session
            if session is None:
                raise RuntimeError("Start this gripper card before requesting an action")
            if action == "stopmotion":
                motion = session.stop_gripper(self._owner_key(instance_id))
                if motion.get("command_id"):
                    motion = session.wait(motion["command_id"], timeout=3.5)
                if motion.get("status") not in ("succeeded", "cancelled"):
                    raise RuntimeError("Gripper stop was not confirmed: " + str(motion.get("error")))
                return {**self._info(instance_id, instance), "motion": motion}
            if getattr(instance, "configuration_error", None):
                raise RuntimeError("Resolve this card's rejected configuration before motion: "
                                   + instance.configuration_error)
            options = {"timeout_s": 120}
            if action != "homing":
                options.update(
                    width_m=self._number(args.get("width_mm"), "width_mm", 0, 80) / 1000,
                    speed_m_s=self._number(args.get("speed_mm_s", instance.config["speed_mm_s"]),
                                           "speed_mm_s", 0.1, 100) / 1000,
                    speed_source="argument" if "speed_mm_s" in args else "config")
            if action == "grasp":
                options.update(
                    force_n=self._number(args.get("force_n", instance.config["force_n"]), "force_n", 30, 70),
                    epsilon_inner_m=self._number(args.get("epsilon_inner_mm", 5), "epsilon_inner_mm", 0, 80) / 1000,
                    epsilon_outer_m=self._number(args.get("epsilon_outer_mm", 5), "epsilon_outer_mm", 0, 80) / 1000)
            action_id = session.command(self._owner_key(instance_id), action, **options)
            instance.last_action_id = action_id
            instance.acp = {"action_id": action_id, "state": "waiting_for_motion"}
            threading.Thread(target=self._complete, args=(instance, session, action_id),
                             name=f"franka-gripper-acp-{instance_id}", daemon=True).start()
            return {"state": "running", "action_id": action_id, "action": action,
                    "target_width_mm": None if action == "homing" else options["width_m"] * 1000,
                    "speed_mm_s": None if action == "homing" else options["speed_m_s"] * 1000,
                    "speed_source": options.get("speed_source")}
