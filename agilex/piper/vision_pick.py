"""RGB-D observation, pick planning and supervised transfer for one PiPER arm."""

from copy import deepcopy
import hashlib
import math
from pathlib import Path
from threading import RLock
import time
from uuid import uuid4

import numpy as np

from agilex.piper.completion import run_async
from agilex.piper.user_motion import tcp_transform
from agilex.piper.vision_kinematics import PiperKinematics
from agilex.piper.vision_rgbd import ObservationSource, camera_point
from agilex.piper.vision_transfer import PickTransfer
from common.vendor_runtime import action_schema, tool


NAME = "vision_pick_and_drop"
OBSERVATION_LIFETIME = 30
SCENE_TOLERANCE_MM = 8
DEFAULT_URDF = Path(__file__).with_name("resource") / "upstream/piper_description.urdf"


class PiperVisionPick:
    def __init__(self, config, namespace, ros2, runtime):
        vision = config["vision_pick"]
        self.can_interface = config.get("can_interface", "can0")
        self.rgb_topic = "/" + "/".join(
            part for part in (namespace.strip("/"), "piper/vision_pick/rgb") if part
        )
        self.lock = RLock()
        self.observation = None
        self.plan = None
        self.last_transfer = None
        self.runtime = runtime
        self.calibration = vision.get("calibration")
        joint_topic = vision.get("joint_state_topic")
        self.wrist = bool(joint_topic) or (
            isinstance(self.calibration, dict) and self.calibration.get("reference_frame") == "link6"
        )
        self.camera_transform = None
        model = None
        if self.calibration is not None:
            value = self.calibration
            if (not isinstance(value, dict) or value.get("arm_can_interface") != self.can_interface
                    or not all(isinstance(value.get(key), str) and value[key].strip()
                               for key in ("id", "optical_frame_id"))):
                raise ValueError("Calibration must identify its CAN interface, camera optical frame and ID")
            rotation = np.asarray(value.get("rotation"), dtype=float)
            translation = np.asarray(value.get("translation_mm"), dtype=float)
            if (rotation.shape != (3, 3) or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-4)
                    or not np.isclose(np.linalg.det(rotation), 1, atol=1e-4)
                    or translation.shape != (3,) or not np.isfinite(translation).all()):
                raise ValueError("Calibration requires a proper 3×3 rotation and a finite XYZ translation")
            expected_frame = "link6" if self.wrist else "base_link"
            if value.get("reference_frame") != expected_frame:
                raise ValueError(f"Calibration must transform the camera optical frame into {expected_frame}")
            self.camera_transform = np.eye(4)
            self.camera_transform[:3, :3] = rotation
            self.camera_transform[:3, 3] = translation
            if self.wrist:
                arm_urdf = Path(vision.get("arm_urdf", DEFAULT_URDF)).read_bytes()
                if hashlib.sha256(arm_urdf).hexdigest() != value.get("arm_urdf_sha256"):
                    raise ValueError("PiPER URDF does not match the wrist calibration")
                model = PiperKinematics(arm_urdf)
        if self.camera_transform is None:
            raise ValueError(f"Configure the installed camera calibration before enabling {NAME}")
        self.transfer = None
        self.transfer_setup_error = None
        try:
            self.transfer = PickTransfer(vision.get("transfer"), tcp_transform(config))
        except (ValueError, RuntimeError) as exc:
            self.transfer_setup_error = str(exc)
        joint_feedback = runtime.read_cached_joint_feedback if self.wrist and not joint_topic else None
        self.source = ObservationSource(ros2, vision, self.rgb_topic, model, joint_feedback=joint_feedback)

    def rgb_output(self):
        return [{"topic": self.rgb_topic, "format": "image/jpeg"}]

    def get_tools(self):
        actions = {
            "start": ([], "Subscribe to RGB-D and publish RGB for VOP"),
            "stop": ([], "Stop this card and cancel any active transfer"),
            "cancel": ([], "Stop an active transfer and keep motor torque"),
            "info": ([], "Read camera, motion permission and transfer status"),
            "observe": ([], "Save a fresh RGB-D observation of a stationary scene"),
            "plan_pick": (["observation_id", "start_point_x", "start_point_y"],
                          "Preview the 3D pick point and any configured drop path"),
        }
        fields = {
            "observation_id": {"type": "string", "title": "Observation ID"},
            "start_point_x": {"type": "number", "minimum": -1, "maximum": 1,
                              "title": "Pick X (positive right)", "description": "Use VOP position[0]."},
            "start_point_y": {"type": "number", "minimum": -1, "maximum": 1,
                              "title": "Pick Y (positive down)", "description": "Use VOP position[1]."},
        }
        if self.transfer is not None:
            actions["grab_to"] = (["plan_id", "confirm_motion"], "Run the reviewed pick and drop plan")
            fields.update({
                "plan_id": {"type": "string", "title": "Plan ID from preview"},
                "confirm_motion": {"type": "boolean", "default": False,
                                   "title": "I checked the planned path and cleared the workspace"},
            })
        schema = action_schema(actions, fields)
        schema["properties"]["action"]["default"] = "info"
        schema.update({"additionalProperties": False, "x-manual-execution": True, "x-resource": "arm",
                       "x-hooks": {hook: {"action": "cancel"}
                                   for hook in ("on_interrupt_motion", "on_interrupt_all")}})
        if self.transfer is not None:
            schema["x-is-dangerous"] = True
            schema["x-completion"] = {"actions": ["grab_to"], "timeout": 450}
        description = ("Pick from aligned RGB-D and move to the configured drop point."
                       if self.transfer is not None else "Preview a 3D pick point from calibrated aligned RGB-D.")
        definition = tool(NAME, "actuator", description,
                          schema, topic_out=self.rgb_output())
        definition["title"] = NAME
        return [definition]

    def info(self):
        source = self.source.status()
        with self.runtime.lifecycle:
            running = self.runtime.active_card == NAME
            stopped = self.runtime.closed or NAME in self.runtime.stopped_cards
        state = "idle"
        if running:
            state = "running"
        elif source["subscribed"] and not stopped:
            rgbd_ready = source["rgb_fresh"] and source["depth_fresh"] and source["camera_info_received"]
            state = "ready" if rgbd_ready and (not self.wrist or source["joints_fresh"]) else "awaiting_feedback"
        return {"state": state,
                "can_interface": self.can_interface, "source": source,
                "rgb_topic": self.rgb_topic, "topic_out": self.rgb_output(),
                "calibration_id": self.calibration["id"],
                "motion_permission": self.runtime.motion_enabled,
                "transfer_setup_error": self.transfer_setup_error,
                "last_transfer": deepcopy(self.last_transfer)}

    def start(self):
        with self.lock:
            with self.runtime.lifecycle:
                if self.runtime.closed or self.runtime.active_card == NAME:
                    raise RuntimeError("Wait until the current transfer has stopped")
                self.runtime.stopped_cards.discard(NAME)
            self.source.start()
            return self.info()

    def stop(self):
        with self.lock:
            result = self.runtime.request_stop(NAME)
            self.observation = self.plan = None
            self.source.stop()
            return result

    def observe(self):
        if self.runtime.active_card == NAME:
            raise RuntimeError("Wait for the current transfer before observing again")
        self.observation = self.plan = None
        snapshot = self.source.snapshot()
        if snapshot["info"].header.frame_id != self.calibration["optical_frame_id"]:
            raise ValueError("Camera optical frame does not match the calibration")
        snapshot.update(id=uuid4().hex, expires=time.monotonic() + OBSERVATION_LIFETIME)
        self.observation = snapshot
        return {"state": "observed", "observation_id": snapshot["id"],
                "expires_in_seconds": OBSERVATION_LIFETIME, "width": snapshot["info"].width,
                "height": snapshot["info"].height, "frame_id": snapshot["info"].header.frame_id,
                "rgb_stamp_ns": snapshot["rgb_stamp_ns"], "depth_stamp_ns": snapshot["depth_stamp_ns"],
                "objects": deepcopy(snapshot["objects"]), "vop_pairing": "stationary_scene_approximate"}

    def plan_pick(self, args):
        snapshot = self.observation
        self.plan = None
        if not snapshot or args["observation_id"] != snapshot["id"] or snapshot["expires"] <= time.monotonic():
            raise ValueError("Observation is missing, mismatched or expired. Run observe again.")
        pick_xy = (args["start_point_x"], args["start_point_y"])
        pixel, point, spread = camera_point(snapshot["depth"], snapshot["info"], pick_xy)
        base_point = self.base_point(snapshot, point)
        preview = {"state": "preview", "observation_id": snapshot["id"], "pick_pixel": pixel,
                   "camera_xyz_mm": point, "depth_patch_spread_mm": spread, "base_xyz_mm": base_point,
                   "base_frame": "base_link", "calibration_id": self.calibration["id"], "executed": False}
        if self.transfer is None:
            return {**preview, "transfer_setup_error": self.transfer_setup_error}
        waypoints = self.transfer.waypoints(base_point)
        plan = self.plan = {"id": uuid4().hex, "observation": snapshot,
                            "xy": pick_xy, "base_point": base_point,
                            "waypoints": waypoints}
        return {**preview, "plan_id": plan["id"], "waypoints_tip_mm_deg": waypoints}

    def base_point(self, snapshot, point):
        transform = self.camera_transform
        if self.wrist:
            transform = snapshot["base_from_link6_mm"] @ transform
        return (transform @ np.r_[point, 1])[:3].tolist()

    def verify_scene(self, plan):
        old = plan["observation"]
        if old["expires"] <= time.monotonic():
            raise ValueError("Pick plan expired. Observe and preview again.")
        live = self.source.snapshot()
        previous, current = old["info"], live["info"]
        if (current.header.frame_id != previous.header.frame_id
                or (current.width, current.height, tuple(current.k), tuple(current.d), current.distortion_model)
                != (previous.width, previous.height, tuple(previous.k), tuple(previous.d), previous.distortion_model)):
            raise RuntimeError("Camera frame or intrinsics changed. Observe and preview again.")
        _, camera_point_mm, _ = camera_point(live["depth"], current, plan["xy"])
        if math.dist(self.base_point(live, camera_point_mm), plan["base_point"]) > SCENE_TOLERANCE_MM:
            raise RuntimeError("Pick point moved. Observe and preview again.")
        return live.get("joint_degrees")

    def grab_to(self, args):
        if self.transfer is None:
            raise RuntimeError(self.transfer_setup_error)
        if args["confirm_motion"] is not True:
            raise ValueError("Confirm that the workspace is clear and the plan has been reviewed")
        self.runtime.require_motion_permission()
        plan = self.plan
        if not plan or plan["id"] != args["plan_id"] or plan["observation"]["expires"] <= time.monotonic():
            raise ValueError("Pick plan is missing, mismatched or expired. Preview again.")
        if self.last_transfer and self.last_transfer.get("recovery_required"):
            raise RuntimeError("Recover the arm and gripper on site before restarting this driver")
        self.plan = self.observation = None

        def transfer(_action, _args):
            report = {"state": "running", "stage": "preflight", "plan_id": plan["id"],
                      "holding_object_possible": False, "release_confirmed": False, "recovery_required": False}
            try:
                return self.transfer.execute(self.runtime, plan, self.verify_scene, report)
            except Exception as exc:
                report.update(state="cancelled" if self.runtime.cancel.is_set() else "failed", error=str(exc))
                raise
            finally:
                with self.lock:
                    self.last_transfer = report

        return run_async(self.runtime, NAME, "grab_to", args, perform_action=transfer)

    def dispatch(self, action, args):
        args = {key: value for key, value in args.items()
                if key not in {"_tool_name", "instance_id", "action"}
                and not (key == "input_topic" and value == "") and not (key == "input_topics" and value == [])}
        required = {"plan_pick": {"observation_id", "start_point_x", "start_point_y"},
                    "grab_to": {"plan_id", "confirm_motion"}}
        if set(args) != required.get(action, set()):
            raise ValueError("Unexpected or missing action parameters")
        if action == "cancel":
            return (self.runtime.request_stop(NAME, only_if_active=True)
                    or {"state": "idle", "operation_stopping": False})
        with self.lock:
            if action in ("info", NAME):
                return self.info()
            if action == "start":
                return self.start()
            if action == "stop":
                return self.stop()
            if action == "observe":
                return self.observe()
            if action == "plan_pick":
                return self.plan_pick(args)
            if action == "grab_to":
                return self.grab_to(args)
        raise ValueError("Unknown PiPER pick-and-drop action")
