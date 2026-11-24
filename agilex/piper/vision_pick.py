"""Read-only RGB-D pick localization for a single PiPER arm."""

from copy import deepcopy
import hashlib
from pathlib import Path
from threading import RLock
import time
from uuid import uuid4

import numpy as np

from agilex.piper.vision_kinematics import PiperKinematics
from agilex.piper.vision_rgbd import ObservationSource, camera_point
from common.vendor_runtime import action_schema, tool


NAME = "piper_vision_pick_and_drop"
OBSERVATION_LIFETIME = 30
DEFAULT_URDF = Path(__file__).with_name("resource") / "upstream/piper_description.urdf"


class PiperVisionPick:
    def __init__(self, config, namespace, ros2):
        vision = config["vision_pick"]
        self.can_interface = config.get("can_interface", "can0")
        self.rgb_topic = "/" + "/".join(
            part for part in (namespace.strip("/"), "piper/vision_pick/rgb") if part
        )
        self.lock = RLock()
        self.observation = None
        self.calibration = vision.get("calibration")
        self.wrist = vision.get("joint_state_topic")
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
        if self.wrist and model is None:
            raise ValueError("Wrist localization requires a validated camera-to-link6 calibration")
        self.source = ObservationSource(ros2, vision, self.rgb_topic, model)

    def get_tools(self):
        schema = action_schema({
            "start": ([], "Subscribe to RGB-D and publish RGB for VOP"),
            "stop": ([], "Stop subscriptions and clear the saved observation"),
            "info": ([], "Read camera and calibration status"),
            "observe": ([], "Save a fresh RGB-D observation of a stationary scene"),
            "plan_pick": (["observation_id", "pick_x", "pick_y"], "Preview the selected 3D point"),
        }, {
            "observation_id": {"type": "string", "title": "Observation ID"},
            "pick_x": {"type": "number", "minimum": -1, "maximum": 1, "title": "Pick X (positive right)"},
            "pick_y": {"type": "number", "minimum": -1, "maximum": 1, "title": "Pick Y (positive down)"},
        })
        schema["properties"]["action"]["default"] = "info"
        return [tool(NAME, "sensor", "Preview a PiPER pick point from aligned RGB-D.",
                     schema, topic_out=[{"topic": self.rgb_topic, "format": "image/jpeg"}])]

    def info(self):
        source = self.source.status()
        return {"state": "ready" if source["subscribed"] else "idle",
                "can_interface": self.can_interface, "source": source,
                "rgb_topic": self.rgb_topic, "calibration_id": self.calibration["id"] if self.calibration else None,
                "execute_enabled": False}

    def start(self):
        with self.lock:
            self.source.start()
            return self.info()

    def stop(self):
        with self.lock:
            self.observation = None
            self.source.stop()
            return self.info()

    def observe(self):
        self.observation = None
        snapshot = self.source.snapshot()
        if self.calibration and snapshot["info"].header.frame_id != self.calibration["optical_frame_id"]:
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
        if not snapshot or args["observation_id"] != snapshot["id"] or snapshot["expires"] <= time.monotonic():
            raise ValueError("Observation is missing, mismatched or expired. Run observe again.")
        pixel, point, spread = camera_point(snapshot["depth"], snapshot["info"], (args["pick_x"], args["pick_y"]))
        base_point = None
        if self.camera_transform is not None:
            transform = self.camera_transform
            if self.wrist:
                transform = snapshot["base_from_link6_mm"] @ transform
            base_point = (transform @ np.r_[point, 1])[:3].tolist()
        return {"state": "preview", "observation_id": snapshot["id"], "pick_pixel": pixel,
                "camera_xyz_mm": point, "depth_patch_spread_mm": spread, "base_xyz_mm": base_point,
                "base_frame": "base_link" if base_point is not None else None,
                "calibration_id": self.calibration["id"] if self.calibration else None, "executed": False}

    def dispatch(self, action, args):
        args = {key: value for key, value in args.items()
                if key not in {"_tool_name", "instance_id", "action"}
                and not (key == "input_topic" and value == "") and not (key == "input_topics" and value == [])}
        if set(args) != ({"observation_id", "pick_x", "pick_y"} if action == "plan_pick" else set()):
            raise ValueError("Unexpected or missing action parameters")
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
        raise ValueError("Unknown PiPER vision preview action")
