"""Validated installation geometry and parameters shared with the ROS launcher."""
import json
import math
from pathlib import Path

try:
    from .evaluate import _finite, rotation
except ImportError:
    from evaluate import _finite, rotation


def load_calibration(path):
    data = json.loads(Path(path).read_text())
    if data.get("fixed_mount") is not True or not data.get("calibration_id"):
        raise ValueError("measured fixed-mount calibration is required")
    try:
        data["translation_m"] = _finite(data["translation_m"], 3)
        q = _finite(data["quaternion_xyzw"], 4)
        rotation(q)
        norm = math.hypot(*q)
        data["quaternion_xyzw"] = tuple(v / norm for v in q)
        data["initial_base_rpy_rad"] = _finite(data["initial_base_rpy_rad"], 3)
    except (KeyError, TypeError) as exc:
        raise ValueError("incomplete installation calibration") from exc
    return data


def validate_mode(start_camera, use_sim_time, publish_mount):
    if use_sim_time and start_camera:
        raise ValueError("replay must not open the camera")
    if use_sim_time and publish_mount:
        raise ValueError("replay must restore recorded static TF; set publish_mount:=false")


def pipeline_settings(calibration):
    return dict(
        camera={"camera_name": "bumi_odom_camera", "camera_namespace": "bumi_odom",
                "enable_color": "true", "enable_depth": "true", "enable_gyro": "true",
                "enable_accel": "true", "unite_imu_method": "2", "enable_sync": "true",
                "align_depth.enable": "true",
                "depth_module.depth_profile": "640x480x30", "rgb_camera.color_profile": "640x480x30"},
        visual={"frame_id": "bumi_odom_base_link", "odom_frame_id": "bumi_odom_odom",
                "publish_tf": False, "wait_imu_to_init": False, "approx_sync": True,
                "approx_sync_max_interval": 0.02, "qos": 2, "qos_camera_info": 2,
                "topic_queue_size": 20, "sync_queue_size": 20, "Odom/ResetCountdown": "0",
                "initial_pose": "0 0 0 " + " ".join(map(str, calibration["initial_base_rpy_rad"]))},
        ekf={"frequency": 30.0, "sensor_timeout": 0.5, "two_d_mode": False,
             "publish_tf": True, "map_frame": "bumi_odom_map", "odom_frame": "bumi_odom_odom",
             "base_link_frame": "bumi_odom_base_link", "world_frame": "bumi_odom_odom",
             "odom0": "/bumi_odom/visual", "odom0_config": [True]*6 + [False]*9,
             "odom0_differential": False, "odom0_relative": False, "odom0_queue_size": 10,
             "imu0": "/bumi_odom/bumi_odom_camera/imu",
             "imu0_config": [False]*9 + [True]*3 + [False]*3,
             "imu0_differential": False, "imu0_remove_gravitational_acceleration": False,
             "imu0_queue_size": 100},
    )
