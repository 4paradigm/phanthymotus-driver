"""Standalone launch file. Camera access is off unless explicitly requested."""
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from configuration import load_calibration, pipeline_settings, validate_mode


def build(context):
    calibration = load_calibration(LaunchConfiguration("calibration_file").perform(context))
    settings = pipeline_settings(calibration)
    sim = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    camera = LaunchConfiguration("start_camera").perform(context).lower() == "true"
    mount = LaunchConfiguration("publish_mount").perform(context).lower() == "true"
    validate_mode(camera, sim, mount)
    actions = []
    if camera:
        share = get_package_share_directory("realsense2_camera")
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(share) / "launch/rs_launch.py")), launch_arguments=settings["camera"].items()))
    t, q = calibration["translation_m"], calibration["quaternion_xyzw"]
    tf_args = ["--x", str(t[0]), "--y", str(t[1]), "--z", str(t[2]),
               "--qx", str(q[0]), "--qy", str(q[1]), "--qz", str(q[2]), "--qw", str(q[3]),
               "--frame-id", "bumi_odom_base_link", "--child-frame-id", "bumi_odom_camera_link"]
    camera_topic = "/bumi_odom/bumi_odom_camera"
    if mount:
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher", namespace="bumi_odom",
                            name="mount", arguments=tf_args, parameters=[{"use_sim_time": sim}]))
    actions.extend([
        Node(package="rtabmap_odom", executable="rgbd_odometry", namespace="bumi_odom", name="visual",
             parameters=[settings["visual"], {"use_sim_time": sim}], remappings=[
                 ("rgb/image", camera_topic + "/color/image_raw"),
                 ("depth/image", camera_topic + "/aligned_depth_to_color/image_raw"),
                 ("rgb/camera_info", camera_topic + "/color/camera_info"),
                 ("imu", "/bumi_odom/unused_visual_imu"),
                 ("odom", "/bumi_odom/visual"), ("odom_info", "/bumi_odom/visual_info")]),
        Node(package="robot_localization", executable="ekf_node", namespace="bumi_odom", name="fusion",
             parameters=[settings["ekf"], {"use_sim_time": sim}],
             remappings=[("odometry/filtered", "/bumi_odom/filtered")]),
        ExecuteProcess(cmd=[sys.executable, str(Path(__file__).with_name("quality.py")),
                            "--output", LaunchConfiguration("output_csv"), "--ros-args",
                            "-p", "use_sim_time:=" + str(sim).lower()], output="screen"),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration_file", description="Measured fixed camera installation JSON"),
        DeclareLaunchArgument("output_csv", description="New output CSV path; existing files are refused"),
        DeclareLaunchArgument("start_camera", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_mount", default_value="true"),
        OpaqueFunction(function=build),
    ])
