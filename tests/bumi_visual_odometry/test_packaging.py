"""Single-container packaging guards for the bundle-local visual odometry card.

The point of these is negative as much as positive: the card must ship inside the
Bumi bundle and must not quietly recreate a second driver/image or the fusion
path, and the planning docs that #408 removed must not come back through this
feature.
"""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
BUMI = ROOT/'noetix/bumi'


def test_image_installs_the_estimator_and_starts_it_inside_the_bundle():
    text = (BUMI/'Dockerfile').read_text()
    assert 'ros-humble-rtabmap-odom' in text
    # rtabmap-odom already declares tf2_ros; a second direct install would not
    # change the installed set, only the apt command.
    assert 'ros-humble-tf2-ros' not in text
    # The base image has no ROS 2 apt source; the Dockerfile has to add the
    # repository and its keyring (Tsinghua mirror with the official fallback).
    assert 'mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu' in text
    assert 'ros-archive-keyring.gpg' in text
    assert 'trusted=yes' not in text
    assert '/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry' in text
    assert 'from tf2_ros import StaticTransformBroadcaster' in text
    assert 'COPY visual_odometry.py /work/visual_odometry.py' in text
    assert 'COPY visual_odometry_probe.py /work/visual_odometry_probe.py' in text
    assert 'COPY visual_odometry_contract.py /work/visual_odometry_contract.py' in text
    assert 'robot-localization' not in text and 'robot_localization' not in text
    assert 'ekf_node' not in text


def test_card_and_plugin_config_are_in_sync():
    driver = yaml.safe_load((BUMI/'driver.yaml').read_text())
    assert {'name': 'rgbd_visual_odometry', 'type': 'sensor'} in driver['cards']
    config = yaml.safe_load((BUMI/'config.yaml').read_text())
    plugin = config['plugins']['visual_odometry']
    assert plugin['enabled'] is True
    assert plugin['max_rate_hz'] == 10
    # The tool name lives in code; keep the driver card pointing at it.
    source = (BUMI/'visual_odometry.py').read_text()
    assert "'name': 'rgbd_visual_odometry'" in source
    assert 'OUTPUT_TOPIC = \'/bumi_visual_odometry/state/odom\'' in source


def test_bundle_entrypoint_wires_and_gates_the_card():
    main = (BUMI/'main.py').read_text()
    assert 'os.environ["BUMI_VISUAL_ODOMETRY"] = "1"' in main
    assert 'from visual_odometry import VisualOdometryPlugin' in main
    assert 'VisualOdometryPlugin(' in main
    assert 'needs plugins.camera.enabled' in main
    device = (BUMI/'device.py').read_text()
    assert 'os.environ.get("BUMI_VISUAL_ODOMETRY") == "1"' in device
    assert 'from visual_inputs import RawCameraPublisher' in device
    assert 'raw RGB-D publish failed' in device
    inputs = (BUMI/'visual_inputs.py').read_text()
    assert 'RawCameraPublisher' in inputs and 'CameraInfo' in inputs
    assert 'BodyImuBridge' not in inputs and 'get_imu_data' not in inputs


def test_estimator_module_has_no_fusion_path():
    text = (BUMI/'visual_odometry.py').read_text()
    assert 'rgbd_odometry' in text and 'rtabmap_odom' in text
    # The module docstring names the fusion non-goals on purpose; the launch
    # parameters and imports are what must not carry them.
    for forbidden in ('ekf_node', 'imu0', 'odom0'):
        assert forbidden not in text
    assert (BUMI/'visual_odometry_contract.py').exists()
    assert not (ROOT/'common/visual_odometry.py').exists()


def test_the_feature_does_not_ship_a_second_driver_or_planning_docs():
    assert not (ROOT/'noetix/bumi_visual_odometry').exists()
    assert not (ROOT/'docs/superpowers').exists()
    # The optional image_repository extension existed only for the two-image
    # shape; the single-container bundle must not carry it.
    assert 'image_repository' not in (ROOT/'build.sh').read_text()
    assert 'image_repository' not in (ROOT/'README_dev.md').read_text()
