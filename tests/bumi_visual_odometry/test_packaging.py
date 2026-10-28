"""Packaging guards for the restored visual odometry driver.

The point of these is negative: #425's fusion path must not quietly reappear as a
dependency, a container setting or a second estimator. Docs may *talk* about EKF
(the non-goal has to be named), so the assertions are about imports, package
names and process/launch files rather than raw text mentions.
"""
from pathlib import Path

import yaml

from noetix.bumi_visual_odometry.service import tool

ROOT = Path(__file__).resolve().parents[2]
SIDE = ROOT/'noetix/bumi_visual_odometry'
BUNDLE = ROOT/'noetix/bumi'


def test_sensor_tool_contract_is_visual_only():
    definition = tool()
    assert definition['name'] == 'rgbd_visual_odometry'
    assert definition['type'] == 'sensor' and definition['multiInstance'] is False
    assert definition['topic_out'] == [{'topic': '/bumi_visual_odometry/state/odom',
                                        'format': 'state/odom',
                                        'message_type': 'std_msgs/msg/String'}]
    assert set(definition['configSchema']['properties']) == {'source_namespace', 'calibration'}
    assert set(definition['inputSchema']['properties']['action']['enum']) == {
        'start', 'stop', 'info', 'reset', 'config'}


def test_sidecar_image_installs_the_visual_stack_only():
    text = (SIDE/'Dockerfile').read_text()
    assert 'ros-humble-rtabmap-odom' in text and 'ros-humble-rtabmap-msgs' in text
    assert 'robot-localization' not in text and 'robot_localization' not in text
    assert 'ekf_node' not in text
    assert '/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry' in text
    # The estimator needs a base frame distinct from the image frame plus a TF
    # to it, so tf2_ros must be in the dependency check.
    assert 'from tf2_ros import StaticTransformBroadcaster' in text
    assert '/opt/bumi-visual-packages.txt' in text and 'VISUAL_BASE_IMAGE' in text
    for name in ('pipeline.py', 'runtime.py', 'service.py', 'ros_probe.py'):
        assert (SIDE/name).exists()
        assert name in text


def test_driver_yaml_and_service_are_the_visual_driver():
    driver = yaml.safe_load((SIDE/'driver.yaml').read_text())
    assert driver['id'] == 'bumi-visual-odometry-driver'
    assert driver['image_name'] == 'noetix-bumi-visual-odometry'
    assert driver['hardware_provider'] == 'noetix'
    assert driver['hardware_model'] == 'bumi'
    # The bundle and this sidecar are two images for one hardware model; the
    # image repository must stay distinct or the later build overwrites the
    # other at the same date+commit tag.
    bundle = yaml.safe_load((ROOT/'noetix/bumi/driver.yaml').read_text())
    bundle_repo = f"{bundle['hardware_provider']}/{bundle['hardware_model']}"
    assert driver['image_repository'] != bundle_repo
    assert driver['image_repository'] == 'noetix/bumi-visual-odometry'
    assert driver['port'] == 15720
    assert driver['cards'] == [{'name': 'rgbd_visual_odometry', 'type': 'sensor'}]
    assert driver['build_context_extras'] == ['../../common']
    fragment = yaml.safe_load((SIDE/'deploy/service.yml').read_text())
    assert set(fragment) == {'noetix-bumi-visual-odometry'}
    service = fragment['noetix-bumi-visual-odometry']
    assert service['restart'] == 'always'
    assert service['container_name'] == 'embodied-noetix-bumi-visual-odometry'
    assert 'ROS_DOMAIN_ID=42' in service['environment']
    assert service.get('privileged') is not True
    assert not any('usb' in str(volume).lower() for volume in service.get('volumes', []))


def test_runtime_publishes_its_own_scoped_static_tf():
    text = (SIDE/'runtime.py').read_text()
    assert 'StaticTransformBroadcaster' in text
    assert "self.segment+'_base'" in text
    assert 'base_frame()' in text
    # RTAB-Map itself must not publish a competing odom->base TF.
    pipeline = (SIDE/'pipeline.py').read_text()
    assert 'publish_tf=False' in pipeline


def test_launch_file_has_no_second_estimator():
    text = (SIDE/'pipeline.py').read_text()
    assert 'rgbd_odometry' in text
    assert 'ekf_node' not in text
    assert 'imu0' not in text and 'odom0' not in text


def test_bundle_raw_branch_has_no_imu_bridge():
    text = (BUNDLE/'visual_inputs.py').read_text()
    assert 'RawCameraPublisher' in text and 'CameraInfo' in text
    assert 'BodyImuBridge' not in text and 'get_imu_data' not in text
    # The merged #408 tree keeps the camera subprocess in device.py (there is no
    # camera_runtime.py any more); the raw branch must hook there and stay
    # non-fatal around both construction and publish.
    device = (BUNDLE/'device.py').read_text()
    assert 'from visual_inputs import RawCameraPublisher' in device
    assert 'raw_inputs.publish(source_frames, frames)' in device
    assert 'raw RGB-D publish failed' in device
    assert 'raw RGB-D branch unavailable' in device
    dockerfile = (BUNDLE/'Dockerfile').read_text()
    assert 'visual_inputs.py' in dockerfile


def test_build_script_honours_the_optional_image_repository():
    text = (ROOT/'build.sh').read_text()
    assert 'image_repository' in text
    assert 'repo:-${provider}/${model}' in text
    assert 'repo:-${hw_provider}/${hw_model}' in text


def test_sidecar_modules_import_without_ros():
    """Importing the module (not constructing the runtime) must not need rclpy."""
    import importlib
    for name in ('noetix.bumi_visual_odometry.pipeline',
                 'noetix.bumi_visual_odometry.service',
                 'noetix.bumi_visual_odometry.runtime'):
        importlib.import_module(name)  # constructing the runtime is what needs ROS
