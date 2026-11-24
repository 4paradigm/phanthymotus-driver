"""Monitor cards in the production S10 Pro bundle, exercised without hardware."""

import json
import math
import xml.etree.ElementTree as ET

import pytest

from deep_robotics.s10_pro.reader import DEFAULT_SOURCES
from test_s10_monitor_runtime import pro_bundle, s10_ros


MONITORS = {
    "battery": "data/json",
    "joints": "sensor/skeleton",
    "lidar_cloud": "sensor/pointcloud",
    "imu": "data/json",
}


def test_pro_catalog_preserves_the_four_monitor_tools_and_ports(pro_bundle):
    tools = {tool["name"]: tool for tool in pro_bundle.bundle.get_all_tools()}
    for name, format_ in MONITORS.items():
        assert tools[name]["type"] == "sensor"
        assert tools[name]["topic_out"] == [{"topic": f"/test/{name}", "format": format_}]
    assert tools["model"]["type"] == "resource"
    assert tools["joint_state"]["topic_out"] == [{"topic": "/test/joint_state", "format": "data/json"}]


def test_monitor_cards_start_and_stop_through_the_bundle(pro_bundle):
    bundle, nodes = pro_bundle.bundle, pro_bundle.nodes
    subscriptions = dict(nodes.robot.subscriptions)
    tools = {tool["name"]: tool for tool in bundle.get_all_tools()}
    for name in MONITORS:
        assert tools[name]["inputSchema"]["properties"] == {}
        for _ in range(2):
            assert bundle.dispatch(name, {"action": "start"}) == {
                "state": "running", "topic_out": tools[name]["topic_out"],
            }
        assert bundle.dispatch(name, {"action": "stop"}) == {
            "state": "idle", "topic_out": tools[name]["topic_out"],
        }
    assert nodes.robot.subscriptions == subscriptions


def test_standalone_reader_catalog_is_limited_to_monitor_subscriptions():
    assert set(DEFAULT_SOURCES) == {"battery", "joints", "imu", "lidar"}


def test_skeleton_marks_the_fixed_mechanical_front_without_changing_dof(pro_bundle):
    device = pro_bundle.device
    root = ET.fromstring(pro_bundle.bundle.dispatch("model", {})["urdf"])
    movable = [joint.get("name") for joint in root.findall("joint") if joint.get("type") != "fixed"]
    assert len(movable) == 16 and set(movable) == set(device.JOINT_NAMES)
    markers = [root.find(f"joint[@name='front_marker_{name}_joint']")
               for name in ("tip", "left", "right", "close")]
    assert all(marker is not None and marker.get("type") == "fixed" for marker in markers)
    assert markers[0].find("parent").get("link") == "front_lidar_link"

    joints = [{"q": (reference - math.radians(offset)) / direction}
              for reference, offset, direction in zip(device.MODEL_REFERENCE_ANGLES,
                                                       device.JOINT_OFFSETS_DEG,
                                                       device.JOINT_DIRECTIONS)]
    pro_bundle.nodes.robot.subscriptions["/JOINTS_DATA_10HZ"]({"data": {"joints_data": joints}})
    skeleton = json.loads(pro_bundle.nodes.publishers["joints"].messages[-1].data)
    assert [joint["name"] for joint in skeleton["joints"]] == list(device.JOINT_NAMES)
    assert [joint["q"] for joint in skeleton["joints"]] == pytest.approx([0.] * 16)


def test_readable_battery_and_imu_payloads(pro_bundle):
    nodes = pro_bundle.nodes
    battery = {"battery_level": 72, "current": -1.25, "voltage": 51.4,
               "battery_temperature": [31., 32.], "cycles": 18}
    nodes.robot.subscriptions["/BATTERY_DATA"]({"data": [battery]})
    assert json.loads(nodes.publishers["battery"].messages[-1].data) == {"battery_0": battery}

    nodes.robot.subscriptions["/IMU"]({
        "header": {"stamp": {"sec": 1793343577, "nanosec": 638000000}},
        "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.},
        "angular_velocity": {"x": .1, "y": .2, "z": .3},
        "linear_acceleration": {"x": 0., "y": 0., "z": 9.8},
    })
    assert json.loads(nodes.publishers["imu"].messages[-1].data) == {
        "orientation_x": 0., "orientation_y": 0., "orientation_z": 0., "orientation_w": 1.,
        "angular_velocity_x": .1, "angular_velocity_y": .2, "angular_velocity_z": .3,
        "linear_acceleration_x": 0., "linear_acceleration_y": 0., "linear_acceleration_z": 9.8,
        "sec": 1793343577, "nanosec": 638000000,
    }


@pytest.mark.parametrize("stamp, expected", [
    ({"sec": 0, "nanosec": 0}, {"sec": 0, "nanosec": 0}),
    ({"sec": -1, "nanosec": 999999999}, {"sec": -1, "nanosec": 999999999}),
    (None, {}),
    ({"sec": 1}, {}),
    ({"sec": True, "nanosec": 0}, {}),
    ({"sec": 1, "nanosec": 1000000000}, {}),
])
def test_imu_timestamp_preserves_ros_time_and_does_not_fabricate_missing_time(pro_bundle, stamp, expected):
    nodes = pro_bundle.nodes
    nodes.robot.subscriptions["/IMU"]({"header": {"stamp": stamp}})
    assert json.loads(nodes.publishers["imu"].messages[-1].data) == expected
