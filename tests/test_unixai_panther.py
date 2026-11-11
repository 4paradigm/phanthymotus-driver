"""UniX AI Panther driver: schema, payload, hub and bridge behaviour, all without ROS.

Everything here runs on a laptop. What it cannot show is that a real robot publishes what the
fixtures say it does; `unixai/panther/README.md` lists what has to be confirmed on the robot.

Run: python3 -m pytest tests/test_unixai_panther.py -q
"""
from __future__ import annotations

import importlib
import importlib.util
import json
import queue
import re
import sys
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "unixai" / "panther"

NAN = float("nan")
INF = float("inf")
URDF_JOINTS = [f"joint_{side}a{i}" for side in "lr" for i in range(8)] + ["joint_head0", "joint_head1"]
ALLOWED_FORMATS = {"data/json", "sensor/skeleton", "state/odom", "image/jpeg", "image/depth-zlib"}


# ── fixtures ─────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def device():
    patch = pytest.MonkeyPatch()
    patch.syspath_prepend(str(DRIVER))
    patch.syspath_prepend(str(ROOT))
    try:
        spec = importlib.util.spec_from_file_location("unixai_panther_device", DRIVER / "device.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        patch.undo()


@pytest.fixture(scope="module")
def payloads(device):
    return importlib.import_module("panther_payloads")


@pytest.fixture(scope="module")
def bridge(device):
    return importlib.import_module("panther_bridge")


@pytest.fixture
def config():
    return yaml.safe_load((DRIVER / "config.yaml").read_text(encoding="utf-8"))


class Clock:
    def __init__(self):
        self.wall = 1_794_300_000.0
        self.mono = 5_000.0

    def advance(self, seconds):
        self.wall += seconds
        self.mono += seconds


class Sink:
    """Stands in for BridgeSink; records what would have gone to domain 42."""

    def __init__(self, accept=True):
        self.accept = accept
        self.jsons, self.images, self.closed = [], [], False

    def json(self, lane, topic, text):
        self.jsons.append((lane, topic, text))
        return self.accept

    def image(self, lane, topic, image_format, data, frame_id):
        self.images.append((lane, topic, image_format, data, frame_id))
        return self.accept

    def stats(self):
        return {}

    def close(self):
        self.closed = True

    def last(self, topic):
        found = [json.loads(t) for _, name, t in self.jsons if name == topic]
        return found[-1] if found else None

    def count(self, topic):
        return sum(1 for _, name, _ in self.jsons if name == topic)


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def sink():
    return Sink()


@pytest.fixture
def hub(device, config, clock, sink):
    return device.Hub(config, "panther", sink, clock=lambda: clock.wall, monotonic=lambda: clock.mono)


ALL_CONTROLLERS = ("movebase3_drive_controller", "dual_arm_forward_position_controller",
                   "left_arm_joint_trajectory_position_controller",
                   "right_arm_joint_trajectory_position_controller", "head_forward_position_controller",
                   "lifting_controller", "left_unix_gripper3_controller", "right_unix_gripper3_controller")


class FakeIO:
    """Stands in for `RosMotionIO`: records every command that would have gone to the robot."""

    def __init__(self):
        self.calls = []
        self.controllers = {name: "active" for name in ALL_CONTROLLERS}
        self.controllers["left_arm_joint_trajectory_position_controller"] = "inactive"
        self.controllers["right_arm_joint_trajectory_position_controller"] = "inactive"
        self.arm_results = []                # outcomes poll_arm_result hands out, in order
        self.fail_on = set()
        self.switch_fail_after = None        # the Nth controller switch (1-based) and later raise
        # Which of the configured base topics the robot would have subscribers on.
        self.subscribers = {"/cmd_vel": 1}
        self.twist_topic = None

    def _do(self, name, *args):
        if name in self.fail_on:
            raise RuntimeError(f"{name} failed")
        self.calls.append((name, *args))

    def named(self, name):
        return [c[1:] for c in self.calls if c[0] == name]

    def twist_candidates(self, candidates):
        return {topic: self.subscribers.get(topic, 0) for topic in candidates}

    def select_twist_topic(self, candidates):
        counts = self.twist_candidates(candidates)
        for topic in candidates:
            if counts.get(topic):
                self.twist_topic = topic
                return topic, counts
        return None, counts

    def list_controllers(self):
        return dict(self.controllers)

    def switch_controllers(self, activate, deactivate):
        if self.switch_fail_after is not None and len(self.named("switch")) >= self.switch_fail_after:
            raise RuntimeError("the controller manager rejected the controller switch")
        self._do("switch", tuple(activate), tuple(deactivate))
        for name in activate:                # the robot does what it was asked, so a test can compare states
            self.controllers[name] = "active"
        for name in deactivate:
            self.controllers[name] = "inactive"

    def publish_twist(self, vx, vy, wz):
        self._do("twist", vx, vy, wz)

    def publish_head(self, values):
        self._do("head", tuple(values))

    def publish_lift(self, position):
        self._do("lift", position)

    def publish_gripper(self, side, mode, position):
        self._do("gripper", side, mode, position)

    def send_arm_goal(self, side, names, positions, duration):
        self._do("goal", side, tuple(positions), duration)
        return f"handle-{side}"

    def poll_arm_result(self, handle, timeout):
        if self.arm_results:
            return self.arm_results.pop(0)
        return {"status": 4, "error_code": 0, "error_string": ""}

    def cancel_arm_goal(self, handle):
        self._do("cancel", handle)


@pytest.fixture
def io():
    return FakeIO()


@pytest.fixture
def completions():
    return []


@pytest.fixture
def motion(device, config, hub, io, completions, clock):
    """Motion with threads run inline. There is no switch to open: readiness comes from the robot."""
    motion_module = importlib.import_module("panther_motion")

    def sleep(seconds):
        clock.advance(seconds)

    return motion_module.Motion(config, io, hub, spawn=lambda target, name: target(),
                                notify=lambda *a: completions.append(a), sleep=sleep,
                                monotonic=lambda: clock.mono)


# ── message builders ─────────────────────────────────────────────────────────

def header(sec=1_794_300_000, nanosec=0, frame_id=""):
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id=frame_id)


def joint_state(names, positions, velocities=None, efforts=None):
    n = len(names)
    return NS(header=header(), name=list(names), position=list(positions),
              velocity=list(velocities if velocities is not None else [NAN] * n),
              effort=list(efforts if efforts is not None else [NAN] * n))


def battery(percentage=1.0, **over):
    fields = dict(header=header(frame_id="base_link"), voltage=50.02, temperature=29.0, current=-2.2,
                  charge=NAN, capacity=NAN, design_capacity=NAN, percentage=percentage,
                  power_supply_status=2, power_supply_health=1, power_supply_technology=0,
                  present=True)
    fields.update(over)
    return NS(**fields)


def imu(quaternion=(0.0, 0.0, 0.0, 1.0), orientation_cov0=0.0, gyro_cov0=0.0, accel_cov0=0.0):
    x, y, z, w = quaternion
    return NS(header=header(frame_id="imu_link"), orientation=NS(x=x, y=y, z=z, w=w),
              orientation_covariance=[orientation_cov0] + [0.0] * 8,
              angular_velocity=NS(x=0.01, y=-0.02, z=0.03), angular_velocity_covariance=[gyro_cov0] + [0.0] * 8,
              linear_acceleration=NS(x=0.1, y=0.2, z=9.81), linear_acceleration_covariance=[accel_cov0] + [0.0] * 8)


def odometry(vx=0.0, vy=0.0, wz=0.0, sec=1_794_300_000):
    return NS(header=header(sec=sec, frame_id="odom"), child_frame_id="base_link",
              twist=NS(twist=NS(linear=NS(x=vx, y=vy, z=0.0), angular=NS(x=0.0, y=0.0, z=wz))))


def status(name, level=b"\x00", message="ok", **values):
    return NS(name=name, level=level, message=message, hardware_id="mh3_0/hardware bridge",
              values=[NS(key=k, value=v) for k, v in values.items()])


def diagnostics(*statuses):
    return NS(header=header(), status=list(statuses))


def normal_diagnostics():
    return diagnostics(
        status("diagnostic_broadcaster: CAN bus", message="CAN bus OK", can0_state="ERROR-ACTIVE"),
        status("diagnostic_broadcaster: Emergency stop", message="急停状态正常", stop_button_status="正常"),
        status("diagnostic_broadcaster: Motor status", message="手臂电机正常"),
    )


def scan(n=1800, fill=2.0, **over):
    fields = dict(header=header(frame_id="link_front_radar"), angle_min=-3.1415927,
                  angle_max=3.1381, angle_increment=3.490658e-3, range_min=0.1, range_max=50.0,
                  ranges=[fill] * n, intensities=[0.0] * n)
    fields.update(over)
    return NS(**fields)


def image(width, height, encoding, data, step=None, bigendian=0):
    return NS(header=header(frame_id="camera_color_optical_frame"), width=width, height=height,
              encoding=encoding, is_bigendian=bigendian, step=step if step is not None else len(data) // height,
              data=data)


# ── bundle metadata ──────────────────────────────────────────────────────────

def test_driver_config_and_cards_agree(device, config):
    driver = yaml.safe_load((DRIVER / "driver.yaml").read_text(encoding="utf-8"))
    cards = {c["name"]: c["type"] for c in driver["cards"]}
    assert set(cards) == set(config["plugins"]) == set(device.CARD_ORDER)
    assert [c["name"] for c in driver["cards"]] == list(device.CARD_ORDER)
    assert cards["model"] == "resource"
    assert {n for n, t in cards.items() if t == "actuator"} == {
        "base_drive", "arm_control", "gripper", "lifting", "head"}
    assert {n for n, t in cards.items() if t == "sensor"} == set(device.SENSOR_CARDS) - {"model"}
    assert driver["port"] == config["mcp_port"] == 15747
    assert driver["mcp_url"] == "http://localhost:15747/mcp"
    assert (driver["hardware_provider"], driver["hardware_model"]) == ("unixai", "panther")
    assert driver["build_context_extras"] == ["../../common"]


def test_port_is_not_used_by_another_driver():
    ports = {}
    for path in ROOT.glob("*/*/driver.yaml"):
        ports.setdefault(yaml.safe_load(path.read_text(encoding="utf-8"))["port"], []).append(path.parent.name)
    assert ports[15747] == ["panther"]


def test_tools_match_the_driver_manifest_and_use_renderable_formats(device, hub, motion, config):
    tools = device.PantherPlugin(hub, motion=motion, config=config).get_tools()
    assert [t["name"] for t in tools] == list(device.CARD_ORDER)
    for definition in tools:
        assert definition["description"] and definition["inputSchema"]["type"] == "object"
        assert definition["type"] in ("sensor", "resource", "actuator")
        assert definition["multiInstance"] is False
        for output in definition.get("topic_out", []):
            assert output["format"] in ALLOWED_FORMATS, "sensor/lidar* does not render from the topic bus"
            assert output["topic"].startswith("/panther/")
    json.dumps(tools, allow_nan=False)


def test_sensor_only_plugin_lists_no_motion_cards(device, hub):
    assert [t["name"] for t in device.PantherPlugin(hub).get_tools()] == list(device.SENSOR_CARDS)


def test_topic_names_are_unique(hub):
    topics = [spec["topic"] for spec in hub.topics.values()] + [v["topic"] for v in hub.views.values()]
    assert len(topics) == len(set(topics))
    assert hub.topics["odom"] == {"topic": "/panther/state/odom", "format": "state/odom"}


def test_service_yml_satisfies_the_fleet_checker():
    spec = importlib.util.spec_from_file_location("check_service_yml", ROOT / "scripts" / "check_service_yml.py")
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    bad, _ = checker.check(str(DRIVER / "deploy" / "service.yml"), "unixai/panther")
    assert bad == []
    service = yaml.safe_load((DRIVER / "deploy" / "service.yml").read_text(encoding="utf-8"))
    (name, body), = service.items()
    assert body["restart"] == "always" and body["network_mode"] == "host"
    assert body["image"] == "__IMAGE__"
    # host_camera.py enters PID 1's namespaces to start the Orbbec driver on the robot: it needs the
    # host PID namespace and CAP_SYS_ADMIN, and nothing more (review: privileged was far too broad).
    assert body["pid"] == "host" and body["cap_add"] == ["SYS_ADMIN"]
    assert "privileged" not in body and "devices" not in body
    assert body["stop_grace_period"] == "30s", "a running arm move is cancelled and restored on SIGTERM"
    assert not any("PASSWORD" in v.upper() or "TOKEN" in v.upper() for v in body["environment"])


def test_robot_side_dds_is_loopback_cyclone_like_the_robots_own(config):
    ros = config["ros"]
    assert ros["robot_domain_id"] == 0 and ros["core_domain_id"] is None
    assert re.fullmatch(r"[a-zA-Z0-9_.:-]+", ros["robot_interface"])
    uri = ET.fromstring(ros["cyclonedds_uri"])
    ns = {"c": "https://cdds.io/config"}
    interface = uri.find(".//c:NetworkInterface", ns)
    assert interface.get("address") == "127.0.0.1" and interface.get("multicast") == "true"
    assert uri.find(".//c:AllowMulticast", ns).text == "spdp"
    # Cyclone's own trace output would otherwise go to the container log, which logsafe cannot guard.
    assert uri.find(".//c:Tracing/c:OutputFile", ns).text == "/dev/null"
    assert config["core_bridge"] == {"domain": 42, "profile": "/opt/phanthy-motus/dds-local.xml"}


def _pin_robot_dds():
    import ast

    tree = ast.parse((DRIVER / "main.py").read_text(encoding="utf-8"))
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "pin_robot_dds")
    namespace = {"os": __import__("os")}
    exec(compile(ast.Module([function], []), "main.py", "exec"), namespace)
    return namespace["pin_robot_dds"]


def test_an_inherited_cyclone_setting_cannot_override_the_configured_one(monkeypatch):
    """vendor_runtime only *fills in* CYCLONEDDS_URI; a stale one would leave the driver deaf, not broken."""
    pin = _pin_robot_dds()
    monkeypatch.setenv("CYCLONEDDS_URI", "file:///somewhere/else.xml")
    monkeypatch.delenv("PANTHER_CYCLONEDDS_URI", raising=False)
    pin()
    assert "CYCLONEDDS_URI" not in __import__("os").environ
    monkeypatch.setenv("PANTHER_CYCLONEDDS_URI", "file:///deliberate.xml")
    pin()
    assert __import__("os").environ["CYCLONEDDS_URI"] == "file:///deliberate.xml"


def test_dockerfile_exposes_the_port_and_copies_what_the_code_imports():
    dockerfile = (DRIVER / "Dockerfile").read_text(encoding="utf-8")
    assert "EXPOSE 15747" in dockerfile
    copied = re.search(r"COPY (main\.py[^\n]*) /work/unixai/panther/", dockerfile).group(1).split()
    for name in ("main.py", "device.py", "contracts.py", "host_camera.py", "panther_motion.py",
                 "panther_payloads.py", "panther_bridge.py", "config.yaml", "driver.yaml"):
        assert name in copied
        assert (DRIVER / name).is_file()
    assert "COPY common/ /work/common/" in dockerfile
    assert "rmw-cyclonedds-cpp" in dockerfile and "rmw-fastrtps-cpp" in dockerfile
    # The ros-base image has the ROS install and key but no ROS apt source; the first bot build
    # failed with "ros-humble-rmw-cyclonedds-cpp has no installation candidate" without this.
    for stage in dockerfile.split("FROM ")[1:]:
        assert stage.index("ros2-archive-keyring.gpg") < stage.index("apt-get update")
    assert dockerfile.count("/etc/apt/sources.list.d/ros2.list") == 2
    assert "ros-base@sha256:" in dockerfile.splitlines()[0], "pinned like the other ROS drivers"
    assert "print('panther imports ok'" in dockerfile, "build fails here, not on the robot"
    # What the motion cards import at run time must be installed and checked at build time.
    for package in ("ros-humble-control-msgs", "ros-humble-controller-manager-msgs",
                    "ros-humble-trajectory-msgs", "ros-humble-geometry-msgs", "ros-humble-tf2-msgs"):
        assert package in dockerfile
    for imported in ("unix_msgs.msg import GripperCommand", "control_msgs.action import FollowJointTrajectory",
                     "controller_manager_msgs.srv import ListControllers, SwitchController"):
        assert imported in dockerfile
    assert "command -v nsenter" in dockerfile
    # The build log must say which packages the base image lacked, so a reviewer can see what the
    # image really grows by (and whether any interface package was already in ros-base).
    assert "installing packages the base image lacks" in dockerfile and "nothing installed" in dockerfile
    assert "AS interface-builder" in dockerfile and "COPY --from=interface-builder" in dockerfile
    assert "COPY vendor/unix_msgs/ " in dockerfile
    assert "systemd" not in dockerfile, "nsenter uses the host's systemctl, so none is installed here"
    assert "numpy==1.26.4" in (DRIVER / "requirements.txt").read_text(encoding="utf-8")


def test_vendor_message_package_matches_the_robots_gripper_command():
    message = (DRIVER / "vendor" / "unix_msgs" / "msg" / "GripperCommand.msg").read_text(encoding="utf-8")
    fields = [line.split() for line in message.splitlines() if line.strip() and not line.startswith("#")]
    assert fields == [["string", "mode"], ["sensor_msgs/JointState", "command"]]
    package = ET.parse(DRIVER / "vendor" / "unix_msgs" / "package.xml").getroot()
    assert package.findtext("name") == "unix_msgs"


def test_main_is_guarded_so_spawned_children_do_not_start_a_second_driver():
    source = (DRIVER / "main.py").read_text(encoding="utf-8")
    assert 'if __name__ == "__main__":' in source
    assert source.index("logsafe.install()") < source.index("from device import")


def _code(path):
    return [line for line in path.read_text(encoding="utf-8").splitlines()
            if not line.strip().startswith(("#", '"""'))]


def test_modules_that_decide_things_do_not_import_ros():
    """Hub, Motion and the payload rules are tested on a laptop only because they cannot reach ROS."""
    for name in ("contracts.py", "panther_motion.py", "panther_payloads.py", "host_camera.py"):
        for line in _code(DRIVER / name):
            if line.startswith(("import ", "from ")):
                assert not re.search(r"\b(rclpy|sensor_msgs|geometry_msgs|std_msgs|nav_msgs|control_msgs|"
                                     r"controller_manager_msgs|trajectory_msgs|unix_msgs|tf2_msgs)\b", line), (name, line)


def test_only_the_expected_modules_publish_call_services_or_touch_the_host():
    """A source-level tripwire: robot-facing writes live in RosMotionIO, host access in host_camera."""
    sources = {p.name: "\n".join(_code(p)) for p in DRIVER.glob("*.py")}
    for name, code in sources.items():
        for forbidden in ("os.system", "os.dup(1)", "dup2("):
            assert forbidden not in code, (name, forbidden)
        if name != "host_camera.py":
            assert "subprocess" not in code, name
            assert "nsenter" not in code, name
        if name != "device.py":
            for forbidden in ("create_publisher(", "create_client(", "ActionClient(", "call_async("):
                # panther_bridge legitimately publishes on domain 42 (Agent Core), nothing on the robot domain.
                lines = [l for l in code.splitlines() if forbidden in l
                         and "node.create_publisher(msg_type, topic, qos)" not in l]
                assert not lines, (name, forbidden, lines)
    device_code = sources["device.py"]
    publishers = device_code.index("class RosMotionIO")
    nodes = device_code.index("class PantherNodes")
    for forbidden in ("create_publisher(", "create_client(", "ActionClient("):
        for position in [m.start() for m in re.finditer(re.escape(forbidden), device_code)]:
            assert publishers < position < nodes, f"{forbidden} outside RosMotionIO"


# ── URDF ─────────────────────────────────────────────────────────────────────

def test_urdf_is_a_clean_kinematic_chain_whose_joints_match_joint_states(device):
    root = ET.parse(DRIVER / "resource" / "panther.urdf").getroot()
    assert root.get("name") == "mh3"
    assert not root.findall(".//mesh") and not root.findall(".//visual") and not root.findall("ros2_control")
    moving = sorted(j.get("name") for j in root.findall("joint") if j.get("type") == "revolute")
    assert moving == sorted(URDF_JOINTS)
    link_names = {l.get("name") for l in root.findall("link")}
    for joint in root.findall("joint"):
        assert joint.find("parent").get("link") in link_names and joint.find("child").get("link") in link_names
    assert {n for n in device._urdf_joint_names() if n.startswith(("joint_la", "joint_ra", "joint_head"))} == set(URDF_JOINTS)


def test_strip_urdf_handles_the_utf16_capture_with_trailing_separator():
    import tempfile

    spec = importlib.util.spec_from_file_location("strip_urdf", DRIVER / "tools" / "strip_urdf.py")
    tool_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool_module)
    document = ('<?xml version="1.0" ?><robot name="r"><link name="a"><visual><geometry><mesh filename="file:///x"/>'
                '</geometry></visual></link><link name="b"/><joint name="j" type="revolute"><parent link="a"/>'
                '<child link="b"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>'
                '<ros2_control name="c"><joint name="j"/></ros2_control></robot>')
    with tempfile.TemporaryDirectory() as directory:
        capture, out = Path(directory) / "capture.txt", Path(directory) / "out.urdf"
        capture.write_bytes(("\ufeff" + document + "\r\n\r\n---\r\n").encode("utf-16-le"))
        assert tool_module.main([str(capture), str(out)]) == 0
        root = ET.parse(out).getroot()
    assert len(root.findall("joint")) == 1 and not root.findall(".//mesh") and not root.findall("ros2_control")
    assert root.find("joint").find("limit").get("upper") == "1"


# ── payloads ─────────────────────────────────────────────────────────────────

def strict(value):
    """json.dumps that refuses NaN/Infinity, the way the dashboard's parser does."""
    return json.loads(json.dumps(value, allow_nan=False))


def test_joint_state_turns_nan_into_null_and_filters_to_the_urdf(payloads):
    msg = joint_state(["joint_la0", "joint_head0", "left_wheel_joint", "joint_ra3"],
                      [0.1, 0.5, 7.0, NAN], [NAN, 0.25, 0.0, 0.0], [NAN, NAN, NAN, NAN])
    result = strict(payloads.joint_state_payload(msg, frozenset(URDF_JOINTS)))
    names = [j["name"] for j in result["joints"]]
    assert names == ["joint_la0", "joint_head0"], "unknown joint and NaN position are left out"
    assert result["dropped"] == 1
    first = result["joints"][0]
    assert first == {"idx": 0, "name": "joint_la0", "q": 0.1, "dq": None, "tau": None}
    assert result["joints"][1]["dq"] == 0.25


def test_joint_state_tolerates_short_arrays(payloads):
    msg = NS(header=header(sec=0), name=["joint_la0", "joint_la1"], position=[0.1], velocity=[], effort=[])
    result = strict(payloads.joint_state_payload(msg))
    assert [j["name"] for j in result["joints"]] == ["joint_la0"]
    assert result["stamp_ms"] is None


def test_battery_percentage_is_a_ratio_and_nan_becomes_null(payloads):
    result = strict(payloads.battery_payload(battery(0.873)))
    assert result["percent"] == 87.3 and result["raw_percentage"] == 0.873
    assert result["charge_ah"] is None and result["capacity_ah"] is None
    assert result["status"] == "discharging" and result["health"] == "good"
    assert result["voltage_v"] == 50.02 and result["current_a"] == -2.2 and result["present"] is True
    assert payloads.battery_payload(battery(NAN))["percent"] is None
    assert payloads.battery_payload(battery(87.0))["percent"] is None, "a value outside 0..1 is not guessed at"
    assert payloads.battery_payload(battery(power_supply_status=9))["status"] == "code_9"


def test_imu_marks_unprovided_fields_null_by_the_ros_convention(payloads):
    full = strict(payloads.imu_payload(imu((0.0, 0.0, 0.7071068, 0.7071068))))
    assert full["orientation_xyzw"] == pytest.approx([0, 0, 0.70711, 0.70711], abs=1e-4)
    assert full["euler_rpy_rad"][2] == pytest.approx(1.5708, abs=1e-3)
    assert full["angular_velocity_rad_s"] == [0.01, -0.02, 0.03]
    assert full["linear_acceleration_m_s2"][2] == 9.81
    no_orientation = payloads.imu_payload(imu(orientation_cov0=-1.0))
    assert no_orientation["orientation_xyzw"] is None and no_orientation["euler_rpy_rad"] is None
    assert no_orientation["angular_velocity_rad_s"] is not None
    assert payloads.imu_payload(imu((0, 0, 0, 0)))["orientation_xyzw"] is None, "an all-zero quaternion is not an attitude"
    assert payloads.imu_payload(imu(gyro_cov0=-1.0))["angular_velocity_rad_s"] is None
    assert payloads.imu_payload(imu(accel_cov0=-1.0))["linear_acceleration_m_s2"] is None


def test_odom_reading_reports_only_the_planar_axes(payloads):
    twist, meta = payloads.odom_reading(odometry(0.3, -0.1, 0.5))
    assert twist == [0.3, -0.1, None, None, None, 0.5]
    assert meta == {"vendor_ms": 1_794_300_000_000, "frame_id": "odom", "child_frame_id": "base_link"}
    assert payloads.odom_reading(odometry(NAN, 0.0, 0.0))[0][0] is None


def test_diagnostic_level_byte_becomes_an_int_and_order_is_irrelevant(payloads):
    entries = payloads.diagnostic_entries(normal_diagnostics())
    assert [e["level"] for e in entries] == [0, 0, 0]
    assert all(isinstance(e["level"], int) for e in entries)
    assert payloads.byte_value(b"\x02") == 2 and payloads.byte_value("\x01") == 1 and payloads.byte_value(3) == 3
    assert payloads.byte_value(b"") == 0 and payloads.byte_value(None) == 0
    strict(entries)


def _diag_state(payloads, message, received_ms=1_000):
    return {e["name"]: {"entry": e, "received_ms": received_ms} for e in payloads.diagnostic_entries(message)}


def test_estop_passes_vendor_text_through_and_never_maps_unknown_strings_to_safe(payloads):
    state = _diag_state(payloads, normal_diagnostics())
    ok = payloads.estop_payload(state, 1_500, 3_000)
    assert ok["found"] and ok["stop_button_status"] == "正常" and ok["message"] == "急停状态正常"
    assert ok["level"] == 0 and ok["age_ms"] == 500 and ok["stale"] is False

    pressed = _diag_state(payloads, diagnostics(
        status("diagnostic_broadcaster: Emergency stop", level=b"\x02", message="急停已按下", stop_button_status="按下")))
    result = payloads.estop_payload(pressed, 1_100, 3_000)
    assert result["level"] == 2 and result["level_name"] == "error"
    assert result["stop_button_status"] == "按下" and result["stale"] is False

    assert payloads.estop_payload(state, 1_000 + 3_001, 3_000)["stale"] is True
    missing = payloads.estop_payload(_diag_state(payloads, diagnostics(status("other"))), 1_000, 3_000)
    assert missing["found"] is False and missing["stale"] is True


def test_health_payload_is_compact_and_lists_only_what_is_wrong(payloads):
    state = _diag_state(payloads, diagnostics(
        status("b", level=b"\x01", message="warm"), status("a"),
        status("lifecycle_manager_navigation: Nav2 Health", level=b"\x02", message="Nav2 is inactive")))
    result = strict(payloads.health_payload(state, 3_000, 5_000))
    assert set(result) == {"count", "worst_level", "worst_level_name", "problems"}
    assert result["count"] == 3 and result["worst_level"] == 2 and result["worst_level_name"] == "error"
    assert result["problems"] == ["b: warm", "lifecycle_manager_navigation: Nav2 Health: Nav2 is inactive"], \
        "sorted by name (the robot's order changes per message); the ok entry 'a' is not listed"
    assert strict(payloads.health_payload({}, 3_000, 5_000))["problems"] == []


def test_a_diagnostic_that_looks_fine_but_has_stopped_updating_is_a_problem(payloads):
    """Entries are kept by name, so a dead publisher's last 'ok' would otherwise look healthy for ever."""
    state = _diag_state(payloads, diagnostics(status("camera: Temperatures"), status("CAN bus")))
    state["CAN bus"]["received_ms"] = 3_000                 # refreshed; the camera entry is 6 s old
    result = payloads.health_payload(state, 7_000, 5_000)
    assert result["problems"] == ["camera: Temperatures: 超过 5 秒没有更新"]
    assert result["worst_level"] == 0, "stale is reported in `problems`, it is not invented as a level"
    state["CAN bus"]["entry"] = {**state["CAN bus"]["entry"], "level": 1, "message": "slow"}
    assert payloads.health_payload(state, 9_000, 5_000)["problems"] == [
        "CAN bus: slow（且超过 5 秒没有更新）", "camera: Temperatures: 超过 5 秒没有更新"]


def test_display_helpers_keep_only_the_informative_fields(payloads):
    shown = payloads.battery_display(payloads.battery_payload(battery(0.97)))
    assert shown == {"percent": 97.0, "voltage_v": 50.02, "current_a": -2.2, "temperature_c": 29.0,
                     "status": "discharging"}
    full = payloads.estop_payload(_diag_state(payloads, normal_diagnostics()), 1_500, 3_000)
    assert set(payloads.estop_display(full)) == {"found", "level", "message", "stop_button_status",
                                                 "age_ms", "stale"}
    assert payloads.estop_display(payloads.estop_payload({}, 1_000, 3_000)) == {
        "found": False, "stale": True, "age_ms": None, "message": "no emergency-stop diagnostic has been received"}
    assert payloads.joint_display(payloads.single_joint(joint_state(["joint_lf"], [0.02], [0.0], [-32.7]))) == {
        "position": 0.02, "effort": -32.7}


def test_laser_keeps_the_nearest_return_in_each_bin(payloads):
    ranges = [5.0] * 1800
    ranges[7] = 0.4                       # a thin obstacle that every-nth-beam sampling steps over
    ranges[100:105] = [NAN, INF, -1.0, 0.05, 80.0]
    result = strict(payloads.laser_payload(scan(ranges=ranges), max_points=360))
    assert result["beams"] == 1800 and result["bins"] == 360 and result["beams_per_bin"] == 5
    assert result["ranges"][1] == 0.4, "bin 1 holds beams 5..9"
    assert result["ranges"][20] is None, "NaN, inf, below range_min and above range_max are all invalid"
    assert result["nearest_m"] == 0.4
    assert result["angle_increment"] == pytest.approx(3.490658e-3 * 5, rel=1e-4)
    assert 0.9 < result["valid_ratio"] < 1.0


def test_laser_handles_empty_and_all_invalid_scans(payloads):
    empty = strict(payloads.laser_payload(scan(ranges=[]), 360))
    assert empty["ranges"] == [] and empty["valid_ratio"] == 0.0 and empty["nearest_m"] is None
    blind = strict(payloads.laser_payload(scan(n=360, fill=INF), 360))
    assert set(blind["ranges"]) == {None} and blind["nearest_m"] is None


def test_laser_topdown_view_is_a_decodable_jpeg(payloads):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    result = payloads.laser_payload(scan(fill=3.0), 360)
    jpeg = payloads.laser_topdown_jpeg(result, size=200)
    picture = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert picture.shape == (200, 200, 3)
    assert picture[100, 100].tolist() != [24, 24, 24], "the robot marker is drawn"


# ── image conversion ─────────────────────────────────────────────────────────

def _decode(jpeg):
    cv2, np = pytest.importorskip("cv2"), pytest.importorskip("numpy")
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_UNCHANGED)


def test_color_jpeg_swaps_rgb_to_bgr_and_scales_wide_frames(payloads):
    pytest.importorskip("cv2")
    red = bytes([255, 0, 0]) * (64 * 48)
    jpeg, size = payloads.encode_color_jpeg(image(64, 48, "rgb8", red))
    assert size == (64, 48)
    pixel = _decode(jpeg)[24, 32]
    assert pixel[2] > 200 and pixel[0] < 60, "rgb8 red must come out red"

    wide = image(1280, 720, "bgr8", bytes([10, 20, 30]) * (1280 * 720))
    _, size = payloads.encode_color_jpeg(wide, max_width=640)
    assert size == (640, 360)


def test_color_jpeg_honours_row_padding_and_other_encodings(payloads):
    pytest.importorskip("cv2")
    row = bytes([0, 0, 255]) * 8 + b"\xaa" * 5                      # 8 px of BGR red plus 5 padding bytes
    padded = image(8, 4, "bgr8", row * 4, step=len(row))
    assert _decode(payloads.encode_color_jpeg(padded)[0])[2, 4][2] > 200
    assert payloads.encode_color_jpeg(image(4, 4, "mono8", bytes([128]) * 16))[1] == (4, 4)
    assert payloads.encode_color_jpeg(image(4, 4, "rgba8", bytes([255, 0, 0, 255]) * 16))[1] == (4, 4)
    assert payloads.encode_color_jpeg(image(4, 4, "bgra8", bytes([0, 0, 255, 255]) * 16))[1] == (4, 4)


@pytest.mark.parametrize("bad, reason", [
    (lambda: image(4, 4, "yuv422", bytes(32)), "colour encoding"),
    (lambda: image(4, 4, "rgb8", bytes(10), step=12), "bytes for a"),
    (lambda: image(4, 4, "rgb8", bytes(48), step=6), "row step"),
    (lambda: image(0, 4, "rgb8", b"", step=0), "empty image"),
])
def test_color_jpeg_rejects_what_it_cannot_convert_loudly(payloads, bad, reason):
    pytest.importorskip("cv2")
    with pytest.raises(payloads.UnsupportedImage, match=reason):
        payloads.encode_color_jpeg(bad())


def test_depth_16uc1_is_resampled_to_640x480_without_blending_values(payloads):
    np = pytest.importorskip("numpy")
    values = np.array([[100, 200, 300, 400], [500, 600, 700, 800], [900, 1000, 1100, 1200]], dtype="<u2")
    data, size, source = payloads.encode_depth_zlib(image(4, 3, "16UC1", values.tobytes()))
    assert size == (640, 480) and source == (4, 3)
    depth = np.frombuffer(zlib.decompress(data), dtype="<u2").reshape(480, 640)
    assert set(np.unique(depth)) == set(values.flatten()), "nearest-neighbour only; no new values appear"
    assert depth[0, 0] == 100 and depth[479, 639] == 1200


def test_depth_native_640x480_passes_through_unchanged(payloads):
    np = pytest.importorskip("numpy")
    depth = (np.arange(480 * 640) % 5000).astype("<u2").reshape(480, 640)
    data, size, _ = payloads.encode_depth_zlib(image(640, 480, "16UC1", depth.tobytes()))
    assert np.array_equal(np.frombuffer(zlib.decompress(data), dtype="<u2").reshape(480, 640), depth)


def test_depth_32fc1_metres_become_millimetres_and_invalid_becomes_zero(payloads):
    np = pytest.importorskip("numpy")
    metres = np.array([[1.234, NAN, -1.0, INF], [0.0, 70.0, 2.0, 0.0004]], dtype="<f4")
    data, _, _ = payloads.encode_depth_zlib(image(4, 2, "32FC1", metres.tobytes()), output_size=(4, 2))
    out = np.frombuffer(zlib.decompress(data), dtype="<u2").reshape(2, 4)
    assert out.tolist() == [[1234, 0, 0, 0], [0, 65535, 2000, 0]]


def test_depth_big_endian_and_unsupported_encodings(payloads):
    np = pytest.importorskip("numpy")
    big = np.array([[258, 513]], dtype=">u2")
    data, _, _ = payloads.encode_depth_zlib(image(2, 1, "16UC1", big.tobytes(), bigendian=1), output_size=(2, 1))
    assert np.frombuffer(zlib.decompress(data), dtype="<u2").tolist() == [258, 513]
    with pytest.raises(payloads.UnsupportedImage, match="depth encoding"):
        payloads.encode_depth_zlib(image(2, 1, "rgb8", bytes(6)))


# ── hub: rates, averaging, staleness ─────────────────────────────────────────

def test_joints_are_throttled_to_the_configured_rate(hub, sink, clock):
    msg = joint_state(URDF_JOINTS, [0.0] * len(URDF_JOINTS))
    for _ in range(300):                                    # 100 Hz for 3 s
        clock.advance(0.01)
        hub.on_joint_state(msg)
    sent = sink.count("/panther/joints")
    assert 29 <= sent <= 31, f"asked for 10 Hz over 3 s, sent {sent}"
    assert hub.stream("joints")["received"] == 300
    assert len(sink.last("/panther/joints")["joints"]) == 18


def test_odom_publishes_the_mean_of_the_window_and_keeps_unmeasured_axes_null(hub, sink, clock):
    hub.on_odom(odometry(9.9))                              # first reading publishes on its own
    sink.jsons.clear()
    for i in range(9):
        clock.advance(0.01)
        hub.on_odom(odometry(vx=i / 10, wz=0.2))
    sample = sink.last("/panther/state/odom")
    assert sample["schema"] == "motus.odom/1" and sample["frame"] == "body"
    assert sample["twist"][0] == pytest.approx(0.4), "mean of 0.0..0.8, not the last reading"
    assert sample["twist"][5] == pytest.approx(0.2)
    assert sample["twist"][2:5] == [None, None, None]
    assert sample["pose"] is None
    assert sample["vendor"]["samples"] == 9 and sample["vendor"]["child_frame_id"] == "base_link"
    assert sample["vendor"]["stamp_source"] == "robot", "the robot's stamp is quoted when its clock agrees with ours"


def test_odom_with_an_unsynchronised_robot_clock_publishes_arrival_time(hub, sink):
    hub.on_odom(odometry(0.1, sec=1_000_000))              # robot clock decades behind the host
    sample = sink.last("/panther/state/odom")
    assert sample["vendor"]["stamp_source"] == "received"
    assert abs(sample["stamp_ms"] - 1_794_300_000_000) < 5_000


def test_odom_interface_declaration_lists_only_measured_axes(hub):
    interface = hub.card_info("odom")["odom_interface"]
    assert interface["provides"] == ["vx", "vy", "wz"] and interface["pose_drift"] == "none"


def _state(sink):
    return sink.last("/panther/robot_state")


def test_robot_state_merges_grippers_and_lift_with_their_age(hub, sink, clock):
    hub.on_gripper_left(joint_state(["joint_lf"], [0.023], [0.0], [-32.763]))
    hub.on_lift(joint_state(["body_joint"], [0.61], [-0.0], [0.0]))
    clock.advance(0.2)
    hub.tick()
    state = _state(sink)
    assert state["grippers"]["left"]["position"] == 0.023 and state["grippers"]["left"]["effort"] == -32.763
    assert state["lift"]["position"] == 0.61, "not clamped to the URDF's stale 0.6 limit"
    assert state["grippers"]["right"] is None, "a joint that never reported is null, not zero"
    assert state["grippers"]["left"]["age_ms"] == 200 and state["grippers"]["left"]["fresh"] is True
    clock.advance(2.0)
    hub.tick()
    assert _state(sink)["lift"]["fresh"] is False and _state(sink)["lift"]["age_ms"] == 2200


def test_robot_state_carries_battery_diagnostics_estop_and_controllers_in_blocks(hub, sink, clock):
    hub.on_battery(battery(0.99))
    hub.on_diagnostics(diagnostics(
        status("diagnostic_broadcaster: CAN bus", message="CAN bus OK", can0_state="ERROR-ACTIVE"),
        status("diagnostic_broadcaster: Emergency stop", message="急停状态正常", stop_button_status="正常"),
        status("lifecycle_manager_navigation: Nav2 Health", level=b"\x02", message="Nav2 is inactive")))
    hub.on_controllers({"lifting_controller": "active", "head_forward_position_controller": "active"})
    clock.advance(0.6)
    hub.tick()
    state = _state(sink)
    strict(state)
    assert set(state) == {"stamp_ms", "ready_for_motion", "not_ready_reasons", "battery", "diagnostics",
                          "estop", "grippers", "lift", "controllers"}
    assert set(state["battery"]) == {"percent", "voltage_v", "current_a", "temperature_c", "status",
                                     "age_ms", "fresh"}
    assert state["battery"]["percent"] == 99.0 and state["battery"]["fresh"] is True
    assert set(state["diagnostics"]) == {"count", "worst_level", "worst_level_name", "problems",
                                         "age_ms", "fresh"}
    assert state["diagnostics"]["count"] == 3 and state["diagnostics"]["fresh"] is True
    assert state["diagnostics"]["problems"] == ["lifecycle_manager_navigation: Nav2 Health: Nav2 is inactive"], \
        "an error elsewhere on the robot (Nav2 is not running) is named, not buried"
    assert set(state["estop"]) == {"found", "level", "message", "stop_button_status", "age_ms", "stale"}
    assert state["estop"]["found"] and state["estop"]["stop_button_status"] == "正常"
    assert state["controllers"]["states"]["lifting_controller"] == "active"


def test_the_trimmed_view_does_not_trim_what_the_motion_gate_reads(hub, clock):
    """The gate reads the hub's full estop payload; only the card's view was shortened."""
    hub.on_joint_state(joint_state(URDF_JOINTS, [0.0] * 18))
    hub.on_diagnostics(diagnostics(status("diagnostic_broadcaster: Emergency stop", level=b"\x02",
                                          message="anything", stop_button_status="whatever")))
    ready, reasons = hub.motion_ready()
    assert not ready and "level is 2" in reasons[0]


def test_robot_state_says_so_when_nothing_has_arrived_instead_of_looking_healthy(hub, sink, clock):
    clock.advance(1)
    hub.tick()
    state = _state(sink)
    assert state["battery"] is None and state["lift"] is None and state["controllers"] is None
    assert state["diagnostics"] is None, "no diagnostics yet is null, like every other block, not an empty list"
    assert state["estop"]["found"] is False and state["estop"]["stale"] is True
    assert state["ready_for_motion"] is False


def test_robot_state_is_published_at_its_rate_and_at_once_on_a_diagnostics_change(hub, sink, clock):
    hub.on_diagnostics(normal_diagnostics())
    assert _state(sink)["estop"]["level"] == 0
    published = sink.count("/panther/robot_state")

    pressed = diagnostics(status("diagnostic_broadcaster: Emergency stop", level=b"\x02",
                                 message="急停已按下", stop_button_status="按下"))
    clock.advance(0.05)
    hub.on_diagnostics(pressed)
    assert sink.count("/panther/robot_state") == published + 1, "a change is sent at once, not at the next slot"
    assert _state(sink)["estop"]["level"] == 2
    assert _state(sink)["diagnostics"]["worst_level"] == 2

    sink.jsons.clear()
    for _ in range(100):                                    # 10 s of 10 Hz timer ticks, nothing changing
        clock.advance(0.1)
        hub.tick()
    assert 19 <= sink.count("/panther/robot_state") <= 21, "2 Hz"


def test_estop_goes_stale_when_diagnostics_stop(hub, sink, clock):
    hub.on_diagnostics(normal_diagnostics())
    assert _state(sink)["estop"]["stale"] is False
    for _ in range(60):                                     # 6 s of timer ticks and no diagnostics
        clock.advance(0.1)
        hub.tick()
    latest = _state(sink)["estop"]
    assert latest["stale"] is True and latest["stop_button_status"] == "正常", "old text kept, flagged as old"
    assert latest["age_ms"] >= 3_000


def test_motion_ready_needs_fresh_joints_and_a_fresh_normal_estop_diagnostic(hub, clock):
    ready, reasons = hub.motion_ready()
    assert not ready and len(reasons) == 2
    hub.on_joint_state(joint_state(URDF_JOINTS, [0.0] * 18))
    hub.on_diagnostics(normal_diagnostics())
    assert hub.motion_ready() == (True, [])

    # An unrelated error elsewhere (an absent camera, say) does not stop the robot moving ...
    hub.on_diagnostics(diagnostics(status("diagnostic_broadcaster: Camera", level=b"\x02", message="missing")))
    assert hub.motion_ready()[0] is True
    # ... but the emergency-stop diagnostic leaving "ok" does, whatever its text says.
    hub.on_diagnostics(diagnostics(status("diagnostic_broadcaster: Emergency stop", level=b"\x02",
                                          message="anything", stop_button_status="whatever")))
    ready, reasons = hub.motion_ready()
    assert not ready and "level is 2" in reasons[0]

    hub.on_diagnostics(normal_diagnostics())
    clock.advance(1.5)                                      # joints are older than 1 s now
    assert hub.motion_ready()[0] is False
    hub.on_joint_state(joint_state(URDF_JOINTS, [0.0] * 18))
    clock.advance(3.5)                                      # and the estop diagnostic is older than 3 s
    hub.on_joint_state(joint_state(URDF_JOINTS, [0.0] * 18))
    ready, reasons = hub.motion_ready()
    assert not ready and "stale" in reasons[0]


def test_joint_positions_lift_and_limits_come_from_the_hub(hub, clock):
    assert hub.joint_positions() is None and hub.lift_position() is None
    hub.on_joint_state(joint_state(["joint_la0", "joint_head0"], [0.1, NAN]))
    assert hub.joint_positions() == {"joint_la0": 0.1}, "a NaN position is not a position"
    hub.on_lift(joint_state(["body_joint"], [0.61]))
    assert hub.lift_position() == 0.61
    clock.advance(2)
    assert hub.joint_positions() is None and hub.lift_position() is None

    limits, source = hub.joint_limits()
    assert source == "bundled URDF" and limits["joint_head1"] == (-1.57, 1.57)
    hub.on_robot_description(NS(data='<robot name="x"><joint name="joint_la0" type="revolute">'
                                     '<limit lower="-1" upper="1"/></joint></robot>'))
    limits, source = hub.joint_limits()
    assert source == "live /robot_description" and limits == {"joint_la0": (-1.0, 1.0)}


def test_laser_json_always_publishes_and_the_view_waits_for_start(hub, sink):
    pytest.importorskip("cv2")
    hub.on_laser("front", scan())
    assert sink.count("/panther/laser_scan") == 1 and not sink.images
    hub.activate("laser_scan")
    hub.on_laser("front", scan())
    assert [i[1] for i in sink.images] == ["/panther/laser_scan_view"]
    assert sink.images[0][2] == "jpeg" and sink.images[0][0] == "views"


def _tf(*edges):
    """A /tf_static message: (parent, child, (x, y, z), (qx, qy, qz, qw))."""
    return NS(transforms=[NS(header=NS(frame_id=parent), child_frame_id=child,
                             transform=NS(translation=NS(x=t[0], y=t[1], z=t[2]),
                                          rotation=NS(x=q[0], y=q[1], z=q[2], w=q[3])))
                          for parent, child, t, q in edges])


REAR_FACING = (0.0, 0.0, 1.0, 0.0)                                          # yaw = pi


def test_front_and_rear_lidars_are_merged_into_one_scan_round_base_link(hub, sink, clock):
    hub.on_tf_static(_tf(("base_link", "link_front_radar", (0.3, 0.0, 0.1), (0, 0, 0, 1)),
                         ("base_link", "link_back_radar", (-0.3, 0.0, 0.1), REAR_FACING)))
    # Each lidar sees a wall 1 m dead ahead of itself: the front one at x=1.3, the rear one at x=-1.3.
    front = scan(n=360, fill=INF, header=header(frame_id="link_front_radar"), angle_min=-3.14159,
                 angle_increment=2 * 3.14159 / 360)
    front.ranges[180] = 1.0
    rear = scan(n=360, fill=INF, header=header(frame_id="link_back_radar"), angle_min=-3.14159,
                angle_increment=2 * 3.14159 / 360)
    rear.ranges[180] = 1.0
    hub.on_laser("front", front)
    clock.advance(0.2)                                      # the next 5 Hz slot
    hub.on_laser("rear", rear)
    merged = sink.last("/panther/laser_scan")
    strict(merged)
    assert merged["frame_id"] == "base_link" and merged["transformed"] is True
    hits = {round(i * merged["angle_increment"] + merged["angle_min"], 2): r
            for i, r in enumerate(merged["ranges"]) if r is not None}
    assert hits[0.0] == pytest.approx(1.3, abs=0.01), "front wall ahead of base_link"
    assert max(hits.items(), key=lambda kv: abs(kv[0]))[1] == pytest.approx(1.3, abs=0.01), "rear wall behind it"
    assert merged["sources"]["front"]["used"] and merged["sources"]["rear"]["used"]
    assert merged["nearest_m"] == pytest.approx(1.3, abs=0.01)


def test_a_lidar_without_a_known_transform_is_left_out_and_the_reason_is_given(hub, sink, clock):
    hub.on_tf_static(_tf(("base_link", "link_front_radar", (0.3, 0, 0), (0, 0, 0, 1))))
    hub.on_laser("front", scan(fill=2.0))
    clock.advance(0.2)
    hub.on_laser("rear", scan(fill=2.0, header=header(frame_id="link_back_radar")))
    merged = sink.last("/panther/laser_scan")
    assert merged["sources"]["front"]["used"] is True
    assert merged["sources"]["rear"]["used"] is False
    assert "no fixed transform from link_back_radar to base_link" in merged["sources"]["rear"]["reason"]


def test_with_no_transform_at_all_the_first_lidar_is_shown_in_its_own_frame(hub, sink):
    hub.on_laser("front", scan(fill=2.0))
    merged = sink.last("/panther/laser_scan")
    assert merged["transformed"] is False and merged["frame_id"] == "link_front_radar"
    assert "no /tf_static transform" in merged["sources"]["front"]["note"]
    assert merged["sources"]["rear"] == {"used": False, "reason": "no data received"}


def test_a_stale_lidar_drops_out_of_the_merged_scan(hub, sink, clock):
    hub.on_laser("rear", scan(fill=2.0, header=header(frame_id="link_back_radar")))
    clock.advance(2.0)
    hub.on_laser("front", scan(fill=3.0))
    merged = sink.last("/panther/laser_scan")
    assert merged["sources"]["rear"] == {"frame_id": "link_back_radar", "age_ms": 2000,
                                         "used": False, "reason": "stale"}
    assert merged["nearest_m"] == 3.0


def test_cameras_do_no_work_until_started_and_unsupported_frames_are_reported(hub, sink, clock):
    pytest.importorskip("cv2")
    frame = image(16, 8, "rgb8", bytes([200, 100, 50]) * 128)
    hub.on_image("camera_color", frame)
    assert not sink.images and hub.stream("camera_color")["received"] == 1
    assert "未启动" in hub.card_info("camera_color")["note"]

    hub.activate("camera_color")
    hub.on_image("camera_color", frame)
    assert sink.images[-1][:3] == ("camera_color", "/panther/camera_color", "jpeg")
    assert hub.card_info("camera_color")["image"]["encoding"] == "rgb8"

    clock.advance(0.2)                                      # past the 10 Hz throttle
    hub.on_image("camera_color", image(4, 4, "yuv422", bytes(32)))
    stream = hub.stream("camera_color")
    assert stream["errors"] == 1 and "colour encoding" in stream["last_error"]
    assert len(sink.images) == 1, "an unsupported frame publishes nothing"


def test_camera_frames_above_the_rate_are_skipped_before_any_conversion(hub, sink, clock, monkeypatch, device):
    calls = []
    monkeypatch.setattr(device, "encode_color_jpeg", lambda *a, **k: calls.append(1) or (b"x", (1, 1)))
    hub.activate("camera_color")
    for _ in range(30):                                     # 30 fps source, 10 fps card
        clock.advance(1 / 30)
        hub.on_image("camera_color", image(2, 2, "rgb8", bytes(12)))
    assert 9 <= len(calls) <= 11, len(calls)


def test_depth_frames_are_published_with_the_compressed_depth_format(hub, sink, device):
    np = pytest.importorskip("numpy")
    hub.activate("camera_depth")
    hub.on_image("camera_depth", image(4, 2, "16UC1", np.arange(8, dtype="<u2").tobytes()))
    lane, topic, image_format, data, _ = sink.images[-1]
    assert (lane, topic, image_format) == ("camera_depth", "/panther/camera_depth", device.DEPTH_FORMAT)
    assert len(zlib.decompress(data)) == 640 * 480 * 2


def test_a_handler_failure_is_counted_and_never_reaches_the_executor(hub, capsys, clock):
    class Hostile:
        def __getattr__(self, name):
            raise RuntimeError(f"broken message: {name}")

    hub.on_joint_state(NS())                                # not a message at all
    hub.on_battery(Hostile())
    hub.on_diagnostics(NS(status=[NS()]))
    assert hub.stream("joints")["errors"] == 1 and "AttributeError" in hub.stream("joints")["last_error"]
    assert hub.stream("robot_state")["errors"] == 1 and "RuntimeError" in hub.stream("robot_state")["last_error"]
    assert "handler failed" in capsys.readouterr().out
    clock.advance(0.2)
    hub.on_joint_state(joint_state(["joint_la0"], [0.1]))   # and the hub still works afterwards
    assert hub.stream("joints")["published"] == 1


def test_a_ten_hertz_timer_that_ticks_slightly_early_does_not_lose_samples(hub, sink, clock):
    hub.on_lift(joint_state(["body_joint"], [0.61]))
    for _ in range(100):
        clock.advance(0.0995)                               # jittery 10 Hz timer
        hub.tick()
    assert 19 <= sink.count("/panther/robot_state") <= 21, "the 2 Hz state keeps its rate under timer jitter"


def test_non_finite_numbers_cannot_reach_the_wire(hub, sink, monkeypatch, device, clock):
    monkeypatch.setattr(device, "battery_payload", lambda msg: {"percent": float("nan")})
    hub.on_battery(battery())
    hub.tick()
    assert sink.count("/panther/robot_state") == 0, "allow_nan=False rejects it before it is sent"
    assert hub.stream("robot_state")["errors"] == 1


def test_a_full_bridge_is_counted_not_blocked_on(device, config, clock):
    full = Sink(accept=False)
    hub = device.Hub(config, "panther", full, clock=lambda: clock.wall, monotonic=lambda: clock.mono)
    hub.on_battery(battery())
    hub.tick()
    assert hub.stream("robot_state")["dropped"] == 1 and hub.stream("robot_state")["published"] == 0


# ── hub: honesty about missing data ──────────────────────────────────────────

def test_robot_link_reports_a_container_that_hears_nothing(hub, clock, capsys):
    assert hub.robot_link()["state"] == "never_received"
    clock.advance(31)
    hub.tick()
    assert "no data from the robot's DDS" in capsys.readouterr().out
    hub.on_battery(battery())
    assert hub.robot_link()["state"] == "receiving"
    clock.advance(10)
    assert hub.robot_link()["state"] == "silent"


def test_a_card_with_no_feed_still_starts_and_says_why(device, hub):
    plugin = device.PantherPlugin(hub)
    started = plugin.dispatch("start", {"_tool_name": "camera_color"})
    assert started["state"] == "running", "agent-core rolls the whole project back if a start fails"
    assert "panther_orbbec" in started["warning"]
    laser = plugin.dispatch("info", {"_tool_name": "laser_scan"})
    assert laser["state"] == "idle" and "雷达" in laser["warning"]
    never = plugin.dispatch("info", {"_tool_name": "robot_state"})
    assert "/battery_state_broadcaster/battery_state" in never["warning"]


def test_stale_feed_is_flagged_on_the_card(device, hub, clock):
    plugin = device.PantherPlugin(hub)
    hub.on_battery(battery())
    assert "warning" not in plugin.dispatch("info", {"_tool_name": "robot_state"})
    clock.advance(6)
    info = plugin.dispatch("info", {"_tool_name": "robot_state"})
    assert "过期" in info["warning"] and info["stream"]["fresh"] is False


def test_camera_info_declares_unknown_optics_rather_than_guessing(device, hub):
    for card in ("camera_color", "camera_depth"):
        declared = hub.camera_info(card)
        assert declared["source"] == "unknown" and declared["half_fov_rad"] is None and declared["K"] is None
        assert declared["id"] == f"unixai/panther/{card}"
    assert hub.camera_info("camera_depth")["width"] == 640 and hub.camera_info("camera_depth")["height"] == 480
    assert hub.camera_info("camera_color")["width"] is None


# ── plugin / bundle ──────────────────────────────────────────────────────────

def test_model_card_returns_the_urdf_as_a_plain_dict(device, hub):
    result = device.PantherPlugin(hub).dispatch("model", {"_tool_name": "model"})
    assert isinstance(result, dict) and result["robot"] == "mh3"
    assert ET.fromstring(result["urdf"].split("-->", 1)[1]).tag == "robot"


def test_every_sensor_answers_start_stop_info_with_a_state(device, hub):
    plugin = device.PantherPlugin(hub)
    for card in device.SENSOR_CARDS[1:]:
        started = plugin.dispatch("start", {"_tool_name": card})
        assert started["state"] == "running" and started["topic_out"], card
        assert hub.is_active(card)
        assert plugin.dispatch("info", {"_tool_name": card})["state"] == "running"
        assert plugin.dispatch("stop", {"_tool_name": card})["state"] == "idle"
        assert not hub.is_active(card)
        strict(plugin.dispatch("info", {"_tool_name": card}))


def test_unknown_cards_and_actions_are_errors_not_none(device, hub):
    plugin = device.PantherPlugin(hub)
    assert plugin.dispatch("start", {"_tool_name": "arm"})["state"] == "error"
    assert plugin.dispatch("move", {"_tool_name": "joints"})["state"] == "error"
    assert "take start, stop and info" in plugin.dispatch("move", {"_tool_name": "joints"})["message"]


def test_disabled_cards_are_not_listed_or_served(device, hub):
    plugin = device.PantherPlugin(hub, enabled={"camera_depth": False})
    assert "camera_depth" not in [t["name"] for t in plugin.get_tools()]
    assert plugin.dispatch("start", {"_tool_name": "camera_depth"})["state"] == "error"


def test_the_shared_bundle_runtime_can_drive_the_plugin(device, hub, motion, config):
    from common.vendor_runtime import DriverBundle, jsonable
    bundle = DriverBundle([device.PantherPlugin(hub, motion=motion, config=config)])
    assert len(bundle.get_all_tools()) == 13
    for name, arguments in (("model", {}), ("robot_state", {"action": "start"}), ("laser_scan", {"action": "info"}),
                            ("odom", {"action": "stop"}), ("base_drive", {"action": "info"}),
                            ("arm_control", {"action": "stop"}), ("head", {"action": "stop"})):
        result = bundle.dispatch(name, arguments)
        assert isinstance(result, dict), name
        json.dumps(jsonable(result), allow_nan=False)
    assert bundle.dispatch("nope", {}) is None


def test_plugin_stop_releases_the_ros_node_and_the_bridge(device, hub, sink):
    closed = []
    plugin = device.PantherPlugin(hub, nodes=NS(close=lambda: closed.append("nodes")), sink=sink)
    plugin.stop()
    assert closed == ["nodes"] and sink.closed


# ── rclpy wiring (against a stand-in for rclpy) ──────────────────────────────

class _Future:
    """An already-completed rclpy future."""

    def __init__(self, result=None):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def exception(self):
        return None

    def result(self):
        return self._result


class _FakeRos:
    """Just enough of rclpy to check what PantherNodes subscribes to, publishes and calls."""

    def __init__(self, monkeypatch):
        import types

        self.nodes, self.subscriptions, self.timers, self.groups = [], [], [], []
        self.publishers, self.published, self.clients, self.action_clients = {}, {}, {}, {}
        self.subscribers = {"/cmd_vel": 1}
        fake = self

        class Publisher:
            def __init__(self, topic):
                self.topic = topic

            def publish(self, message):
                fake.published.setdefault(self.topic, []).append(message)

        class Client:
            def __init__(self, name):
                self.name = name
                self.controllers = [NS(name="lifting_controller", state="active")]
                self.requests = []

            def wait_for_service(self, timeout_sec=None):
                return True

            def service_is_ready(self):
                return True

            def call_async(self, request):
                self.requests.append(request)
                return _Future(NS(controller=self.controllers, ok=True))

        class Node:
            def __init__(self, name, **kwargs):
                self.name, self.kwargs, self.destroyed = name, kwargs, False
                fake.nodes.append(self)

            def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
                fake.subscriptions.append(NS(type=msg_type, topic=topic, callback=callback, qos=qos, **kwargs))

            def create_timer(self, period, callback, **kwargs):
                fake.timers.append(NS(period=period, callback=callback, **kwargs))

            def destroy_node(self):
                self.destroyed = True

            def create_publisher(self, msg_type, topic, qos):
                fake.publishers[topic] = msg_type
                return Publisher(topic)

            def create_client(self, srv_type, name):
                fake.clients[name] = srv_type
                return Client(name)

            def count_publishers(self, topic):
                return 0

            def count_subscribers(self, topic):
                return fake.subscribers.get(topic, 0)

        class ActionClient:
            def __init__(self, node, action_type, name):
                fake.action_clients[name] = action_type

        class QoSProfile:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)

        class Group:
            def __init__(self):
                fake.groups.append(self)

        def module(name, **attributes):
            instance = types.ModuleType(name)
            instance.__dict__.update(attributes)
            monkeypatch.setitem(sys.modules, name, instance)
            return instance

        def message_types(*names):
            return {name: type(name, (), {}) for name in names}

        class Twist:
            def __init__(self):
                self.linear, self.angular = NS(x=0.0, y=0.0, z=0.0), NS(x=0.0, y=0.0, z=0.0)

        class SwitchRequest:
            def __init__(self):
                self.activate_controllers, self.deactivate_controllers = [], []
                self.strictness, self.activate_asap, self.timeout = 0, False, NS(sec=0)

        module("rclpy")
        module("rclpy.node", Node=Node)
        module("rclpy.action", ActionClient=ActionClient)
        module("rclpy.qos", QoSProfile=QoSProfile,
               HistoryPolicy=NS(KEEP_LAST="keep_last"),
               ReliabilityPolicy=NS(BEST_EFFORT="best_effort", RELIABLE="reliable"),
               DurabilityPolicy=NS(VOLATILE="volatile", TRANSIENT_LOCAL="transient_local"))
        module("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=Group)
        self.sensor = message_types("BatteryState", "Image", "Imu", "JointState", "LaserScan")
        module("sensor_msgs"); module("sensor_msgs.msg", **self.sensor)
        self.nav = message_types("Odometry")
        module("nav_msgs"); module("nav_msgs.msg", **self.nav)
        self.diag = message_types("DiagnosticArray")
        module("diagnostic_msgs"); module("diagnostic_msgs.msg", **self.diag)
        module("std_msgs"); module("std_msgs.msg", **message_types("String", "Float64MultiArray"))
        module("tf2_msgs"); module("tf2_msgs.msg", **message_types("TFMessage"))
        module("geometry_msgs"); module("geometry_msgs.msg", Twist=Twist)
        module("unix_msgs"); module("unix_msgs.msg", **message_types("GripperCommand"))
        module("control_msgs"); module("control_msgs.action", FollowJointTrajectory=type(
            "FollowJointTrajectory", (), {"Goal": type("Goal", (), {})}))
        module("builtin_interfaces"); module("builtin_interfaces.msg", Duration=lambda sec, nanosec: NS(sec=sec, nanosec=nanosec))
        module("trajectory_msgs"); module("trajectory_msgs.msg", **message_types("JointTrajectory", "JointTrajectoryPoint"))
        module("controller_manager_msgs"); module("controller_manager_msgs.srv", **{
            "ListControllers": type("ListControllers", (), {"Request": staticmethod(lambda: NS())}),
            "SwitchController": type("SwitchController", (), {"Request": staticmethod(SwitchRequest)})})
        self.executor = NS(added=[], removed=[])
        self.executor.add_node = self.executor.added.append
        self.executor.remove_node = self.executor.removed.append
        self.ros2 = NS(ctx_robot="robot-context", executor_robot=self.executor)


MOTION_PLUGINS = ("base_drive", "arm_control", "gripper", "lifting", "head")


def test_nodes_subscribe_to_exactly_the_configured_robot_topics(device, config, hub, monkeypatch):
    fake = _FakeRos(monkeypatch)
    nodes = device.PantherNodes(config, fake.ros2, hub)

    (node,) = fake.nodes
    assert node.kwargs == {"context": "robot-context", "enable_rosout": False, "start_parameter_services": False}
    assert fake.executor.added == [node]

    wanted = {
        ("JointState", "/joint_states"), ("BatteryState", "/battery_state_broadcaster/battery_state"),
        ("Imu", "/imu_sensor_broadcaster/imu"), ("Odometry", "/odom"),
        ("JointState", "/left_gripper/joint_states"), ("JointState", "/right_gripper/joint_states"),
        ("JointState", "/lifting_controller/joint_states"), ("DiagnosticArray", "/diagnostics"),
        ("LaserScan", "/lidar_front/scan"), ("LaserScan", "/lidar_rear/scan"),
        ("Image", "/camera/color/image_raw"), ("Image", "/camera/depth/image_raw"),
        ("TFMessage", "/tf_static"), ("String", "/robot_description"),
    }
    assert {(s.type.__name__, s.topic) for s in fake.subscriptions} == wanted
    assert len(fake.subscriptions) == len(wanted), "one subscription per topic"
    for subscription in fake.subscriptions:
        if subscription.topic in ("/tf_static", "/robot_description"):
            # Published once and latched: a volatile subscriber that starts later never sees it.
            assert (subscription.qos.reliability, subscription.qos.durability) == ("reliable", "transient_local")
        else:
            # Best-effort is compatible with reliable and best-effort publishers alike, and cannot
            # back-pressure the robot's own controllers.
            assert subscription.qos.reliability == "best_effort"
            assert subscription.qos.durability == "volatile" and subscription.qos.depth >= 1

    assert [t.period for t in fake.timers] == [0.1, 1.0]
    assert fake.timers[0].callback == hub.tick

    nodes.close()
    assert fake.executor.removed == [node] and node.destroyed


def test_a_sensor_only_configuration_adds_no_publisher_client_or_action_to_the_robot_graph(
        device, config, hub, monkeypatch):
    for name in MOTION_PLUGINS:
        config["plugins"][name]["enabled"] = False
    fake = _FakeRos(monkeypatch)
    nodes = device.PantherNodes(config, fake.ros2, hub)
    assert nodes.motion_io is None
    assert not fake.publishers and not fake.clients and not fake.action_clients
    assert [t.period for t in fake.timers] == [0.1]


def test_motion_io_creates_exactly_the_command_endpoints_and_nothing_else(device, config, hub, monkeypatch):
    fake = _FakeRos(monkeypatch)
    device.PantherNodes(config, fake.ros2, hub)
    assert set(fake.publishers) == {"/head_forward_position_controller/commands",
                                    "/lifting_controller/commands", "/left_gripper/gripper_command",
                                    "/right_gripper/gripper_command"}, \
        "the drive publisher waits for a move to pick its topic"
    assert set(fake.clients) == {"/controller_manager/list_controllers", "/controller_manager/switch_controller"}
    assert set(fake.action_clients) == {
        "/left_arm_joint_trajectory_position_controller/follow_joint_trajectory",
        "/right_arm_joint_trajectory_position_controller/follow_joint_trajectory"}


def test_motion_io_sends_the_messages_the_controllers_expect(device, config, hub, monkeypatch):
    fake = _FakeRos(monkeypatch)
    io = device.PantherNodes(config, fake.ros2, hub).motion_io
    assert io.publish_twist(0.1, -0.1, 0.2) is None and fake.published == {}, "no topic chosen yet"
    topic, counts = io.select_twist_topic(["/cmd_vel", "/cmd_vel_teleop"])
    assert topic == "/cmd_vel" and counts == {"/cmd_vel": 1, "/cmd_vel_teleop": 0}
    io.publish_twist(0.1, -0.1, 0.2)
    twist = fake.published["/cmd_vel"][-1]
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.1, -0.1, 0.2)
    io.publish_head([0.1, -0.2])
    assert fake.published["/head_forward_position_controller/commands"][-1].data == [0.1, -0.2]
    io.publish_lift(0.55)
    lift = fake.published["/lifting_controller/commands"][-1]
    assert (lift.name, lift.position) == ([""], [0.55])
    io.publish_gripper("right", "parallel", 0.4)
    gripper = fake.published["/right_gripper/gripper_command"][-1]
    assert gripper.mode == "parallel" and gripper.command.position == [0.4]
    assert "/left_gripper/gripper_command" not in fake.published

    assert io.list_controllers() == {"lifting_controller": "active"}
    delivered = []
    io.poll_controllers(delivered.append)
    assert delivered == [{"lifting_controller": "active"}]


def test_motion_io_sends_a_trajectory_goal_polls_its_result_and_cancels_it(device, config, hub, monkeypatch):
    fake = _FakeRos(monkeypatch)
    io = device.PantherNodes(config, fake.ros2, hub).motion_io
    seen = []

    class Handle:
        accepted = True

        def get_result_async(self):
            return _Future(NS(status=4, result=NS(error_code=0, error_string="")))

        def cancel_goal_async(self):
            seen.append("cancelled")

    class Server:
        up, accept = True, True

        def wait_for_server(self, timeout_sec):
            return self.up

        def send_goal_async(self, goal):
            seen.append(goal)
            handle = Handle()
            handle.accepted = self.accept
            return _Future(handle)

    server = Server()
    io._arm["left"] = server
    handle = io.send_arm_goal("left", ["joint_la0", "joint_la1"], [0.1, -0.1], 2.5)
    goal = seen[0]
    point = goal.trajectory.points[0]
    assert goal.trajectory.joint_names == ["joint_la0", "joint_la1"] and point.positions == [0.1, -0.1]
    assert (point.time_from_start.sec, point.time_from_start.nanosec) == (2, 500_000_000)
    assert io.poll_arm_result(handle, 0.1) == {"status": 4, "error_code": 0, "error_string": ""}
    io.cancel_arm_goal(handle)
    assert seen[-1] == "cancelled"
    server.accept = False
    with pytest.raises(RuntimeError, match="rejected"):
        io.send_arm_goal("left", ["joint_la0"], [0.0], 2.0)
    server.up = False
    with pytest.raises(RuntimeError, match="unavailable"):
        io.send_arm_goal("left", ["joint_la0"], [0.0], 2.0)


def test_switching_controllers_asks_for_a_strict_switch(device, config, hub, monkeypatch):
    fake = _FakeRos(monkeypatch)
    io = device.PantherNodes(config, fake.ros2, hub).motion_io
    sent = []
    original = io._switch.call_async
    io._switch.call_async = lambda request: sent.append(request) or original(request)
    io.switch_controllers(["left_arm"], ["forward"])
    (request,) = sent
    assert request.activate_controllers == ["left_arm"] and request.deactivate_controllers == ["forward"]
    assert request.strictness == 2 and request.activate_asap is True


def test_heavy_streams_run_in_their_own_callback_groups(device, config, hub, monkeypatch):
    fake = _FakeRos(monkeypatch)
    device.PantherNodes(config, fake.ros2, hub)
    by_topic = {s.topic: s for s in fake.subscriptions}
    heavy = [by_topic[t].callback_group for t in
             ("/camera/color/image_raw", "/camera/depth/image_raw", "/lidar_front/scan", "/lidar_rear/scan")]
    assert len({id(g) for g in heavy}) == 4, "a slow JPEG encode must not queue behind the lidar or the joints"
    assert not hasattr(by_topic["/joint_states"], "callback_group")


def test_subscription_callbacks_reach_the_matching_hub_handlers(device, config, hub, sink, clock, monkeypatch):
    fake = _FakeRos(monkeypatch)
    device.PantherNodes(config, fake.ros2, hub)
    by_topic = {s.topic: s for s in fake.subscriptions}

    by_topic["/joint_states"].callback(joint_state(URDF_JOINTS, [0.0] * 18))
    by_topic["/battery_state_broadcaster/battery_state"].callback(battery(0.5))
    by_topic["/imu_sensor_broadcaster/imu"].callback(imu())
    by_topic["/odom"].callback(odometry(0.1))
    by_topic["/diagnostics"].callback(normal_diagnostics())
    by_topic["/tf_static"].callback(_tf(("base_link", "link_front_radar", (0.3, 0, 0), (0, 0, 0, 1)),
                                        ("base_link", "link_back_radar", (-0.3, 0, 0), REAR_FACING)))
    by_topic["/lidar_front/scan"].callback(scan())
    clock.advance(0.2)
    by_topic["/lidar_rear/scan"].callback(scan(header=header(frame_id="link_back_radar")))
    by_topic["/left_gripper/joint_states"].callback(joint_state(["joint_lf"], [0.02]))
    by_topic["/right_gripper/joint_states"].callback(joint_state(["joint_rf"], [0.03]))
    by_topic["/lifting_controller/joint_states"].callback(joint_state(["body_joint"], [0.61]))
    by_topic["/robot_description"].callback(NS(data='<robot name="x"><joint name="joint_la0" type="revolute">'
                                                    '<limit lower="-1" upper="1"/></joint></robot>'))
    hub.activate("camera_depth")
    by_topic["/camera/color/image_raw"].callback(image(2, 2, "rgb8", bytes(12)))
    by_topic["/camera/depth/image_raw"].callback(image(2, 2, "16UC1", bytes(8)))
    clock.advance(0.6)                                      # past the 2 Hz robot_state slot
    fake.timers[0].callback()

    for topic in ("/panther/joints", "/panther/imu", "/panther/state/odom", "/panther/robot_state",
                  "/panther/laser_scan"):
        assert sink.count(topic) >= 1, topic
    assert sink.last("/panther/laser_scan")["frame_id"] == "base_link", "both lidars placed by /tf_static"
    assert sink.last("/panther/robot_state")["grippers"]["right"]["position"] == 0.03
    assert hub.joint_limits()[1] == "live /robot_description"
    assert [i[1] for i in sink.images] == ["/panther/camera_depth"], "colour is not started, depth is"
    assert all(hub.stream(c)["errors"] == 0 for c in device.STREAM_CARDS), \
        {c: hub.stream(c)["last_error"] for c in device.STREAM_CARDS}


def test_every_topic_the_nodes_use_is_in_the_config(config):
    keys = {"joints", "battery", "imu", "odom", "gripper_left", "gripper_right", "lift", "diagnostics",
            "laser_front", "laser_rear", "camera_color", "camera_depth", "tf_static", "robot_description"}
    assert keys <= set(config["topics"])


# ── bridge ───────────────────────────────────────────────────────────────────

class FakeProcess:
    def __init__(self, target, args, name, daemon):
        self.target, self.args, self.name = target, args, name
        self.alive, self.started, self.pid = False, False, 4000

    def start(self):
        self.alive = self.started = True

    def is_alive(self):
        return self.alive

    def join(self, timeout=None):
        self.alive = False

    def terminate(self):
        self.alive = False


class FakeContext:
    def __init__(self):
        self.processes = []

    def Queue(self, maxsize):
        return queue.Queue(maxsize)

    def Process(self, target, args, name, daemon):
        process = FakeProcess(target, args, name, daemon)
        self.processes.append(process)
        return process


@pytest.fixture
def core(bridge):
    context, ticks = FakeContext(), [100.0]
    instance = bridge.CoreBridge("/profile.xml", 42, context=context, clock=lambda: ticks[0])
    instance.ticks, instance.context = ticks, context
    return instance


def test_a_lane_costs_nothing_until_its_first_sample(core):
    core.register_lane("camera_color", 1, True)
    assert core.context.processes == []
    assert core.send("camera_color", ("json", "/t", "{}")) is True
    assert len(core.context.processes) == 1 and core.context.processes[0].started
    assert core.context.processes[0].args[1:4] == ("/profile.xml", 42, "camera_color")
    core.send("camera_color", ("json", "/t", "{}"))
    assert len(core.context.processes) == 1, "one child per lane"


def test_latest_only_lane_keeps_the_newest_sample(core):
    core.register_lane("cam", 1, True)
    for n in range(5):
        assert core.send("cam", ("json", "/t", str(n))) is True
    messages = core._lanes["cam"]["queue"]
    assert messages.get_nowait()[2] == "4" and messages.empty()


def test_a_full_ordered_lane_drops_and_counts_instead_of_blocking(core):
    core.register_lane("state", 2, False)
    results = [core.send("state", ("json", "/t", str(n))) for n in range(4)]
    assert results == [True, True, False, False]
    stats = core.stats()["state"]
    assert stats["enqueued"] == 2 and stats["dropped"] == 2 and stats["alive"] is True


def test_a_dead_child_is_restarted_but_not_in_a_tight_loop(core):
    core.register_lane("state", 4, False)
    core.send("state", ("json", "/t", "a"))
    core.context.processes[0].alive = False
    core.ticks[0] += 1.0
    core.send("state", ("json", "/t", "b"))
    assert len(core.context.processes) == 1, "inside the backoff window"
    core.ticks[0] += 10.0
    core.send("state", ("json", "/t", "c"))
    assert len(core.context.processes) == 2 and core.stats()["state"]["restarts"] == 1


def test_send_never_raises_into_a_ros_callback(bridge):
    class Exploding(FakeContext):
        def Queue(self, maxsize):
            raise OSError("no semaphores")

    broken = bridge.CoreBridge("/p.xml", 42, context=Exploding())
    broken.register_lane("x", 1, True)
    assert broken.send("x", ("json", "/t", "{}")) is False
    assert broken.stats()["x"]["last_error"].startswith("OSError")


def test_lane_registration_conflicts_are_rejected_and_repeats_are_idempotent(core):
    core.register_lane("a", 1, True)
    core.register_lane("a", 1, True)
    with pytest.raises(ValueError, match="different queue settings"):
        core.register_lane("a", 8, False)


def test_stop_sends_the_sentinel_and_joins_the_children(core):
    core.register_lane("a", 4, False)
    core.send("a", ("json", "/t", "{}"))
    child_queue = core._lanes["a"]["queue"]
    core.stop()
    items = []
    while not child_queue.empty():
        items.append(child_queue.get_nowait())
    assert items[-1] is None
    assert core.context.processes[0].alive is False
    assert core.send("a", ("json", "/t", "{}")) is False, "nothing is sent after stop"


def test_put_latest_discards_the_oldest(bridge):
    messages = queue.Queue(2)
    for n in range(5):
        assert bridge.put_latest(messages, n)
    assert [messages.get_nowait(), messages.get_nowait()] == [3, 4]


def test_child_refuses_to_publish_without_the_isolation_profile(bridge, capsys, monkeypatch):
    """FastDDS ignores a missing profile and uses every interface -- delivery breaks and domain 42 leaks."""
    from common import logsafe
    # install() swaps sys.stdout for the whole process; the real child wants that, pytest does not.
    monkeypatch.setattr(logsafe, "install", lambda *args, **kwargs: None)
    imported_before = "rclpy" in sys.modules
    bridge._run_publisher(queue.Queue(1), "/definitely/not/here.xml", 42, "state")
    assert "refusing to publish" in capsys.readouterr().out
    assert ("rclpy" in sys.modules) == imported_before, "no ROS was initialised"


def test_log_text_from_outside_is_escaped_and_capped(bridge):
    assert bridge.safe_text("a\x1b[31mred\nline") == "a\\x1b[31mred\\nline"
    assert len(bridge.safe_text("x" * 500)) == 120


# ── contracts: the rules that decide whether a command may be sent ───────────

@pytest.fixture(scope="module")
def contracts(device):
    return importlib.import_module("contracts")


@pytest.fixture(scope="module")
def motion_module(device):
    return importlib.import_module("panther_motion")


ARM_LIMITS = {name: (-1.0, 1.0) for name in [f"joint_la{i}" for i in range(8)]}


def test_a_joint_target_is_checked_for_finiteness_and_range_only(contracts):
    """There is no per-command step limit: inside the range a joint goes straight to the target."""
    current = {name: 0.0 for name in contracts.LEFT_ARM_JOINTS}
    names = contracts.LEFT_ARM_JOINTS
    assert contracts.validate_joint_target(names, [0.1] * 8, ARM_LIMITS, current) == [0.1] * 8
    assert contracts.validate_joint_target(names, [0.9] * 8, ARM_LIMITS, current) == [0.9] * 8, \
        "a 0.9 rad move from 0 is one command, not two"
    with pytest.raises(ValueError, match="exactly 8"):
        contracts.validate_joint_target(names, [0.1] * 7, ARM_LIMITS, current)
    with pytest.raises(ValueError, match="must be finite"):
        contracts.validate_joint_target(names, [NAN] + [0.0] * 7, ARM_LIMITS, current)
    with pytest.raises(ValueError, match="must be a number"):
        contracts.validate_joint_target(names, [True] + [0.0] * 7, ARM_LIMITS, current)
    with pytest.raises(ValueError, match=r"outside its range -1.0 ~ 1.0 rad \(URDF limit\)"):
        contracts.validate_joint_target(names, [1.5] + [0.0] * 7, ARM_LIMITS, current)
    with pytest.raises(ValueError, match="no joint limits known for: joint_la7"):
        contracts.validate_joint_target(names, [0.0] * 8, {k: v for k, v in ARM_LIMITS.items() if k != "joint_la7"}, current)
    with pytest.raises(ValueError, match="fresh joint state is missing"):
        contracts.validate_joint_target(names, [0.0] * 8, ARM_LIMITS, {})
    with pytest.raises(ValueError, match="must be finite"):
        contracts.validate_joint_target(names, [0.0] * 8, ARM_LIMITS, {**current, "joint_la0": NAN})


def test_switch_requests_are_filled_for_both_generations_of_the_interface(contracts):
    new = NS(activate_controllers=[], deactivate_controllers=[], strictness=0, activate_asap=False, timeout=NS(sec=0))
    contracts.populate_switch_request(new, ["a"], ["b"])
    assert (new.activate_controllers, new.deactivate_controllers, new.strictness, new.activate_asap) == (["a"], ["b"], 2, True)
    old = NS(start_controllers=[], stop_controllers=[], strictness=0, start_asap=False, timeout=NS(sec=0))
    contracts.populate_switch_request(old, ["a"], ["b"])
    assert (old.start_controllers, old.stop_controllers, old.start_asap) == (["a"], ["b"], True)
    with pytest.raises(RuntimeError, match="unsupported"):
        contracts.populate_switch_request(NS(timeout=NS(sec=0)), [], [])


def test_the_bundled_urdf_gives_limits_for_every_joint_the_motion_cards_move(contracts):
    limits = contracts.parse_joint_limits((DRIVER / "resource" / "panther.urdf").read_text(encoding="utf-8"))
    for name in (*contracts.LEFT_ARM_JOINTS, *contracts.RIGHT_ARM_JOINTS, *contracts.HEAD_JOINTS):
        assert name in limits and limits[name][0] < limits[name][1], name
    assert contracts.parse_joint_limits("") == {}


# ── Motion: gate, stop, completion ───────────────────────────────────────────

def open_gate(hub, positions=None, lift=True):
    hub.on_joint_state(joint_state(URDF_JOINTS, positions or [0.0] * len(URDF_JOINTS)))
    hub.on_diagnostics(normal_diagnostics())
    if lift:
        hub.on_lift(joint_state(["body_joint"], [0.61]))


DRIVE = {"linear_x": 0.1, "duration_s": 0.5}
# 0.1 rad from the all-zero pose, and inside every bundled-URDF limit (la4 and ra4 top out at 0).
LEFT_TARGET = [0.1, -0.1, 0.1, 0.1, -0.1, 0.1, 0.1, 0.1]
RIGHT_TARGET = [-0.1, -0.1, -0.1, 0.1, -0.1, 0.1, 0.1, 0.1]
LEFT_BOXES = {f"la{i}": value for i, value in enumerate(LEFT_TARGET)}
RIGHT_BOXES = {f"ra{i}": value for i, value in enumerate(RIGHT_TARGET)}
ARM = {**LEFT_BOXES}
RIGHT_ARM = {**RIGHT_BOXES}
BOTH_ARMS = {**LEFT_BOXES, **RIGHT_BOXES}


def test_start_is_ready_while_the_robot_is_not_ready_and_says_why(motion, hub, io):
    for card in ("base_drive", "arm_control", "gripper", "lifting", "head"):     # no joints, no estop yet
        started = motion.dispatch(card, "start", {})
        assert started["state"] == "ready", card
        assert started["ready_for_motion"] is False and started["accepts_motion"] is False
        assert len(started["not_ready_reasons"]) == 2
    assert io.calls == []


def test_start_stays_ready_when_the_controller_manager_is_down(motion, io):
    io.list_controllers = lambda: (_ for _ in ()).throw(RuntimeError("down"))
    assert motion.dispatch("arm_control", "start", {})["state"] == "ready"


def test_stop_makes_the_card_idle_and_start_or_a_command_makes_it_ready_again(motion, hub, io):
    open_gate(hub)
    for card in ("base_drive", "arm_control", "gripper", "lifting", "head"):
        assert motion.dispatch(card, "start", {})["state"] == "ready"
        assert motion.dispatch(card, "stop", {})["state"] == "idle"
        assert motion.dispatch(card, "info", {})["state"] == "idle", card
        assert motion.dispatch(card, "start", {})["state"] == "ready", card
    motion.dispatch("head", "stop", {})
    motion.dispatch("head", "look", {"head0_rad": 0.1, "head1_rad": 0.1})
    assert motion.dispatch("head", "info", {})["state"] == "ready", "using a card brings it back"


def test_motion_is_refused_without_fresh_joints_or_with_the_estop_not_normal(motion, hub, io):
    result = motion.dispatch("base_drive", "move", DRIVE)
    assert result["state"] == "error" and "motion refused" in result["message"]
    open_gate(hub)
    hub.on_diagnostics(diagnostics(status("diagnostic_broadcaster: Emergency stop", level=b"\x02",
                                          message="x", stop_button_status="y")))
    result = motion.dispatch("base_drive", "move", DRIVE)
    assert result["state"] == "error" and "level is 2" in result["message"]
    assert io.calls == []


def test_a_drive_runs_for_its_duration_then_zeroes_and_reports_completion(motion, hub, io, completions):
    open_gate(hub)
    result = motion.dispatch("base_drive", "move", DRIVE)
    assert result["state"] == "moving" and result["action_id"].startswith("panther_drive_")
    sent = io.named("twist")
    moving = [c for c in sent if c != (0.0, 0.0, 0.0)]
    assert 9 <= len(moving) <= 11 and set(moving) == {(0.1, 0.0, 0.0)}
    assert sent[-5:] == [(0.0, 0.0, 0.0)] * 5, "ends with repeated zero velocity"
    (action_id, status, detail, tool), = completions
    assert (action_id, status, tool) == (result["action_id"], "completed", "base_drive")


def test_the_base_picks_the_configured_topic_that_the_robot_subscribes_to(motion, hub, io, config):
    open_gate(hub)
    io.subscribers = {"/cmd_vel": 0, "/cmd_vel_teleop": 2}
    result = motion.dispatch("base_drive", "move", DRIVE)
    assert result["topic"] == "/cmd_vel_teleop" and result["subscribers"]["/cmd_vel_teleop"] == 2
    assert io.twist_topic == "/cmd_vel_teleop"
    io.calls.clear()
    io.subscribers = {"/cmd_vel": 1, "/cmd_vel_teleop": 2}
    assert motion.dispatch("base_drive", "move", DRIVE)["topic"] == "/cmd_vel", \
        "the vendor-documented controller topic wins when both have subscribers"


def test_the_base_refuses_rather_than_publishing_where_nobody_listens(motion, hub, io, config):
    open_gate(hub)
    io.subscribers = {}
    result = motion.dispatch("base_drive", "move", DRIVE)
    assert result["state"] == "error" and "no robot node subscribes" in result["message"]
    assert "/cmd_vel" in result["message"] and "/cmd_vel_teleop" in result["message"]
    assert io.named("twist") == [], "nothing was sent"


def test_base_info_shows_the_subscriber_count_of_each_candidate(motion, hub, io):
    open_gate(hub)
    io.subscribers = {"/cmd_vel": 1, "/cmd_vel_teleop": 0}
    info = motion.dispatch("base_drive", "info", {})
    assert info["cmd_topics"] == {"/cmd_vel": 1, "/cmd_vel_teleop": 0}


def test_a_drive_that_breaks_the_caps_or_needs_a_dead_controller_never_publishes(motion, hub, io):
    open_gate(hub)
    assert "linear speed" in motion.dispatch("base_drive", "move", {**DRIVE, "linear_x": 0.5})["message"]
    io.controllers["movebase3_drive_controller"] = "inactive"
    assert "movebase3_drive_controller" in motion.dispatch("base_drive", "move", DRIVE)["message"]
    assert io.named("twist") == []


def test_stopping_a_drive_zeroes_at_once_and_the_worker_reports_cancelled(motion, hub, io, completions, clock):
    open_gate(hub)
    original_sleep = motion._sleep
    state = {"stopped": False}

    def hooked_sleep(seconds):
        if not state["stopped"] and len(io.named("twist")) == 3:
            state["stopped"] = True
            assert motion.stop("base_drive")["state"] == "idle"     # as if another request arrived
        original_sleep(seconds)

    motion._sleep = hooked_sleep
    motion.dispatch("base_drive", "move", {**DRIVE, "duration_s": 3.0})
    sent = io.named("twist")
    assert sum(1 for c in sent if c[0]) == 3, "no further motion command after stop"
    assert sent[3:8] == [(0.0, 0.0, 0.0)] * 5
    assert completions[-1][1] == "cancelled"


def test_a_failing_publish_zeroes_and_reports_error(motion, hub, io, completions):
    open_gate(hub)
    io.fail_on = {"twist"}
    motion.dispatch("base_drive", "move", DRIVE)
    assert completions[-1][1] == "error" and "twist failed" in completions[-1][2]["message"]


def test_a_new_drive_supersedes_the_old_one(motion, hub, io, completions):
    open_gate(hub)
    original_sleep = motion._sleep
    state = {"nested": False}

    def hooked_sleep(seconds):
        if not state["nested"] and len(io.named("twist")) == 2:
            state["nested"] = True
            motion.dispatch("base_drive", "move", {**DRIVE, "linear_x": -0.1, "duration_s": 0.2})
        original_sleep(seconds)

    motion._sleep = hooked_sleep
    motion.dispatch("base_drive", "move", {**DRIVE, "duration_s": 2.0})
    assert [c[1] for c in completions].count("cancelled") == 1 and [c[1] for c in completions].count("completed") == 1


LEFT_P2P = "left_arm_joint_trajectory_position_controller"
RIGHT_P2P = "right_arm_joint_trajectory_position_controller"
FORWARD = "dual_arm_forward_position_controller"


def _switches(io):
    return [c for c in io.calls if c[0] == "switch"]


def _touched(io):
    """Every controller any switch activated or deactivated."""
    return {name for _, activate, deactivate in _switches(io) for name in (*activate, *deactivate)}


def test_an_arm_move_switches_controllers_sends_the_goal_and_restores_them(motion, hub, io, completions):
    open_gate(hub)
    before = dict(io.controllers)
    result = motion.dispatch("arm_control", "move", ARM)
    assert result["state"] == "moving" and result["action_id"].startswith("panther_arm_")
    assert result["controllers_activated"] == [LEFT_P2P] and result["controllers_deactivated"] == [FORWARD]
    assert [c[0] for c in io.calls] == ["switch", "goal", "switch"]
    assert io.calls[0] == ("switch", (LEFT_P2P,), (FORWARD,)), "only the arm being moved, not both"
    assert io.calls[1] == ("goal", "left", tuple(LEFT_TARGET), 2.0), "the shortest allowed duration"
    assert io.calls[2] == ("switch", (FORWARD,), (LEFT_P2P,)), "the exact inverse"
    assert io.controllers == before, "every controller is in the state it was in before the move"
    (action_id, status, detail, tool), = completions
    assert (status, tool, detail["controllers_restored"]) == ("completed", "arm_control", True)
    assert detail["left"]["error_code"] == 0


def test_a_one_arm_move_never_touches_the_other_arms_trajectory_controller(motion, hub, io, completions):
    """Review finding: it used to activate both trajectory controllers and deactivate both afterwards."""
    open_gate(hub)
    motion.dispatch("arm_control", "move", ARM)
    assert RIGHT_P2P not in _touched(io)
    io.calls.clear()
    motion.dispatch("arm_control", "move", RIGHT_ARM)
    assert LEFT_P2P not in _touched(io) and RIGHT_P2P in _touched(io)


def test_a_trajectory_controller_that_was_already_active_for_the_other_arm_stays_active(
        motion, hub, io, completions):
    open_gate(hub)
    io.controllers[RIGHT_P2P] = "active"                 # someone else is using the right arm
    io.controllers[FORWARD] = "inactive"
    before = dict(io.controllers)
    motion.dispatch("arm_control", "move", ARM)          # a move of the *left* arm
    assert _switches(io) == [("switch", (LEFT_P2P,), ()), ("switch", (), (LEFT_P2P,))]
    assert RIGHT_P2P not in _touched(io)
    assert io.controllers == before and io.controllers[RIGHT_P2P] == "active"
    assert completions[-1][1] == "completed"


def test_a_requested_controller_that_was_already_active_is_not_switched_off_afterwards(
        motion, hub, io, completions):
    open_gate(hub)
    io.controllers[LEFT_P2P] = "active"
    io.controllers[FORWARD] = "inactive"
    before = dict(io.controllers)
    result = motion.dispatch("arm_control", "move", ARM)
    assert result["controllers_activated"] == [] and result["controllers_deactivated"] == []
    assert _switches(io) == [], "nothing to change, so no switch request at all"
    assert [c[0] for c in io.calls] == ["goal"] and io.controllers == before


def test_both_arms_activate_both_trajectory_controllers_and_restore_the_forward_one(motion, hub, io, completions):
    open_gate(hub)
    before = dict(io.controllers)
    motion.dispatch("arm_control", "move", BOTH_ARMS)
    assert _switches(io) == [("switch", (LEFT_P2P, RIGHT_P2P), (FORWARD,)),
                             ("switch", (FORWARD,), (LEFT_P2P, RIGHT_P2P))]
    assert io.controllers == before


def test_the_controller_state_is_restored_after_a_failure_and_after_stop_too(motion, hub, io, completions):
    open_gate(hub)
    io.controllers[RIGHT_P2P] = "active"
    io.controllers[FORWARD] = "inactive"
    before = dict(io.controllers)
    io.arm_results = [{"status": 6, "error_code": -4, "error_string": "path tolerance violated"}]
    motion.dispatch("arm_control", "move", ARM)
    assert completions[-1][1] == "error" and io.controllers == before
    assert RIGHT_P2P not in _touched(io)


def test_an_arm_move_leaves_the_forward_controller_off_if_it_was_off(motion, hub, io, completions):
    open_gate(hub)
    io.controllers[FORWARD] = "inactive"
    motion.dispatch("arm_control", "move", ARM)
    assert io.calls[0][2] == () and io.calls[2][1] == (), "no controller is activated that was not active before"
    assert io.controllers[FORWARD] == "inactive"


def test_both_arms_take_their_own_targets(motion, hub, io, completions):
    open_gate(hub)
    motion.dispatch("arm_control", "move", BOTH_ARMS)
    assert [c[1:3] for c in io.calls if c[0] == "goal"] == [("left", tuple(LEFT_TARGET)),
                                                            ("right", tuple(RIGHT_TARGET))]


def test_a_failed_trajectory_cancels_the_goal_restores_the_controllers_and_says_so(motion, hub, io, completions):
    open_gate(hub)
    io.arm_results = [{"status": 6, "error_code": -4, "error_string": "path tolerance violated"}]
    motion.dispatch("arm_control", "move", ARM)
    status, detail = completions[-1][1], completions[-1][2]
    assert status == "error" and "path tolerance violated" in detail["message"]
    assert io.named("cancel") == [("handle-left",)]
    assert io.calls[-1][0] == "switch" and detail["controllers_restored"] is True


def test_failing_to_restore_the_controllers_is_an_error_and_is_logged_loudly(motion, hub, io, completions, capsys):
    open_gate(hub)
    io.switch_fail_after = 1
    motion.dispatch("arm_control", "move", ARM)
    status, detail = completions[-1][1], completions[-1][2]
    assert status == "error" and detail["controllers_restored"] is False and "rejected" in detail["restore_error"]
    assert "could not restore the arm controllers" in capsys.readouterr().out


def test_a_trajectory_that_never_finishes_times_out_and_restores(motion, hub, io, completions, clock):
    open_gate(hub)
    io.poll_arm_result = lambda handle, timeout: clock.advance(2.0)             # never done; time passes
    motion.dispatch("arm_control", "move", ARM)
    assert completions[-1][1] == "error" and "did not finish" in completions[-1][2]["message"]
    assert io.calls[-1][0] == "switch"


def _running_motion(device, config, hub, completions, clock, motion_module, io):
    """Real worker thread, and an arm goal that never finishes on its own."""
    import time

    class Hanging(FakeIO):
        def poll_arm_result(self, handle, timeout):
            time.sleep(0.005)
            return None

    io.__class__ = Hanging
    return motion_module.Motion(config, io, hub, notify=lambda *a: completions.append(a),
                                monotonic=lambda: clock.mono)


def _wait_for(condition, seconds=3.0):
    import time

    end = time.time() + seconds
    while time.time() < end:
        if condition():
            return True
        time.sleep(0.005)
    return False


def test_stopping_the_arm_cancels_the_goal_restores_the_controllers_and_reports_cancelled(
        device, config, hub, io, completions, clock, motion_module):
    motion = _running_motion(device, config, hub, completions, clock, motion_module, io)
    open_gate(hub)
    assert motion.dispatch("arm_control", "move", ARM)["state"] == "moving"
    assert _wait_for(lambda: io.named("goal"))
    assert motion.dispatch("arm_control", "info", {})["busy"] is True
    busy = motion.dispatch("arm_control", "move", ARM)
    assert busy["state"] == "error" and "already moving" in busy["message"]

    stopped = motion.stop("arm_control")
    assert stopped["state"] == "idle"
    assert completions[-1][1] == "cancelled"
    assert io.named("cancel") == [("handle-left",)]
    assert io.calls[-1][0] == "switch" and io.calls[-1][1] == ("dual_arm_forward_position_controller",)
    assert motion.dispatch("arm_control", "info", {})["busy"] is False
    assert motion.stop("arm_control") == {"state": "idle"}, "stop with nothing running is a no-op"


def test_a_lift_command_needs_a_fresh_reading(motion, hub, io):
    open_gate(hub, lift=False)
    result = motion.dispatch("lifting", "set_position", {"position": 0.6})
    assert result["state"] == "error" and "fresh lift reading" in result["message"]
    assert io.calls == []


def test_info_never_raises_even_when_the_controller_manager_is_unreachable(motion, hub, io):
    open_gate(hub)
    io.list_controllers = lambda: (_ for _ in ()).throw(RuntimeError("no list_controllers"))
    for card in ("base_drive", "arm_control", "gripper", "lifting", "head"):
        info = motion.dispatch(card, "start", {})
        assert info["state"] == "ready" and "no list_controllers" in info["controllers_error"], card


def test_info_reports_which_controllers_are_missing(motion, hub, io):
    open_gate(hub)
    del io.controllers["lifting_controller"]
    assert motion.dispatch("lifting", "info", {})["controllers"] == {"lifting_controller": "missing"}


def test_unsupported_actions_are_errors(motion):
    for card in ("base_drive", "arm_control", "gripper", "lifting", "head"):
        result = motion.dispatch(card, "fly", {})
        assert result["state"] == "error" and "unsupported" in result["message"], card


# ── motion cards as tools ────────────────────────────────────────────────────

def test_a_disabled_motion_card_is_not_listed(device, hub, motion, config):
    config["plugins"]["head"]["enabled"] = False
    plugin = device.PantherPlugin(hub, motion=motion, config=config,
                                  enabled={n: s["enabled"] for n, s in config["plugins"].items()})
    assert "head" not in [t["name"] for t in plugin.get_tools()]


def test_the_plugin_routes_motion_calls_and_turns_failures_into_error_results(device, hub, motion, config, io):
    open_gate(hub)
    plugin = device.PantherPlugin(hub, motion=motion, config=config)
    ok = plugin.dispatch("look", {"_tool_name": "head", "head0_rad": 0.1, "head1_rad": 0.1})
    assert ok["state"] == "commanded"
    refused = plugin.dispatch("look", {"_tool_name": "head", "head0_rad": 9.0})
    assert refused["state"] == "error" and "outside its range" in refused["message"]
    assert plugin.dispatch("stop", {"_tool_name": "gripper"})["state"] == "idle"
    motion.stop = lambda card: (_ for _ in ()).throw(RuntimeError("bus down"))
    failed = plugin.dispatch("stop", {"_tool_name": "base_drive"})
    assert failed["state"] == "error" and "bus down" in failed["message"], "a failed stop is never reported as idle"


def test_without_motion_the_actuator_cards_say_so(device, hub):
    result = device.PantherPlugin(hub).dispatch("move", {"_tool_name": "base_drive"})
    assert result["state"] == "error" and "not available" in result["message"]


def test_plugin_stop_stops_motion_first_then_releases_everything(device, hub, sink):
    order = []
    plugin = device.PantherPlugin(
        hub, nodes=NS(close=lambda: order.append("nodes")), sink=NS(close=lambda: order.append("bridge")),
        motion=NS(stop_all=lambda: order.append("motion")), camera=NS(close=lambda: order.append("camera")))
    plugin.stop()
    assert order == ["motion", "camera", "nodes", "bridge"]


# ── completion callback to Agent Core ────────────────────────────────────────

class _Response:
    status = 200

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def test_completion_is_posted_to_agent_core_in_the_documented_shape(motion_module, monkeypatch):
    import ssl
    import urllib.request

    sent = []
    monkeypatch.setattr(urllib.request, "urlopen", lambda request, timeout, context: sent.append((request, context)) or _Response())
    monkeypatch.delenv("AGENT_CORE_URL", raising=False)
    monkeypatch.delenv("AGENT_CORE_CA_CERT", raising=False)
    motion_module.post_acp_callback("panther_arm_1", "completed", {"left": {"error_code": 0}}, "arm_control")
    (request, context), = sent
    body = json.loads(request.data)
    assert request.full_url == "https://localhost:15678/api/acp/complete" and request.get_method() == "POST"
    assert body["action_id"] == "panther_arm_1" and body["status"] == "completed" and body["tool"] == "arm_control"
    assert body["result"] == {"left": {"error_code": 0}} and isinstance(body["ts"], float)
    assert context.check_hostname is False, "Agent Core's certificate has no subjectAltName"
    assert context.verify_mode is ssl.CERT_NONE, "no CA to verify against: the guidance's CERT_NONE"


def test_a_supplied_ca_still_verifies_the_chain_without_hostname_checking(motion_module, monkeypatch):
    """Agent Core's certificate has no subjectAltName, so a supplied CA used to still fail with
    CERTIFICATE_VERIFY_FAILED. The chain must stay verified; only the hostname check goes."""
    import ssl
    import urllib.request

    contexts, sent = [], []
    monkeypatch.setenv("AGENT_CORE_CA_CERT", "/etc/phanthy/agent-core-ca.pem")
    monkeypatch.setattr(motion_module, "ssl", NS(
        create_default_context=lambda cafile=None: contexts.append(
            NS(cafile=cafile, check_hostname=True, verify_mode=ssl.CERT_REQUIRED)) or contexts[-1],
        CERT_NONE=ssl.CERT_NONE))
    monkeypatch.setattr(urllib.request, "urlopen", lambda request, timeout, context: sent.append(context) or _Response())
    motion_module.post_acp_callback("panther_head_2", "completed", {"head0_rad": 0.1}, "head")
    context, = contexts
    assert context.cafile == "/etc/phanthy/agent-core-ca.pem"
    assert context.check_hostname is False
    assert context.verify_mode is ssl.CERT_REQUIRED, "the operator's CA is there to verify the chain"
    assert sent == [context]


def test_completion_is_retried_three_times_then_reported_not_swallowed(motion_module, monkeypatch, capsys):
    import urllib.request

    attempts = []
    monkeypatch.setattr(urllib.request, "urlopen", lambda *a, **k: attempts.append(1) or (_ for _ in ()).throw(OSError("refused")))
    monkeypatch.setattr(motion_module.time, "sleep", lambda s: None)
    with pytest.raises(RuntimeError, match="after 3 attempts"):
        motion_module.post_acp_callback("a", "error", {}, "base_drive")
    assert len(attempts) == 3

    class Inline:
        def __init__(self, target, daemon, name):
            self.target = target

        def start(self):
            self.target()

    monkeypatch.setattr(motion_module, "threading", NS(Thread=Inline))
    motion_module.notify_async("panther_drive_9", "completed", {}, "base_drive")
    assert "callback failed action_id=panther_drive_9" in capsys.readouterr().out


# ── host camera: lazy start of the Orbbec driver ─────────────────────────────

@pytest.fixture(scope="module")
def host_camera(device):
    return importlib.import_module("host_camera")


class FakeRun:
    def __init__(self, host_camera, active="inactive", start_rc=1, raises=None):
        self.prefix = list(host_camera.HostCamera.NSENTER)
        self.active, self.start_rc, self.raises, self.calls = active, start_rc, raises, []

    def __call__(self, command, check, capture_output, text, timeout):
        self.calls.append(list(command))
        if self.raises:
            raise self.raises
        tail = list(command)[len(self.prefix):]
        if tail[:2] == ["systemctl", "is-active"]:
            return NS(stdout=self.active + "\n", returncode=0 if self.active == "active" else 3)
        if tail[:2] == ["systemctl", "start"]:
            return NS(stdout="", returncode=self.start_rc)
        return NS(stdout="", returncode=0)

    def tails(self):
        return [c[len(self.prefix):] for c in self.calls]


TOPICS = {"color": "/camera/color/image_raw", "depth": "/camera/depth/image_raw"}


def _camera(host_camera, config, clock, run, publishers=None, age=lambda kind: None):
    counts = publishers if publishers is not None else {}
    return host_camera.HostCamera(config["camera"]["host"], TOPICS, lambda topic: counts.get(topic, 0), age,
                                  run=run, sleep=clock.advance, monotonic=lambda: clock.mono, watchdog=False)


def test_an_existing_camera_publisher_is_never_competed_with(host_camera, config, clock):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run, {TOPICS["color"]: 1})
    info = camera.start("color")
    assert run.calls == [], "another process owns the device: no host command at all"
    assert info["host_service_managed"] is False and info["publisher_count"] == 1


def test_starting_a_camera_creates_a_transient_host_unit_through_nsenter(host_camera, config, clock):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    info = camera.start("color")
    assert all(call[:len(run.prefix)] == run.prefix for call in run.calls), "every command enters PID 1's namespaces"
    assert run.prefix[:3] == ["nsenter", "-t", "1"]
    tails = run.tails()
    assert tails[0] == ["systemctl", "is-active", "panther_orbbec.service"]
    assert tails[1] == ["systemctl", "start", "panther_orbbec.service"]
    create = tails[2]
    assert create[0] == "systemd-run" and "--unit=panther_orbbec.service" in create and "--uid=unix_ai" in create
    assert "--property=Restart=on-failure" in create
    script = create[-1]
    assert "gemini2.launch.py" in script and "ROS_DOMAIN_ID=0" in script and "rmw_cyclonedds_cpp" in script
    assert "/home/unix_ai/orbbec_ws" in script and "file:///home/unix_ai/config/cyclonedds.xml" in script
    assert info["host_service_managed"] is True


def test_an_already_running_host_unit_is_reused_but_not_adopted(host_camera, config, clock):
    """A unit that is already up may be managed by something else (the name is fixed), and the
    driver cannot tell it from its own leftover, so it must not stop or restart it."""
    run = FakeRun(host_camera, active="active")
    camera = _camera(host_camera, config, clock, run)
    info = camera.start("depth")
    assert info["host_service_managed"] is False and "start_error" not in info
    assert [t for t in run.tails()] == [["systemctl", "is-active", "panther_orbbec.service"]]
    camera.close()
    assert len(run.calls) == 1, "container exit must not stop a unit this process did not create"


def test_a_failed_unit_is_restarted_not_duplicated(host_camera, config, clock):
    run = FakeRun(host_camera, start_rc=0)
    _camera(host_camera, config, clock, run).start("color")
    assert [t[0] for t in run.tails()] == ["systemctl", "systemctl"], "no systemd-run for a unit that restarts"


def test_a_camera_that_cannot_be_started_is_reported_not_raised(host_camera, config, clock, capsys):
    run = FakeRun(host_camera, raises=FileNotFoundError("nsenter"))
    info = _camera(host_camera, config, clock, run).start("color")
    assert "FileNotFoundError" in info["start_error"], "agent-core rolls the project back if a start raises"
    assert "camera source could not be started" in capsys.readouterr().out


def test_stopping_a_card_never_stops_the_shared_host_service(host_camera, config, clock):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("color")
    camera.start("depth")
    before = len(run.calls)
    camera.stop("color")
    camera.stop("depth")
    assert len(run.calls) == before and not any("stop" in t[:2] for t in run.tails()), \
        "other programs on the robot may be using the camera; only the container stopping ends it"


def test_stopping_the_container_stops_the_unit_this_driver_manages_once_and_without_blocking(
        host_camera, config, clock, capsys):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("color")
    camera.start("depth")
    before = len(run.calls)
    camera.close()
    camera.close()                                           # shutdown reaches it from two places
    stops = [t for t in run.tails()[before:]]
    assert stops == [["systemctl", "stop", "--no-block", "panther_orbbec.service"]], \
        "one request, not waiting for the camera to exit (the grace period may be needed for an arm)"
    assert all(call[:len(run.prefix)] == run.prefix for call in run.calls[before:])
    assert "asked systemd to stop panther_orbbec.service" in capsys.readouterr().out


def test_the_unit_is_created_with_a_bounded_stop_timeout(host_camera, config, clock):
    run = FakeRun(host_camera)
    _camera(host_camera, config, clock, run).start("color")
    create = next(t for t in run.tails() if t[0] == "systemd-run")
    assert "--property=TimeoutStopSec=15s" in create, "systemd's default would wait 90 s for a stuck camera"


def test_stopping_the_container_leaves_a_camera_someone_else_is_publishing_alone(host_camera, config, clock):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run, {TOPICS["color"]: 1})
    camera.start("color")
    camera.close()
    assert run.calls == [], "nothing was started here, so nothing is stopped here"


def test_stopping_a_container_that_never_started_a_camera_stops_nothing(host_camera, config, clock):
    run = FakeRun(host_camera)
    _camera(host_camera, config, clock, run).close()
    assert run.calls == []


def test_a_unit_this_driver_created_is_stopped_on_exit(host_camera, config, clock):
    """The only unit `close()` stops is one this process started (here through `systemd-run`)."""
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("depth")
    camera.close()
    assert run.tails()[-1] == ["systemctl", "stop", "--no-block", "panther_orbbec.service"]


def test_a_failed_stop_is_logged_with_the_manual_command_and_never_raised(host_camera, config, clock, capsys):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("color")
    run.raises = TimeoutError("nsenter hung")
    camera.close()                                           # must not raise: shutdown has to finish
    out = capsys.readouterr().out
    assert "could not stop panther_orbbec.service: TimeoutError" in out
    assert "sudo systemctl stop panther_orbbec.service" in out


def test_the_watchdog_cannot_restart_the_unit_after_the_container_has_begun_to_stop(
        host_camera, config, clock):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("color")
    camera.close()
    calls = len(run.calls)
    clock.advance(100)                                       # long past any stall limit
    assert camera.check() is False and len(run.calls) == calls


def test_plugin_stop_closes_the_camera_after_the_motion_and_before_the_node(device, hub):
    order = []
    plugin = device.PantherPlugin(
        hub, nodes=NS(close=lambda: order.append("nodes")), sink=NS(close=lambda: order.append("bridge")),
        motion=NS(stop_all=lambda: order.append("motion")), camera=NS(close=lambda: order.append("camera")))
    plugin.stop()
    assert order.index("motion") < order.index("camera") < order.index("nodes") < order.index("bridge")


def test_the_watchdog_restarts_a_stalled_unit_a_bounded_number_of_times(host_camera, config, clock, capsys):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("color")
    clock.advance(10)
    assert camera.check() is False, "inside the first-frame grace"
    restarts = 0
    for _ in range(5):
        clock.advance(31)
        restarts += bool(camera.check())
    assert restarts == 2 == config["camera"]["host"]["max_watchdog_restarts"]
    assert sum(1 for t in run.tails() if t[:2] == ["systemctl", "restart"]) == 2
    assert camera.info("color")["watchdog_restarts"] == 2
    assert "restarted panther_orbbec.service (2/2)" in capsys.readouterr().out


def test_the_watchdog_leaves_a_flowing_or_foreign_camera_alone(host_camera, config, clock):
    run = FakeRun(host_camera)
    flowing = _camera(host_camera, config, clock, run, age=lambda kind: 0.1)
    flowing.start("color")
    clock.advance(100)
    assert flowing.check() is False

    foreign = _camera(host_camera, config, clock, FakeRun(host_camera), {TOPICS["color"]: 1})
    foreign.start("color")
    clock.advance(100)
    assert foreign.check() is False, "the driver does not own that camera, so it never restarts it"


def test_camera_plugin_starts_and_stops_the_host_runtime_with_the_card(device, hub):
    calls = []
    camera = NS(start=lambda kind: calls.append(("start", kind)) or {"host_service_managed": True},
                stop=lambda kind: calls.append(("stop", kind)) or {}, info=lambda kind: {"kind": kind},
                close=lambda: calls.append(("close",)))
    plugin = device.PantherPlugin(hub, camera=camera)
    started = plugin.dispatch("start", {"_tool_name": "camera_color"})
    assert started["state"] == "running" and started["camera_runtime"] == {"host_service_managed": True}
    assert plugin.dispatch("info", {"_tool_name": "camera_depth"})["camera_runtime"] == {"kind": "depth"}
    plugin.dispatch("stop", {"_tool_name": "camera_depth"})
    plugin.dispatch("start", {"_tool_name": "laser_scan"})
    plugin.stop()
    assert calls == [("start", "color"), ("stop", "depth"), ("close",)], "only camera cards touch the camera runtime"


def test_a_stream_of_unsupported_frames_logs_a_transition_and_a_throttled_sample(hub, clock, capsys):
    hub.activate("camera_color")
    for _ in range(300):                                   # 33 s of frames the driver cannot convert
        clock.advance(0.11)
        hub.on_image("camera_color", image(4, 4, "yuv422", bytes(32)))
    lines = [l for l in capsys.readouterr().out.splitlines() if "camera_color" in l]
    assert hub.stream("camera_color")["errors"] == 300, "every dropped frame is counted"
    assert 2 <= len(lines) <= 3, f"one line per frame would be 300: {len(lines)}"
    assert "frames dropped so far" in lines[-1]


# ── static transforms and the merged scan ────────────────────────────────────

def test_static_tf_composes_a_chain_including_rotation(payloads):
    tf = payloads.StaticTf()
    assert tf.add_message(_tf(("base_link", "torso", (1, 0, 0), (0, 0, 0, 1)),
                              ("/torso", "/lidar", (0, 2, 0), (0, 0, 0.70710678, 0.70710678)))) == 2
    rotation, translation = tf.lookup("lidar", "base_link")
    x = rotation[0][0] * 1.0 + translation[0]
    y = rotation[1][0] * 1.0 + translation[1]
    assert (x, y) == pytest.approx((1.0, 3.0), abs=1e-6), "1 m ahead of a lidar yawed 90 degrees, 2 m left of the torso"
    identity, zero = tf.lookup("base_link", "base_link")
    assert identity[0][0] == 1.0 and zero == [0.0, 0.0, 0.0]
    assert tf.lookup("lidar", "elsewhere") is None and tf.lookup("nowhere", "base_link") is None


def test_static_tf_ignores_unusable_transforms_and_cycles(payloads):
    tf = payloads.StaticTf()
    assert tf.add_message(_tf(("a", "b", (0, 0, 0), (0, 0, 0, 0)), ("a", "c", (NAN, 0, 0), (0, 0, 0, 1)))) == 0
    tf.add_message(_tf(("a", "b", (0, 0, 0), (0, 0, 0, 1)), ("b", "a", (0, 0, 0), (0, 0, 0, 1))))
    assert tf.lookup("a", "z") is None, "a loop in the tree ends, it does not hang"


def test_merging_keeps_the_nearest_return_where_two_lidars_overlap(payloads):
    near = scan(n=360, fill=INF, angle_min=-3.14159, angle_increment=2 * 3.14159 / 360)
    far = scan(n=360, fill=INF, angle_min=-3.14159, angle_increment=2 * 3.14159 / 360)
    near.ranges[180], far.ranges[180] = 1.0, 2.0
    far.ranges[90] = 0.05                                      # below range_min: invalid
    merged = strict(payloads.merge_scans([(near, None), (far, None)], "base_link", 360))
    assert merged["ranges"][180] == pytest.approx(1.0, abs=1e-3) and merged["ranges"][90] is None
    assert merged["nearest_m"] == pytest.approx(1.0, abs=1e-3) and merged["beams"] == 720 and merged["bins"] == 360
    empty = strict(payloads.merge_scans([], "base_link", 360))
    assert set(empty["ranges"]) == {None} and empty["nearest_m"] is None and empty["valid_ratio"] == 0.0


# ── the lidar sees the robot's own body ──────────────────────────────────────

def _own_body_scan(frame="link_front_radar"):
    """A lidar whose beams 0..9 hit something 7 cm away (its own chassis), and one wall at 2 m."""
    # range_min is below 7 cm on purpose: a return under range_min is already discarded as invalid,
    # before any filtering, so it would prove nothing about the self-filter.
    msg = scan(n=360, fill=INF, header=header(frame_id=frame), angle_min=-3.14159,
               angle_increment=2 * 3.14159 / 360, range_min=0.05)
    for index in range(10):
        msg.ranges[index] = 0.07
    msg.ranges[180] = 2.0
    return msg


FRONT_AT_NOSE = ((1.0, 0.0, 0.0), (0, 1.0, 0.0), (0.0, 0.0, 1.0)), [0.3, 0.0, 0.1]


def test_points_within_the_radius_of_base_link_are_dropped_and_counted(payloads):
    msg = _own_body_scan()
    # The lidar sits 0.3 m ahead of base_link, so its own chassis at 7 cm is 0.23-0.37 m from the
    # origin; the part of it inside the radius goes, and the wall stays.
    kept = strict(payloads.merge_scans([(msg, FRONT_AT_NOSE)], "base_link", 360, 0.35))
    unfiltered = strict(payloads.merge_scans([(msg, FRONT_AT_NOSE)], "base_link", 360, 0.0))
    assert kept["self_filter_radius_m"] == 0.35 and unfiltered["self_filter_radius_m"] == 0.0
    assert kept["self_filtered_beams"] > 0 and unfiltered["self_filtered_beams"] == 0
    assert kept["nearest_m"] > unfiltered["nearest_m"], "the chassis no longer is the nearest obstacle"
    assert kept["nearest_m"] == pytest.approx(2.3, abs=0.02), "the wall 2 m ahead of a lidar that is 0.3 m ahead of base_link"
    assert all(v >= 0.35 for v in kept["ranges"] if v is not None), "nothing inside the radius survives"


def test_a_point_just_outside_the_radius_is_kept(payloads):
    msg = scan(n=360, fill=INF, angle_min=-3.14159, angle_increment=2 * 3.14159 / 360)
    msg.ranges[180], msg.ranges[90] = 0.36, 0.34              # at the origin, 0.36 m and 0.34 m away
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), [0.0, 0.0, 0.0]
    result = payloads.merge_scans([(msg, identity)], "base_link", 360, 0.35)
    assert result["nearest_m"] == pytest.approx(0.36, abs=1e-3)
    assert result["self_filtered_beams"] == 1


def test_a_radius_of_zero_filters_nothing(payloads):
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), [0.0, 0.0, 0.0]
    result = payloads.merge_scans([(_own_body_scan(), identity)], "base_link", 360, 0.0)
    assert result["self_filtered_beams"] == 0 and result["nearest_m"] == pytest.approx(0.07, abs=1e-3)


def test_a_scan_not_moved_into_base_link_is_never_filtered(payloads):
    """Without a transform the body's position is unknown, so 0.07 m might be a real obstacle."""
    result = payloads.merge_scans([(_own_body_scan(), None)], "link_front_radar", 360, 0.35)
    assert result["self_filtered_beams"] == 0 and result["nearest_m"] == pytest.approx(0.07, abs=1e-3)


def test_the_hub_filters_the_chassis_out_of_the_merged_scan_using_the_configured_radius(hub, sink):
    hub.on_tf_static(_tf(("base_link", "link_front_radar", (0.3, 0.0, 0.1), (0, 0, 0, 1))))
    hub.on_laser("front", _own_body_scan())
    merged = sink.last("/panther/laser_scan")
    assert merged["self_filter_radius_m"] == 0.35 and merged["self_filtered_beams"] > 0
    assert merged["nearest_m"] > 0.35, "a 7 cm 'obstacle' that is the robot itself is not reported"
    assert hub.card_info("laser_scan")["scan"]["self_filtered_beams"] == merged["self_filtered_beams"]


def test_the_hub_says_when_it_could_not_filter_because_there_is_no_transform(hub, sink):
    hub.on_laser("front", _own_body_scan())
    merged = sink.last("/panther/laser_scan")
    assert merged["transformed"] is False and merged["self_filtered_beams"] == 0
    assert "own body is not filtered" in merged["sources"]["front"]["note"].replace("robot's ", "")


def test_the_filter_radius_is_configurable_and_zero_turns_it_off(device, config, sink, clock):
    assert config["laser"]["self_filter_radius_m"] == 0.35
    config["laser"]["self_filter_radius_m"] = 0.0
    off = device.Hub(config, "panther", sink, clock=lambda: clock.wall, monotonic=lambda: clock.mono)
    off.on_tf_static(_tf(("base_link", "link_front_radar", (0.3, 0.0, 0.1), (0, 0, 0, 1))))
    off.on_laser("front", _own_body_scan())
    assert sink.last("/panther/laser_scan")["self_filtered_beams"] == 0

def test_a_restart_already_decided_on_cannot_follow_the_stop_and_bring_the_camera_back(
        host_camera, config, clock):
    """The race: the watchdog has read 'stalled' and is about to restart when close() runs.

    The previous guard sat where the state is read, so it could not help here; the restart
    must be re-checked after close() has begun, under the same lock close() stops the unit with.
    """
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("color")
    clock.advance(31)                                        # first-frame grace is over: stalled
    original = camera._frame_age
    camera._frame_age = lambda kind: (camera.close(), original(kind))[1]   # close() lands mid-check
    assert camera.check() is False, "the restart that was decided on is withdrawn"
    restarts = [t for t in run.tails() if t[:2] == ["systemctl", "restart"]]
    stops = [t for t in run.tails() if t[:2] == ["systemctl", "stop"]]
    assert restarts == [] and len(stops) == 1, "the unit is stopped and stays stopped"
    assert camera.info("color")["watchdog_restarts"] == 0


def test_the_restart_and_the_final_stop_never_run_at_the_same_time(host_camera, config, clock):
    import threading

    run, inside, overlap = FakeRun(host_camera), [], []
    original = run.__call__

    def slow(command, check, capture_output, text, timeout):
        tail = list(command)[len(run.prefix):]
        if tail[:2] in (["systemctl", "restart"], ["systemctl", "stop"]):
            inside.append(tail[1])
            if len(inside) > 1:
                overlap.append(list(inside))
            threading.Event().wait(0.05)
            inside.remove(tail[1])
        return original(command, check, capture_output, text, timeout)

    camera = host_camera.HostCamera(config["camera"]["host"], TOPICS, lambda t: 0, lambda kind: None,
                                    run=slow, sleep=clock.advance, monotonic=lambda: clock.mono, watchdog=False)
    camera.start("color")
    clock.advance(31)
    watchdog = threading.Thread(target=camera.check)
    watchdog.start()
    camera.close()
    watchdog.join(5)
    assert overlap == [], "two systemctl calls on the unit overlapped"

# ── arm stop racing a worker that has not started yet ───────────────────────────────────────

class DeferredSpawn:
    """Holds the worker back, as a busy machine can: the move has been accepted, the thread has not run."""

    def __init__(self):
        self.pending = []

    def __call__(self, target, name):
        self.pending.append(target)

    def run_all(self):
        pending, self.pending = self.pending, []
        for target in pending:
            target()


@pytest.fixture
def deferred_motion(device, config, hub, io, completions, clock, motion_module):
    spawn = DeferredSpawn()
    instance = motion_module.Motion(config, io, hub, spawn=spawn, notify=lambda *a: completions.append(a),
                                    sleep=clock.advance, monotonic=lambda: clock.mono)
    instance.spawn = spawn
    return instance


def test_a_stop_before_the_worker_begins_means_no_controller_switch_and_no_goal(
        deferred_motion, hub, io, completions):
    """Review finding: the worker used to switch controllers and send every goal before it first
    looked at the cancel flag, so a stop issued right after a move could be followed by motion."""
    open_gate(hub)
    started = deferred_motion.dispatch("arm_control", "move", ARM)
    assert started["state"] == "moving" and io.calls == [], "accepted, but nothing has happened yet"

    stopped = deferred_motion.dispatch("arm_control", "stop", {})
    assert stopped["state"] == "idle"
    deferred_motion.spawn.run_all()                          # the thread finally gets to run

    assert io.named("switch") == [] and io.named("goal") == [], "stopped before it began: nothing was sent"
    assert completions[-1][1] == "cancelled" and completions[-1][3] == "arm_control"
    assert deferred_motion.dispatch("arm_control", "info", {})["busy"] is False


def test_a_stop_between_two_arms_goals_sends_no_further_goal_and_restores(deferred_motion, hub, io, completions):
    open_gate(hub)
    original = io.send_arm_goal

    def stop_after_the_first_goal(side, names, positions, duration):
        handle = original(side, names, positions, duration)
        if side == "left":
            deferred_motion._arm_cancel.set()                # a stop lands between the two sends
        return handle

    io.send_arm_goal = stop_after_the_first_goal
    deferred_motion.dispatch("arm_control", "move", BOTH_ARMS)
    deferred_motion.spawn.run_all()
    assert [c[1] for c in io.calls if c[0] == "goal"] == ["left"], "the right arm's goal was never sent"
    assert io.named("cancel") == [("handle-left",)], "the goal that was sent is cancelled"
    assert io.controllers[FORWARD] == "active" and completions[-1][1] == "cancelled", "controllers restored"


def test_a_stop_during_the_controller_switch_cancels_before_any_goal(deferred_motion, hub, io, completions):
    open_gate(hub)
    original = io.switch_controllers

    def stop_during_switch(activate, deactivate):
        original(activate, deactivate)
        deferred_motion._arm_cancel.set()

    io.switch_controllers = stop_during_switch
    deferred_motion.dispatch("arm_control", "move", ARM)
    deferred_motion.spawn.run_all()
    assert io.named("goal") == [], "no goal after the stop"
    assert io.controllers[FORWARD] == "active" and io.controllers[LEFT_P2P] == "inactive", "switched back"
    assert completions[-1][1] == "cancelled"


def test_a_stop_with_nothing_running_leaves_no_trace_that_could_cancel_the_next_move(
        deferred_motion, hub, io, completions):
    """The flag must not be set by a stop that found nothing to stop."""
    open_gate(hub)
    assert deferred_motion.dispatch("arm_control", "stop", {})["state"] == "idle"
    assert not deferred_motion._arm_cancel.is_set(), "an idle stop must not leave the cancel flag set"
    deferred_motion.dispatch("arm_control", "move", ARM)
    deferred_motion.spawn.run_all()
    assert [c[0] for c in io.calls] == ["switch", "goal", "switch"] and completions[-1][1] == "completed"


def test_a_new_move_starts_clean_even_if_the_previous_one_was_cancelled(deferred_motion, hub, io, completions):
    """The flag a stop sets must not outlive the move it cancelled."""
    open_gate(hub)
    deferred_motion.dispatch("arm_control", "move", ARM)
    deferred_motion.dispatch("arm_control", "stop", {})
    deferred_motion.spawn.run_all()
    assert completions[-1][1] == "cancelled" and deferred_motion._arm_cancel.is_set(), \
        "precondition: the cancelled move left the flag set"
    io.calls.clear()
    deferred_motion.dispatch("arm_control", "move", ARM)
    assert not deferred_motion._arm_cancel.is_set(), "accepting a new move clears the old stop"
    deferred_motion.spawn.run_all()
    assert [c[0] for c in io.calls] == ["switch", "goal", "switch"] and completions[-1][1] == "completed", \
        "a stale cancel flag must not silently cancel a fresh move"

def test_two_stops_and_a_move_in_any_order_never_leave_the_arm_busy(deferred_motion, hub, io):
    open_gate(hub)
    deferred_motion.dispatch("arm_control", "move", ARM)
    deferred_motion.dispatch("arm_control", "stop", {})
    deferred_motion.dispatch("arm_control", "stop", {})
    deferred_motion.spawn.run_all()
    assert deferred_motion.dispatch("arm_control", "info", {})["busy"] is False
    again = deferred_motion.dispatch("arm_control", "move", ARM)
    assert again["state"] == "moving", "a new move is accepted after a cancelled one"


def test_a_controller_restore_failure_is_logged_escaped_and_capped(deferred_motion, hub, io, completions, capsys):
    open_gate(hub)
    io.switch_fail_after = 1
    original = io.switch_controllers

    def hostile(activate, deactivate):
        try:
            original(activate, deactivate)
        except RuntimeError:
            raise RuntimeError("bad\x1b[31m\nINJECTED line " + "x" * 600)

    io.switch_controllers = hostile
    deferred_motion.dispatch("arm_control", "move", ARM)
    deferred_motion.spawn.run_all()
    out = capsys.readouterr().out
    warning = next(line for line in out.splitlines() if "could not restore the arm controllers" in line)
    assert "\x1b" not in out and "INJECTED" in warning and "\\x1b" in warning, "controls are escaped, on one line"
    assert not any(line.startswith("INJECTED") for line in out.splitlines()), "no forged second record"
    assert len(warning) < 400, "capped, not 600+ characters"
    assert len(completions[-1][2]["restore_error"]) < 400

def _race_once(motion_module, config, hub, io, delay):
    """One real-thread move-vs-stop interleaving. Returns (events, violations)."""
    import threading
    import time

    events, guard = [], threading.Lock()

    def note(what):
        with guard:
            events.append(what)

    io.calls.clear()
    io.controllers.update({FORWARD: "active", LEFT_P2P: "inactive", RIGHT_P2P: "inactive"})
    originals = (io.send_arm_goal, io.switch_controllers, io.cancel_arm_goal)

    def goal(*args):
        note("GOAL")
        return originals[0](*args)

    def switch(activate, deactivate):
        note(f"switch +{len(activate)}/-{len(deactivate)}")
        return originals[1](activate, deactivate)

    def cancel(handle):
        note("cancel-goal")
        return originals[2](handle)

    io.send_arm_goal, io.switch_controllers, io.cancel_arm_goal = goal, switch, cancel
    try:
        motion = motion_module.Motion(config, io, hub, notify=lambda *a: note(f"completion:{a[1]}"),
                                      sleep=lambda s: None)
        mover = threading.Thread(target=lambda: motion.dispatch("arm_control", "move", ARM))
        mover.start()
        if delay:
            time.sleep(delay)
        result = motion.dispatch("arm_control", "stop", {})
        note(f"STOP-RETURNED:{result['state']}")
        mover.join(5)
        end = time.time() + 5
        while motion.dispatch("arm_control", "info", {})["busy"] and time.time() < end:
            time.sleep(0.001)
    finally:
        io.send_arm_goal, io.switch_controllers, io.cancel_arm_goal = originals

    violations = []
    if any(e == "STOP-RETURNED:idle" for e in events):
        after = events[events.index("STOP-RETURNED:idle") + 1:]
        # After stop() has said idle the robot must be settled: nothing issued but the clean-up.
        if any(e == "GOAL" or e.startswith("switch +") for e in after):
            violations.append("motion issued after stop returned idle")
    if any(e == "GOAL" for e in events) and "cancel-goal" not in events and "completion:completed" not in events:
        violations.append("a goal was sent and neither cancelled nor completed")
    if io.controllers != {**io.controllers, FORWARD: "active", LEFT_P2P: "inactive", RIGHT_P2P: "inactive"}:
        violations.append("controllers not restored")
    if motion.dispatch("arm_control", "info", {})["busy"]:
        violations.append("still busy")
    return events, violations


def test_a_move_and_a_stop_racing_on_real_threads_leave_the_robot_settled(
        device, config, hub, io, motion_module):
    """The invariants that must hold in EVERY interleaving of a move and a stop on real threads:

    * once stop() has returned `idle`, nothing new is issued (no further switch-on, no goal);
    * a goal that was sent is cancelled or completed, never left running;
    * the controllers are back as they were, and the arm is not left busy.
    """
    open_gate(hub)
    bad = []
    for attempt in range(80):
        events, violations = _race_once(motion_module, config, hub, io, delay=0.0004 * (attempt % 6))
        if violations:
            bad.append((attempt, violations, events))
    assert bad == [], "\n".join(f"#{a} {v} {e}" for a, v, e in bad[:3])

def test_a_stop_that_lands_while_the_move_is_still_being_checked_cancels_it(
        device, config, hub, io, completions, clock, motion_module):
    """The window the stress test found: stop() arrives after move() began but before it was marked
    as running. It used to answer 'idle' and then the whole move ran."""
    open_gate(hub)
    motion = motion_module.Motion(config, io, hub, spawn=lambda t, n: t(),
                                  notify=lambda *a: completions.append(a), sleep=clock.advance,
                                  monotonic=lambda: clock.mono)
    original = io.list_controllers
    landed = []

    def stop_lands_during_validation():
        landed.append(motion.dispatch("arm_control", "stop", {}))     # the stop arrives mid-check
        return original()

    io.list_controllers = stop_lands_during_validation
    result = motion.dispatch("arm_control", "move", ARM)
    assert landed == [{"state": "idle"}]
    assert result["state"] == "error" and "stopped while this move was being checked" in result["message"]
    assert io.calls == [] and completions == [], "nothing was switched, sent or reported"
    io.list_controllers = original
    assert motion.dispatch("arm_control", "move", ARM)["state"] == "moving", "and a later move is fine"


def test_a_stop_before_a_move_began_does_not_cancel_it(device, config, hub, io, completions, clock, motion_module):
    """Only a stop that arrives *during* a move's checking cancels it; an earlier one is history."""
    open_gate(hub)
    motion = motion_module.Motion(config, io, hub, spawn=lambda t, n: t(),
                                  notify=lambda *a: completions.append(a), sleep=clock.advance,
                                  monotonic=lambda: clock.mono)
    motion.dispatch("arm_control", "stop", {})
    motion.dispatch("arm_control", "stop", {})
    assert motion.dispatch("arm_control", "move", ARM)["state"] == "moving"
    assert completions[-1][1] == "completed"

# ── nsenter itself failing must be reported as nsenter failing ───────────────────────────────

class DeniedRun:
    """What a container that lacks a capability sees: nsenter exits non-zero with its own message,
    for every command, including the ones the driver calls with check=False."""

    def __init__(self, host_camera, message, returncode=1):
        self.prefix = list(host_camera.HostCamera.NSENTER)
        self.message, self.returncode, self.calls = message, returncode, []

    def __call__(self, command, check, capture_output, text, timeout):
        self.calls.append(list(command))
        return NS(stdout="", stderr=self.message, returncode=self.returncode)

    def tails(self):
        return [c[len(self.prefix):] for c in self.calls]


DENIED = "nsenter: cannot open /proc/1/ns/mnt: Permission denied"


def test_a_denied_nsenter_is_reported_as_nsenter_and_names_the_missing_privilege(host_camera, config, clock, capsys):
    """With check=False on is-active/start, a denied nsenter used to look like 'the unit does not
    exist', and the error that surfaced was systemd-run's, pointing at the wrong thing."""
    run = DeniedRun(host_camera, DENIED)
    info = _camera(host_camera, config, clock, run).start("color")
    assert "nsenter" in info["start_error"] and "Permission denied" in info["start_error"]
    assert "systemd-run" not in info["start_error"]
    assert "cap_add" in info["start_error"] and "SYS_ADMIN" in info["start_error"], \
        "says what to check, not only what failed"
    assert [t[0] for t in run.tails()] == ["systemctl"], "stopped at the first denied call; no further host commands"
    assert "could not enter the host's namespaces" in capsys.readouterr().out


def test_a_denied_nsenter_is_not_mistaken_for_a_unit_that_is_merely_not_running(host_camera, config, clock):
    """`systemctl is-active` legitimately exits non-zero for a unit that does not exist; only nsenter's
    own message identifies a failure to enter the namespaces."""
    ordinary = FakeRun(host_camera)                          # is-active=inactive rc 3, no stderr
    _camera(host_camera, config, clock, ordinary).start("color")
    assert [t[0] for t in ordinary.tails()] == ["systemctl", "systemctl", "systemd-run"], \
        "an inactive unit still goes on to be created"


@pytest.mark.parametrize("message", [
    "nsenter: cannot open /proc/1/ns/pid: Permission denied",
    "nsenter: reassociate to namespace 'ns/ipc' failed: Operation not permitted",
    "nsenter: neither filename nor target pid supplied for ns/net",
    "/usr/bin/nsenter: cannot open /proc/1/ns/uts: No such file or directory",
])
def test_every_way_nsenter_can_refuse_is_recognised(host_camera, config, clock, message):
    info = _camera(host_camera, config, clock, DeniedRun(host_camera, message)).start("depth")
    assert "could not enter the host" in info["start_error"], message


@pytest.mark.parametrize("stderr", [
    "Unit panther_orbbec.service could not be found (started by nsenter earlier).",
    "systemctl: nsenter: not a denial, the word is only in the middle of a systemctl line",
    "Failed to start panther_orbbec.service: see `journalctl` (the nsenter: prefix is on a later line)\nnsenter: x",
    "mynsenter: looks similar but is a different program",
    "xnsenter: neither is this",
])
def test_a_message_that_is_not_nsenters_own_refusal_is_not_taken_for_one(host_camera, config, clock, stderr):
    """Only a message that STARTS with `nsenter:` (optionally with a path in front) is nsenter refusing.
    Cases where a looser pattern would fire: the word appearing mid-line, on a later line, or as the
    tail of another program's name."""
    class Quirky(FakeRun):
        def __call__(self, command, check, capture_output, text, timeout):
            result = super().__call__(command, check, capture_output, text, timeout)
            result.stderr = stderr
            return result

    info = _camera(host_camera, config, clock, Quirky(host_camera)).start("color")
    assert "start_error" not in info, stderr


def test_the_denial_pattern_is_anchored_at_the_start_of_the_first_line(host_camera):
    refuse = host_camera.HostCamera._nsenter_refused
    assert refuse(NS(stderr="nsenter: cannot open x: Permission denied", returncode=1))
    assert refuse(NS(stderr="/usr/bin/nsenter: cannot open x", returncode=1))
    assert refuse(NS(stderr="nsenter: a\nsecond line", returncode=1)) == "nsenter: a"
    assert refuse(NS(stderr="nsenter: cannot open x", returncode=0)) is None, "a zero exit is not a refusal"
    assert refuse(NS(stderr="", returncode=1)) is None and refuse(NS(returncode=1)) is None
    assert refuse(NS(stderr=None, returncode=1)) is None

def test_a_stop_that_cannot_enter_the_namespaces_says_so_and_still_does_not_raise(host_camera, config, clock, capsys):
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("color")
    run_denied = DeniedRun(host_camera, DENIED)
    camera._run = run_denied
    camera.close()
    out = capsys.readouterr().out
    assert "could not stop panther_orbbec.service" in out and "sudo systemctl stop panther_orbbec.service" in out

# ── the container gets the privilege nsenter needs and no more ──────────────────────────────────

def _service_body():
    (_, body), = yaml.safe_load((DRIVER / "deploy" / "service.yml").read_text(encoding="utf-8")).items()
    return body


def test_the_container_is_not_privileged_and_asks_for_exactly_one_capability():
    """Review finding: privileged gives every capability, every host device and no seccomp/AppArmor
    confinement, when the one thing that needs it is entering PID 1's namespaces."""
    body = _service_body()
    assert body.get("privileged") in (None, False), "privileged: true is back"
    assert body["cap_add"] == ["SYS_ADMIN"], "exactly the capability setns(2) needs, nothing else"
    for broadening in ("devices", "device_cgroup_rules", "ipc", "userns_mode", "cgroup", "security_opt"):
        assert broadening not in body, f"{broadening} widens the container beyond what nsenter needs"
    for capability in ("ALL", "SYS_PTRACE", "SYS_MODULE", "NET_ADMIN", "SYS_RAWIO", "DAC_READ_SEARCH"):
        assert capability not in body["cap_add"], capability


def test_the_privilege_is_explained_next_to_where_it_is_granted():
    """A reader of service.yml must find why, and what to do if a robot refuses, without hunting."""
    text = (DRIVER / "deploy" / "service.yml").read_text(encoding="utf-8")
    anchor = re.search(r"^  pid: host$", text, re.M).start()                  # the setting, not a comment that quotes it
    block = re.sub(r"\s*#\s*|\s+", " ", text[:anchor])                          # comment wrapping is not content
    for fragment in ("CAP_SYS_ADMIN", "setns(2)", "privileged: true", "seccomp", "Acceptance step 7",
                     "do not go back to"):
        assert fragment in block, fragment


def test_nothing_in_the_driver_still_claims_it_needs_privileged():
    """The docs must not describe the old design. Mentioning the word is fine (the explanations say what
    it replaced, and `docker inspect` prints a `Privileged` field); what is not fine is saying it is
    needed, required or why the container is it."""
    claims = re.compile(r"(needs?|requires?|required|why the container is|must be|has to be)\W+(`?privileged)"
                        r"|privileged\W+(is|are)\W+(needed|required|necessary)"
                        r"|privileged\s*\+\s*pid", re.I)
    for name in ("README.md", "host_camera.py", "config.yaml", "device.py", "panther_motion.py"):
        text = (DRIVER / name).read_text(encoding="utf-8")
        flattened = re.sub(r"\s*\n\s*(#\s*)?", " ", text)               # a claim may wrap across lines
        found = claims.search(flattened)
        assert not found, f"{name} still says privileged is needed: ...{flattened[max(0, found.start() - 60):found.end() + 60]}..."
    service = (DRIVER / "deploy" / "service.yml").read_text(encoding="utf-8")
    assert not claims.search(re.sub(r"\s*\n\s*#?\s*", " ", service)), "service.yml's comment still says it is needed"


def test_the_claim_detector_actually_detects_the_old_claims():
    """A checker that never fires proves nothing: it must catch the sentences this PR removed."""
    claims = re.compile(r"(needs?|requires?|required|why the container is|must be|has to be)\W+(`?privileged)"
                        r"|privileged\W+(is|are)\W+(needed|required|necessary)"
                        r"|privileged\s*\+\s*pid", re.I)
    for old in ("This is why the container is `privileged` with `pid: host`",
                "which needs privileged + pid: host", "This needs `privileged: true` and `pid: host`",
                "privileged is required for nsenter", "the container must be privileged"):
        assert claims.search(old), old
    for fine in ("not `privileged: true`, which would also give every capability",
                 "`docker inspect` prints {{.HostConfig.Privileged}}", "only ever tried under `privileged`",
                 "do not return to `privileged`", "needs pid: host and the SYS_ADMIN capability, not privileged"):
        assert not claims.search(fine), fine

def test_host_camera_only_ever_runs_commands_through_nsenter_with_fixed_arguments(host_camera, config, clock):
    """What the one capability is spent on: every host command goes through nsenter and none of its
    arguments comes from outside the driver."""
    run = FakeRun(host_camera)
    camera = _camera(host_camera, config, clock, run)
    camera.start("color")
    camera.close()
    assert run.calls, "precondition: the camera did run host commands"
    for call in run.calls:
        assert call[:len(run.prefix)] == run.prefix and run.prefix[0] == "nsenter"
        assert call[len(run.prefix)] in ("systemctl", "systemd-run"), call
    assert run.prefix == ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--"], \
        "the namespaces entered are the ones the capability and pid: host were granted for"

def test_the_driver_documents_are_not_corrupted_by_the_tools_that_wrote_them():
    """Shell quoting once ate backticks and turned `\n` into a newline in this README, which would have
    sent an operator a broken command. Control characters and a dropped initial letter are the signs."""
    for path in [*DRIVER.glob("*.md"), *DRIVER.glob("*.py"), *DRIVER.glob("*.yaml"), *DRIVER.glob("deploy/*.yml")]:
        text = path.read_text(encoding="utf-8")
        for number, line in enumerate(text.splitlines(), 1):
            assert not re.search(r"[\x00-\x08\x0b\x0c\x0e-\x1f]", line), f"{path.name}:{number} has a control character"
        assert "\ufffd" not in text, f"{path.name} has a replacement character"
    readme = (DRIVER / "README.md").read_text(encoding="utf-8")
    for command in ("docker inspect embodied-unixai-panther", "systemctl is-active panther_orbbec",
                    "ros2 run tf2_ros tf2_echo"):
        assert f"`{command}" in readme, f"{command!r} is not in a code span"
    assert "must print `false [SYS_ADMIN] host`" in readme, "the expected inspect output is exact"
    assert "`nsenter` may read `/proc/1/ns/*`" in readme


def test_the_acceptance_step_for_the_reduced_privilege_has_a_failure_branch():
    """A check that only says what success looks like leaves an operator stuck when it fails."""
    readme = (DRIVER / "README.md").read_text(encoding="utf-8")
    step = readme[readme.index("7. **Cameras, with the reduced privilege**"):readme.index("8. **Odometry**")]
    assert "could not\n      enter the host's namespaces" in step or "could not enter the host's namespaces" in step.replace("\n      ", " ")
    assert "do not return to `privileged`" in step.replace("\n      ", " ")
    assert "record what it was" in step.replace("\n      ", " ")

# ── the motion cards after removing confirmation and the switch ───────────────────────────────

@pytest.fixture(scope="module")
def BUNDLED_LIMITS(device):
    return importlib.import_module("contracts").parse_joint_limits(
        (DRIVER / "resource" / "panther.urdf").read_text(encoding="utf-8"))


def _tools(motion_module, config, limits):
    return {t["name"]: t for t in motion_module.motion_tools(config, limits)}


def test_no_confirmation_field_and_no_switch_anywhere(motion_module, config, BUNDLED_LIMITS):
    """The operator asked for both to go: with intelligent control on, a call moves the robot."""
    text = json.dumps(motion_module.motion_tools(config, BUNDLED_LIMITS), ensure_ascii=False)
    assert "confirm" not in text and "commissioning" not in text
    assert "safety" not in config
    for name in ("panther_motion.py", "contracts.py", "device.py", "config.yaml", "Dockerfile"):
        body = (DRIVER / name).read_text(encoding="utf-8")
        assert "confirm_motion" not in body and "commissioning_motion_enabled" not in body, name
    assert not (DRIVER / "motion_switch.py").exists()
    assert "PANTHER_COMMISSIONING" not in (DRIVER / "deploy" / "service.yml").read_text(encoding="utf-8")


def test_a_ready_robot_moves_on_a_plain_call(motion, hub, io):
    open_gate(hub)
    assert motion.dispatch("base_drive", "move", DRIVE)["state"] == "moving"
    assert motion.dispatch("head", "look", {"head0_rad": 0.1})["state"] == "commanded"
    assert motion.dispatch("lifting", "set_position", {"position": 0.62})["state"] == "commanded"
    assert motion.dispatch("gripper", "open", {})["state"] == "commanded"
    assert io.named("twist") and io.named("head") and io.named("lift") and io.named("gripper")


def test_the_robots_own_state_still_refuses_motion(motion, hub, io):
    """What replaced the switch: conditions that clear by themselves when the robot is fine."""
    for card, action, args in (("base_drive", "move", DRIVE), ("arm_control", "move", ARM),
                               ("gripper", "set", {"position": 0.5}), ("lifting", "set_position", {"position": 0.62}),
                               ("head", "look", {"head0_rad": 0.1}), ("arm_control", "home", {}), ("head", "home", {})):
        result = motion.dispatch(card, action, args)
        assert result["state"] == "error" and "motion refused" in result["message"], card
    assert io.calls == []


# ── which cards keep an action, and what each one offers ───────────────────────────────────────

EXPECTED_ACTIONS = {"base_drive": ["move", "stop"], "arm_control": ["move", "home", "stop"],
                    "gripper": ["set", "open", "close"], "head": ["look", "home", "stop"]}


def test_lifting_has_no_action_box_and_the_others_offer_only_what_they_do(motion_module, config, BUNDLED_LIMITS):
    tools = _tools(motion_module, config, BUNDLED_LIMITS)
    lifting = tools["lifting"]["inputSchema"]
    assert "action" not in lifting["properties"] and "required" not in lifting and "x-action-params" not in lifting
    assert list(lifting["properties"]) == ["position"]
    for name, actions in EXPECTED_ACTIONS.items():
        schema = tools[name]["inputSchema"]
        assert schema["properties"]["action"]["enum"] == actions, name
        assert schema["properties"]["action"]["default"] == actions[0], "the motion is preselected"
        assert [o["const"] for o in schema["properties"]["action"]["oneOf"]] == actions
        assert set(schema["x-action-params"]) == set(actions)
        for action, entry in schema["x-action-params"].items():
            assert set(entry["params"]) <= set(schema["properties"]), (name, action)
            assert entry["description"], (name, action)
        assert not ({"start", "info"} & set(actions)), "lifecycle actions are the framework's, not a menu item"


def test_a_call_without_an_action_runs_the_cards_one_motion(motion, hub, io):
    """The bundle passes the tool name when there is no action (README_dev); lifting is always called so."""
    open_gate(hub)
    assert motion.dispatch("lifting", "lifting", {"position": 0.62})["state"] == "commanded"
    assert motion.dispatch("lifting", None, {"position": 0.63})["state"] == "commanded"
    assert motion.dispatch("head", "head", {"head0_rad": 0.1})["state"] == "commanded"
    assert motion.dispatch("gripper", "", {"position": 0.5})["state"] == "commanded"
    assert [c[0] for c in io.calls].count("lift") == 2


def test_the_bundle_routes_a_lifting_call_without_an_action(device, hub, motion, config, io):
    from common.vendor_runtime import DriverBundle
    open_gate(hub)
    bundle = DriverBundle([device.PantherPlugin(hub, motion=motion, config=config)])
    assert bundle.dispatch("lifting", {"position": 0.62})["state"] == "commanded"
    assert bundle.dispatch("lifting", {"action": "start"})["state"] == "ready", "lifecycle still reaches it"
    assert bundle.dispatch("lifting", {"action": "stop"})["state"] == "idle"
    assert io.named("lift") == [(0.62,)]


# ── every box shows its range, and the range shown is the one enforced ─────────────────────────

def _boxes(tools):
    for name, definition in tools.items():
        for field, spec in definition["inputSchema"]["properties"].items():
            if spec.get("type") == "number":
                yield name, field, spec


def test_every_number_box_shows_its_range_in_the_title_and_the_description(motion_module, config, BUNDLED_LIMITS):
    boxes = list(_boxes(_tools(motion_module, config, BUNDLED_LIMITS)))
    assert len(boxes) == 4 + 16 + 1 + 1 + 2, [f for _, f, _ in boxes]
    for name, field, spec in boxes:
        low, high = spec["minimum"], spec["maximum"]
        shown = f"{low:g} ~ {high:g}"
        assert shown in spec["title"] and shown in spec["description"], (name, field, spec)
        assert low < high, (name, field)


def test_the_arm_and_head_boxes_show_the_urdf_limits_of_their_own_joint(motion_module, config, BUNDLED_LIMITS):
    props = {n: t["inputSchema"]["properties"] for n, t in _tools(motion_module, config, BUNDLED_LIMITS).items()}
    for i in range(8):
        for side in ("la", "ra"):
            low, high = BUNDLED_LIMITS[f"joint_{side}{i}"]
            assert (props["arm_control"][f"{side}{i}"]["minimum"], props["arm_control"][f"{side}{i}"]["maximum"]) == (low, high)
    assert (props["head"]["head0_rad"]["minimum"], props["head"]["head0_rad"]["maximum"]) == BUNDLED_LIMITS["joint_head0"]
    assert (props["head"]["head1_rad"]["minimum"], props["head"]["head1_rad"]["maximum"]) == BUNDLED_LIMITS["joint_head1"]
    assert props["arm_control"]["la0"]["title"] == "左臂 la0（rad，-1.046 ~ 0.697）"
    assert props["arm_control"]["ra7"]["title"] == "右臂 ra7（rad，-1.57 ~ 1.57）"


def test_no_step_limit_is_stated_or_implied_anywhere(motion_module, config, BUNDLED_LIMITS):
    """The operator asked for one command to reach any value inside the range."""
    props = {n: t["inputSchema"]["properties"] for n, t in _tools(motion_module, config, BUNDLED_LIMITS).items()}
    for field in [f"la{i}" for i in range(8)] + [f"ra{i}" for i in range(8)]:
        assert "相差不超过" not in props["arm_control"][field]["description"]
    assert "相差不超过" not in props["head"]["head0_rad"]["description"]
    assert "相差不超过" not in props["lifting"]["position"]["description"]
    assert "max_target_delta" not in json.dumps(config)
    assert "duration_s" not in props["arm_control"], "the arm's duration is derived, not asked for"


def test_a_full_range_arm_move_is_one_command(motion, hub, io):
    open_gate(hub)
    low, high = -1.046, 0.697                      # la0's whole URDF range
    assert motion.dispatch("arm_control", "move", {"la0": high})["state"] == "moving"
    assert motion.dispatch("arm_control", "move", {"la0": low})["state"] == "moving"
    assert len(io.named("goal")) == 2


def test_a_full_range_head_move_is_one_command(motion, hub, io):
    positions = [0.0] * len(URDF_JOINTS)
    positions[URDF_JOINTS.index("joint_head1")] = -1.57
    open_gate(hub, positions)
    assert motion.dispatch("head", "look", {"head1_rad": 1.57})["state"] == "commanded"
    assert io.named("head") == [((0.0, 1.57),)]


def test_a_full_range_lift_move_is_one_command(motion, hub, io):
    open_gate(hub)
    hub.on_lift(joint_state(["body_joint"], [0.61]))
    assert motion.dispatch("lifting", None, {"position": 0.0})["state"] == "commanded"
    assert io.named("lift") == [(0.0,)]


@pytest.mark.parametrize("card, action, field, other", [
    ("base_drive", "move", "linear_x", {"duration_s": 0.5}),
    ("base_drive", "move", "linear_y", {"duration_s": 0.5}),
    ("base_drive", "move", "angular_z", {"duration_s": 0.5}),
    ("base_drive", "move", "duration_s", {}),
    ("gripper", "set", "position", {}),
])
def test_a_value_on_the_shown_boundary_is_accepted_and_one_just_past_it_refused(
        motion_module, motion, hub, io, config, BUNDLED_LIMITS, card, action, field, other):
    spec = _tools(motion_module, config, BUNDLED_LIMITS)[card]["inputSchema"]["properties"][field]
    open_gate(hub)
    for value in (spec["minimum"], spec["maximum"]):
        result = motion.dispatch(card, action, {**other, field: value})
        assert result["state"] in ("moving", "commanded"), (field, value, result)
        motion.stop(card)
    for value in (spec["minimum"] - 1e-6, spec["maximum"] + 1e-6):
        result = motion.dispatch(card, action, {**other, field: value})
        assert result["state"] == "error", (field, value)
        assert f"{spec['minimum']:g} ~ {spec['maximum']:g}".replace("-", "") in result["message"].replace("-", ""), \
            "the refusal quotes the same range the box shows"


def test_every_arm_joint_box_accepts_its_shown_limit_and_refuses_beyond_it(motion_module, device, config, hub, io,
                                                                         BUNDLED_LIMITS, clock):
    """The arm also has a step limit, so the robot is placed next to each limit before trying it."""
    props = _tools(motion_module, config, BUNDLED_LIMITS)["arm_control"]["inputSchema"]["properties"]
    motion_module_instance = motion_module.Motion(config, io, hub, spawn=lambda t, n: t(), notify=lambda *a: None,
                                                  sleep=clock.advance, monotonic=lambda: clock.mono)
    for field in [f"la{i}" for i in range(8)] + [f"ra{i}" for i in range(8)]:
        joint = "joint_" + field
        for bound in ("minimum", "maximum"):
            limit = props[field][bound]
            near = limit + (0.1 if bound == "minimum" else -0.1)
            positions = [near if name == joint else 0.0 for name in URDF_JOINTS]
            open_gate(hub, positions)
            ok = motion_module_instance.dispatch("arm_control", "move", {field: limit})
            assert ok["state"] == "moving", (field, bound, ok)
            past = limit + (-1e-6 if bound == "minimum" else 1e-6)
            refused = motion_module_instance.dispatch("arm_control", "move", {field: past})
            assert refused["state"] == "error" and "outside its range" in refused["message"], (field, bound)


def test_the_head_boxes_accept_their_shown_limits_and_refuse_beyond(motion_module, motion, config, hub, io, BUNDLED_LIMITS):
    props = _tools(motion_module, config, BUNDLED_LIMITS)["head"]["inputSchema"]["properties"]
    for field, joint in (("head0_rad", "joint_head0"), ("head1_rad", "joint_head1")):
        for bound in ("minimum", "maximum"):
            limit = props[field][bound]
            positions = [limit + (0.1 if bound == "minimum" else -0.1) if n == joint else 0.0 for n in URDF_JOINTS]
            open_gate(hub, positions)
            assert motion.dispatch("head", "look", {field: limit})["state"] == "commanded", (field, bound)
            past = limit + (-1e-6 if bound == "minimum" else 1e-6)
            assert "outside its range" in motion.dispatch("head", "look", {field: past})["message"], (field, bound)


def test_the_lift_box_accepts_its_shown_limits_and_refuses_beyond(motion_module, motion, config, hub, io, BUNDLED_LIMITS):
    spec = _tools(motion_module, config, BUNDLED_LIMITS)["lifting"]["inputSchema"]["properties"]["position"]
    open_gate(hub)
    hub.on_lift(joint_state(["body_joint"], [spec["maximum"] - 0.05]))
    assert motion.dispatch("lifting", None, {"position": spec["maximum"]})["state"] == "commanded"
    assert "0 ~ 0.65" in motion.dispatch("lifting", None, {"position": spec["maximum"] + 1e-6})["message"]
    hub.on_lift(joint_state(["body_joint"], [0.05]))
    assert motion.dispatch("lifting", None, {"position": spec["minimum"]})["state"] == "commanded"
    assert "0 ~ 0.65" in motion.dispatch("lifting", None, {"position": -1e-6})["message"]


def test_the_displayed_ranges_follow_the_config_caps(motion_module, config, BUNDLED_LIMITS):
    config["plugins"]["base_drive"]["max_linear_mps"] = 0.05
    config["plugins"]["lifting"]["max_position"] = 0.62
    props = {n: t["inputSchema"]["properties"] for n, t in _tools(motion_module, config, BUNDLED_LIMITS).items()}
    assert (props["base_drive"]["linear_x"]["minimum"], props["base_drive"]["linear_x"]["maximum"]) == (-0.05, 0.05)
    assert "-0.05 ~ 0.05" in props["base_drive"]["linear_x"]["title"]
    assert props["lifting"]["position"]["maximum"] == 0.62 and "0 ~ 0.62" in props["lifting"]["position"]["title"]


def test_the_cards_show_the_bundled_limits_and_say_so_when_the_robot_differs(device, hub, capsys):
    limits = hub.bundled_joint_limits()
    assert hub.limit_differences(limits) == []
    live = dict(limits)
    live["joint_la0"] = (-1.0, 0.5)
    del live["joint_head1"]
    differ = hub.limit_differences(live)
    assert differ == ["joint_la0: robot -1~0.5, shown -1.046~0.697", "joint_head1: not in the robot's URDF"]
    urdf = (DRIVER / "resource" / "panther.urdf").read_text(encoding="utf-8").replace(
        '<limit effort="10" lower="-0.348" upper="1.5"', '<limit effort="10" lower="-0.2" upper="1.5"')
    hub.on_robot_description(NS(data=urdf))
    out = capsys.readouterr().out
    assert "WARNING: the robot's joint limits differ" in out and "joint_head0: robot -0.2~1.5" in out


# ── arm: one box per joint ─────────────────────────────────────────────────────────────────────

def test_an_empty_box_keeps_that_joint_where_it_is(motion, hub, io):
    current = [0.0] * len(URDF_JOINTS)
    current[URDF_JOINTS.index("joint_la4")] = -0.5
    open_gate(hub, current)
    motion.dispatch("arm_control", "move", {"la2": 0.3})
    (side, positions, _), = io.named("goal")
    assert side == "left" and positions == (0.0, 0.0, 0.3, 0.0, -0.5, 0.0, 0.0, 0.0), \
        "la2 moves; every other joint is sent its current position, la4 included"


def test_only_the_arms_with_a_filled_box_move(motion, hub, io):
    open_gate(hub)
    motion.dispatch("arm_control", "move", {"ra7": 0.2})
    assert [c[0] for c in io.named("goal")] == ["right"]
    io.calls.clear()
    motion.dispatch("arm_control", "move", {"la0": 0.1, "ra0": 0.1})
    assert [c[0] for c in io.named("goal")] == ["left", "right"]


def test_an_arm_move_with_every_box_empty_is_refused_and_sends_nothing(motion, hub, io):
    open_gate(hub)
    for args in ({}, {f"la{i}": None for i in range(8)}, {"la0": "", "ra3": "  "}):
        result = motion.dispatch("arm_control", "move", args)
        assert result["state"] == "error" and "fill in at least one joint" in result["message"], args
    assert io.calls == []


def test_the_canvas_may_send_a_number_as_text_but_never_words(motion, hub, io):
    open_gate(hub)
    assert motion.dispatch("arm_control", "move", {"la0": "0.1", "duration_s": "2"})["state"] == "moving"
    for bad in ("abc", True, "nan", float("inf")):
        result = motion.dispatch("arm_control", "move", {"la0": bad})
        assert result["state"] == "error" and ("number" in result["message"] or "finite" in result["message"]), bad


def test_the_arm_refuses_with_nothing_sent_when_a_value_is_wrong(motion, hub, io):
    open_gate(hub)
    for args, text in (({"la0": 9.0}, "outside its range"),
                       ({"la0": 0.1, "ra0": 9.0}, "outside its range")):
        result = motion.dispatch("arm_control", "move", args)
        assert result["state"] == "error" and text in result["message"], (args, result)
    assert io.calls == [], "nothing switched and nothing sent, including the valid left arm of the last case"


def test_the_arm_duration_follows_the_distance_and_the_configured_speed(motion, hub, io, config):
    open_gate(hub)
    near = motion.dispatch("arm_control", "move", {"la0": 0.1})["duration_s"]
    assert near == config["plugins"]["arm_control"]["min_duration_s"], "short moves take the minimum"
    far = motion.dispatch("arm_control", "move", {"la1": 0.785})["duration_s"]   # a 0.785 rad move
    assert far == pytest.approx(0.785 / config["plugins"]["arm_control"]["speed_rad_s"], rel=1e-2)
    assert far > near
    assert far <= config["plugins"]["arm_control"]["max_duration_s"]


# ── arm and head: home ─────────────────────────────────────────────────────────────────────────

def test_arm_home_sends_every_joint_of_the_chosen_arms_to_zero(motion, hub, io, completions):
    positions = [0.2 if name.startswith(("joint_la", "joint_ra")) and not name.endswith(("a4", "a2")) else 0.0
                 for name in URDF_JOINTS]
    open_gate(hub, positions)
    result = motion.dispatch("arm_control", "home", {"side": "both"})
    assert result["state"] == "moving" and result["action"] == "home"
    goals = io.named("goal")
    assert [g[0] for g in goals] == ["left", "right"]
    assert all(g[1] == (0.0,) * 8 for g in goals)
    assert completions[-1][1] == "completed" and completions[-1][3] == "arm_control"


def test_arm_home_is_not_held_to_the_step_limit_but_stretches_its_duration(motion, hub, io):
    positions = [0.0] * len(URDF_JOINTS)
    positions[URDF_JOINTS.index("joint_la3")] = 2.1                 # far beyond one 0.35 rad step
    open_gate(hub, positions)
    result = motion.dispatch("arm_control", "home", {"side": "left"})
    assert result["state"] == "moving"
    assert result["duration_s"] == pytest.approx(2.1 / 0.35), "average speed of the furthest joint is 0.35 rad/s"
    (side, target, duration), = io.named("goal")
    assert side == "left" and target == (0.0,) * 8 and duration == pytest.approx(6.0)


def test_arm_home_takes_at_least_the_minimum_duration(motion, hub, io):
    positions = [0.0] * len(URDF_JOINTS)
    positions[URDF_JOINTS.index("joint_ra0")] = 0.05
    open_gate(hub, positions)
    assert motion.dispatch("arm_control", "home", {"side": "right"})["duration_s"] == 2.0


def test_arm_home_that_would_take_too_long_is_refused(motion, hub, io, config):
    config["plugins"]["arm_control"]["home_speed_rad_s"] = 0.05
    positions = [0.0] * len(URDF_JOINTS)
    positions[URDF_JOINTS.index("joint_la1")] = -2.0
    open_gate(hub, positions)
    result = motion.dispatch("arm_control", "home", {"side": "left"})
    assert result["state"] == "error" and "over the 20.0 s limit" in result["message"]
    assert io.calls == []


def test_arm_home_uses_home_pose_from_config_and_checks_it_against_the_limits(motion, hub, io, config):
    open_gate(hub)
    config["plugins"]["arm_control"]["home_pose"] = {"joint_la2": 0.5}
    motion.dispatch("arm_control", "home", {"side": "left"})
    assert io.named("goal")[0][1] == (0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
    config["plugins"]["arm_control"]["home_pose"] = {"joint_la2": 5.0}
    result = motion.dispatch("arm_control", "home", {"side": "left"})
    assert result["state"] == "error" and "outside its range" in result["message"], "a bad home_pose is never sent"


def test_arm_home_defaults_to_both_arms_and_refuses_a_bad_side(motion, hub, io):
    open_gate(hub)
    assert motion.dispatch("arm_control", "home", {})["side"] == "both"
    assert "side must be" in motion.dispatch("arm_control", "home", {"side": "middle"})["message"]


def test_head_home_steps_to_zero_at_the_configured_speed(motion, hub, io, completions):
    positions = [0.0] * len(URDF_JOINTS)
    positions[URDF_JOINTS.index("joint_head1")] = 1.4
    positions[URDF_JOINTS.index("joint_head0")] = 0.7
    open_gate(hub, positions)
    result = motion.dispatch("head", "home", {})
    assert result["state"] == "moving" and result["duration_s"] == pytest.approx(4.0, abs=0.06)
    sent = [c[0] for c in io.named("head")]
    assert sent[-1] == (0.0, 0.0)
    assert len(sent) == 80, "1.4 rad at 0.35 rad/s, one step every 1/20 s"
    steps = [max(abs(a - b) for a, b in zip(x, y)) for x, y in zip([(0.7, 1.4)] + sent, sent)]
    assert max(steps) <= 0.35 / 20 + 1e-9, "no step faster than the configured speed"
    assert completions[-1][1] == "completed" and completions[-1][3] == "head"


def test_head_stop_halts_a_home_where_it_is(motion, hub, io, completions):
    positions = [0.0] * len(URDF_JOINTS)
    positions[URDF_JOINTS.index("joint_head1")] = 1.0
    open_gate(hub, positions)
    original = io.publish_head

    def hooked(values):
        original(values)
        if len(io.named("head")) == 5:
            motion.stop("head")

    io.publish_head = hooked
    motion.dispatch("head", "home", {})
    assert len(io.named("head")) == 5, "nothing more after the stop"
    assert io.named("head")[-1][0][1] > 0.8, "it stops part way, it does not jump to the target"
    assert completions[-1][1] == "cancelled"


def test_a_head_look_supersedes_a_home_in_progress(motion, hub, io, completions):
    positions = [0.0] * len(URDF_JOINTS)
    positions[URDF_JOINTS.index("joint_head1")] = 1.0
    open_gate(hub, positions)
    original = io.publish_head
    state = {"done": False}

    def hooked(values):
        original(values)
        if not state["done"] and len(io.named("head")) == 3:
            state["done"] = True
            motion.dispatch("head", "look", {"head1_rad": 0.9})

    io.publish_head = hooked
    motion.dispatch("head", "home", {})
    assert io.named("head")[-1][0] == (0.0, 0.9) and completions[-1][1] == "cancelled"


def test_head_look_with_one_box_keeps_the_other_joint(motion, hub, io):
    positions = [0.0] * len(URDF_JOINTS)
    positions[URDF_JOINTS.index("joint_head0")] = 0.3
    open_gate(hub, positions)
    motion.dispatch("head", "look", {"head1_rad": -0.2})
    assert io.named("head") == [((0.3, -0.2),)]
    assert "fill in head0_rad" in motion.dispatch("head", "look", {})["message"]


def test_stop_all_stops_the_drive_the_arm_and_a_head_home(motion, hub, io, capsys):
    motion.stop_all()
    assert io.named("twist") == [(0.0, 0.0, 0.0)] * 5
    io.fail_on = {"twist"}
    motion.stop_all()
    assert "stopping base_drive failed" in capsys.readouterr().out
    assert motion._head_generation == 2, "the head was told to stop both times"


# ── gripper ─────────────────────────────────────────────────────────────────────────────────────

def test_gripper_both_checks_both_controllers_before_sending_to_either(motion, hub, io):
    open_gate(hub)
    result = motion.dispatch("gripper", "set", {"side": "both", "position": 0.4})
    assert result["side"] == "both" and io.named("gripper") == [("left", "parallel", 0.4), ("right", "parallel", 0.4)]
    io.calls.clear()
    io.controllers["right_unix_gripper3_controller"] = "inactive"
    result = motion.dispatch("gripper", "set", {"side": "both", "position": 0.6})
    assert result["state"] == "error" and "right_unix_gripper3_controller" in result["message"]
    assert io.named("gripper") == [], "the left one was not sent either"


def test_gripper_defaults_to_both_hands(motion, hub, io):
    open_gate(hub)
    assert motion.dispatch("gripper", "set", {"position": 0.5})["side"] == "both"
    assert len(io.named("gripper")) == 2


def test_gripper_open_is_zero_and_close_is_one(motion, hub, io):
    """0 = open, 1 = closed: confirmed by the operator on the robot; the vendor SDK says the opposite."""
    open_gate(hub)
    motion.dispatch("gripper", "open", {"side": "left"})
    motion.dispatch("gripper", "close", {"side": "right"})
    assert io.named("gripper") == [("left", "parallel", 0.0), ("right", "parallel", 1.0)]


def test_gripper_always_sends_parallel_and_has_no_mode_choice(motion, hub, io, motion_module, config, BUNDLED_LIMITS):
    schema = _tools(motion_module, config, BUNDLED_LIMITS)["gripper"]["inputSchema"]
    assert "mode" not in schema["properties"], "the three modes were indistinguishable on the robot"
    open_gate(hub)
    motion.dispatch("gripper", "set", {"side": "left", "position": 0.4, "mode": "centric"})
    assert io.named("gripper") == [("left", "parallel", 0.4)], "an old canvas value cannot change the mode"


def test_gripper_open_and_close_ignore_a_leftover_position(motion, hub, io):
    """The canvas may still send the box's last value: a preset must not take it."""
    open_gate(hub)
    motion.dispatch("gripper", "open", {"side": "left", "position": 0.7})
    assert io.named("gripper") == [("left", "parallel", 0.0)]


def test_gripper_refuses_what_it_cannot_send(motion, hub, io):
    open_gate(hub)
    for args, text in (({"position": 1.5}, "0 ~ 1"), ({"position": NAN}, "finite"), ({}, "position is required"),
                       ({"side": "middle", "position": 0.5}, "left, right or both")):
        result = motion.dispatch("gripper", "set", args)
        assert result["state"] == "error" and text in result["message"], args
    assert io.calls == []


# ── interrupts reach the cards that have something to stop ────────────────────────────────────

def test_interrupt_hooks_call_stop_on_the_cards_that_have_motion_in_flight(motion_module, config, BUNDLED_LIMITS):
    """Core's emergency interrupt (on_interrupt_motion) and full interrupt must stop the base, the
    arms and a head home. These hooks were missing before confirmation and the switch went away."""
    tools = _tools(motion_module, config, BUNDLED_LIMITS)
    for name in ("base_drive", "arm_control", "head"):
        hooks = tools[name]["inputSchema"]["x-hooks"]
        assert hooks == {"on_interrupt_motion": {"action": "stop"}, "on_interrupt_all": {"action": "stop"}}, name
        assert "stop" in tools[name]["inputSchema"]["properties"]["action"]["enum"], "the hook's action exists"
    for name in ("gripper", "lifting"):
        assert "x-hooks" not in tools[name]["inputSchema"], "one position command each: nothing in flight to stop"


def test_a_hook_call_really_stops_the_drive(device, hub, motion, config, io):
    """What Core sends for the hook: the hook's action through the normal dispatch."""
    from common.vendor_runtime import DriverBundle
    open_gate(hub)
    bundle = DriverBundle([device.PantherPlugin(hub, motion=motion, config=config)])
    assert bundle.dispatch("base_drive", {"action": "stop"})["state"] == "idle"
    assert io.named("twist")[-5:] == [(0.0, 0.0, 0.0)] * 5


def test_completion_is_declared_for_every_action_that_runs_in_a_worker(motion_module, config, BUNDLED_LIMITS):
    tools = {n: t["inputSchema"] for n, t in _tools(motion_module, config, BUNDLED_LIMITS).items()}
    assert tools["base_drive"]["x-completion"] == {"actions": ["move"], "timeout": 13}
    assert tools["arm_control"]["x-completion"] == {"actions": ["move", "home"], "timeout": 35}
    assert tools["head"]["x-completion"] == {"actions": ["home"], "timeout": 30}
    for name in ("gripper", "lifting"):
        assert "x-completion" not in tools[name]
    assert tools["base_drive"]["x-resource"] == "base" and set(tools["arm_control"]["x-resource"]) == {"arm_l", "arm_r"}
    assert tools["head"]["x-resource"] == "head" and tools["lifting"]["x-resource"] == "waist"


def test_the_head_home_timeout_covers_its_longest_possible_run(motion_module, config, BUNDLED_LIMITS):
    """The furthest a head joint can be from 0 is its limit; at 0.35 rad/s that must fit in the timeout."""
    furthest = max(abs(v) for j in ("joint_head0", "joint_head1") for v in BUNDLED_LIMITS[j])
    timeout = _tools(motion_module, config, BUNDLED_LIMITS)["head"]["inputSchema"]["x-completion"]["timeout"]
    assert furthest / config["plugins"]["head"]["home_speed_rad_s"] < timeout

def test_the_readme_range_table_matches_the_ranges_the_cards_show(motion_module, config, BUNDLED_LIMITS):
    """The README restates the ranges for people who read it instead of the canvas; it must not drift."""
    readme = (DRIVER / "README.md").read_text(encoding="utf-8")
    props = {n: t["inputSchema"]["properties"] for n, t in _tools(motion_module, config, BUNDLED_LIMITS).items()}

    def shown(card, field):
        return f"{props[card][field]['minimum']:g} ~ {props[card][field]['maximum']:g}"

    for card, field, row in (("base_drive", "linear_x", "| `linear_x`, `linear_y` |"),
                             ("base_drive", "angular_z", "| `angular_z` |"),
                             ("base_drive", "duration_s", "| `duration_s` (base) |"),
                             ("gripper", "position", "| `position` (gripper) |"),
                             ("lifting", "position", "| `position` (lift) |"),
                             ("head", "head0_rad", "| `head0_rad` |"),
                             ("head", "head1_rad", "| `head1_rad` |")):
        line = next(l for l in readme.splitlines() if l.startswith(row))
        assert shown(card, field) in line, (row, shown(card, field), line)
    for field in ("la0", "ra7"):
        assert f"`{field}` {shown('arm_control', field)}" in readme, field
    assert "左臂 la0（rad，-1.046 ~ 0.697）" in readme, "the README quotes a real box title"

# ── review: a stop must not be followed by a motion publish ──────────────────────────────────

def test_a_stop_landing_between_the_generation_check_and_the_publish_is_not_overtaken(motion, hub, io, completions):
    """Review finding: the worker checked the generation, released the lock, then published. A stop that
    landed in that gap sent its zeros, and the worker's nonzero twist went out after them.

    Deterministic: the stop is run from inside the check itself, i.e. in exactly that gap."""
    open_gate(hub)
    original = motion._superseded
    landed = []

    def check_then_stop_lands(generation):
        answer = original(generation)
        if not landed and io.named("twist"):              # after the first twist, a stop arrives here
            landed.append(True)
            motion.stop("base_drive")
        return answer

    motion._superseded = check_then_stop_lands
    motion.dispatch("base_drive", "move", {**DRIVE, "duration_s": 3.0})
    sent = io.named("twist")
    last_motion = max(i for i, c in enumerate(sent) if c != (0.0, 0.0, 0.0))
    first_stop_zero = next(i for i, c in enumerate(sent) if i > 0 and c == (0.0, 0.0, 0.0))
    assert last_motion < first_stop_zero, f"a motion twist went out after the stop's zeros: {sent}"
    assert completions[-1][1] == "cancelled"


def test_no_motion_publish_can_follow_a_stop_on_real_threads(device, config, hub, io, motion_module):
    """Many real interleavings: after stop() has returned, no nonzero twist may be published."""
    import threading
    import time

    open_gate(hub)
    for attempt in range(40):
        io.calls.clear()
        motion = motion_module.Motion(config, io, hub, notify=lambda *a: None, sleep=lambda s: time.sleep(0.0005))
        stopped, late = threading.Event(), []
        original = io.publish_twist

        def twist(vx, vy, wz, _o=original):
            if stopped.is_set() and (vx, vy, wz) != (0.0, 0.0, 0.0):
                late.append((vx, vy, wz))
            _o(vx, vy, wz)

        io.publish_twist = twist
        motion.dispatch("base_drive", "move", {**DRIVE, "duration_s": 3.0})
        time.sleep(0.0003 * (attempt % 7))
        motion.stop("base_drive")
        stopped.set()
        time.sleep(0.01)
        motion.stop("base_drive")
        io.publish_twist = original
        assert late == [], f"attempt {attempt}: motion published after stop returned: {late}"