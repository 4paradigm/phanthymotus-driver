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
    assert cards["model"] == "resource"
    assert {t for n, t in cards.items() if n != "model"} == {"sensor"}, "first batch is read-only"
    assert driver["port"] == config["mcp_port"] == 15747
    assert driver["mcp_url"] == "http://localhost:15747/mcp"
    assert (driver["hardware_provider"], driver["hardware_model"]) == ("unixai", "panther")
    assert driver["build_context_extras"] == ["../../common"]


def test_port_is_not_used_by_another_driver():
    ports = {}
    for path in ROOT.glob("*/*/driver.yaml"):
        ports.setdefault(yaml.safe_load(path.read_text(encoding="utf-8"))["port"], []).append(path.parent.name)
    assert ports[15747] == ["panther"]


def test_tools_match_the_driver_manifest_and_use_renderable_formats(device, hub):
    tools = device.PantherPlugin(hub).get_tools()
    assert [t["name"] for t in tools] == list(device.CARD_ORDER)
    for definition in tools:
        assert definition["description"] and definition["inputSchema"]["type"] == "object"
        assert definition["type"] in ("sensor", "resource")
        assert definition["multiInstance"] is False
        for output in definition.get("topic_out", []):
            assert output["format"] in ALLOWED_FORMATS, "sensor/lidar* does not render from the topic bus"
            assert output["topic"].startswith("/panther/")
    json.dumps(tools, allow_nan=False)


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
    assert "privileged" not in body and "devices" not in body, "a read-only driver needs no hardware access"
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
    for name in ("main.py", "device.py", "panther_payloads.py", "panther_bridge.py", "config.yaml", "driver.yaml"):
        assert name in copied
    assert "COPY common/ /work/common/" in dockerfile
    assert "rmw-cyclonedds-cpp" in dockerfile and "rmw-fastrtps-cpp" in dockerfile
    # The ros-base image has the ROS install and key but no ROS apt source; the first bot build
    # failed with "ros-humble-rmw-cyclonedds-cpp has no installation candidate" without this.
    assert dockerfile.index("ros2-archive-keyring.gpg") < dockerfile.index("apt-get update")
    assert "/etc/apt/sources.list.d/ros2.list" in dockerfile
    assert "ros-base@sha256:" in dockerfile.splitlines()[0], "pinned like the other ROS drivers"
    assert "print('panther imports ok'" in dockerfile, "build fails here, not on the robot"
    assert "numpy==1.26.4" in (DRIVER / "requirements.txt").read_text(encoding="utf-8")


def test_main_is_guarded_so_spawned_children_do_not_start_a_second_driver():
    source = (DRIVER / "main.py").read_text(encoding="utf-8")
    assert 'if __name__ == "__main__":' in source
    assert source.index("logsafe.install()") < source.index("from device import")


def test_driver_never_writes_to_the_robot():
    """Read-only is the whole contract of this batch; a source-level tripwire for anyone adding a card."""
    source = "\n".join(p.read_text(encoding="utf-8") for p in DRIVER.glob("*.py"))
    for forbidden in ("create_publisher(", "create_client(", "create_service(", "call_async(", "create_action",
                      "geometry_msgs", "trajectory_msgs", "Twist", "SetBool", "switch_controller",
                      "subprocess", "os.system", "os.dup(1)", "dup2("):
        code = [line for line in source.splitlines() if forbidden in line and not line.strip().startswith(("#", '"""'))]
        # panther_bridge legitimately publishes on domain 42 (Agent Core), nothing on the robot domain.
        code = [line for line in code if "node.create_publisher(msg_type, topic, qos)" not in line]
        assert not code, f"{forbidden!r}: {code}"


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


def test_health_payload_reports_worst_level_and_per_status_age(payloads):
    state = _diag_state(payloads, diagnostics(status("b", level=b"\x01", message="warm"), status("a")))
    result = strict(payloads.health_payload(state, 3_000, 5_000))
    assert [s["name"] for s in result["statuses"]] == ["a", "b"], "sorted: the robot's order changes per message"
    assert result["worst_level"] == 1 and result["worst_level_name"] == "warn"
    assert result["statuses"][0]["age_ms"] == 2_000 and result["statuses"][0]["stale"] is False


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


def test_gripper_and_lift_are_merged_with_their_age(hub, sink, clock):
    hub.on_gripper_left(joint_state(["joint_lf"], [0.023], [0.0], [-32.763]))
    hub.on_lift(joint_state(["body_joint"], [0.61], [-0.0], [0.0]))
    clock.advance(0.2)
    hub.tick()
    state = sink.last("/panther/gripper_lift_state")
    assert state["left_gripper"]["position"] == 0.023 and state["left_gripper"]["effort"] == -32.763
    assert state["lift"]["position"] == 0.61, "not clamped to the URDF's stale 0.6 limit"
    assert state["right_gripper"] is None, "a joint that never reported is null, not zero"
    assert state["left_gripper"]["age_ms"] == 200


def test_gripper_lift_is_silent_until_something_arrives(hub, sink, clock):
    clock.advance(1)
    hub.tick()
    assert sink.count("/panther/gripper_lift_state") == 0


def test_health_and_estop_publish_on_change_and_on_a_timer(hub, sink, clock):
    hub.on_diagnostics(normal_diagnostics())
    assert sink.last("/panther/health")["worst_level"] == 0
    assert sink.last("/panther/estop")["stop_button_status"] == "正常"
    published = sink.count("/panther/estop")

    pressed = diagnostics(status("diagnostic_broadcaster: Emergency stop", level=b"\x02",
                                 message="急停已按下", stop_button_status="按下"))
    clock.advance(0.05)
    hub.on_diagnostics(pressed)
    assert sink.count("/panther/estop") == published + 1, "a change is published at once, not on the next second"
    assert sink.last("/panther/estop")["level"] == 2
    assert sink.last("/panther/health")["worst_level"] == 2


def test_estop_goes_stale_when_diagnostics_stop(hub, sink, clock):
    hub.on_diagnostics(normal_diagnostics())
    assert sink.last("/panther/estop")["stale"] is False
    for _ in range(60):                                     # 6 s of timer ticks and no diagnostics
        clock.advance(0.1)
        hub.tick()
    latest = sink.last("/panther/estop")
    assert latest["stale"] is True and latest["stop_button_status"] == "正常", "old text kept, flagged as old"
    assert latest["age_ms"] >= 3_000


def test_laser_json_always_publishes_and_the_view_waits_for_start(hub, sink):
    pytest.importorskip("cv2")
    hub.on_laser("front", scan())
    assert sink.count("/panther/laser_scan_front") == 1 and not sink.images
    hub.activate("laser_scan_front")
    hub.on_laser("front", scan())
    assert [i[1] for i in sink.images] == ["/panther/laser_scan_front_view"]
    assert sink.images[0][2] == "jpeg" and sink.images[0][0] == "views"


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
    assert hub.stream("battery")["errors"] == 1 and "RuntimeError" in hub.stream("battery")["last_error"]
    assert "handler failed" in capsys.readouterr().out
    clock.advance(0.2)
    hub.on_joint_state(joint_state(["joint_la0"], [0.1]))   # and the hub still works afterwards
    assert hub.stream("joints")["published"] == 1


def test_a_ten_hertz_timer_that_ticks_slightly_early_does_not_lose_samples(hub, sink, clock):
    hub.on_lift(joint_state(["body_joint"], [0.61]))
    for _ in range(100):
        clock.advance(0.0995)                               # jittery 10 Hz timer
        hub.tick()
    assert 99 <= sink.count("/panther/gripper_lift_state") <= 100


def test_non_finite_numbers_cannot_reach_the_wire(hub, sink, monkeypatch, device):
    monkeypatch.setattr(device, "battery_payload", lambda msg: {"percent": float("nan")})
    hub.on_battery(battery())
    assert sink.count("/panther/battery") == 0, "allow_nan=False rejects it before it is sent"
    assert hub.stream("battery")["errors"] == 1


def test_a_full_bridge_is_counted_not_blocked_on(device, config, clock):
    full = Sink(accept=False)
    hub = device.Hub(config, "panther", full, clock=lambda: clock.wall, monotonic=lambda: clock.mono)
    hub.on_battery(battery())
    assert hub.stream("battery") ["dropped"] == 1 and hub.stream("battery")["published"] == 0


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
    assert "orbbec_camera" in started["warning"]
    rear = plugin.dispatch("info", {"_tool_name": "laser_scan_rear"})
    assert rear["state"] == "idle" and "后雷达" in rear["warning"]
    never = plugin.dispatch("info", {"_tool_name": "battery"})
    assert "/battery_state_broadcaster/battery_state" in never["warning"]


def test_stale_feed_is_flagged_on_the_card(device, hub, clock):
    plugin = device.PantherPlugin(hub)
    hub.on_battery(battery())
    assert "warning" not in plugin.dispatch("info", {"_tool_name": "battery"})
    clock.advance(6)
    info = plugin.dispatch("info", {"_tool_name": "battery"})
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
    for card in device.CARD_ORDER[1:]:
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


def test_the_shared_bundle_runtime_can_drive_the_plugin(device, hub):
    from common.vendor_runtime import DriverBundle, jsonable
    bundle = DriverBundle([device.PantherPlugin(hub)])
    assert len(bundle.get_all_tools()) == 12
    for name, arguments in (("model", {}), ("battery", {"action": "start"}), ("estop", {"action": "info"}),
                            ("odom", {"action": "stop"})):
        result = bundle.dispatch(name, arguments)
        assert isinstance(result, dict)
        json.dumps(jsonable(result), allow_nan=False)
    assert bundle.dispatch("nope", {}) is None


def test_plugin_stop_releases_the_ros_node_and_the_bridge(device, hub, sink):
    closed = []
    plugin = device.PantherPlugin(hub, nodes=NS(close=lambda: closed.append("nodes")), sink=sink)
    plugin.stop()
    assert closed == ["nodes"] and sink.closed


# ── rclpy wiring (against a stand-in for rclpy) ──────────────────────────────

class _FakeRos:
    """Just enough of rclpy to check what PantherNodes subscribes to, and nothing else."""

    def __init__(self, monkeypatch):
        import types

        self.nodes, self.subscriptions, self.timers, self.groups = [], [], [], []
        fake = self

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

            def create_publisher(self, *args, **kwargs):
                raise AssertionError("the robot-domain node must never publish")

            create_client = create_service = create_publisher

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

        module("rclpy")
        module("rclpy.node", Node=Node)
        module("rclpy.qos", QoSProfile=QoSProfile,
               HistoryPolicy=NS(KEEP_LAST="keep_last"),
               ReliabilityPolicy=NS(BEST_EFFORT="best_effort", RELIABLE="reliable"),
               DurabilityPolicy=NS(VOLATILE="volatile"))
        module("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=Group)
        self.sensor = message_types("BatteryState", "Image", "Imu", "JointState", "LaserScan")
        module("sensor_msgs"); module("sensor_msgs.msg", **self.sensor)
        self.nav = message_types("Odometry")
        module("nav_msgs"); module("nav_msgs.msg", **self.nav)
        self.diag = message_types("DiagnosticArray")
        module("diagnostic_msgs"); module("diagnostic_msgs.msg", **self.diag)
        self.executor = NS(added=[], removed=[])
        self.executor.add_node = self.executor.added.append
        self.executor.remove_node = self.executor.removed.append
        self.ros2 = NS(ctx_robot="robot-context", executor_robot=self.executor)


def test_nodes_subscribe_read_only_to_exactly_the_configured_robot_topics(device, config, hub, monkeypatch):
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
    }
    assert {(s.type.__name__, s.topic) for s in fake.subscriptions} == wanted
    assert len(fake.subscriptions) == len(wanted), "one subscription per topic"
    for subscription in fake.subscriptions:
        # Best-effort is compatible with reliable and best-effort publishers alike, and cannot
        # back-pressure the robot's own controllers.
        assert subscription.qos.reliability == "best_effort"
        assert subscription.qos.durability == "volatile" and subscription.qos.depth >= 1

    (timer,) = fake.timers
    assert timer.period == 0.1 and timer.callback == hub.tick

    nodes.close()
    assert fake.executor.removed == [node] and node.destroyed


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
    by_topic["/lidar_front/scan"].callback(scan())
    by_topic["/lidar_rear/scan"].callback(scan(header=header(frame_id="link_back_radar")))
    by_topic["/left_gripper/joint_states"].callback(joint_state(["joint_lf"], [0.02]))
    by_topic["/right_gripper/joint_states"].callback(joint_state(["joint_rf"], [0.03]))
    by_topic["/lifting_controller/joint_states"].callback(joint_state(["body_joint"], [0.61]))
    hub.activate("camera_depth")
    by_topic["/camera/color/image_raw"].callback(image(2, 2, "rgb8", bytes(12)))
    by_topic["/camera/depth/image_raw"].callback(image(2, 2, "16UC1", bytes(8)))
    clock.advance(0.1)
    fake.timers[0].callback()

    for topic in ("/panther/joints", "/panther/battery", "/panther/imu", "/panther/state/odom", "/panther/health",
                  "/panther/estop", "/panther/laser_scan_front", "/panther/laser_scan_rear",
                  "/panther/gripper_lift_state"):
        assert sink.count(topic) >= 1, topic
    assert sink.last("/panther/laser_scan_rear")["frame_id"] == "link_back_radar"
    assert [i[1] for i in sink.images] == ["/panther/camera_depth"], "colour is not started, depth is"
    assert all(hub.stream(c)["errors"] == 0 for c in device.CARD_ORDER[1:]), \
        {c: hub.stream(c)["last_error"] for c in device.CARD_ORDER[1:]}


def test_every_topic_the_nodes_use_is_in_the_config(config):
    keys = {"joints", "battery", "imu", "odom", "gripper_left", "gripper_right", "lift", "diagnostics",
            "laser_front", "laser_rear", "camera_color", "camera_depth"}
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
