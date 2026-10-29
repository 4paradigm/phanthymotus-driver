"""Cross-check the shipped four-card configuration and deployment metadata."""
from pathlib import Path
import sys
from types import SimpleNamespace

import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

from can_client import validate_position_vector
from adaptive_grasp_device import AdaptiveGraspPlugin
from hand_device import build_plugins
from common.vendor_runtime import resolve_namespace


def load(name):
    return yaml.safe_load((DRIVER / name).read_text())


def test_manifest_matches_enabled_runtime_cards_without_opening_can():
    config = load("config.yaml")
    manifest = load("driver.yaml")
    plugins = build_plugins(config, "", None)  # Do not start lifecycle workers.
    definitions = [p.get_tool() for p in plugins]
    actual = {d["name"]: d["type"] for d in definitions}
    assert actual == {d["name"]: d["type"] for d in manifest["cards"]}
    assert actual == {"hand": "actuator", "hand_gesture": "actuator",
                      "hand_grasp": "actuator", "hand_tactile": "sensor"}
    assert config["plugins"]["hand"]["enabled"]  # Sensor shares this controller.


def test_shipped_grasp_config_uses_adaptive_contact_settings():
    config = load("config.yaml")["plugins"]["hand_grasp"]
    plugin = AdaptiveGraspPlugin(None, config)
    assert validate_position_vector(plugin.prep_pose) == plugin.prep_pose
    assert validate_position_vector(plugin.close_limit_pose) == plugin.close_limit_pose
    assert plugin.tactile_threshold == 15
    assert plugin.step_size == 5
    assert set(plugin.get_tool()["inputSchema"]["x-completion"]["actions"]) == {"adaptive_grasp", "release"}


def test_can_port_and_dds_settings_agree_across_deployment_files():
    config = load("config.yaml")
    manifest = load("driver.yaml")
    service = next(iter(load("deploy/service.yml").values()))
    assert config["mcp_port"] == manifest["port"] == 15734
    assert manifest["mcp_url"].endswith(":" + str(config["mcp_port"]) + "/mcp")
    assert config["can_channel"] == "can0" and config["baudrate"] == 1_000_000
    assert config["left_can_id"] == 0x28 and config["right_can_id"] == 0x27
    assert config["ros"]["core_domain_id"] == config["ros"]["robot_domain_id"] == 42
    assert "ROS_DOMAIN_ID=42" in service["environment"]
    assert service["container_name"] == "embodied-linkerhand-l20"
    assert service["network_mode"] == "host"


def test_tactile_defaults_produce_the_declared_sensor_topic():
    config = load("config.yaml")
    settings = config["plugins"]["hand_tactile"]
    assert 1 <= settings["poll_hz"] <= 50
    assert isinstance(settings["auto_start"], bool)
    assert settings["matrix_request_interval_s"] > 0
    assert 1 <= settings["contact_threshold"] <= 255
    plugins = build_plugins(config, "", None)
    sensor = next(p for p in plugins if p.PREFIX == "hand_tactile")
    assert sensor.topic_out() == [
        {"topic": f"/hand/{config['side']}/tactile", "format": "sensor/tactile"}]
    namespace = resolve_namespace(config)
    runtime_plugins = build_plugins(config, namespace, None)
    runtime_sensor = next(p for p in runtime_plugins if p.PREFIX == "hand_tactile")
    assert runtime_sensor.topic_out() == [
        {"topic": f"/{namespace}/hand/{config['side']}/tactile", "format": "sensor/tactile"}]


def test_tactile_uses_core_renderer_without_image_dependencies():
    assert 'tactile_heatmap.py' not in (DRIVER / 'Dockerfile').read_text()
    requirements = (DRIVER / 'requirements.txt').read_text().lower()
    assert 'pillow' not in requirements and 'numpy' not in requirements


def test_actuator_shared_can_change_does_not_rewrite_sensor_target_or_topics():
    config = load("config.yaml")
    plugins = build_plugins(config, "topic_test", None)
    hand = next(p for p in plugins if p.PREFIX == "hand")
    sensor = next(p for p in plugins if p.PREFIX == "hand_tactile")
    original_topics = sensor.topic_out("right", "can0")
    other_adapter_topics = sensor.topic_out("right", "can1")

    # Hand settings are shared by the three actuator cards. Tactile retains
    # its own initial default and per-instance adapter selection.
    hand.dispatch("config", {"can_channel": "can1"})

    assert hand.config["can_channel"] == "can1"
    assert sensor.default_channel == "can0"
    assert sensor._resolve_target({"bound_side": "right"}) == ("right", "can0")
    assert sensor.topic_out("right", "can0") == original_topics
    assert sensor.topic_out("right", "can1") == other_adapter_topics
    assert original_topics != other_adapter_topics
