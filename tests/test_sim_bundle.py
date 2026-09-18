"""Packaging: the bundle assembles, and its four descriptions agree.

`driver.yaml`'s card list is hand-synced with `config.yaml`'s plugin switches,
exactly as the other bundles do it — which means it drifts unless something
checks. The marketplace listing is built from `driver.yaml`, so drift shows up as
a card that either never appears or appears and does nothing.

This file also runs `scripts/check_service_yml.py` from pytest. The repo has no
CI at all, so a lint nobody runs is a lint that does not exist; invoking it here
puts the DDS isolation contract into the one suite people do run.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_bundle.py -q
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

BUNDLE = ROOT / "simulator" / "generic"

from common.vendor_runtime import DriverBundle  # noqa: E402
from simulator.generic.plugins import build_plugins  # noqa: E402


def load(name: str) -> dict:
    with (BUNDLE / name).open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope="module")
def config() -> dict:
    return load("config.yaml")


@pytest.fixture(scope="module")
def bundle(config):
    built = DriverBundle(build_plugins(config, "sim", None))
    yield built
    built.stop_all()


# ── the four descriptions agree ──────────────────────────────────────────────

def test_driver_yaml_cards_match_the_tools_actually_served(config, bundle):
    declared = {card["name"] for card in load("driver.yaml")["cards"]}
    served = {tool["name"] for tool in bundle.get_all_tools()}

    assert declared == served, (
        f"declared but not served: {sorted(declared - served)}; "
        f"served but not declared: {sorted(served - declared)}")


def test_driver_yaml_card_types_match_the_tool_types(bundle):
    """A `sensor` listed as an `actuator` changes whether the barrier exempts it
    and whether a `viewer` peer can reach it."""
    declared = {card["name"]: card["type"] for card in load("driver.yaml")["cards"]}
    for tool in bundle.get_all_tools():
        assert declared[tool["name"]] == tool["type"], f"{tool['name']} type mismatch"


def test_config_plugin_switches_cover_every_card(config):
    declared = {card["name"] for card in load("driver.yaml")["cards"]}
    assert set(config["plugins"]) == declared


def test_ports_agree_across_config_driver_yaml_and_dockerfile(config):
    driver = load("driver.yaml")
    dockerfile = (BUNDLE / "Dockerfile").read_text(encoding="utf-8")

    assert config["mcp_port"] == driver["port"] == 15711
    assert driver["mcp_url"] == "http://localhost:15711/mcp"
    assert "EXPOSE 15711" in dockerfile


def test_the_bundle_sits_two_levels_deep():
    """Three repo-wide checks glob `*/*/` — check_service_yml.py, the Dockerfile
    COPY test and the image tag in build.sh. A one-level bundle is skipped by all
    three *and still builds*, because build.sh also globs `*/driver.yaml`. That
    combination is the worst case: it looks compliant."""
    assert (BUNDLE / "driver.yaml").relative_to(ROOT).parts[:2] == ("simulator", "generic")
    assert load("driver.yaml")["hardware_provider"] == "simulator"


def test_namespace_is_not_the_hostname(config):
    """`vendor_runtime.resolve_namespace` falls back to socket.gethostname(),
    which on a rig already running a real driver would put the simulator's topics
    on top of the robot's."""
    assert config["ros_namespace"] == "sim"


# ── deployment contract ──────────────────────────────────────────────────────

def test_service_yml_passes_the_dds_isolation_lint():
    """The repo has no CI, so a lint nobody runs is a lint that does not exist."""
    result = subprocess.run([sys.executable, str(ROOT / "scripts" / "check_service_yml.py"),
                             "simulator/generic"],
                            cwd=ROOT, capture_output=True, text=True)

    assert result.returncode == 0, result.stdout + result.stderr


def test_service_yml_claims_no_hardware():
    """A simulator that reaches outside its container has stopped being one."""
    service = load("deploy/service.yml")["simulator-generic"]

    assert "privileged" not in service
    assert "device_cgroup_rules" not in service
    assert not any(str(v).startswith("/dev") for v in service.get("volumes", []))


def test_service_yml_records_what_a_run_was_measured_against():
    """A score with no configuration attached is noise."""
    environment = load("deploy/service.yml")["simulator-generic"]["environment"]
    keys = {entry.split("=", 1)[0] for entry in environment}

    assert {"SIM_TIER", "SIM_LLM_MODEL", "IMAGE_TAG", "GIT_SHA"} <= keys


def test_scenarios_can_be_added_without_rebuilding_the_image():
    service = load("deploy/service.yml")["simulator-generic"]
    mounts = [str(v) for v in service["volumes"]]

    assert any("/opt/phanthy-motus/data/sim" in mount for mount in mounts)
    assert any(entry.startswith("SIM_SCENARIO_DIR=") for entry in service["environment"])


def test_dockerfile_stays_thin():
    """Every one of these exists in a vendor bundle and none of it is needed
    here — there is no hardware to talk to."""
    dockerfile = (BUNDLE / "Dockerfile").read_text(encoding="utf-8").lower()
    for unwanted in ("colcon", "build-essential", "cmake", "espeak", "alsa-utils",
                     "sshpass", "opencv", "cyclonedds"):
        line = next((l for l in dockerfile.splitlines()
                     if unwanted in l and not l.strip().startswith("#")), None)
        assert line is None, f"Dockerfile pulls in {unwanted}: {line}"


# ── it actually boots ────────────────────────────────────────────────────────

def test_build_plugins_works_without_ros(bundle):
    """Everything above the publisher is ROS-free, so the whole schema and
    dispatch surface can be exercised on a laptop."""
    tools = bundle.get_all_tools()

    assert len(tools) == 15
    assert {"nav", "tts", "loco", "map", "sim_scenario", "sim_report"} <= {t["name"] for t in tools}


def test_the_default_scenario_is_loaded_at_boot(bundle):
    state = bundle.dispatch("sim_scenario", {"action": "read"})

    assert state["scenario"] == "exhibition_tour"
    assert state["waypoints"][0] == "入口"


def test_waypoints_reach_both_the_navigator_and_the_map(bundle):
    waypoints = bundle.dispatch("nav", {"action": "list_waypoints"})["waypoints"]

    assert [w["name"] for w in waypoints] == ["入口", "一号展区", "洗手间", "二号展区", "三号展区"]


def test_every_card_answers_info_or_declines_cleanly(bundle):
    """start-project is strict: one card that fails start rolls the whole project
    back and the robot comes up with nothing running.

    `resource` cards are exempt, and not by choice: `DriverBundle.dispatch` does
    not pop an `action` for that kind, so a resource tool is called by its own
    name and cannot tell a lifecycle verb from a request for the resource. They
    are checked below for returning their payload instead.
    """
    from common import lifecycle

    for tool in bundle.get_all_tools():
        if tool["type"] == "resource":
            continue
        result = bundle.dispatch(tool["name"], {"action": "info"})
        assert isinstance(result, dict)
        assert "state" in result or lifecycle.is_declined(result), f"{tool['name']}: {result}"


def test_resource_cards_return_their_resource(bundle):
    model = bundle.dispatch("model", {})
    report = bundle.dispatch("sim_report", {})

    assert model["joint_names"] == ["head_yaw", "head_pitch"]
    for name in model["joint_names"]:
        # The skeleton renderer matches by name; one mismatch draws nothing and
        # reports nothing about why.
        assert f'<joint name="{name}"' in model["urdf"]
    assert report["scenario"] == "exhibition_tour"


def test_the_shipped_scenario_loads_with_no_warnings(bundle):
    result = bundle.dispatch("sim_scenario", {"action": "load", "scenario": "exhibition_tour"})

    assert result["warnings"] == [], result["warnings"]
