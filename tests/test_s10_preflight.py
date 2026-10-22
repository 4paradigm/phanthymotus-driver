"""Offline, noninitializing ROS message contract checks for S10 deployment."""

from __future__ import annotations

import copy
import hashlib
import json
import re
import sys
from collections import Counter
from pathlib import Path
from types import ModuleType

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from deep_robotics.s10_pro import preflight, reader  # noqa: E402


def primitive(name):
    return {"kind": "primitive", "name": name}


def message(type_name):
    return {"kind": "message", "type": type_name}


def string(*, wide=False, max_size=None):
    return {"kind": "string", "wide": wide, "max_size": max_size}


def array_of(value, size):
    return {"kind": "array", "size": size, "value": value}


def sequence_of(value, max_size=None):
    return {"kind": "sequence", "max_size": max_size, "value": value}


@pytest.mark.parametrize("text,expected", [
    ("float", primitive("float32")),
    ("float32", primitive("float32")),
    ("double", primitive("float64")),
    ("float64", primitive("float64")),
    ("bool", primitive("bool")),
    ("boolean", primitive("bool")),
    ("uint8", primitive("uint8")),
    ("int32", primitive("int32")),
    ("string", string()),
    ("wstring", string(wide=True)),
    ("string<=24", string(max_size=24)),
    ("wstring<=12", string(wide=True, max_size=12)),
    ("sensor_msgs/msg/Imu", message("sensor_msgs/msg/Imu")),
    ("sensor_msgs/Imu", message("sensor_msgs/msg/Imu")),
    ("float[3]", array_of(primitive("float32"), 3)),
    ("array<double, 4>", array_of(primitive("float64"), 4)),
    ("uint8[<=32]", sequence_of(primitive("uint8"), 32)),
    ("sequence<uint8>", sequence_of(primitive("uint8"))),
    ("sequence<double, 12>", sequence_of(primitive("float64"), 12)),
    ("bounded_sequence<float, 8>", sequence_of(primitive("float32"), 8)),
    ("sequence<string<=20, 3>", sequence_of(string(max_size=20), 3)),
    ("sequence<array<float, 3>, 4>", sequence_of(array_of(primitive("float32"), 3), 4)),
    ("array<sequence<robot_msgs/Pose, 5>, 2>",
     array_of(sequence_of(message("robot_msgs/msg/Pose"), 5), 2)),
    ("sequence<sequence<uint8, 8>, 2>",
     sequence_of(sequence_of(primitive("uint8"), 8), 2)),
])
def test_parse_field_types_into_canonical_descriptors(text, expected):
    assert preflight.parse_field_type(text) == expected


def test_relative_message_names_resolve_with_owner_package_through_containers():
    assert preflight.parse_field_type("Pose", owner_package="robot_msgs") == message("robot_msgs/msg/Pose")
    assert preflight.parse_field_type("sequence<Pose[2], 5>", owner_package="robot_msgs") == (
        sequence_of(array_of(message("robot_msgs/msg/Pose"), 2), 5))


@pytest.mark.parametrize("text", ["", "sequence<>", "sequence<float", "array<float>",
    "bounded_sequence<float>", "sequence<float, nope>", "float[-3]", "string<=oops",
    "int32[", "array<float,2,3>", "sequence<float>trailing", "pkg/srv/Command"])
def test_malformed_type_expressions_fail_without_guessing(text):
    with pytest.raises(ValueError):
        preflight.parse_field_type(text)


def fake_type(fields):
    """The resolver returns classes: constructing one is forbidden."""
    class Message:
        def __init__(self, *args, **kwargs):
            pytest.fail("preflight must inspect message classes without instantiation")

        @classmethod
        def get_fields_and_field_types(cls):
            return dict(fields)

    return Message


def resolver_for(definitions):
    calls = Counter()

    def resolve(type_name):
        calls[type_name] += 1
        assert type_name in definitions, f"unexpected type lookup {type_name}"
        return fake_type(definitions[type_name])

    return resolve, calls


def sample_specs():
    return {
        "imu": {"topic": "/IMU", "type": "robot_msgs/msg/Sample"},
        "imu_copy": {"topic": "/IMU_103", "type": "robot_msgs/msg/Sample"},
    }


def sample_definitions():
    return {
        "robot_msgs/msg/Sample": {
            "stamp": "builtin_interfaces/Time",
            "position": "robot_msgs/Vector3",
            "history": "sequence<robot_msgs/Vector3, 5>",
            "acceleration": "array<float, 3>",
            "labels": "sequence<string<=20, 2>",
        },
        "robot_msgs/msg/Vector3": {"x": "double", "y": "double", "z": "double"},
        "builtin_interfaces/msg/Time": {"sec": "int32", "nanosec": "uint32"},
    }


def collect(definitions=None, specs=None):
    getter, calls = resolver_for(definitions if definitions is not None else sample_definitions())
    return preflight.collect_contract(sample_specs() if specs is None else specs, getter), calls


def test_collect_recurses_nested_types_and_memoizes_shared_dependencies_without_instances():
    specs = sample_specs()
    contract, calls = collect(specs=specs)
    assert contract["schema"] == "s10_pro.ros_contract/1"
    assert contract["sources"] == specs
    assert set(contract["types"]) == {
        "robot_msgs/msg/Sample", "robot_msgs/msg/Vector3", "builtin_interfaces/msg/Time"}
    assert calls == {key: 1 for key in contract["types"]}
    fields = contract["types"]["robot_msgs/msg/Sample"]["fields"]
    assert [entry["name"] for entry in fields] == ["stamp", "position", "history", "acceleration", "labels"]
    assert fields[2] == {"name": "history", "type": sequence_of(message("robot_msgs/msg/Vector3"), 5)}
    assert fields[3] == {"name": "acceleration", "type": array_of(primitive("float32"), 3)}
    assert fields[4] == {"name": "labels", "type": sequence_of(string(max_size=20), 2)}
    assert re.fullmatch(r"[0-9a-f]{64}", contract["fingerprint_sha256"])
    # The contract can be moved between machines without generated Python types.
    assert json.loads(json.dumps(contract, allow_nan=False)) == contract


def test_collect_canonicalizes_declared_source_and_nested_message_aliases():
    specs = {"pose": {"topic": "/pose", "type": "robot_msgs/Sample"}}
    defs = {"robot_msgs/msg/Sample": {"value": "Vector3"},
            "robot_msgs/msg/Vector3": {"x": "double"}}
    contract, calls = collect(defs, specs)
    assert contract["sources"]["pose"]["type"] == "robot_msgs/msg/Sample"
    assert contract["types"]["robot_msgs/msg/Sample"]["fields"] == [
        {"name": "value", "type": message("robot_msgs/msg/Vector3")}]
    assert calls == {"robot_msgs/msg/Sample": 1, "robot_msgs/msg/Vector3": 1}


def test_fingerprint_is_stable_for_canonical_aliases_and_changes_with_nested_layout():
    expected, _ = collect()
    aliases = sample_definitions()
    aliases["robot_msgs/msg/Vector3"] = {"x": "float64", "y": "float64", "z": "float64"}
    aliases["robot_msgs/msg/Sample"]["acceleration"] = "float32[3]"
    actual, _ = collect(aliases)
    assert actual["fingerprint_sha256"] == expected["fingerprint_sha256"]
    changed = sample_definitions()
    changed["robot_msgs/msg/Vector3"]["z"] = "float"
    different, _ = collect(changed)
    assert different["fingerprint_sha256"] != expected["fingerprint_sha256"]


def test_comparison_ignores_host_package_versions_fingerprint_and_topic_remapping():
    expected, _ = collect()
    actual = copy.deepcopy(expected)
    expected.update(host={"architecture": "aarch64"}, packages={"drdds": "1.0.23"})
    actual.update(host={"architecture": "x86_64"}, packages={"drdds": "9.9.9"})
    actual["fingerprint_sha256"] = hashlib.sha256(b"different-host").hexdigest()
    actual["sources"]["imu"]["topic"] = "/remapped/imu"
    actual["types"] = dict(reversed(list(actual["types"].items())))
    result = preflight.compare_contracts(actual, expected)
    assert result == {"compatible": True, "mismatches": []}


def test_incomplete_reference_cannot_hide_a_nested_definition_change():
    actual, _ = collect()
    reference = copy.deepcopy(actual)
    del reference["types"]["robot_msgs/msg/Vector3"]
    result = preflight.compare_contracts(actual, reference)
    assert result == {"compatible": False, "mismatches": [
        {"path": "$", "reason": "invalid_or_incomplete_contract"}]}


@pytest.mark.parametrize("invalid", [None, [], {}, {"schema": "unknown"},
    {"schema": "s10_pro.ros_contract/1", "sources": [], "types": {}}])
def test_malformed_reference_is_an_explained_incompatibility(invalid):
    actual, _ = collect()
    assert preflight.compare_contracts(actual, invalid)["compatible"] is False


def test_full_preflight_reports_can_be_compared_without_host_values():
    contract, _ = collect()
    report = {"schema": "s10_pro.preflight/1", "contract": contract,
              "host": {"private": "never part of a mismatch"}}
    assert preflight.compare_contracts(contract, report) == {"compatible": True, "mismatches": []}


def test_vendor_metadata_reads_only_a_plain_version_token(tmp_path):
    path = tmp_path / "version"
    assert preflight._version_metadata(path) == {"status": "missing"}
    path.mkdir()
    assert preflight._version_metadata(path) == {"status": "not_regular_file"}
    path.rmdir()
    path.write_text("v1.2.3+build.4\n", encoding="utf-8")
    assert preflight._version_metadata(path) == {"status": "read", "version": "v1.2.3+build.4"}
    path.write_text("version=1.2.3\npassword=never-disclose\n", encoding="utf-8")
    result = preflight._version_metadata(path)
    assert result == {"status": "unrecognized"}
    assert "never-disclose" not in json.dumps(result)


def test_unavailable_ros_import_reports_only_the_exception_class(monkeypatch, tmp_path, capsys):
    def missing():
        raise ModuleNotFoundError("private path and credential must not appear")

    monkeypatch.setattr(preflight, "_message_getter", missing)
    assert preflight.main(["--config", str(write_config(tmp_path)), "--public-contract"]) == 1
    captured = capsys.readouterr()
    assert json.loads(captured.out) == {"schema": "s10_pro.preflight/1", "status": "error",
                                            "error_type": "ModuleNotFoundError"}
    assert "private" not in captured.out + captured.err


@pytest.mark.parametrize("change", ["source_missing", "source_added", "source_type", "field_name",
                                      "nested_type", "field_order", "array_size", "sequence_bound", "string_bound"])
def test_comparison_rejects_source_and_recursive_field_contract_changes(change):
    expected, _ = collect()
    actual = copy.deepcopy(expected)
    fields = actual["types"]["robot_msgs/msg/Sample"]["fields"]
    if change == "source_missing":
        del actual["sources"]["imu_copy"]
    elif change == "source_added":
        actual["sources"]["extra"] = {"topic": "/extra", "type": "robot_msgs/msg/Sample"}
    elif change == "source_type":
        actual["sources"]["imu"]["type"] = "robot_msgs/msg/Vector3"
    elif change == "field_name":
        fields[0]["name"] = "timestamp"
    elif change == "nested_type":
        actual["types"]["robot_msgs/msg/Vector3"]["fields"][2]["type"] = primitive("float32")
    elif change == "field_order":
        fields[0], fields[1] = fields[1], fields[0]
    elif change == "array_size":
        fields[3]["type"]["size"] = 4
    elif change == "sequence_bound":
        fields[2]["type"]["max_size"] = 6
    elif change == "string_bound":
        fields[4]["type"]["value"]["max_size"] = 21
    result = preflight.compare_contracts(actual, expected)
    assert result["compatible"] is False
    assert result["mismatches"], f"{change} was not explained"


def install_no_ros_initialization_trap(monkeypatch):
    module = ModuleType("rclpy")

    def forbidden(*args, **kwargs):
        pytest.fail("preflight must not initialize ROS, create nodes, or call vendor startup code")

    module.init = forbidden
    module.create_node = forbidden
    monkeypatch.setitem(sys.modules, "rclpy", module)


def cli_fake_resolver():
    definitions = {entry["type"]: {"sample": "float"} for entry in reader.DEFAULT_SOURCES.values()}
    return resolver_for(definitions)


def write_config(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("topics:\n  imu: /custom/IMU\n", encoding="utf-8")
    return path


def test_cli_public_contract_has_no_host_metadata_or_ros_initialization(monkeypatch, tmp_path, capsys):
    install_no_ros_initialization_trap(monkeypatch)
    getter, calls = cli_fake_resolver()
    monkeypatch.setattr(preflight, "_message_getter", lambda: getter)
    monkeypatch.setenv("PRELIGHT_PRIVATE_CREDENTIAL", "must-never-be-disclosed")
    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    monkeypatch.setattr(preflight, "host_metadata", lambda: pytest.fail("public mode must not gather host metadata"))
    code = preflight.main(["--config", str(write_config(tmp_path)), "--public-contract"])
    captured = capsys.readouterr()
    assert code == 0
    payload = json.loads(captured.out)
    assert set(payload) == {"schema", "sources", "types", "fingerprint_sha256"}
    assert payload["sources"]["imu"]["topic"] == "/custom/IMU"
    assert "must-never-be-disclosed" not in captured.out + captured.err
    assert all(count == 1 for count in calls.values())


def test_cli_comparison_returns_exit_two_for_incompatible_nested_fields(monkeypatch, tmp_path, capsys):
    install_no_ros_initialization_trap(monkeypatch)
    getter, _ = cli_fake_resolver()
    config = reader.load_config(write_config(tmp_path))
    expected = preflight.collect_contract(reader.source_specs(config), getter)
    imu_type = expected["sources"]["imu"]["type"]
    expected["types"][imu_type]["fields"][0]["type"] = primitive("float64")
    contract_path = tmp_path / "contract.json"
    contract_path.write_text(json.dumps(expected), encoding="utf-8")
    monkeypatch.setattr(preflight, "_message_getter", lambda: getter)
    code = preflight.main(["--config", str(tmp_path / "config.yaml"),
                           "--compare-contract", str(contract_path)])
    captured = capsys.readouterr()
    assert code == 2
    payload = json.loads(captured.out)
    assert payload["schema"] == "s10_pro.preflight/1"
    assert payload["comparison"]["compatible"] is False
    assert payload["comparison"]["mismatches"]


def test_cli_comparison_accepts_version_and_topic_only_differences(monkeypatch, tmp_path, capsys):
    install_no_ros_initialization_trap(monkeypatch)
    getter, _ = cli_fake_resolver()
    config_path = write_config(tmp_path)
    expected = preflight.collect_contract(reader.source_specs(reader.load_config(config_path)), getter)
    expected["sources"]["imu"]["topic"] = "/other/IMU"
    expected["packages"] = {"drdds": "some-other-release"}
    contract_path = tmp_path / "contract.json"
    contract_path.write_text(json.dumps(expected), encoding="utf-8")
    monkeypatch.setattr(preflight, "_message_getter", lambda: getter)
    assert preflight.main(["--config", str(config_path), "--compare-contract", str(contract_path)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["comparison"]["compatible"] is True
    assert payload["comparison"]["mismatches"] == []


def test_cli_public_contract_comparison_remains_free_of_host_metadata(monkeypatch, tmp_path, capsys):
    getter, _ = cli_fake_resolver()
    config_path = write_config(tmp_path)
    expected = preflight.collect_contract(reader.source_specs(reader.load_config(config_path)), getter)
    contract_path = tmp_path / "contract.json"
    contract_path.write_text(json.dumps(expected), encoding="utf-8")
    monkeypatch.setattr(preflight, "_message_getter", lambda: getter)
    monkeypatch.setattr(preflight, "host_metadata", lambda: pytest.fail("public mode must not gather host metadata"))
    assert preflight.main(["--config", str(config_path), "--public-contract",
                           "--compare-contract", str(contract_path)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "s10_pro.ros_contract/1"
    assert payload["comparison"] == {"compatible": True, "mismatches": []}
    assert "host" not in payload
    assert "contract" not in payload


def test_host_metadata_uses_only_public_environment_allowlist(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("host metadata must not run subprocesses or vendor startup scripts")

    monkeypatch.setattr("subprocess.Popen", forbidden)
    monkeypatch.setattr("subprocess.run", forbidden)
    monkeypatch.setattr("subprocess.check_output", forbidden)
    monkeypatch.setattr("os.system", forbidden)
    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    monkeypatch.setenv("SSH_AUTH_SOCK", "/private/ssh/socket")
    monkeypatch.setenv("S10_ADMIN_PASSWORD", "secret-probe-must-not-appear")
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/private/vendor/profile.xml")
    metadata = preflight.host_metadata()
    encoded = json.dumps(metadata, ensure_ascii=False)
    assert "jazzy" in encoded and "rmw_fastrtps_cpp" in encoded
    assert "secret-probe-must-not-appear" not in encoded
    assert "/private/ssh/socket" not in encoded
    assert "/private/vendor/profile.xml" not in encoded
