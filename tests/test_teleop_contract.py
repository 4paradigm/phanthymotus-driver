"""The same serialized fixtures must be admitted by both Driver consumers."""
import copy
import json
import pytest
from common.teleop_contract import (
    COMMAND_SCHEMA, FEEDBACK_SCHEMA, COMMAND_TOPIC, STATE_TOPIC, TRACKING_FRAME, command_from_input,
    topics, binding_from_topic, validate_input, validate_operation, validate_feedback,
)

NOW = 1_000_000_000
OPTIONS = dict(instance_id="pico_1", clock_id="boot-test", now_ns=NOW)


def input_frame():
    pose = {"tracked": True, "position": [0., 0., 1.],
            "orientation_xyzw": [0., 0., 0., 1.]}
    return {"schema": COMMAND_SCHEMA, "kind": "input", "instance_id": "pico-1",
            "device_id": "paired-headset", "connection_epoch": 10, "space_epoch": 2,
            "sequence": 1, "source_monotonic_ns": 10, "received_monotonic_ns": NOW,
            "clock_id": "boot-test", "tracking_frame": TRACKING_FRAME,
            "head_reference": copy.deepcopy(pose),
            "left": dict(pose, grip=0., trigger=0.),
            "right": dict(pose, grip=0., trigger=0.)}


def operation():
    value = {key: value for key, value in input_frame().items() if key in (
        "schema", "instance_id", "device_id", "connection_epoch", "space_epoch",
        "sequence", "received_monotonic_ns", "clock_id")}
    value.update(kind="operation", request_id="start-001", action="begin",
                 expires_monotonic_ns=NOW + 5_000_000_000)
    return value


def feedback():
    return {"schema": FEEDBACK_SCHEMA, "instance_id": "pico_1",
            "control_instance_id": "teleop_control", "server_epoch": "boot-controller",
            "sequence": 1, "emitted_monotonic_ns": NOW, "clock_id": "boot-test",
            "source_sequence": 1, "connection_epoch": 10, "space_epoch": 2,
            "operator_session_id": None, "mapping_epoch": 0, "state": "ready",
            "reason": None, "capabilities": ["dual_arm"],
            "execution": {"armed": True, "started": False, "mode": "shadow"},
            "receipts": [{"request_id": "start-001", "action": "begin",
                          "device_id": "paired-headset", "connection_epoch": 10, "space_epoch": 2,
                          "status": "completed", "error": None, "result": {}}]}


@pytest.mark.parametrize("factory,validator", [
    (input_frame, validate_input), (operation, validate_operation), (feedback, validate_feedback)])
def test_json_round_trip_and_detached_snapshot(factory, validator):
    value = json.loads(json.dumps(factory()))
    original = copy.deepcopy(value)
    result = validator(value, **OPTIONS)
    assert result == original
    value["instance_id"] = "changed"
    assert result["instance_id"] == original["instance_id"]


@pytest.mark.parametrize("arguments", [(), ("pico", "default"), ("/robot/fleet", "pico-1")])
def test_fixed_topics_do_not_derive_source_identity(arguments):
    assert topics(*arguments) == (COMMAND_TOPIC, STATE_TOPIC) == (
        "/teleop/command", "/teleop/state")
    assert binding_from_topic(topics(*arguments)[0]) == ("", None)


@pytest.mark.parametrize("value", ["a", "/x/command", "/x/teleop//command",
    "/x/teleop/pico/feedback", "/x//teleop/pico/command", "/../teleop/pico/command",
    "/x/teleop/pico-1/command", "/pico/teleop/default/command", "/teleop/state"])
def test_invalid_binding_rejected(value):
    with pytest.raises(ValueError): binding_from_topic(value)


@pytest.mark.parametrize("key,value", [
    ("schema", "motus.xr.input/1"), ("schema", "asr/1"), ("kind", "operation"),
    ("clock_id", "other-boot"), ("instance_id", "other-headset"),
    ("sequence", True), ("connection_epoch", -1), ("sequence", 1 << 63),
    ("tracking_frame", "robot_base"), ("received_monotonic_ns", NOW+1),
    ("received_monotonic_ns", NOW-300_000_001)])
def test_input_rejects_wrong_identity_clock_or_stale_data(key, value):
    frame = input_frame(); frame[key] = value
    with pytest.raises(ValueError): validate_input(frame, **OPTIONS)


@pytest.mark.parametrize("key,value", [("grip", float("nan")), ("trigger", 1.1),
    ("position", [0, 1, float("inf")]), ("orientation_xyzw", [0, 0, 0, 0]),
    ("tracked", "true")])
def test_invalid_controller_rejected(key, value):
    frame = input_frame(); frame["left"][key] = value
    with pytest.raises(ValueError): validate_input(frame, **OPTIONS)


def test_tracking_loss_is_an_admissible_stop_signal_without_pose():
    frame = input_frame()
    frame["left"].update(tracked=False, position=None, orientation_xyzw=None)
    assert validate_input(frame, **OPTIONS)["left"]["tracked"] is False


def test_wrapping_does_not_refresh_a_stale_pose():
    frame = input_frame(); frame["schema"] = "motus.xr.input/1"
    frame["received_monotonic_ns"] -= 300_000_001
    wrapped = command_from_input(frame)
    assert wrapped["received_monotonic_ns"] == frame["received_monotonic_ns"]
    with pytest.raises(ValueError): validate_input(wrapped, **OPTIONS)


@pytest.mark.parametrize("change", [dict(action="release_all"), dict(request_id=""),
    dict(expires_monotonic_ns=NOW), dict(expires_monotonic_ns=NOW+5_000_000_001),
    dict(kind="input")])
def test_invalid_operation_and_deadlines_rejected(change):
    value = operation(); value.update(change)
    with pytest.raises(ValueError): validate_operation(value, **OPTIONS)


def test_operation_is_not_a_pose_and_can_request_stop_without_tracking():
    value = operation(); value["action"] = "stop"
    assert validate_operation(value, **OPTIONS)["action"] == "stop"
    with pytest.raises(ValueError): validate_input(value, **OPTIONS)


@pytest.mark.parametrize("change", [dict(schema="motus.motion.feedback/1"),
    dict(clock_id="other"), dict(instance_id="other"), dict(receipts=[{}]),
    dict(emitted_monotonic_ns=NOW+1), dict(execution=True)])
def test_bad_feedback_rejected(change):
    value = feedback(); value.update(change)
    with pytest.raises(ValueError): validate_feedback(value, **OPTIONS)


def test_duplicate_and_unbounded_receipts_rejected():
    value = feedback(); value["receipts"] *= 2
    with pytest.raises(ValueError): validate_feedback(value, **OPTIONS)
    value["receipts"] *= 32
    with pytest.raises(ValueError): validate_feedback(value, **OPTIONS)


@pytest.mark.parametrize("key,value", [("device_id", None), ("connection_epoch", None),
    ("connection_epoch", -1), ("space_epoch", True)])
def test_receipt_requires_original_request_identity(key, value):
    message = feedback(); message["receipts"][0][key] = value
    with pytest.raises(ValueError): validate_feedback(message, **OPTIONS)


def test_stop_receipt_identity_is_independent_of_last_pose_epoch():
    message = feedback(); receipt = message["receipts"][0]
    receipt.update(action="stop", connection_epoch=11)
    assert validate_feedback(message, **OPTIONS)["receipts"][0]["connection_epoch"] == 11


def test_optional_controls_and_future_tracking_survive_round_trip():
    frame = input_frame()
    frame['left']['controls'] = {
        'buttons': {'x': {'available': True, 'pressed': True, 'touched': False},
                    'grip': {'available': True, 'value': .7},
                    'menu': {'available': False}},
        'axes': {'thumbstick': {'available': True, 'value': [.3, -.4]}},
        'future_sensor': {'sample': [1, 2]},
    }
    frame['extensions'] = {'trackers': {'left_foot': {'tracked': False}}}
    frame['future_metadata'] = {'revision': 2}
    admitted = validate_input(json.loads(json.dumps(frame)), **OPTIONS)
    assert admitted == frame
    frame['left']['controls']['axes']['thumbstick']['value'][0] = 1
    assert admitted['left']['controls']['axes']['thumbstick']['value'] == [.3, -.4]


@pytest.mark.parametrize('controls', [
    None, [], {'buttons': []},
    {'buttons': {'x': {'available': 'yes', 'pressed': True}}},
    {'buttons': {'x': {'available': True, 'pressed': 1}}},
    {'buttons': {'x': {'available': True, 'value': 1.1}}},
    {'buttons': {'x': {'available': False, 'value': 0}}},
    {'axes': {'thumbstick': {'available': True, 'value': [.3]}}},
    {'axes': {'thumbstick': {'available': True, 'value': [2, 0]}}},
    {'axes': {'thumbstick': {'available': False, 'value': [0, 0]}}},
])
def test_malformed_known_controls_are_not_silently_accepted(controls):
    frame = input_frame()
    frame['left']['controls'] = controls
    with pytest.raises(ValueError):
        validate_input(frame, **OPTIONS)


@pytest.mark.parametrize('extension', [[], 'unknown', {'future': float('nan')}])
def test_extension_keeps_finite_json_object_contract(extension):
    frame = input_frame()
    frame['extensions'] = extension
    with pytest.raises(ValueError):
        validate_input(frame, **OPTIONS)
