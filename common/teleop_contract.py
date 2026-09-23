"""Versioned two-Driver teleoperation wire contract; no ROS or robot dependencies.

The device publishes input only. Optional controls/extensions preserve device
capabilities without changing the mandatory dual-arm fields. Historical operation
and feedback helpers remain for existing robot consumers. All deadlines use the
receiving host boot clock, not the headset clock.
"""
from __future__ import annotations

import copy
import json
import math
import re

COMMAND_SCHEMA = "motus.teleop.command/1"
FEEDBACK_SCHEMA = "motus.teleop.feedback/1"
COMMAND_FORMAT = "data/teleop-cmd"
FEEDBACK_FORMAT = "data/teleop-state"
COMMAND_TOPIC = "/teleop/command"
STATE_TOPIC = "/teleop/state"
TRACKING_FRAME = "tracking_x_forward_y_left_z_up"
MAX_INPUT_AGE_NS = 300_000_000
MAX_OPERATION_AGE_NS = 5_000_000_000
MAX_FEEDBACK_AGE_NS = 1_000_000_000
MAX_WIRE_BYTES = 65_536
ACTIONS = frozenset({"begin", "finish", "stop", "calibrate"})
TERMINAL_RECEIPTS = frozenset({"completed", "failed"})
_TOKEN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")


def _text(value, name, *, maximum=256):
    if not isinstance(value, str) or not value or len(value) > maximum:
        raise ValueError("invalid_" + name)
    return value


def _integer(value, name, *, minimum=0):
    if type(value) is not int or not minimum <= value <= (1 << 63) - 1:
        raise ValueError("invalid_" + name)
    return value


def _number(value, name, *, lower=None, upper=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("invalid_" + name)
    if (lower is not None and value < lower) or (upper is not None and value > upper):
        raise ValueError("invalid_" + name)
    return value


def canonical_instance(value):
    normalized = _text(value, "instance_id").replace("-", "_")
    if not _TOKEN.fullmatch(normalized):
        raise ValueError("invalid_instance_id")
    return normalized


def topics(namespace=None, instance_id=None):
    """Fixed single-source topics; legacy arguments never select a destination.

    State is the robot card's monitor output, not a device feedback subscription.
    Device identity belongs in the message, independently of these topic names.
    """
    return COMMAND_TOPIC, STATE_TOPIC


def binding_from_topic(input_topic):
    value = _text(input_topic, "input_topic", maximum=512)
    if value != COMMAND_TOPIC:
        raise ValueError("invalid_input_topic")
    # Keep the consumer tuple shape without inventing an identity from a topic.
    # The consumer must bind/validate the source from the received message.
    return "", None


def _wire(value):
    if not isinstance(value, dict):
        raise ValueError("invalid_message")
    try:
        encoded = json.dumps(value, allow_nan=False, separators=(",", ":"))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("invalid_json") from exc
    if len(encoded.encode("utf-8")) > MAX_WIRE_BYTES:
        raise ValueError("message_too_large")


def _identity(value, *, instance_id, clock_id):
    if canonical_instance(value.get("instance_id")) != canonical_instance(instance_id):
        raise ValueError("input_binding_mismatch")
    _text(value.get("device_id"), "device_id")
    if value.get("clock_id") != clock_id:
        raise ValueError("input_clock_mismatch")
    _text(clock_id, "clock_id")
    for key in ("sequence", "connection_epoch", "space_epoch", "received_monotonic_ns"):
        _integer(value.get(key), key)


def _fresh(value, now_ns, max_age_ns):
    _integer(now_ns, "now_ns")
    _integer(max_age_ns, "max_age_ns", minimum=1)
    if not 0 <= now_ns - value <= max_age_ns:
        raise ValueError("input_stale")


def _vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError("invalid_" + name)
    return [_number(v, name) for v in value]


def _pose(value, name, *, controller=False):
    if not isinstance(value, dict) or type(value.get("tracked")) is not bool:
        raise ValueError("invalid_tracking")
    if value["tracked"]:
        _vector(value.get("position"), 3, name + "_position")
        q = _vector(value.get("orientation_xyzw"), 4, name + "_orientation")
        if abs(sum(v * v for v in q) - 1) > .002:
            raise ValueError("invalid_quaternion")
    elif value.get("position") is not None or value.get("orientation_xyzw") is not None:
        raise ValueError("invalid_tracking")
    if controller:
        for key in ("grip", "trigger"):
            _number(value.get(key), key, lower=0, upper=1)
        if "controls" in value:
            validate_controls(value["controls"])


def validate_controls(value):
    """Validate named optional inputs without stripping future capabilities.

    Unavailable inputs carry no fabricated measurement. Unknown optional fields
    are retained; the enclosing message still enforces finite JSON and size.
    """
    if not isinstance(value, dict):
        raise ValueError("invalid_controls")
    for group in ("buttons", "axes"):
        entries = value.get(group, {})
        if not isinstance(entries, dict):
            raise ValueError("invalid_controls_" + group)
        for name, state in entries.items():
            _text(name, "control_name", maximum=64)
            if not isinstance(state, dict) or type(state.get("available")) is not bool:
                raise ValueError("invalid_control_availability")
            measurements = ("value", "pressed", "touched") if group == "buttons" else ("value",)
            if not state["available"]:
                if any(key in state for key in measurements):
                    raise ValueError("unavailable_control_has_measurement")
                continue
            if group == "buttons":
                if "value" in state:
                    _number(state["value"], "button_value", lower=0, upper=1)
                for key in ("pressed", "touched"):
                    if key in state and type(state[key]) is not bool:
                        raise ValueError("invalid_button_" + key)
            else:
                vector = state.get("value")
                if not isinstance(vector, list) or not vector:
                    raise ValueError("invalid_axis_value")
                if name == "thumbstick" and len(vector) != 2:
                    raise ValueError("invalid_thumbstick_dimensions")
                for component in vector:
                    _number(component, "axis_value", lower=-1, upper=1)
    return value


def validate_input(value, *, instance_id, clock_id, now_ns, max_age_ns=MAX_INPUT_AGE_NS):
    _wire(value)
    if value.get("schema") != COMMAND_SCHEMA or value.get("kind") != "input":
        raise ValueError("invalid_input_schema")
    _identity(value, instance_id=instance_id, clock_id=clock_id)
    _integer(value.get("source_monotonic_ns"), "source_monotonic_ns")
    _fresh(value["received_monotonic_ns"], now_ns, max_age_ns)
    if value.get("tracking_frame") != TRACKING_FRAME:
        raise ValueError("input_frame_mismatch")
    _pose(value.get("head_reference"), "head_reference")
    for side in ("left", "right"):
        _pose(value.get(side), side, controller=True)
    if "extensions" in value and not isinstance(value["extensions"], dict):
        raise ValueError("invalid_extensions")
    return copy.deepcopy(value)


def command_from_input(value):
    """Wrap a normalized internal XR frame without refreshing its timestamp."""
    out = copy.deepcopy(value)
    if out.get("schema") not in ("motus.xr.input/1", COMMAND_SCHEMA):
        raise ValueError("invalid_input_schema")
    out.update(schema=COMMAND_SCHEMA, kind="input")
    return out


def validate_operation(value, *, instance_id, clock_id, now_ns,
                       max_age_ns=MAX_OPERATION_AGE_NS):
    _wire(value)
    if value.get("schema") != COMMAND_SCHEMA or value.get("kind") != "operation":
        raise ValueError("invalid_operation_schema")
    _identity(value, instance_id=instance_id, clock_id=clock_id)
    _text(value.get("request_id"), "request_id", maximum=128)
    if value.get("action") not in ACTIONS:
        raise ValueError("unsupported_operation")
    received = value["received_monotonic_ns"]
    expiry = _integer(value.get("expires_monotonic_ns"), "expires_monotonic_ns")
    _fresh(received, now_ns, max_age_ns)
    if not received < expiry <= received + max_age_ns or now_ns >= expiry:
        raise ValueError("operation_expired")
    return copy.deepcopy(value)


def validate_feedback(value, *, instance_id, clock_id, now_ns,
                      max_age_ns=MAX_FEEDBACK_AGE_NS):
    _wire(value)
    if value.get("schema") != FEEDBACK_SCHEMA:
        raise ValueError("invalid_feedback_schema")
    if canonical_instance(value.get("instance_id")) != canonical_instance(instance_id):
        raise ValueError("feedback_binding_mismatch")
    if value.get("clock_id") != clock_id:
        raise ValueError("feedback_clock_mismatch")
    _text(value.get("control_instance_id"), "control_instance_id")
    _text(value.get("server_epoch"), "server_epoch")
    for key in ("sequence", "mapping_epoch", "emitted_monotonic_ns"):
        _integer(value.get(key), key)
    _fresh(value["emitted_monotonic_ns"], now_ns, max_age_ns)
    for key in ("source_sequence", "connection_epoch", "space_epoch"):
        if value.get(key) is not None:
            _integer(value[key], key, minimum=-1 if key == "source_sequence" else 0)
    if value.get("operator_session_id") is not None:
        _text(value["operator_session_id"], "operator_session_id")
    _text(value.get("state"), "state", maximum=64)
    if value.get("reason") is not None:
        _text(value["reason"], "reason", maximum=1024)
    if (not isinstance(value.get("capabilities"), list)
            or any(not isinstance(v, str) for v in value["capabilities"])
            or not isinstance(value.get("execution"), dict)):
        raise ValueError("invalid_feedback_state")
    receipts = value.get("receipts")
    if not isinstance(receipts, list) or len(receipts) > 32:
        raise ValueError("invalid_receipts")
    keys = set()
    for receipt in receipts:
        if not isinstance(receipt, dict):
            raise ValueError("invalid_receipt")
        key = _text(receipt.get("request_id"), "request_id", maximum=128)
        if key in keys:
            raise ValueError("duplicate_receipt")
        keys.add(key)
        _text(receipt.get("device_id"), "receipt_device_id")
        for field in ("connection_epoch", "space_epoch"):
            _integer(receipt.get(field), "receipt_" + field)
        if (receipt.get("action") not in ACTIONS or
                receipt.get("status") not in {"accepted", "completed", "failed"} or
                not isinstance(receipt.get("result"), dict)):
            raise ValueError("invalid_receipt")
        if receipt.get("error") is not None:
            _text(receipt["error"], "receipt_error", maximum=1024)
    return copy.deepcopy(value)
