import math
import sys
import time
import types
from unittest.mock import patch

from can_runtime import BunkerCAN
import protocol


class FakeStop:
    def __init__(self):
        self.waits = 0

    def wait(self, _timeout):
        self.waits += 1
        return self.waits > 1


class FakeBus:
    def __init__(self):
        self.sent = []

    def send(self, message, timeout=None):
        self.sent.append((message, timeout))


def _can_module():
    return types.SimpleNamespace(
        Message=lambda **kwargs: types.SimpleNamespace(**kwargs),
    )


def _assert_raises(exception_type, message, callback):
    try:
        callback()
    except exception_type as exc:
        assert message in str(exc)
    else:
        raise AssertionError(f"expected {exception_type.__name__}")


def test_target_requires_fresh_normal_chassis_in_can_mode():
    can = BunkerCAN({}, "can1")
    _assert_raises(RuntimeError, "missing or stale", lambda: can.set_target(0.1, 0.0))

    can._system_state = {"vehicle_state": protocol.VEHICLE_ESTOP,
                         "control_mode": protocol.CONTROL_CAN}
    can._system_state_at = time.monotonic()
    _assert_raises(RuntimeError, "异常或急停", lambda: can.set_target(0.1, 0.0))

    can._system_state = {"vehicle_state": protocol.VEHICLE_NORMAL,
                         "control_mode": protocol.CONTROL_RC}
    can._system_state_at = time.monotonic()
    _assert_raises(RuntimeError, "CAN 控制模式", lambda: can.set_target(0.1, 0.0))


def test_target_uses_configured_one_meter_per_second_and_sixty_degree_limits():
    can = BunkerCAN({"max_linear_m_s": 1.0, "max_angular_deg_s": 60}, "can1")
    can._system_state = {"vehicle_state": protocol.VEHICLE_NORMAL,
                         "control_mode": protocol.CONTROL_CAN}
    can._system_state_at = time.monotonic()

    assert can.validate_target(1.0, math.radians(60)) == (1.0, math.radians(60))
    _assert_raises(ValueError, "exceeds configured limit",
                   lambda: can.validate_target(1.01, 0.0))
    _assert_raises(ValueError, "exceeds configured limit",
                   lambda: can.validate_target(0.0, 1.0473))


def test_motion_lease_expiry_sends_one_zero_velocity_frame():
    can = BunkerCAN({}, "can1")
    can._bus = FakeBus()
    can._stop = FakeStop()
    can._system_state = {"vehicle_state": protocol.VEHICLE_NORMAL,
                         "control_mode": protocol.CONTROL_CAN}
    can._system_state_at = time.monotonic()
    can._target = (0.1, 0.2)
    can._target_deadline = time.monotonic() + 1

    with patch.dict(sys.modules, {"can": _can_module()}):
        can._transmit_loop()
        assert can._tx_active
        can._target_deadline = time.monotonic() - 1
        can._stop = FakeStop()
        can._transmit_loop()

    assert len(can._bus.sent) == 2
    stop_message, timeout = can._bus.sent[-1]
    assert stop_message.arbitration_id == protocol.MOTION_COMMAND_ID
    assert bytes(stop_message.data) == bytes(8)
    assert timeout == 0.05


def test_critical_fault_during_motion_cancels_target_and_sends_zero():
    can = BunkerCAN({}, "can1")
    can._bus = FakeBus()
    can._stop = FakeStop()
    can._system_state = {"vehicle_state": protocol.VEHICLE_NORMAL,
                         "control_mode": protocol.CONTROL_CAN,
                         "error_flags": 0x04}
    can._system_state_at = time.monotonic()
    can._target = (0.1, 0.2)
    can._target_deadline = time.monotonic() + 1
    can._tx_active = True

    with patch.dict(sys.modules, {"can": _can_module()}):
        can._transmit_loop()

    assert can._target is None
    assert not can._tx_active
    assert len(can._bus.sent) == 1
    message, _timeout = can._bus.sent[0]
    assert bytes(message.data) == bytes(8)


def test_odometry_window_preserves_receive_wall_timestamp():
    can = BunkerCAN({}, "can1")
    can._odom_samples.append((10.0, 1_800_000_000_123,
                              [0.15, None, None, None, None, None]))

    rows, _ = can.odometry_window(now=10.1)

    assert rows == [(1_800_000_000_123,
                     [0.15, None, None, None, None, None])]
