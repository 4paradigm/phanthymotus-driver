"""Franka Hand commands on a shared, independent gripper SDK connection."""

from __future__ import annotations

import json
import math
import os
import time

from hardware import RobotSession, TERMINAL_STATUSES


def _number(value, name, low, high, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high
            or (positive and value == 0)):
        raise ValueError(f"{name} is outside its permitted range")
    return value


class GripperSession(RobotSession):
    """Reuse session ownership, deadlines, completion and stop handling for a Hand."""

    def __init__(self, address, *, reader_path=None, **options):
        self._motion_fault = None
        super().__init__(address, reader_path=reader_path or os.environ.get(
            "FRANKA_GRIPPER_READER", "/usr/local/bin/franka-gripper"), **options)

    def _latch_motion_fault_locked(self, error):
        if self._motion_fault is None:
            self._motion_fault = ("Gripper stop could not be confirmed; stop all gripper cards for this robot, then restart them "
                                  "before another motion: " + str(error))
        if self._sample is not None:
            self._sample.update(control_ready=False, control_error=self._motion_fault)

    def _break_locked(self, error):
        record = self._motions.get(self._active)
        if record is not None and not record.get("_native_stop_confirmed", False):
            self._latch_motion_fault_locked(error)
        super()._break_locked(error)

    def command(self, owner, action, *, width_m=None, speed_m_s=0.02,
                speed_source="argument", force_n=30, epsilon_inner_m=0.005,
                epsilon_outer_m=0.005, timeout_s=30):
        """Submit an explicit gripper action; connecting never homes or moves it."""
        if action not in ("set_position", "grasp", "homing"):
            raise ValueError("Unknown gripper action")
        _number(timeout_s, "timeout_s", 1, 120)
        target = {"action": action}
        if action == "homing":
            return self._submit_motion(owner, target, "HOME", (timeout_s,), timeout_s)

        _number(width_m, "width_m", 0, 0.08)
        _number(speed_m_s, "speed_m_s", 0, 0.1, positive=True)
        if speed_source not in ("argument", "config"):
            raise ValueError("Unknown gripper speed source")
        target.update(width_m=width_m, speed_m_s=speed_m_s, speed_source=speed_source)
        verb, fields = "MOVE", (speed_m_s, width_m, timeout_s)
        if action == "grasp":
            _number(force_n, "force_n", 30, 70)
            _number(epsilon_inner_m, "epsilon_inner_m", 0, 0.08)
            _number(epsilon_outer_m, "epsilon_outer_m", 0, 0.08)
            target.update(force_n=force_n, epsilon_inner_m=epsilon_inner_m,
                          epsilon_outer_m=epsilon_outer_m)
            verb = "GRASP"
            fields = (speed_m_s, width_m, force_n, epsilon_inner_m, epsilon_outer_m, timeout_s)

        def calibrated_width(sample):
            maximum = sample["max_width"]
            if maximum <= 0:
                raise RuntimeError("Gripper must be homed before setting its width or grasping")
            if width_m > maximum:
                raise ValueError("width_m exceeds the gripper's measured max_width")

        return self._submit_motion(owner, target, verb, fields, timeout_s,
                                   validate=calibrated_width)

    def stop_gripper(self, owner):
        """Stop this Hand, including holding force after an already completed grasp."""
        command_id = self._submit_motion(
            owner, {"action": "stopmotion"}, "STOP", (), 0, stop=True)
        return self.motion(command_id)

    def _accept_event(self, event):
        if "requires_restart" in event:
            restart = event["requires_restart"]
            if (not isinstance(restart, bool) or event.get("status") != "failed"
                    or event.get("event") not in ("motion", "stop")):
                raise ValueError("Invalid native gripper restart requirement")
            with self._lock:
                command_id = event.get("command_id")
                if isinstance(command_id, str) and command_id == self._active:
                    record = self._motions[command_id]
                    record["requires_restart"] = restart
                    # Only this explicit native result certifies that a failed
                    # operation was stopped and physically observed at rest.
                    record["_native_stop_confirmed"] = not restart
                    if restart:
                        self._latch_motion_fault_locked(event.get("error") or "Native stop failed")
        parameters = event.get("parameters")
        if parameters is not None:
            if (event.get("event") != "motion" or event.get("status") != "accepted"
                    or not isinstance(parameters, dict)):
                raise ValueError("Invalid native gripper command parameters")
            action = parameters.get("action")
            if action not in ("set_position", "grasp", "homing"):
                raise ValueError("Invalid native gripper command action")
            expected_keys = {"action"}
            if action != "homing":
                _number(parameters.get("width_m"), "Native width_m", 0, 0.08)
                _number(parameters.get("speed_m_s"), "Native speed_m_s", 0, 0.1, positive=True)
                expected_keys.update(("width_m", "speed_m_s"))
            if action == "grasp":
                _number(parameters.get("force_n"), "Native force_n", 30, 70)
                _number(parameters.get("epsilon_inner_m"), "Native epsilon_inner_m", 0, 0.08)
                _number(parameters.get("epsilon_outer_m"), "Native epsilon_outer_m", 0, 0.08)
                expected_keys.update(("force_n", "epsilon_inner_m", "epsilon_outer_m"))
            if set(parameters) != expected_keys:
                raise ValueError("Unexpected native gripper command parameters")
            with self._lock:
                if self._active is not None and event.get("command_id") == self._active:
                    target = self._motions[self._active]["target"]
                    if any(target.get(key) != value for key, value in parameters.items()):
                        raise ValueError("Native gripper parameters differ from the submitted command")
                    self._motions[self._active]["native_parameters"] = dict(parameters)
        if event.get("event") == "motion" and event.get("status") in TERMINAL_STATUSES:
            with self._lock:
                command_id = event.get("command_id")
                record = self._motions.get(command_id) if isinstance(command_id, str) else None
                wait_for_stop = (command_id == self._active and record is not None
                                 and record.get("_stop_ack_required")
                                 and record.get("stop_status") != "stopped")
            if wait_for_stop:
                # The SDK action can finish before it reads our queued STOP.
                # Validate its event now, but retain ownership until STOP is ACKed.
                super()._accept_event(dict(event, status="running"))
                with self._changed:
                    if command_id == self._active:
                        self._motions[command_id]["_terminal_after_stop"] = (
                            event["status"], event.get("error"))
                return
        super()._accept_event(event)
        if event.get("event") == "stop" and event.get("status") in ("stopped", "idle"):
            with self._changed:
                command_id = event["command_id"]
                if command_id != self._active:
                    return
                record = self._motions[command_id]
                if record["target"] == {"action": "stopmotion"}:
                    # Only an idle STOP is complete on its ACK. An interrupted
                    # motion still waits for its own terminal motion event.
                    if event["status"] == "stopped":
                        self._finish_locked("succeeded")
                    else:
                        self._break_locked("Native backend did not confirm the requested gripper stop")
                elif record.get("_stop_ack_required"):
                    if event["status"] != "stopped":
                        self._break_locked("Native backend did not confirm the requested gripper stop")
                    elif "_terminal_after_stop" in record:
                        self._finish_locked(*record["_terminal_after_stop"])

    def _accept(self, line):
        record = json.loads(line)
        if not isinstance(record, dict):
            raise ValueError("Reader state must be a JSON object")
        if "event" in record:
            self._accept_event(record)
            return False
        sample = {}
        # Preserve SDK measurements: a closed Hand can report a negative width.
        # An unhomed max_width may be zero; measured maxima may exceed 80 mm.
        sample["width"] = _number(record.get("width"), "Reader width", -math.inf, math.inf)
        sample["max_width"] = _number(record.get("max_width"), "Reader max_width", 0, math.inf)
        for name in ("is_grasped", "control_ready"):
            value = record.get(name, False if name == "control_ready" else None)
            if not isinstance(value, bool):
                raise ValueError(f"Reader {name} must be boolean")
            sample[name] = value
        temperature = record.get("temperature")
        if (isinstance(temperature, bool) or not isinstance(temperature, int)
                or not 0 <= temperature <= 65535):
            raise ValueError("Reader temperature must be an unsigned 16-bit integer")
        error = record.get("control_error")
        if error is not None and not isinstance(error, str):
            raise ValueError("Reader control_error must be text or null")
        sample.update(temperature=temperature, control_error=error,
                      timestamp_ms=int(time.time() * 1000))
        with self._lock:
            if self._stop.is_set() or self._connection_broken:
                return False
            if self._motion_fault is not None:
                sample.update(control_ready=False, control_error=self._motion_fault)
            self._sample, self._received = sample, time.monotonic()
            self._state, self._error = "connected", None
        return True
