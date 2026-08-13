"""G1 teleoperation MCP facade and in-process aiortc event loop."""

from __future__ import annotations

import asyncio
import base64
import copy
import json
import math
import secrets
import threading
import time
from collections.abc import Callable, Mapping
from typing import Any

from .descriptor import (
    PREFLIGHT_SCHEMA,
    PROFILE_ID,
    SIGNALING_AUDIENCE,
    tool_definitions,
)
from .capture import (
    MAX_CAPTURE_CA_BASE64_CHARS,
    MAX_CAPTURE_CA_PEM_BYTES,
    CaptureError,
    CaptureManager,
)
from .capture_server import CaptureWssServer, capture_certificate_base64
from .ik_diagnostic import G1TeleopIkDiagnostic
from .protocol import ProtocolError, TicketCodec, TicketVerifier
from .rtc import RtcManager, RtcRequestError
from .runtime import G1TeleopRuntime

_REQUIRED_MODE_MACHINE = 4
_LIVE_LOWSTATE_MAX_AGE_S = 0.1


class G1TeleopService:
    """One teleoperation service embedded in the existing root G1 Driver."""

    def __init__(
        self,
        runtime: G1TeleopRuntime,
        *,
        ticket_secret: str | bytes | None = None,
        startup_preflight: dict,
        ik_diagnostic: G1TeleopIkDiagnostic,
        capture_config: Mapping[str, object],
        live_low_state_probe: Callable[[], Mapping[str, object]] | None = None,
        start_capture_listener: bool = True,
        offer_timeout_s: float = 5.0,
        ticket_ttl_max_seconds: int = 30,
        ticket_replay_cache_entries: int = 4096,
    ):
        self.runtime = runtime
        if (
            not callable(getattr(ik_diagnostic, "dispatch", None))
            or not callable(getattr(ik_diagnostic, "status", None))
        ):
            raise ValueError("G1 teleoperation requires an IK diagnostic facade")
        self._ik_diagnostic = ik_diagnostic
        if runtime.actuation_enabled:
            if not callable(live_low_state_probe):
                raise ValueError("Live service requires a read-only LowState health probe")
        elif live_low_state_probe is not None:
            raise ValueError("Shadow service forbids a hardware LowState health probe")
        self._live_low_state_probe = live_low_state_probe
        self._offer_timeout = float(offer_timeout_s)
        validated_preflight = self._validate_factory_preflight(startup_preflight)
        if ticket_secret is None:
            ticket_secret = secrets.token_bytes(32)
        ticket_codec = TicketCodec(ticket_secret)
        verifier = TicketVerifier(
            ticket_codec,
            audience=SIGNALING_AUDIENCE,
            max_ttl_seconds=int(ticket_ttl_max_seconds),
            max_replay_entries=int(ticket_replay_cache_entries),
        )
        self.rtc = RtcManager(runtime, verifier)
        capture = dict(capture_config)
        ca_certificate = capture.get("ca_certificate_base64")
        if ca_certificate is None:
            ca_certificate = capture_certificate_base64(capture)
        if (
            not isinstance(ca_certificate, str)
            or not ca_certificate
            or len(ca_certificate) > MAX_CAPTURE_CA_BASE64_CHARS
        ):
            raise ValueError("capture.ca_certificate_base64 must be bounded base64 text")
        try:
            decoded_ca_certificate = base64.b64decode(ca_certificate, validate=True)
        except (ValueError, TypeError) as exc:
            raise ValueError("capture.ca_certificate_base64 must be valid base64") from exc
        if (
            not decoded_ca_certificate
            or len(decoded_ca_certificate) > MAX_CAPTURE_CA_PEM_BYTES
            or b"-----BEGIN CERTIFICATE-----" not in decoded_ca_certificate
            or b"PRIVATE KEY" in decoded_ca_certificate
        ):
            raise ValueError(
                "capture.ca_certificate_base64 must decode to a bounded public PEM chain"
            )
        self.capture = CaptureManager(
            runtime,
            self.rtc,
            ticket_codec,
            pairing_ttl_seconds=int(capture.get("pairing_ttl_seconds", 60)),
            ticket_ttl_seconds=int(capture.get("ticket_ttl_seconds", 20)),
            presence_interval_ms=int(capture.get("presence_interval_ms", 1000)),
            presence_timeout_ms=int(capture.get("presence_timeout_ms", 5000)),
            state_file=capture.get("state_file") or None,
            public_wss_url=str(capture["public_wss_url"]),
            ca_certificate_base64=ca_certificate,
        )
        self._capture_server = (
            CaptureWssServer(self.capture, capture)
            if start_capture_listener
            else None
        )
        self._instance_id: str | None = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._run_loop,
            name="g1-teleop-rtc",
            daemon=True,
        )
        self._closed = False
        self._close_lock = threading.Lock()
        self._registration_lock = threading.Lock()
        self._registration_stop = threading.Event()
        self._registration_thread: threading.Thread | None = None
        self._registration_status = {
            "state": "not_started",
            "attempts": 0,
            "successes": 0,
            "last_http_status": None,
            "last_error": None,
            "tls_verification": "pinned_certificate",
        }
        self._startup_preflight = validated_preflight
        self._rtc_loop_ready = False
        self._thread.start()
        loop_probe = threading.Event()
        self._loop.call_soon_threadsafe(loop_probe.set)
        if not loop_probe.wait(timeout=2.0):
            self._stop_failed_loop_start()
            raise RuntimeError("G1 teleoperation RTC event loop failed its startup probe")
        self._rtc_loop_ready = True
        try:
            if self._capture_server is not None:
                self._run_async(
                    self._capture_server.start(),
                    timeout=self._offer_timeout,
                )
            self._startup_preflight = self._complete_startup_preflight(
                self._startup_preflight
            )
        except Exception:
            if self._capture_server is not None:
                try:
                    self._run_async(self._capture_server.close())
                except Exception:
                    pass
            self._stop_failed_loop_start()
            raise

    @property
    def registration_status(self) -> dict:
        with self._registration_lock:
            return dict(self._registration_status)

    def update_registration_status(self, **updates) -> None:
        allowed = {
            "state",
            "attempts",
            "successes",
            "last_http_status",
            "last_error",
            "tls_verification",
        }
        if set(updates) - allowed:
            raise ValueError("invalid registration status field")
        bounded = dict(updates)
        error = bounded.get("last_error")
        if error is not None:
            bounded["last_error"] = str(error)[:256]
        with self._registration_lock:
            self._registration_status.update(bounded)

    def launch_registration_worker(self, target) -> None:
        with self._registration_lock:
            if self._registration_thread is not None:
                raise RuntimeError("registration worker already exists")
            thread = threading.Thread(
                target=target,
                daemon=True,
                name="g1-register",
            )
            self._registration_thread = thread
        thread.start()

    def registration_wait(self, timeout: float) -> bool:
        return self._registration_stop.wait(timeout)

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def get_tools(self) -> list[dict]:
        return tool_definitions(
            mode=self.runtime.mode,
            driver_id=self.runtime.driver_id,
            driver_name=self.runtime.driver_name,
            robot_id=self.runtime.robot_id,
            signaling_enabled=self.rtc.enabled,
        )

    def preflight_status(self) -> dict:
        """Return bounded startup evidence plus current RTC-loop liveness."""

        result = copy.deepcopy(self._startup_preflight)
        loop_running = bool(
            self._rtc_loop_ready
            and not self._closed
            and self._thread.is_alive()
            and self._loop.is_running()
        )
        result["rtc"]["event_loop_running"] = loop_running
        result["rtc"]["ready"] = bool(self.rtc.enabled and loop_running)
        result["ready"] = bool(result["ready"] and result["rtc"]["ready"])
        if not result["ready"]:
            result["stage"] = "service_runtime"
            result["code"] = "rtc_event_loop_unavailable"
            result["message"] = "RTC event loop is not running"
        return result

    def dispatch(self, tool_name: str, arguments: Any) -> dict:
        if not isinstance(arguments, dict):
            raise ProtocolError("invalid_arguments", "tool arguments must be an object")
        args = dict(arguments)
        instance_id = self._pop_instance_id(args)
        if tool_name == "teleop_state":
            action = args.pop("action", None)
            if args:
                raise ProtocolError(
                    "invalid_arguments",
                    "teleop_state accepts only action and an optional instance_id",
                )
            if action is None or action == "status":
                return self._status_with_capture(instance_id)
            if action in ("start", "info", "stop"):
                return self._passive_card_status(
                    self._status_with_capture(instance_id),
                    instance_id=instance_id,
                    action=action,
                )
            raise ProtocolError(
                "unknown_action",
                f"unknown teleop_state action: {action}",
            )
        if tool_name == "teleop_ik":
            action = args.pop("action", None)
            if not isinstance(action, str):
                raise ProtocolError("missing_action", "teleop_ik requires an action")
            if action in ("start", "info", "stop"):
                self._require_exact(args, set(), action)
                return self._passive_card_status(
                    self._project_ik_output_contract(self._ik_diagnostic.status()),
                    instance_id=instance_id,
                    action=action,
                )
            result = self._ik_diagnostic.dispatch(action, args)
            return self._project_ik_output_contract(result)
        if tool_name != "teleop_session":
            raise ProtocolError("unknown_tool", f"unknown tool: {tool_name}")
        action = args.pop("action", None)
        if not isinstance(action, str):
            raise ProtocolError("missing_action", "teleop_session requires an action")
        if action == "start":
            self._require_exact(args, set(), action)
            if (
                self._instance_id is not None
                and instance_id is not None
                and self._instance_id != instance_id
                and self.runtime.status()["authority_valid"]
            ):
                raise ProtocolError(
                    "instance_conflict",
                    "another card instance owns the local teleoperation session",
                )
            result = self.runtime.prepare_local_session()
            self._instance_id = instance_id or self._instance_id
            self._run_async(self.capture.issue_assignment_if_connected())
            return self._card_session_result(result, action=action)
        if action == "info":
            self._require_exact(args, set(), action)
            return self._card_session_result(self.runtime.status(), action=action)
        if action == "stop":
            self._require_exact(args, set(), action)
            result = self.runtime.release_local(reason="lifecycle_stop")
            self._close_all_rtc()
            self._run_async(self.capture.revoke_assignment("lifecycle_stop"))
            projected = self._card_session_result(result, action=action)
            self._instance_id = None
            return projected
        if action == "pair_headset":
            self._require_exact(args, set(), action)
            if not self.runtime.status()["authority_valid"]:
                raise ProtocolError(
                    "session_inactive",
                    "start the teleop_session card before pairing",
                )
            result = self._run_async(self.capture.create_pairing())
            if not result.get("wss_url") or not result.get("ca_certificate_base64"):
                raise ProtocolError(
                    "capture_tls_unavailable",
                    "Capture WSS URL or public CA bootstrap is unavailable",
                )
            result["state"] = "pairing_ready"
            return result
        if action == "revoke_headset":
            self._require_exact(args, set(), action)
            result = self.runtime.release_local(reason="capture_revoked")
            self._close_all_rtc()
            try:
                result["headset"] = self._run_async(self.capture.revoke_headset())
            except CaptureError as exc:
                raise ProtocolError(exc.code, str(exc)) from exc
            return result
        if action == "pause":
            self._require_exact(args, set(), action)
            result = self.runtime.pause_local()
            self._close_all_rtc()
            self._run_async(self.capture.revoke_assignment("operator_pause"))
            return result
        if action in {"release", "emergency_stop"}:
            self._require_exact(args, set(), action)
            reason = "emergency_stop" if action == "emergency_stop" else "operator_release"
            result = self.runtime.release_local(reason=reason)
            self._close_all_rtc()
            self._run_async(self.capture.revoke_assignment(reason))
            return result
        if action == "status":
            self._require_exact(args, set(), action)
            return self._status_with_capture(instance_id)
        raise ProtocolError("unknown_action", f"unknown teleop_session action: {action}")

    def _card_session_result(self, status: Mapping[str, object], *, action: str) -> dict:
        projected = copy.deepcopy(dict(status))
        projected["instance_id"] = self._instance_id
        projected["lifecycle_action"] = action
        projected["lifecycle_state"] = (
            "ready" if projected.get("dispatch", {}).get("ready") else "fault"
        )
        projected["topic_out"] = []
        projected["capture_control"] = self._run_async(self.capture.status())
        return projected

    def _status_with_capture(self, instance_id: str | None = None) -> dict:
        result = self.runtime.status()
        result["instance_id"] = self._instance_id or instance_id
        result["capture_control"] = self._capture_status()
        result["topic_out"] = []
        return result

    def _capture_status(self) -> dict:
        if self._closed or self._loop.is_closed() or not self._loop.is_running():
            return {
                "paired_devices": 0,
                "pending_pairings": 0,
                "connected": False,
                "capture_id": None,
                "assignment_id": None,
                "state": "service_closed" if self._closed else "unavailable",
            }
        return self._run_async(self.capture.status())

    def _project_ik_output_contract(self, result: Mapping[str, object]) -> dict:
        projected = copy.deepcopy(dict(result))
        runtime = self.runtime.status()
        projected["diagnostic_hardware_output"] = False
        projected["diagnostic_publisher_present"] = False
        projected["diagnostic_output_active"] = False
        projected["actuation_enabled"] = runtime["actuation_enabled"]
        projected["publisher_present"] = runtime["publisher_present"]
        projected["output_active"] = runtime["output_active"]
        projected.pop("hardware_output", None)
        projected.pop("publisher_created", None)
        return projected

    @staticmethod
    def _passive_card_status(
        status: Mapping[str, object],
        *,
        instance_id: str | None,
        action: str,
    ) -> dict:
        """Return a Canvas lifecycle response without mutating its owner."""

        return {
            **copy.deepcopy(dict(status)),
            "instance_id": instance_id,
            "compatibility_lifecycle_only": True,
            "lifecycle_action": action,
            "lifecycle_action_output_applied": False,
            "topic_out": [],
        }

    def _accept_capture_offer(self, payload: Any) -> dict:
        """Private test seam; network signaling is owned by Capture WSS."""

        if self._closed:
            raise RtcRequestError(503, "service_closed", "teleoperation service is closed")
        future = asyncio.run_coroutine_threadsafe(self.rtc.accept_offer(payload), self._loop)
        try:
            return future.result(timeout=self._offer_timeout)
        except TimeoutError as exc:
            future.cancel()
            raise RtcRequestError(504, "offer_timeout", "RTC negotiation timed out") from exc

    def blocks_arm_gesture(self) -> bool:
        state = self.runtime.status()["state"]
        return state in {
            "prepared_shadow",
            "active_shadow",
            "prepared_live",
            "active_live",
            "hold",
            "paused",
        }

    def health(self) -> dict:
        status = self.runtime.status()
        registration = self.registration_status
        preflight = self.preflight_status()
        live_low_state = self._live_low_state_health()
        # Do not hold the lifecycle lock across the potentially blocking
        # LowState read: final-dispatch close must never wait for diagnostics.
        # This short final section makes the result linearizable with close().
        # If close finishes during the probe, this response is red; if this
        # response is green, close cannot finish until it has returned.
        with self._close_lock:
            if self._closed:
                preflight = self.preflight_status()
                if live_low_state is not None:
                    live_low_state = self._live_low_state_result(
                        code="service_closed"
                    )
            return {
                "ready": bool(
                    not self._closed
                    and preflight["ready"]
                    and status["dispatch"]["ready"]
                    and registration["state"] == "registered"
                    and (
                        live_low_state is None
                        or live_low_state["ready"] is True
                    )
                ),
                "mode": status["mode"],
                "actuation_enabled": status["actuation_enabled"],
                "publisher_present": status["publisher_present"],
                "output_active": status["output_active"],
                "profile_id": status["profile_id"],
                "capability_digest": status["capability_digest"],
                "state": status["state"],
                "dispatch": {
                    "kind": status["dispatch"]["kind"],
                    "state": status["dispatch"]["state"],
                    "stop_acknowledged": status["dispatch"]["stop_acknowledged"],
                    "fault_code": status["dispatch"]["fault_code"],
                },
                "preflight": preflight,
                "live_low_state": live_low_state,
                "registration": registration,
                "mcp_access": "loopback-only",
                "capture_control": self._capture_status(),
            }

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._registration_stop.set()

            def cleanup(label: str, operation) -> None:
                try:
                    operation()
                except Exception as exc:  # noqa: BLE001 -- keep safety cleanup independent
                    print(
                        f"[teleop] {label} cleanup FAILED: "
                        f"{type(exc).__name__}: {exc}",
                        flush=True,
                    )

            registration_thread = self._registration_thread
            if (
                registration_thread is not None
                and registration_thread is not threading.current_thread()
            ):
                cleanup(
                    "registration worker",
                    lambda: registration_thread.join(timeout=3.5),
                )
            cleanup("RTC peers", self._close_all_rtc)
            if self._capture_server is not None:
                cleanup(
                    "Capture WSS",
                    lambda: self._run_async(self._capture_server.close()),
                )
            # Runtime close owns the final-dispatch safe-stop and sole possible
            # arm publisher. It must run even if RTC/registration cleanup fails.
            cleanup("runtime", self.runtime.close)
            if not self._loop.is_closed():
                cleanup(
                    "RTC event-loop stop",
                    lambda: self._loop.call_soon_threadsafe(self._loop.stop),
                )
            if self._thread is not threading.current_thread():
                cleanup("RTC event-loop thread", lambda: self._thread.join(timeout=1.0))
            if not self._thread.is_alive() and not self._loop.is_closed():
                cleanup("RTC event loop", self._loop.close)

    def _live_low_state_health(self) -> dict | None:
        """Read current Live LowState without touching the output adapter."""

        probe = self._live_low_state_probe
        if probe is None:
            return None
        if self._closed:
            return self._live_low_state_result(code="service_closed")
        try:
            sample = probe()
        except Exception:  # noqa: BLE001 -- a health probe must fail closed
            return self._live_low_state_result(code="low_state_unavailable")
        if not isinstance(sample, Mapping):
            return self._live_low_state_result(code="low_state_invalid")

        mode_machine = sample.get("mode_machine")
        sampled = sample.get("sample_monotonic")
        arm_joint_count = self._finite_vector_count(
            sample.get("joint_positions"),
            expected=10,
        )
        velocity_count = self._finite_vector_count(
            sample.get("joint_velocities"),
            expected=10,
        )
        motor_joint_count = self._finite_vector_count(
            sample.get("all_joint_positions"),
            expected=35,
        )
        if (
            isinstance(mode_machine, bool)
            or not isinstance(mode_machine, int)
            or isinstance(sampled, bool)
            or not isinstance(sampled, (int, float))
            or not math.isfinite(float(sampled))
            or arm_joint_count is None
            or velocity_count is None
            or motor_joint_count is None
        ):
            return self._live_low_state_result(code="low_state_invalid")

        age_s = time.monotonic() - float(sampled)
        age_ms = round(min(60_000.0, max(0.0, age_s * 1000.0)), 3)
        if not math.isfinite(age_s) or age_s < 0.0 or age_s > _LIVE_LOWSTATE_MAX_AGE_S:
            return self._live_low_state_result(
                code="low_state_stale",
                mode_machine=mode_machine,
                sample_age_ms=age_ms,
                arm_joint_count=arm_joint_count,
                motor_joint_count=motor_joint_count,
            )
        if mode_machine != _REQUIRED_MODE_MACHINE:
            return self._live_low_state_result(
                code="mode_machine_not_ai",
                mode_machine=mode_machine,
                sample_age_ms=age_ms,
                arm_joint_count=arm_joint_count,
                motor_joint_count=motor_joint_count,
            )
        return self._live_low_state_result(
            code=None,
            mode_machine=mode_machine,
            sample_age_ms=age_ms,
            arm_joint_count=arm_joint_count,
            motor_joint_count=motor_joint_count,
        )

    @staticmethod
    def _finite_vector_count(value: object, *, expected: int) -> int | None:
        if isinstance(value, (str, bytes, bytearray)):
            return None
        try:
            values = list(value)
        except (TypeError, ValueError):
            return None
        if len(values) != expected:
            return None
        for item in values:
            if (
                isinstance(item, bool)
                or not isinstance(item, (int, float))
                or not math.isfinite(float(item))
            ):
                return None
        return len(values)

    @staticmethod
    def _live_low_state_result(
        *,
        code: str | None,
        mode_machine: int | None = None,
        sample_age_ms: float | None = None,
        arm_joint_count: int | None = None,
        motor_joint_count: int | None = None,
    ) -> dict:
        return {
            "ready": code is None,
            "code": code,
            "mode_machine": mode_machine,
            "required_mode_machine": _REQUIRED_MODE_MACHINE,
            "sample_age_ms": sample_age_ms,
            "arm_joint_count": arm_joint_count,
            "motor_joint_count": motor_joint_count,
        }

    def _validate_factory_preflight(self, preflight: dict) -> dict:
        if not isinstance(preflight, dict):
            raise ValueError("startup_preflight must be an object")
        result = copy.deepcopy(preflight)
        required = {
            "schema",
            "ready",
            "stage",
            "code",
            "message",
            "mode",
            "profile_id",
            "hardware_output",
            "publisher_created",
            "low_state",
            "ik",
            "identity",
            "dispatch",
        }
        if set(result) != required:
            raise ValueError("startup_preflight fields do not match the V1 contract")
        if (
            result["schema"] != PREFLIGHT_SCHEMA
            or result["mode"] != self.runtime.mode
            or result["profile_id"] != PROFILE_ID
            or result["hardware_output"] is not self.runtime.actuation_enabled
            or result["publisher_created"] is not self.runtime.actuation_enabled
            or result["ready"] is not False
            or result["stage"] != "service_startup"
            or result["code"] is not None
            or result["message"] is not None
        ):
            raise ValueError("startup_preflight factory binding is invalid")
        low_state = result["low_state"]
        ik = result["ik"]
        identity = result["identity"]
        dispatch = result["dispatch"]
        sample_age_ms = (
            low_state.get("sample_age_ms") if isinstance(low_state, dict) else None
        )
        warmup_ms = ik.get("warmup_ms") if isinstance(ik, dict) else None
        expected_dispatch_kind = (
            "hardware" if self.runtime.actuation_enabled else "recording"
        )
        if (
            not isinstance(low_state, dict)
            or set(low_state) != {
                "ready",
                "mode_machine",
                "required_mode_machine",
                "sample_age_ms",
                "arm_joint_count",
                "motor_joint_count",
            }
            or low_state.get("ready") is not True
            or low_state.get("mode_machine") != 4
            or low_state.get("required_mode_machine") != 4
            or low_state.get("arm_joint_count") != 10
            or low_state.get("motor_joint_count") != 35
            or isinstance(sample_age_ms, bool)
            or not isinstance(sample_age_ms, (int, float))
            or not math.isfinite(float(sample_age_ms))
            or not 0.0 <= float(sample_age_ms) <= 100.0
            or not isinstance(ik, dict)
            or set(ik) != {"ready", "warmup_ms", "model", "solver"}
            or ik.get("ready") is not True
            or ik.get("model") != "g1_body23.urdf"
            or ik.get("solver") != "pinocchio-casadi-ipopt"
            or isinstance(warmup_ms, bool)
            or not isinstance(warmup_ms, (int, float))
            or not math.isfinite(float(warmup_ms))
            or not 0.0 <= float(warmup_ms) <= 600_000.0
            or not isinstance(identity, dict)
            or set(identity) != {"driver_id", "robot_id", "capability_digest"}
            or identity.get("driver_id") != self.runtime.driver_id
            or identity.get("robot_id") != self.runtime.robot_id
            or identity.get("capability_digest") != self.runtime.capability_digest
            or not isinstance(dispatch, dict)
            or set(dispatch) != {
                "ready",
                "kind",
                "state",
                "stop_acknowledged",
                "fault_code",
            }
            or dispatch.get("ready") is not True
            or dispatch.get("kind") != expected_dispatch_kind
            or dispatch.get("state") != "safe_unarmed"
            or dispatch.get("stop_acknowledged") is not True
            or dispatch.get("fault_code") is not None
        ):
            raise ValueError("startup_preflight safety evidence is invalid")
        try:
            json.dumps(result, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ValueError("startup_preflight must be bounded JSON data") from exc
        return result

    def _complete_startup_preflight(self, preflight: dict) -> dict:
        tools = self.get_tools()
        tool_names = ["teleop_session", "teleop_state", "teleop_ik"]
        if [tool.get("name") for tool in tools] != tool_names:
            raise RuntimeError("G1 teleoperation descriptor tools are incomplete")
        expected_identity = {
            "driver_id": self.runtime.driver_id,
            "robot_id": self.runtime.robot_id,
            "profile_id": self.runtime.profile_id,
            "capability_digest": self.runtime.capability_digest,
            "mode": self.runtime.mode,
            "actuation_enabled": self.runtime.actuation_enabled,
        }
        for tool in tools:
            x_teleop = tool.get("x-teleop")
            if not isinstance(x_teleop, dict):
                raise RuntimeError("G1 teleoperation descriptor binding is missing")
            if any(x_teleop.get(key) != value for key, value in expected_identity.items()):
                raise RuntimeError("G1 teleoperation descriptor identity is inconsistent")
            signaling = x_teleop.get("signaling")
            if (
                not isinstance(signaling, dict)
                or signaling.get("audience") != SIGNALING_AUDIENCE
                or signaling.get("path") != "/ws/teleop-capture"
            ):
                raise RuntimeError("G1 teleoperation RTC descriptor is inconsistent")
        result = copy.deepcopy(preflight)
        result["descriptor"] = {
            "ready": True,
            "tool_names": tool_names,
            **expected_identity,
        }
        result["rtc"] = {
            "ready": True,
            "ticket_verifier": self.rtc.enabled,
            "event_loop_running": True,
            "audience": SIGNALING_AUDIENCE,
            "offer_path": None,
            "capture_path": "/ws/teleop-capture",
        }
        result["ready"] = True
        result["stage"] = "complete"
        result["code"] = None
        result["message"] = None
        try:
            json.dumps(result, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise RuntimeError("completed startup preflight is not JSON-safe") from exc
        return result

    def _stop_failed_loop_start(self) -> None:
        self._closed = True
        self._registration_stop.set()
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        if not self._thread.is_alive() and not self._loop.is_closed():
            self._loop.close()

    def _close_all_rtc(self) -> None:
        if self._loop.is_closed():
            return
        future = asyncio.run_coroutine_threadsafe(self.rtc.close_all(), self._loop)
        try:
            future.result(timeout=self._offer_timeout)
        except TimeoutError:
            future.cancel()

    def _run_async(self, awaitable, *, timeout: float | None = None):
        if self._loop.is_closed():
            raise RuntimeError("G1 teleoperation event loop is closed")
        future = asyncio.run_coroutine_threadsafe(awaitable, self._loop)
        try:
            return future.result(timeout=timeout or self._offer_timeout)
        except TimeoutError:
            future.cancel()
            raise

    @staticmethod
    def _pop_instance_id(args: dict) -> str | None:
        instance_id = args.pop("instance_id", None)
        if instance_id is None:
            return None
        if not isinstance(instance_id, str) or len(instance_id) > 128:
            raise ProtocolError(
                "invalid_arguments",
                "instance_id must be a string of at most 128 characters",
            )
        return instance_id

    @staticmethod
    def _require_exact(args: dict, expected: set[str], action: str) -> None:
        if set(args) != expected:
            raise ProtocolError(
                "invalid_arguments",
                f"{action} requires exactly {sorted(expected)}",
            )


__all__ = ["PREFLIGHT_SCHEMA", "G1TeleopService"]
