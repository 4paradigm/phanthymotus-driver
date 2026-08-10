"""G1 teleoperation MCP facade and in-process aiortc event loop."""

from __future__ import annotations

import asyncio
import copy
import hmac
import json
import math
import re
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
from .protocol import ProtocolError, TicketCodec, TicketVerifier
from .rtc import RtcManager, RtcRequestError
from .runtime import G1TeleopRuntime

_IDENTITY_KEYS = {"boot_id", "session_id", "epoch", "fence"}
_DRIVER_BEARER_RE = re.compile(r"^[A-Za-z0-9._~+/=-]{24,4096}$")
_REQUIRED_MODE_MACHINE = 4
_LIVE_LOWSTATE_MAX_AGE_S = 0.1


class G1TeleopService:
    """One teleoperation service embedded in the existing root G1 Driver."""

    def __init__(
        self,
        runtime: G1TeleopRuntime,
        *,
        driver_token: str,
        ticket_secret: str,
        startup_preflight: dict,
        live_low_state_probe: Callable[[], Mapping[str, object]] | None = None,
        offer_timeout_s: float = 5.0,
        ticket_ttl_max_seconds: int = 30,
        ticket_replay_cache_entries: int = 4096,
    ):
        if not isinstance(driver_token, str) or not _DRIVER_BEARER_RE.fullmatch(driver_token):
            raise ValueError(
                "MOTUS_DRIVER_TOKEN must contain 24-4096 restricted ASCII Bearer characters"
            )
        self.runtime = runtime
        if runtime.actuation_enabled:
            if not callable(live_low_state_probe):
                raise ValueError("Live service requires a read-only LowState health probe")
        elif live_low_state_probe is not None:
            raise ValueError("Shadow service forbids a hardware LowState health probe")
        self._live_low_state_probe = live_low_state_probe
        self._driver_token = driver_token
        self._offer_timeout = float(offer_timeout_s)
        validated_preflight = self._validate_factory_preflight(startup_preflight)
        verifier = TicketVerifier(
            TicketCodec(ticket_secret),
            audience=SIGNALING_AUDIENCE,
            max_ttl_seconds=int(ticket_ttl_max_seconds),
            max_replay_entries=int(ticket_replay_cache_entries),
        )
        self.rtc = RtcManager(runtime, verifier)
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
            self._startup_preflight = self._complete_startup_preflight(
                self._startup_preflight
            )
        except Exception:
            self._stop_failed_loop_start()
            raise

    @property
    def driver_token(self) -> str:
        """Bearer credential used by Core for this embedded Driver."""

        return self._driver_token

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

    def authorized(self, authorization_header: str | None) -> bool:
        if not isinstance(authorization_header, str) or not authorization_header.startswith("Bearer "):
            return False
        return hmac.compare_digest(authorization_header[7:], self._driver_token)

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
        if tool_name == "teleop_state":
            if args:
                raise ProtocolError("invalid_arguments", "teleop_state accepts no arguments")
            return self.runtime.status()
        if tool_name != "teleop_session":
            raise ProtocolError("unknown_tool", f"unknown tool: {tool_name}")
        action = args.pop("action", None)
        if not isinstance(action, str):
            raise ProtocolError("missing_action", "teleop_session requires an action")
        if action in ("prepare_shadow", "prepare_live"):
            self._require_exact(args, {"session_id", "epoch", "fence"}, action)
            result = self.runtime.prepare(action.removeprefix("prepare_"), args)
            self._close_all_rtc()
            return result
        if action in ("heartbeat", "pause", "release", "soft_stop"):
            self._require_exact(args, _IDENTITY_KEYS, action)
            operation = getattr(self.runtime, action)
            result = operation(args)
            if action in ("pause", "release", "soft_stop"):
                self._close_all_rtc()
            return result
        if action == "status":
            self._require_exact(args, set(), action)
            return self.runtime.status()
        if action == "stop":
            self._require_exact(args, set(), action)
            result = self.runtime.release(lifecycle=True)
            self._close_all_rtc()
            return result
        raise ProtocolError("unknown_action", f"unknown teleop_session action: {action}")

    def accept_offer(self, payload: Any) -> dict:
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
        if [tool.get("name") for tool in tools] != ["teleop_session", "teleop_state"]:
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
                or signaling.get("path") != "/offer"
            ):
                raise RuntimeError("G1 teleoperation RTC descriptor is inconsistent")
        result = copy.deepcopy(preflight)
        result["descriptor"] = {
            "ready": True,
            "tool_names": ["teleop_session", "teleop_state"],
            **expected_identity,
        }
        result["rtc"] = {
            "ready": True,
            "ticket_verifier": self.rtc.enabled,
            "event_loop_running": True,
            "audience": SIGNALING_AUDIENCE,
            "offer_path": "/offer",
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

    @staticmethod
    def _require_exact(args: dict, expected: set[str], action: str) -> None:
        if set(args) != expected:
            raise ProtocolError(
                "invalid_arguments",
                f"{action} requires exactly {sorted(expected)}",
            )


__all__ = ["PREFLIGHT_SCHEMA", "G1TeleopService"]
