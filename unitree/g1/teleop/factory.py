"""Production composition root for teleoperation inside the root G1 Driver."""

from __future__ import annotations

import math
import os
import time
from pathlib import Path

import numpy as np

from .adapter import G1ControllerPoseMapper, G1DualArmAdapter
from .descriptor import PREFLIGHT_SCHEMA, PROFILE_ID
from .hardware import REQUIRED_MODE_MACHINE, G1ArmSdkPort, G1LowStateReader
from .ik import G123PinocchioIk
from .runtime import G1TeleopRuntime
from .service import G1TeleopService

_PREFLIGHT_STAGE_CODES = {
    "low_state_init": "low_state_reader_unavailable",
    "ik_init": "ik_dependencies_unavailable",
    "low_state_probe": "low_state_preflight_failed",
    "ik_warmup": "ik_warmup_failed",
    "live_publisher_init": "arm_sdk_publisher_unavailable",
    "runtime_startup": "final_dispatch_startup_failed",
    "service_startup": "rtc_or_descriptor_startup_failed",
}
_PREFLIGHT_PUBLIC_MESSAGES = {
    "low_state_reader_unavailable": "G1 LowState reader is unavailable",
    "ik_dependencies_unavailable": "G1 teleoperation IK dependencies are unavailable",
    "low_state_preflight_failed": "G1 LowState startup safety probe failed",
    "ik_warmup_failed": "G1 teleoperation IK warm-up failed",
    "arm_sdk_publisher_unavailable": "G1 arm publisher startup failed",
    "final_dispatch_startup_failed": "G1 final dispatch did not start safe",
    "rtc_or_descriptor_startup_failed": "G1 teleoperation service startup failed",
    "startup_preflight_failed": "G1 teleoperation startup preflight failed",
    "teleop_configuration_invalid": "G1 teleoperation configuration is invalid",
}


class G1TeleopPreflightError(RuntimeError):
    """Startup failure with a bounded, operator-safe status projection."""

    def __init__(
        self,
        *,
        stage: str,
        mode: str,
        message: str,
        publisher_created: bool = False,
    ):
        del message
        code = _PREFLIGHT_STAGE_CODES.get(stage, "startup_preflight_failed")
        public_message = _PREFLIGHT_PUBLIC_MESSAGES[code]
        super().__init__(public_message)
        self.preflight_status = {
            "schema": PREFLIGHT_SCHEMA,
            "ready": False,
            "stage": stage,
            "code": code,
            "message": public_message,
            "mode": mode,
            "profile_id": PROFILE_ID,
            "hardware_output": mode == "live",
            "publisher_created": bool(publisher_created),
        }


def project_preflight_error(exc: BaseException, *, mode: str = "unknown") -> dict:
    """Return JSON-safe startup evidence without exposing configuration secrets."""

    projected = getattr(exc, "preflight_status", None)
    if isinstance(projected, dict):
        result = dict(projected)
        result["message"] = _PREFLIGHT_PUBLIC_MESSAGES.get(
            result.get("code"),
            _PREFLIGHT_PUBLIC_MESSAGES["startup_preflight_failed"],
        )
        return result
    return {
        "schema": PREFLIGHT_SCHEMA,
        "ready": False,
        "stage": "configuration",
        "code": "teleop_configuration_invalid",
        "message": _PREFLIGHT_PUBLIC_MESSAGES["teleop_configuration_invalid"],
        "mode": mode if mode in ("shadow", "live") else "unknown",
        "profile_id": PROFILE_ID,
        "hardware_output": mode == "live",
        "publisher_created": False,
    }


def build_g1_teleop_service(config: dict) -> G1TeleopService | None:
    """Build one embedded service after the root process initializes DDS.

    Live output requires both ``mode: live`` and ``live.enabled: true``.  There
    is no fallback from a requested live profile to Shadow.
    """

    teleop = config.get("teleop")
    if teleop is None:
        return None
    if not isinstance(teleop, dict):
        raise ValueError("teleop config must be an object")
    if teleop.get("enabled") is not True:
        return None
    plugins = config.get("plugins", {})
    if not isinstance(plugins, dict):
        raise ValueError("plugins config must be an object")
    if plugins.get("arm", {}).get("enabled") is True:
        raise ValueError(
            "teleop.enabled=true conflicts with plugins.arm.enabled=true; "
            "the V1 arm authority must be exclusive"
        )
    mode = teleop.get("mode", "shadow")
    if mode not in ("shadow", "live"):
        raise ValueError("teleop.mode must be 'shadow' or 'live'")
    if teleop.get("profile_id", PROFILE_ID) != PROFILE_ID:
        raise ValueError(f"only teleop.profile_id={PROFILE_ID!r} is supported")
    if int(teleop.get("mode_machine", REQUIRED_MODE_MACHINE)) != REQUIRED_MODE_MACHINE:
        raise ValueError("G1_23 teleoperation requires mode_machine=4")
    if bool(teleop.get("base_enabled", False)) or bool(teleop.get("hands_enabled", False)):
        raise ValueError("G1_23 V1 is arm-only; base and hands must remain disabled")
    lease_timeout_ms = int(teleop.get("lease_timeout_ms", 1000))
    if not 750 <= lease_timeout_ms <= 10_000:
        raise ValueError("teleop.lease_timeout_ms must be in Core's [750, 10000] range")
    live = teleop.get("live", {})
    if not isinstance(live, dict):
        raise ValueError("teleop.live config must be an object")
    if mode == "live" and live.get("enabled") is not True:
        raise ValueError("teleop live output requires explicit teleop.live.enabled=true")
    velocity_limit = float(live.get("velocity_limit_rad_s", 0.5))
    if mode == "live" and not math.isclose(
        velocity_limit,
        0.5,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError(
            "unitree_g1_23_dual_arm_controller_v1 fixes "
            "velocity_limit_rad_s=0.5"
        )

    driver_token = os.environ.get("MOTUS_DRIVER_TOKEN")
    ticket_secret = os.environ.get("MOTUS_TELEOP_TICKET_SECRET")
    if not driver_token:
        raise ValueError("MOTUS_DRIVER_TOKEN is required when G1 teleoperation is enabled")
    if ticket_secret is None:
        raise ValueError(
            "MOTUS_TELEOP_TICKET_SECRET is required when G1 teleoperation is enabled"
        )

    resource_root = Path(__file__).resolve().parents[1] / "resource"
    urdf_path = Path(str(teleop.get("urdf_path", resource_root / "g1_body23.urdf")))
    low_state = None
    arm_sdk = None
    runtime = None
    stage = "low_state_init"
    try:
        low_state = G1LowStateReader(
            ready_timeout_s=float(teleop.get("lowstate_ready_timeout_seconds", 2.0))
        )
        stage = "ik_init"
        ik_solver = G123PinocchioIk(urdf_path)
        stage = "low_state_probe"
        initial_state = low_state.read_arm_state()
        initial_q = np.asarray(initial_state.get("joint_positions"), dtype=float)
        initial_dq = np.asarray(initial_state.get("joint_velocities"), dtype=float)
        all_joint_q = np.asarray(
            initial_state.get("all_joint_positions"),
            dtype=float,
        )
        sampled = float(initial_state.get("sample_monotonic"))
        mode_machine = initial_state.get("mode_machine")
        age = time.monotonic() - sampled
        if (
            initial_q.shape != (10,)
            or initial_dq.shape != (10,)
            or all_joint_q.shape != (35,)
            or not np.all(np.isfinite(initial_q))
            or not np.all(np.isfinite(initial_dq))
            or not np.all(np.isfinite(all_joint_q))
            or not math.isfinite(sampled)
            or age < 0.0
            or age > 0.1
            or mode_machine != REQUIRED_MODE_MACHINE
        ):
            raise RuntimeError("fresh G1 LowState mode_machine=4 is required")
        stage = "ik_warmup"
        warm_up = getattr(ik_solver, "warm_up", None)
        if not callable(warm_up):
            raise RuntimeError("G1_23 IK does not expose the required warm-up probe")
        warmup_result = warm_up(
            initial_q,
            initial_dq,
        )
        warmup_ms = (
            warmup_result.get("warmup_ms")
            if isinstance(warmup_result, dict)
            else None
        )
        if (
            not isinstance(warmup_result, dict)
            or warmup_result.get("ready") is not True
            or isinstance(warmup_ms, bool)
            or not isinstance(warmup_ms, (int, float))
            or not math.isfinite(float(warmup_ms))
            or float(warmup_ms) < 0.0
            or float(warmup_ms) > 600_000.0
            or ik_solver.ready() is not True
        ):
            raise RuntimeError("G1_23 IK warm-up did not establish readiness")
        if mode == "live":
            stage = "live_publisher_init"
            arm_sdk = G1ArmSdkPort(
                low_state,
                control_hz=float(live.get("control_hz", 250.0)),
                ramp_seconds=float(live.get("ramp_seconds", 2.0)),
                release_seconds=float(live.get("release_seconds", 0.05)),
                velocity_limit_rad_s=velocity_limit,
                command_timeout_s=float(
                    live.get(
                        "command_timeout_seconds",
                        int(teleop.get("pose_timeout_ms", 200)) / 1000.0,
                    )
                ),
            )
        adapter = G1DualArmAdapter(
            mode=mode,
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=ik_solver,
            low_state_reader=low_state,
            arm_sdk=arm_sdk,
        )
        driver_id = str(
            os.environ.get("MOTUS_DRIVER_ID", teleop.get("driver_id", "unitree-g1"))
        )
        driver_name = str(teleop.get("driver_name", "Unitree G1 Bundle"))
        robot_id = str(
            os.environ.get("MOTUS_ROBOT_ID", teleop.get("robot_id", driver_id))
        )
        stage = "runtime_startup"
        runtime = G1TeleopRuntime(
            mode=mode,
            adapter=adapter,
            driver_id=driver_id,
            driver_name=driver_name,
            robot_id=robot_id,
            lease_timeout_ms=lease_timeout_ms,
            pose_timeout_ms=int(teleop.get("pose_timeout_ms", 200)),
            watchdog_interval_ms=int(teleop.get("watchdog_interval_ms", 25)),
            dispatch_io_timeout_ms=int(teleop.get("dispatch_io_timeout_ms", 150)),
            dispatch_ack_timeout_ms=int(teleop.get("dispatch_ack_timeout_ms", 200)),
        )
        startup = runtime.status()
        if (
            startup.get("state") != "idle"
            or startup.get("dispatch", {}).get("ready") is not True
            or startup.get("dispatch", {}).get("stop_acknowledged") is not True
        ):
            raise RuntimeError("G1 teleoperation final dispatch did not start safe")
        startup_preflight = {
            "schema": PREFLIGHT_SCHEMA,
            "ready": False,
            "stage": "service_startup",
            "code": None,
            "message": None,
            "mode": mode,
            "profile_id": PROFILE_ID,
            "hardware_output": mode == "live",
            "publisher_created": arm_sdk is not None,
            "low_state": {
                "ready": True,
                "mode_machine": int(mode_machine),
                "required_mode_machine": REQUIRED_MODE_MACHINE,
                "sample_age_ms": round(age * 1000.0, 3),
                "arm_joint_count": int(initial_q.size),
                "motor_joint_count": int(all_joint_q.size),
            },
            "ik": {
                "ready": True,
                "warmup_ms": round(float(warmup_ms), 3),
                "model": "g1_body23.urdf",
                "solver": "pinocchio-casadi-ipopt",
            },
            "identity": {
                "driver_id": driver_id,
                "robot_id": robot_id,
                "capability_digest": runtime.capability_digest,
            },
            "dispatch": {
                "ready": True,
                "kind": startup["dispatch"]["kind"],
                "state": startup["dispatch"]["state"],
                "stop_acknowledged": True,
                "fault_code": None,
            },
        }
        stage = "service_startup"
        return G1TeleopService(
            runtime,
            driver_token=driver_token,
            ticket_secret=ticket_secret,
            startup_preflight=startup_preflight,
            live_low_state_probe=(
                low_state.read_arm_state if mode == "live" else None
            ),
            offer_timeout_s=float(teleop.get("offer_timeout_seconds", 5.0)),
            ticket_ttl_max_seconds=int(teleop.get("ticket_ttl_max_seconds", 30)),
            ticket_replay_cache_entries=int(
                teleop.get("ticket_replay_cache_entries", 4096)
            ),
        )
    except Exception as exc:
        if runtime is not None:
            runtime.close()
        else:
            if arm_sdk is not None:
                arm_sdk.close()
            if low_state is not None:
                low_state.close()
        if isinstance(exc, G1TeleopPreflightError):
            raise
        raise G1TeleopPreflightError(
            stage=stage,
            mode=mode,
            message=str(exc),
            publisher_created=arm_sdk is not None,
        ) from exc


__all__ = [
    "G1TeleopPreflightError",
    "build_g1_teleop_service",
    "project_preflight_error",
]
