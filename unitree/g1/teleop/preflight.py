"""Executable G1 target-image and zero-output Shadow startup probes."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import platform
import time
from pathlib import Path

from .descriptor import PREFLIGHT_SCHEMA, PROFILE_ID

EXPECTED_NUMPY_VERSION = "1.26.4"
EXPECTED_CASADI_VERSION = "3.6.7"
EXPECTED_PINOCCHIO_VERSION = "3.1.0"

_PREFLIGHT_PUBLIC_MESSAGES = {
    "target_dependency_import_failed": "Target image dependencies are unavailable",
    "numpy_version_mismatch": "Target image NumPy version is incompatible",
    "casadi_version_mismatch": "Target image CasADi version is incompatible",
    "pinocchio_version_mismatch": "Target image Pinocchio version is incompatible",
    "pinocchio_casadi_import_failed": "Target image Pinocchio/CasADi bridge is unavailable",
    "target_ik_warmup_failed": "Target image IK warm-up failed",
    "shadow_config_load_failed": "G1 Shadow configuration could not be loaded",
    "shadow_config_invalid": "G1 Shadow configuration is invalid",
    "shadow_mode_required": "G1 Shadow preflight requires Shadow mode",
    "network_interface_required": "G1 network interface is required",
    "shadow_dds_init_failed": "G1 Shadow DDS startup failed",
    "shadow_startup_failed": "G1 Shadow startup failed",
    "preflight_command_failed": "G1 teleoperation preflight command failed",
}


class PreflightCommandError(RuntimeError):
    def __init__(
        self,
        *,
        stage: str,
        code: str,
        message: str,
        mode: str = "image",
    ):
        del message
        public_message = _PREFLIGHT_PUBLIC_MESSAGES.get(
            code,
            _PREFLIGHT_PUBLIC_MESSAGES["preflight_command_failed"],
        )
        super().__init__(public_message)
        self.preflight_status = {
            "schema": PREFLIGHT_SCHEMA,
            "ready": False,
            "stage": stage,
            "code": code,
            "message": public_message,
            "mode": mode,
            "profile_id": PROFILE_ID,
            "hardware_output": False,
            "publisher_created": False,
        }


def _initialize_dds(network_interface: str) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(0, network_interface)


def run_target_image_preflight(urdf_path: str | Path) -> dict:
    """Cold-import target dependencies and execute one real G1_23 IPOPT solve."""

    started = time.monotonic()
    try:
        dependencies = {
            name: importlib.import_module(name)
            for name in ("numpy", "casadi", "pinocchio")
        }
    except Exception as exc:
        raise PreflightCommandError(
            stage="target_dependencies",
            code="target_dependency_import_failed",
            message=f"{type(exc).__name__}: {exc}",
        ) from exc

    for dependency, expected, code in (
        ("numpy", EXPECTED_NUMPY_VERSION, "numpy_version_mismatch"),
        ("casadi", EXPECTED_CASADI_VERSION, "casadi_version_mismatch"),
        ("pinocchio", EXPECTED_PINOCCHIO_VERSION, "pinocchio_version_mismatch"),
    ):
        actual = str(getattr(dependencies[dependency], "__version__", "unknown"))
        if actual != expected:
            raise PreflightCommandError(
                stage="target_dependencies",
                code=code,
                message=f"expected {dependency}=={expected}, got {actual}",
            )

    try:
        dependencies.update({
            name: importlib.import_module(name)
            for name in (
                "aiortc",
                "av",
                "cryptography",
                "teleop.factory",
                "teleop.service",
            )
        })
    except Exception as exc:
        raise PreflightCommandError(
            stage="target_dependencies",
            code="target_dependency_import_failed",
            message=f"{type(exc).__name__}: {exc}",
        ) from exc

    try:
        from pinocchio import casadi as pinocchio_casadi  # noqa: F401
    except Exception as exc:
        raise PreflightCommandError(
            stage="target_dependencies",
            code="pinocchio_casadi_import_failed",
            message=f"{type(exc).__name__}: {exc}",
        ) from exc

    try:
        from .ik import G123PinocchioIk

        solver = G123PinocchioIk(urdf_path)
        numpy_module = dependencies["numpy"]
        warmup = solver.warm_up(numpy_module.zeros(10), numpy_module.zeros(10))
        warmup_ms = warmup.get("warmup_ms") if isinstance(warmup, dict) else None
        if (
            not isinstance(warmup, dict)
            or warmup.get("ready") is not True
            or solver.ready() is not True
            or isinstance(warmup_ms, bool)
            or not isinstance(warmup_ms, (int, float))
            or not math.isfinite(float(warmup_ms))
            or float(warmup_ms) < 0.0
            or float(warmup_ms) > 600_000.0
        ):
            raise RuntimeError("G1_23 cold IPOPT solve did not establish readiness")
    except Exception as exc:
        if isinstance(exc, PreflightCommandError):
            raise
        raise PreflightCommandError(
            stage="target_ik_warmup",
            code="target_ik_warmup_failed",
            message=f"{type(exc).__name__}: {exc}",
        ) from exc

    elapsed_ms = max(0.0, (time.monotonic() - started) * 1000.0)
    return {
        "schema": PREFLIGHT_SCHEMA,
        "ready": True,
        "stage": "complete",
        "code": None,
        "message": None,
        "mode": "image",
        "profile_id": PROFILE_ID,
        "hardware_output": False,
        "publisher_created": False,
        "platform": {
            "machine": platform.machine()[:64],
            "python": platform.python_version()[:32],
        },
        "dependencies": {
            name: str(getattr(module, "__version__", "available"))[:64]
            for name, module in sorted(dependencies.items())
        },
        "ik": {
            "ready": True,
            "warmup_ms": round(float(warmup_ms), 3),
            "total_preflight_ms": round(min(600_000.0, elapsed_ms), 3),
            "model": "g1_body23.urdf",
            "solver": "pinocchio-casadi-ipopt",
        },
    }


def run_shadow_preflight(*, config_path: str | Path, network_interface: str) -> dict:
    """Run the production Shadow factory once and return its startup evidence."""

    try:
        import yaml

        with Path(config_path).open(encoding="utf-8") as handle:
            config = yaml.safe_load(handle)
    except Exception as exc:
        raise PreflightCommandError(
            stage="shadow_configuration",
            code="shadow_config_load_failed",
            message=f"{type(exc).__name__}: {exc}",
            mode="shadow",
        ) from exc
    if not isinstance(config, dict):
        raise PreflightCommandError(
            stage="shadow_configuration",
            code="shadow_config_invalid",
            message="G1 config must be a YAML object",
            mode="shadow",
        )
    teleop = config.get("teleop")
    if (
        not isinstance(teleop, dict)
        or teleop.get("enabled") is not True
        or teleop.get("mode", "shadow") != "shadow"
    ):
        raise PreflightCommandError(
            stage="shadow_configuration",
            code="shadow_mode_required",
            message="preflight requires teleop.enabled=true and teleop.mode=shadow",
            mode="shadow",
        )
    if not isinstance(network_interface, str) or not network_interface.strip():
        raise PreflightCommandError(
            stage="shadow_configuration",
            code="network_interface_required",
            message="a non-empty G1 network interface is required",
            mode="shadow",
        )

    try:
        _initialize_dds(network_interface.strip())
    except Exception as exc:
        raise PreflightCommandError(
            stage="shadow_dds_init",
            code="shadow_dds_init_failed",
            message=f"{type(exc).__name__}: {exc}",
            mode="shadow",
        ) from exc

    service = None
    try:
        from .factory import build_g1_teleop_service

        service = build_g1_teleop_service(config)
        if service is None:
            raise RuntimeError("G1 Shadow factory returned no service")
        status = service.preflight_status()
        if (
            status.get("ready") is not True
            or status.get("mode") != "shadow"
            or status.get("hardware_output") is not False
            or status.get("publisher_created") is not False
        ):
            raise RuntimeError("G1 Shadow startup preflight did not reach zero-output readiness")
        return status
    except Exception as exc:
        if isinstance(getattr(exc, "preflight_status", None), dict):
            raise
        raise PreflightCommandError(
            stage="shadow_factory",
            code="shadow_startup_failed",
            message=f"{type(exc).__name__}: {exc}",
            mode="shadow",
        ) from exc
    finally:
        if service is not None:
            service.close()


def project_command_error(exc: BaseException) -> dict:
    projected = getattr(exc, "preflight_status", None)
    if isinstance(projected, dict):
        result = dict(projected)
        result["message"] = _PREFLIGHT_PUBLIC_MESSAGES.get(
            result.get("code"),
            _PREFLIGHT_PUBLIC_MESSAGES["preflight_command_failed"],
        )
        return result
    return {
        "schema": PREFLIGHT_SCHEMA,
        "ready": False,
        "stage": "preflight_command",
        "code": "preflight_command_failed",
        "message": _PREFLIGHT_PUBLIC_MESSAGES["preflight_command_failed"],
        "mode": "unknown",
        "profile_id": PROFILE_ID,
        "hardware_output": False,
        "publisher_created": False,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python3 -m teleop.preflight")
    commands = parser.add_subparsers(dest="command", required=True)
    image = commands.add_parser("image", help="cold-load target dependencies and IK")
    image.add_argument(
        "--urdf",
        default=str(Path(__file__).resolve().parents[1] / "resource" / "g1_body23.urdf"),
    )
    shadow = commands.add_parser("shadow", help="probe production zero-output Shadow startup")
    shadow.add_argument("--network-interface", required=True)
    shadow.add_argument(
        "--config",
        default=os.environ.get("CONFIG_PATH", "/work/config.teleop-shadow.example.yaml"),
    )
    args = parser.parse_args(argv)
    try:
        if args.command == "image":
            result = run_target_image_preflight(args.urdf)
        else:
            result = run_shadow_preflight(
                config_path=args.config,
                network_interface=args.network_interface,
            )
    except Exception as exc:
        print(json.dumps(project_command_error(exc), allow_nan=False, sort_keys=True))
        return 1
    print(json.dumps(result, allow_nan=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "PreflightCommandError",
    "main",
    "project_command_error",
    "run_shadow_preflight",
    "run_target_image_preflight",
]
