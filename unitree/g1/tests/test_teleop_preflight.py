from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import teleop.factory as factory_module
import teleop.ik as ik_module
import teleop.service as service_module
import yaml
from teleop.descriptor import PREFLIGHT_SCHEMA, PROFILE_ID
from teleop.preflight import (
    EXPECTED_CASADI_VERSION,
    EXPECTED_NUMPY_VERSION,
    EXPECTED_PINOCCHIO_VERSION,
    PreflightCommandError,
    main,
    run_shadow_preflight,
    run_target_image_preflight,
)


class ImageIk:
    def __init__(self, urdf_path):
        self.urdf_path = str(urdf_path)
        self.warmed = False

    def warm_up(self, q, dq):
        self.warmed = True
        return {"ready": True, "warmup_ms": 12.5}

    def ready(self):
        return self.warmed


class ShadowService:
    def __init__(self):
        self.closed = False

    def preflight_status(self):
        return {
            "schema": PREFLIGHT_SCHEMA,
            "ready": True,
            "stage": "complete",
            "code": None,
            "message": None,
            "mode": "shadow",
            "profile_id": PROFILE_ID,
            "hardware_output": False,
            "publisher_created": False,
        }

    def close(self):
        self.closed = True


class G1PreflightCommandTests(unittest.TestCase):
    def _dependency_modules(
        self,
        *,
        numpy_version="1.26.4",
        casadi_version="3.6.7",
        pinocchio_version="3.1.0",
    ):
        pinocchio = types.ModuleType("pinocchio")
        pinocchio.__version__ = pinocchio_version
        pinocchio.casadi = types.SimpleNamespace()
        return {
            "aiortc": types.SimpleNamespace(__version__="1.14.0"),
            "av": types.SimpleNamespace(__version__="14.0.0"),
            "casadi": types.SimpleNamespace(__version__=casadi_version),
            "cryptography": types.SimpleNamespace(__version__="46.0.0"),
            "numpy": types.SimpleNamespace(
                __version__=numpy_version,
                zeros=lambda size: [0.0] * size,
            ),
            "pinocchio": pinocchio,
            "teleop.factory": factory_module,
            "teleop.service": service_module,
        }

    def test_target_image_entry_runs_cold_ik_and_returns_json_safe_evidence(self):
        modules = self._dependency_modules()
        with (
            patch.dict(sys.modules, {"pinocchio": modules["pinocchio"]}),
            patch.object(ik_module, "G123PinocchioIk", ImageIk),
            patch("teleop.preflight.importlib.import_module", side_effect=modules.__getitem__),
        ):
            result = run_target_image_preflight("/work/resource/g1_body23.urdf")
        self.assertTrue(result["ready"])
        self.assertEqual("image", result["mode"])
        self.assertFalse(result["hardware_output"])
        self.assertFalse(result["publisher_created"])
        self.assertEqual(12.5, result["ik"]["warmup_ms"])
        self.assertEqual(EXPECTED_NUMPY_VERSION, result["dependencies"]["numpy"])
        self.assertEqual(EXPECTED_CASADI_VERSION, result["dependencies"]["casadi"])
        self.assertEqual(
            EXPECTED_PINOCCHIO_VERSION,
            result["dependencies"]["pinocchio"],
        )
        json.dumps(result, allow_nan=False)

    def test_target_image_rejects_casadi_372_before_pinocchio_casadi_import(self):
        modules = self._dependency_modules(casadi_version="3.7.2")
        pinocchio = modules["pinocchio"]
        del pinocchio.casadi
        bridge_accesses = []

        def reject_bridge_import(name):
            if name == "casadi":
                bridge_accesses.append(name)
                raise RuntimeError("incompatible pinocchio.casadi ABI")
            raise AttributeError(name)

        pinocchio.__getattr__ = reject_bridge_import
        with (
            patch.dict(sys.modules, {"pinocchio": pinocchio}),
            patch("teleop.preflight.importlib.import_module", side_effect=modules.__getitem__),
        ):
            with self.assertRaises(PreflightCommandError) as caught:
                run_target_image_preflight("unused.urdf")
        self.assertEqual("casadi_version_mismatch", caught.exception.preflight_status["code"])
        self.assertEqual(
            "Target image CasADi version is incompatible",
            caught.exception.preflight_status["message"],
        )
        self.assertEqual([], bridge_accesses)

    def test_target_image_other_numeric_mismatches_have_stable_public_codes(self):
        for kwargs, code, message in (
            (
                {"numpy_version": "2.2.6"},
                "numpy_version_mismatch",
                "Target image NumPy version is incompatible",
            ),
            (
                {"pinocchio_version": "3.2.0"},
                "pinocchio_version_mismatch",
                "Target image Pinocchio version is incompatible",
            ),
        ):
            with self.subTest(code=code):
                modules = self._dependency_modules(**kwargs)
                with (
                    patch.dict(sys.modules, {"pinocchio": modules["pinocchio"]}),
                    patch(
                        "teleop.preflight.importlib.import_module",
                        side_effect=modules.__getitem__,
                    ),
                ):
                    with self.assertRaises(PreflightCommandError) as caught:
                        run_target_image_preflight("unused.urdf")
                self.assertEqual(code, caught.exception.preflight_status["code"])
                self.assertEqual(message, caught.exception.preflight_status["message"])

    def test_target_image_bridge_import_failure_has_stable_dependency_code(self):
        modules = self._dependency_modules()
        del modules["pinocchio"].casadi
        with (
            patch.dict(sys.modules, {"pinocchio": modules["pinocchio"]}),
            patch("teleop.preflight.importlib.import_module", side_effect=modules.__getitem__),
        ):
            with self.assertRaises(PreflightCommandError) as caught:
                run_target_image_preflight("unused.urdf")
        self.assertEqual(
            "pinocchio_casadi_import_failed",
            caught.exception.preflight_status["code"],
        )
        self.assertEqual(
            "Target image Pinocchio/CasADi bridge is unavailable",
            caught.exception.preflight_status["message"],
        )

    def test_shadow_entry_reuses_factory_and_closes_without_hardware_output(self):
        service = ShadowService()
        with tempfile.NamedTemporaryFile("w", suffix=".yaml") as config:
            config.write("teleop:\n  enabled: true\n  mode: shadow\n")
            config.flush()
            with (
                patch(
                    "teleop.preflight._initialize_dds"
                ) as initialize_dds,
                patch(
                    "teleop.factory.build_g1_teleop_service",
                    return_value=service,
                ) as build_service,
            ):
                result = run_shadow_preflight(
                    config_path=config.name,
                    network_interface="eth0",
                )
        initialize_dds.assert_called_once_with("eth0")
        build_service.assert_called_once()
        self.assertTrue(service.closed)
        self.assertTrue(result["ready"])
        self.assertFalse(result["publisher_created"])

    def test_shadow_entry_rejects_live_before_initializing_dds(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml") as config:
            config.write("teleop:\n  enabled: true\n  mode: live\n  live:\n    enabled: true\n")
            config.flush()
            with patch(
                "teleop.preflight._initialize_dds"
            ) as initialize_dds:
                with self.assertRaises(PreflightCommandError) as caught:
                    run_shadow_preflight(
                        config_path=config.name,
                        network_interface="eth0",
                    )
        initialize_dds.assert_not_called()
        self.assertEqual("shadow_mode_required", caught.exception.preflight_status["code"])

    def test_cli_projects_failure_as_one_json_object(self):
        secret = "operator-secret-in-private-path"
        failure = PreflightCommandError(
            stage="target_dependencies",
            code="target_dependency_import_failed",
            message=f"pinocchio unavailable at /private/{secret}",
        )
        output = io.StringIO()
        with (
            patch("teleop.preflight.run_target_image_preflight", side_effect=failure),
            contextlib.redirect_stdout(output),
        ):
            exit_code = main(["image", "--urdf", "unused.urdf"])
        self.assertEqual(1, exit_code)
        payload = json.loads(output.getvalue())
        self.assertFalse(payload["ready"])
        self.assertEqual("target_dependency_import_failed", payload["code"])
        self.assertEqual(
            "Target image dependencies are unavailable",
            payload["message"],
        )
        self.assertNotIn(secret, output.getvalue())
        self.assertNotIn("/private/", output.getvalue())

    def test_docker_build_uses_the_executable_target_image_smoke(self):
        dockerfile = (Path(__file__).parents[1] / "Dockerfile").read_text()
        self.assertIn(
            "/opt/g1-teleop/bin/python -m teleop.preflight image",
            dockerfile,
        )
        self.assertIn("--urdf /work/resource/g1_body23.urdf", dockerfile)
        self.assertNotIn("solver = G123PinocchioIk", dockerfile)

    def test_compose_has_no_obsolete_driver_or_ticket_secret_contract(self):
        service = (Path(__file__).parents[1] / "deploy" / "service.yml").read_text()
        self.assertNotIn("MOTUS_DRIVER_TOKEN", service)
        self.assertNotIn("MOTUS_TELEOP_TICKET_SECRET", service)
        self.assertNotIn("MOTUS_DRIVER_TOKEN is required", service)

    def test_target_image_manifest_owns_the_supported_numeric_abi(self):
        environment = yaml.safe_load(
            (Path(__file__).parents[1] / "environment.teleop.yml").read_text()
        )
        self.assertEqual("g1-teleop", environment["name"])
        self.assertEqual(["conda-forge", "nodefaults"], environment["channels"])
        self.assertEqual(
            [
                "python=3.10",
                "pinocchio=3.1.0",
                "casadi=3.6.7",
                "numpy=1.26.4",
                "pip",
            ],
            environment["dependencies"],
        )

        requirements = {
            line.strip()
            for line in (Path(__file__).parents[1] / "requirements.txt")
            .read_text()
            .splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        }
        self.assertIn("numpy==1.26.4", requirements)
        self.assertFalse(any(line.startswith("casadi") for line in requirements))
        self.assertFalse(any(line.startswith("pinocchio") for line in requirements))

    def test_docker_uses_only_the_pinned_conda_numeric_runtime(self):
        dockerfile = (Path(__file__).parents[1] / "Dockerfile").read_text()
        self.assertIn("ARG MINIFORGE_VERSION=24.7.1-2", dockerfile)
        self.assertIn("amd64)", dockerfile)
        self.assertIn('miniforge_arch="x86_64"', dockerfile)
        self.assertIn("arm64)", dockerfile)
        self.assertIn('miniforge_arch="aarch64"', dockerfile)
        self.assertIn(
            "636f7faca2d51ee42b4640ce160c751a46d57621ef4bf14378704c87c5db4fe3",
            dockerfile,
        )
        self.assertIn(
            "7bf60bce50f57af7ea4500b45eeb401d9350011ab34c9c45f736647d8dba9021",
            dockerfile,
        )
        self.assertIn('echo "unsupported TARGETARCH: ${TARGETARCH}"', dockerfile)
        self.assertIn("exit 64", dockerfile)
        self.assertIn("/opt/miniforge3/bin/conda env create", dockerfile)
        self.assertIn("--prefix /opt/g1-teleop", dockerfile)
        self.assertNotIn("ros-humble-pinocchio", dockerfile)
        self.assertNotIn("pip3 install", dockerfile)
        self.assertNotIn("python3 -m pip", dockerfile)
        self.assertEqual(
            2,
            dockerfile.count("/opt/g1-teleop/bin/python -m pip install"),
        )
        self.assertEqual(
            2,
            dockerfile.count(
                "export PYTHONPATH=/opt/g1-teleop/lib/python3.10/site-packages:/work:"
            ),
        )
        self.assertNotIn("LD_LIBRARY_PATH=/opt/g1-teleop/lib", dockerfile)
        self.assertEqual(
            2,
            dockerfile.count("export LD_LIBRARY_PATH=/opt/cyclonedds/lib:"),
        )
        self.assertIn(
            "exec /opt/g1-teleop/bin/python /work/main.py",
            dockerfile,
        )


if __name__ == "__main__":
    unittest.main()
