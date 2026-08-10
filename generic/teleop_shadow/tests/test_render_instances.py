from __future__ import annotations

import contextlib
import copy
import io
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import render_instances as renderer
import yaml
from render_instances import (
    CORE_CA_TARGET,
    RenderError,
    _validate_core_ca_file_on_host,
    atomic_write,
    load_instances,
    main,
    parse_instances_document,
    render_compose,
    require_distinct_input_output,
    validate_image,
)

ROOT = Path(__file__).resolve().parents[1]


def _instances_document() -> dict:
    return {
        "schema_version": 2,
        "core_ca_file": "/opt/phanthy-motus/data/certs/cert.pem",
        "instances": [
            {
                "service": "teleop-shadow-lab-b",
                "container": "motus-teleop-shadow-lab-b",
                "driver_id": "teleop-shadow-lab-b",
                "driver_name": "Lab B Quest Teleop Shadow",
                "robot_id": "teleop-shadow-lab-b",
                "mcp_port": 15712,
                "driver_token_env": "MOTUS_TELEOP_SHADOW_LAB_B_DRIVER_TOKEN",
                "ticket_secret_env": "MOTUS_TELEOP_SHADOW_LAB_B_TICKET_SECRET",
            },
            {
                "service": "teleop-shadow-lab-a",
                "container": "motus-teleop-shadow-lab-a",
                "driver_id": "teleop-shadow-lab-a",
                "driver_name": "Lab A Quest Teleop Shadow",
                "robot_id": "teleop-shadow-lab-a",
                "mcp_port": 15711,
                "driver_token_env": "MOTUS_TELEOP_SHADOW_LAB_A_DRIVER_TOKEN",
                "ticket_secret_env": "MOTUS_TELEOP_SHADOW_LAB_A_TICKET_SECRET",
            },
        ],
    }


class RenderInstancesTests(unittest.TestCase):
    def setUp(self):
        validation = mock.patch(
            "render_instances._validate_core_ca_file_on_host",
            side_effect=lambda path: str(path),
        )
        validation.start()
        self.addCleanup(validation.stop)

    def test_two_instances_render_canonical_safe_compose(self):
        deployment = parse_instances_document(_instances_document())
        rendered = render_compose(deployment, "registry.example/motus/teleop-shadow:1.2.3")
        compose = yaml.safe_load(rendered)

        self.assertEqual(
            ["teleop-shadow-lab-a", "teleop-shadow-lab-b"],
            list(compose["services"]),
        )
        expected = {
            "teleop-shadow-lab-a": (
                "motus-teleop-shadow-lab-a",
                "teleop-shadow-lab-a",
                "teleop-shadow-lab-a",
                15711,
                "MOTUS_TELEOP_SHADOW_LAB_A_DRIVER_TOKEN",
                "MOTUS_TELEOP_SHADOW_LAB_A_TICKET_SECRET",
            ),
            "teleop-shadow-lab-b": (
                "motus-teleop-shadow-lab-b",
                "teleop-shadow-lab-b",
                "teleop-shadow-lab-b",
                15712,
                "MOTUS_TELEOP_SHADOW_LAB_B_DRIVER_TOKEN",
                "MOTUS_TELEOP_SHADOW_LAB_B_TICKET_SECRET",
            ),
        }
        for service_name, (
            container,
            driver_id,
            robot_id,
            port,
            driver_token_env,
            ticket_secret_env,
        ) in expected.items():
            service = compose["services"][service_name]
            self.assertEqual("host", service["network_mode"])
            self.assertEqual(container, service["container_name"])
            self.assertNotIn("ports", service)
            self.assertNotIn("privileged", service)
            self.assertIn("MOTUS_BIND_HOST=127.0.0.1", service["environment"])
            self.assertIn(f"MOTUS_MCP_PORT={port}", service["environment"])
            self.assertIn(
                f"MOTUS_MCP_URL=http://localhost:{port}/mcp",
                service["environment"],
            )
            self.assertIn(f"MOTUS_DRIVER_ID={driver_id}", service["environment"])
            self.assertIn(f"MOTUS_ROBOT_ID={robot_id}", service["environment"])
            self.assertEqual(
                [
                    (
                        f"MOTUS_DRIVER_TOKEN=${{{driver_token_env}:?set {driver_token_env} "
                        "before Docker Compose validation}"
                    ),
                    (
                        "MOTUS_TELEOP_TICKET_SECRET="
                        f"${{{ticket_secret_env}:?set {ticket_secret_env} "
                        "before Docker Compose validation}"
                    ),
                ],
                [
                    entry
                    for entry in service["environment"]
                    if entry.startswith(("MOTUS_DRIVER_TOKEN=", "MOTUS_TELEOP_TICKET_SECRET="))
                ],
            )

            self.assertEqual(1, len(service["volumes"]))
            ca_mount = service["volumes"][0]
            self.assertEqual("bind", ca_mount["type"])
            self.assertEqual(deployment.core_ca_file, ca_mount["source"])
            self.assertEqual(CORE_CA_TARGET, ca_mount["target"])
            self.assertIs(True, ca_mount["read_only"])
            self.assertIs(False, ca_mount["bind"]["create_host_path"])

    def test_rendering_is_deterministic_across_input_order(self):
        first = _instances_document()
        second = copy.deepcopy(first)
        second["instances"].reverse()

        first_render = render_compose(parse_instances_document(first), "teleop-shadow:local")
        second_render = render_compose(parse_instances_document(second), "teleop-shadow:local")

        self.assertEqual(first_render, second_render)

    def test_every_runtime_identity_and_port_must_be_unique(self):
        for field in ("service", "container", "driver_id", "robot_id", "mcp_port"):
            with self.subTest(field=field):
                document = _instances_document()
                document["instances"][1][field] = document["instances"][0][field]
                with self.assertRaisesRegex(RenderError, field):
                    parse_instances_document(document)

    def test_every_secret_environment_reference_is_globally_unique(self):
        duplicate_cases = (
            ("driver_token_env", "driver_token_env"),
            ("ticket_secret_env", "ticket_secret_env"),
        )
        for source_field, destination_field in duplicate_cases:
            with self.subTest(source_field=source_field, destination_field=destination_field):
                document = _instances_document()
                document["instances"][1][destination_field] = (
                    document["instances"][0][source_field]
                )
                with self.assertRaisesRegex(RenderError, "secret environment name"):
                    parse_instances_document(document)

    def test_schema_v1_and_missing_secret_references_fail_with_migration_signal(self):
        document = _instances_document()
        document["schema_version"] = 1
        with self.assertRaisesRegex(RenderError, "schema_version must be exactly 2"):
            parse_instances_document(document)

        for field in ("driver_token_env", "ticket_secret_env"):
            with self.subTest(field=field):
                document = _instances_document()
                document["instances"][0].pop(field)
                with self.assertRaisesRegex(RenderError, field):
                    parse_instances_document(document)

    def test_unknown_fields_and_secret_fields_are_rejected(self):
        document = _instances_document()
        document["unknown"] = True
        with self.assertRaisesRegex(RenderError, "unknown fields"):
            parse_instances_document(document)

        document = _instances_document()
        document["instances"][0]["MOTUS_DRIVER_TOKEN"] = "must-not-be-here"
        with self.assertRaisesRegex(RenderError, "MOTUS_DRIVER_TOKEN"):
            parse_instances_document(document)

    def test_identifiers_ports_names_and_paths_reject_injection(self):
        mutations = (
            ("service", "Bad/Service"),
            ("container", "../container"),
            ("driver_id", "driver\nINJECTED=1"),
            ("robot_id", "${MOTUS_DRIVER_TOKEN}"),
            ("robot_id", "r" * 65),
            ("driver_name", "${MOTUS_DRIVER_TOKEN}"),
            ("mcp_port", 15699),
            ("mcp_port", 15800),
            ("mcp_port", "15711"),
            ("mcp_port", True),
            ("driver_token_env", "lowercase_secret"),
            ("driver_token_env", "MOTUS-DRIVER-TOKEN-A"),
            ("driver_token_env", "PATH"),
            ("driver_token_env", "MOTUS_DRIVER_TOKEN"),
            ("driver_token_env", "MOTUS_TELEOP_LAB_A_TICKET_SECRET"),
            ("ticket_secret_env", "${MOTUS_TELEOP_TICKET_SECRET}"),
            ("ticket_secret_env", "HOME"),
            ("ticket_secret_env", "MOTUS_TELEOP_TICKET_SECRET"),
            ("ticket_secret_env", "MOTUS_TELEOP_LAB_A_DRIVER_TOKEN"),
            ("ticket_secret_env", "T" * 129),
        )
        for field, value in mutations:
            with self.subTest(field=field, value=value):
                document = _instances_document()
                document["instances"][0][field] = value
                with self.assertRaises(RenderError):
                    parse_instances_document(document)

        for path in (
            "relative/cert.pem",
            "/opt/phanthy-motus/../secret.pem",
            "/opt/phanthy-motus/${CORE_CA}",
            "/opt//phanthy-motus/cert.pem",
            "/etc/shadow",
            "/opt/phanthy-motus/tls/private.key",
        ):
            with self.subTest(path=path):
                document = _instances_document()
                document["core_ca_file"] = path
                with self.assertRaises(RenderError):
                    parse_instances_document(document)

        for image in ("", "../image", "image\nservices:", "${SECRET_IMAGE}"):
            with self.subTest(image=image), self.assertRaises(RenderError):
                validate_image(image)

    def test_duplicate_yaml_keys_are_rejected(self):
        text = """\
schema_version: 2
core_ca_file: /opt/phanthy-motus/data/certs/cert.pem
core_ca_file: /tmp/overridden.pem
instances: []
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "instances.yml"
            path.write_text(text, encoding="utf-8")
            with self.assertRaisesRegex(RenderError, "duplicate key"):
                load_instances(path)

    def test_renderer_never_reads_or_materializes_process_secrets(self):
        deployment = parse_instances_document(_instances_document())
        secret_environment = {
            "MOTUS_TELEOP_SHADOW_LAB_A_DRIVER_TOKEN": "driver-secret-a-sentinel-0001",
            "MOTUS_TELEOP_SHADOW_LAB_A_TICKET_SECRET": (
                "ticket-secret-a-sentinel-at-least-32-bytes"
            ),
            "MOTUS_TELEOP_SHADOW_LAB_B_DRIVER_TOKEN": "driver-secret-b-sentinel-0002",
            "MOTUS_TELEOP_SHADOW_LAB_B_TICKET_SECRET": (
                "ticket-secret-b-sentinel-at-least-32-bytes"
            ),
            # The v1 deployment names must no longer influence schema v2 output.
            "MOTUS_DRIVER_TOKEN": "legacy-driver-secret-sentinel",
            "MOTUS_TELEOP_TICKET_SECRET": "legacy-ticket-secret-sentinel",
        }
        with mock.patch.dict(
            os.environ,
            secret_environment,
        ):
            rendered = render_compose(deployment, "teleop-shadow:local")

        for secret in secret_environment.values():
            self.assertNotIn(secret, rendered)
        self.assertNotIn("${MOTUS_DRIVER_TOKEN:?", rendered)
        self.assertNotIn("${MOTUS_TELEOP_TICKET_SECRET:?", rendered)
        for environment_name in secret_environment:
            if environment_name in ("MOTUS_DRIVER_TOKEN", "MOTUS_TELEOP_TICKET_SECRET"):
                continue
            self.assertEqual(1, rendered.count(f"${{{environment_name}:?"))

    def test_atomic_output_replaces_complete_file_and_leaves_no_temp(self):
        deployment = parse_instances_document(_instances_document())
        rendered = render_compose(deployment, "teleop-shadow:local")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "teleop-shadow.compose.yml"
            output.write_text("old-partial-content", encoding="utf-8")

            atomic_write(output, rendered)

            self.assertEqual(rendered, output.read_text(encoding="utf-8"))
            self.assertEqual(0o644, output.stat().st_mode & 0o777)
            self.assertEqual([], list(output.parent.glob(f".{output.name}.*.tmp")))

    def test_atomic_output_rejects_broken_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "teleop-shadow.compose.yml"
            missing = Path(directory) / "must-not-be-created"
            output.symlink_to(missing)

            with self.assertRaisesRegex(RenderError, "regular file path"):
                atomic_write(output, "services: {}\n")

            self.assertTrue(output.is_symlink())
            self.assertFalse(missing.exists())

    def test_input_output_aliases_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "instances.yml"
            source.write_text("schema_version: 2\n", encoding="utf-8")
            with self.assertRaisesRegex(RenderError, "must not replace"):
                require_distinct_input_output(source, source)

            hard_link = Path(directory) / "hard-link.yml"
            os.link(source, hard_link)
            with self.assertRaisesRegex(RenderError, "must not replace"):
                require_distinct_input_output(source, hard_link)

    def test_cli_stdout_and_dry_run_write_no_output_file(self):
        with tempfile.TemporaryDirectory() as directory:
            instances = Path(directory) / "instances.yml"
            instances.write_text(
                yaml.safe_dump(_instances_document(), sort_keys=False),
                encoding="utf-8",
            )
            for flag in ("--stdout", "--dry-run"):
                with self.subTest(flag=flag):
                    output = Path(directory) / f"{flag[2:]}.compose.yml"
                    stdout = io.StringIO()
                    stderr = io.StringIO()
                    with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                        return_code = main([
                            "--instances",
                            str(instances),
                            "--image",
                            "teleop-shadow:local",
                            "--output",
                            str(output),
                            flag,
                        ])
                    self.assertEqual(0, return_code, stderr.getvalue())
                    self.assertFalse(output.exists())
                    self.assertEqual(
                        ["teleop-shadow-lab-a", "teleop-shadow-lab-b"],
                        list(yaml.safe_load(stdout.getvalue())["services"]),
                    )
                    self.assertNotIn("secret-sentinel", stdout.getvalue())

    def test_cli_writes_a_complete_atomic_output_file(self):
        with tempfile.TemporaryDirectory() as directory:
            instances = Path(directory) / "instances.yml"
            output = Path(directory) / "teleop-shadow.compose.yml"
            instances.write_text(
                yaml.safe_dump(_instances_document(), sort_keys=False),
                encoding="utf-8",
            )

            stdout = io.StringIO()
            stderr = io.StringIO()
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                return_code = main([
                    "--instances",
                    str(instances),
                    "--image",
                    "teleop-shadow:local",
                    "--output",
                    str(output),
                ])

            self.assertEqual(0, return_code, stderr.getvalue())
            self.assertEqual("", stdout.getvalue())
            self.assertIn("rendered 2 teleop-shadow instances", stderr.getvalue())
            self.assertEqual(2, len(yaml.safe_load(output.read_text())["services"]))
            self.assertEqual([], list(output.parent.glob(f".{output.name}.*.tmp")))

    def test_cli_refuses_to_overwrite_its_instances_input(self):
        with tempfile.TemporaryDirectory() as directory:
            instances = Path(directory) / "instances.yml"
            original = yaml.safe_dump(_instances_document(), sort_keys=False)
            instances.write_text(original, encoding="utf-8")
            stderr = io.StringIO()

            with contextlib.redirect_stderr(stderr):
                return_code = main([
                    "--instances",
                    str(instances),
                    "--image",
                    "teleop-shadow:local",
                    "--output",
                    str(instances),
                ])

            self.assertEqual(2, return_code)
            self.assertIn("must not replace", stderr.getvalue())
            self.assertEqual(original, instances.read_text(encoding="utf-8"))

    def test_bundled_example_is_directly_renderable(self):
        deployment = load_instances(ROOT / "deploy/instances.example.yml")
        rendered = render_compose(deployment, "teleop-shadow:local")

        self.assertEqual(2, len(yaml.safe_load(rendered)["services"]))

    @unittest.skipUnless(shutil.which("docker"), "Docker Compose is unavailable")
    def test_rendered_document_passes_real_docker_compose_secret_preflight(self):
        version = subprocess.run(
            ["docker", "compose", "version"],
            check=False,
            capture_output=True,
            text=True,
        )
        if version.returncode != 0:
            self.skipTest("Docker Compose plugin is unavailable")
        deployment = parse_instances_document(_instances_document())
        rendered = render_compose(deployment, "teleop-shadow:local")
        with tempfile.TemporaryDirectory() as directory:
            compose = Path(directory) / "compose.yml"
            compose.write_text(rendered, encoding="utf-8")
            secret_environment = {
                "MOTUS_TELEOP_SHADOW_LAB_A_DRIVER_TOKEN": "driver-a-secret-sentinel",
                "MOTUS_TELEOP_SHADOW_LAB_A_TICKET_SECRET": (
                    "ticket-a-secret-sentinel-at-least-32-bytes"
                ),
                "MOTUS_TELEOP_SHADOW_LAB_B_DRIVER_TOKEN": "driver-b-secret-sentinel",
                "MOTUS_TELEOP_SHADOW_LAB_B_TICKET_SECRET": (
                    "ticket-b-secret-sentinel-at-least-32-bytes"
                ),
            }
            environment = {**os.environ, **secret_environment}
            valid = subprocess.run(
                ["docker", "compose", "-f", str(compose), "config", "--quiet"],
                check=False,
                capture_output=True,
                text=True,
                env=environment,
            )
            self.assertEqual(0, valid.returncode, valid.stderr)

            environment.pop("MOTUS_TELEOP_SHADOW_LAB_B_TICKET_SECRET")
            missing = subprocess.run(
                ["docker", "compose", "-f", str(compose), "config", "--quiet"],
                check=False,
                capture_output=True,
                text=True,
                env=environment,
            )
            self.assertNotEqual(0, missing.returncode)
            for secret in secret_environment.values():
                self.assertNotIn(secret, missing.stderr)


class CoreCaValidationTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("openssl"), "OpenSSL is unavailable")
    def test_ca_must_be_public_x509_inside_the_real_deployment_root(self):
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory).resolve()
            root = temporary / "opt" / "phanthy-motus"
            tls_dir = root / "tls"
            tls_dir.mkdir(parents=True)
            certificate = tls_dir / "core-ca.pem"
            private_key = temporary / "private-key.pem"
            subprocess.run(
                [
                    "openssl",
                    "req",
                    "-x509",
                    "-newkey",
                    "rsa:2048",
                    "-keyout",
                    str(private_key),
                    "-out",
                    str(certificate),
                    "-days",
                    "1",
                    "-nodes",
                    "-subj",
                    "/CN=motus-renderer-test",
                ],
                check=True,
                capture_output=True,
            )
            with mock.patch.object(renderer, "CORE_CA_SOURCE_ROOT", root):
                self.assertEqual(
                    str(certificate.resolve()),
                    _validate_core_ca_file_on_host(certificate),
                )

                managed_link = tls_dir / "managed-link.pem"
                managed_link.symlink_to(certificate)
                self.assertEqual(
                    str(certificate.resolve()),
                    _validate_core_ca_file_on_host(managed_link),
                )

                private_inside_root = tls_dir / "privkey.pem"
                shutil.copyfile(private_key, private_inside_root)
                with self.assertRaisesRegex(RenderError, "private-key PEM"):
                    _validate_core_ca_file_on_host(private_inside_root)

                combined = tls_dir / "combined.pem"
                combined.write_bytes(
                    certificate.read_bytes()
                    + b"\n-----BEGIN PRIVATE KEY-----\nsecret\n-----END PRIVATE KEY-----\n"
                )
                with self.assertRaisesRegex(RenderError, "private-key PEM"):
                    _validate_core_ca_file_on_host(combined)

                public_plus_garbage = tls_dir / "garbage.pem"
                public_plus_garbage.write_bytes(
                    certificate.read_bytes() + b"not-a-certificate"
                )
                with self.assertRaisesRegex(RenderError, "only public X.509"):
                    _validate_core_ca_file_on_host(public_plus_garbage)

                outside = temporary / "outside-ca.pem"
                shutil.copyfile(certificate, outside)
                escaped = tls_dir / "escaped.pem"
                escaped.symlink_to(outside)
                with self.assertRaisesRegex(RenderError, "inside /opt/phanthy-motus"):
                    _validate_core_ca_file_on_host(escaped)

                if hasattr(os, "mkfifo"):
                    fifo = tls_dir / "fifo.pem"
                    os.mkfifo(fifo)
                    with self.assertRaisesRegex(RenderError, "regular file"):
                        _validate_core_ca_file_on_host(fifo)


if __name__ == "__main__":
    unittest.main()
