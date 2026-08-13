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
    REGISTRATION_COORDINATION_FILE,
    REGISTRATION_COORDINATION_TARGET,
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
        "schema_version": 4,
        "core_ca_file": "/opt/phanthy-motus/data/certs/cert.pem",
        "registration_coordination_dir": "/opt/phanthy-motus/data/teleop-registration",
        "instances": [
            {
                "service": "teleop-shadow-lab-b",
                "container": "motus-teleop-shadow-lab-b",
                "driver_id": "teleop-shadow-lab-b",
                "driver_name": "Lab B PICO Teleop Shadow",
                "robot_id": "teleop-shadow-lab-b",
                "mcp_port": 15712,
                "capture_port": 15812,
                "capture_wss_url": "wss://192.0.2.111:15812/ws/teleop-capture",
                "capture_tls_dir": "/opt/phanthy-motus/data/teleop-capture-tls/lab-b",
                "capture_state_dir": "/opt/phanthy-motus/data/teleop-shadow/lab-b",
            },
            {
                "service": "teleop-shadow-lab-a",
                "container": "motus-teleop-shadow-lab-a",
                "driver_id": "teleop-shadow-lab-a",
                "driver_name": "Lab A Quest Teleop Shadow",
                "robot_id": "teleop-shadow-lab-a",
                "mcp_port": 15711,
                "capture_port": 15811,
                "capture_wss_url": "wss://192.0.2.110:15811/ws/teleop-capture",
                "capture_tls_dir": "/opt/phanthy-motus/data/teleop-capture-tls/lab-a",
                "capture_state_dir": "/opt/phanthy-motus/data/teleop-shadow/lab-a",
            },
        ],
    }


class RenderInstancesTests(unittest.TestCase):
    def setUp(self):
        ca_validation = mock.patch(
            "render_instances._validate_core_ca_file_on_host",
            side_effect=lambda path: str(path),
        )
        directory_validation = mock.patch(
            "render_instances._validate_managed_directory_on_host",
            side_effect=lambda path, _field: str(path),
        )
        overlap_validation = mock.patch(
            "render_instances._host_paths_overlap",
            side_effect=renderer._paths_overlap,
        )
        ca_validation.start()
        directory_validation.start()
        overlap_validation.start()
        self.addCleanup(ca_validation.stop)
        self.addCleanup(directory_validation.stop)
        self.addCleanup(overlap_validation.stop)

    def test_two_instances_render_driver_owned_dual_listener_compose(self):
        deployment = parse_instances_document(_instances_document())
        rendered = render_compose(deployment, "registry.example/motus/teleop-shadow:2.0")
        compose = yaml.safe_load(rendered)
        self.assertEqual(
            ["teleop-shadow-lab-a", "teleop-shadow-lab-b"],
            list(compose["services"]),
        )
        for instance in deployment.instances:
            service = compose["services"][instance.service]
            environment = service["environment"]
            self.assertEqual("host", service["network_mode"])
            self.assertIn("MOTUS_BIND_HOST=127.0.0.1", environment)
            self.assertIn(f"MOTUS_MCP_PORT={instance.mcp_port}", environment)
            self.assertIn("MOTUS_CAPTURE_BIND_HOST=0.0.0.0", environment)
            self.assertIn(f"MOTUS_CAPTURE_PORT={instance.capture_port}", environment)
            self.assertIn(f"MOTUS_CAPTURE_WSS_URL={instance.capture_wss_url}", environment)
            self.assertIn(
                f"MOTUS_REGISTRATION_COORDINATION_FILE={REGISTRATION_COORDINATION_FILE}",
                environment,
            )
            self.assertFalse(
                any(entry.startswith("MOTUS_REGISTRATION_SLOT=") for entry in environment)
            )
            self.assertFalse(any(entry.startswith("MOTUS_DRIVER_TOKEN=") for entry in environment))
            self.assertFalse(any(entry.startswith("MOTUS_TELEOP_TICKET_SECRET=") for entry in environment))
            self.assertEqual(4, len(service["volumes"]))
            core_ca, capture_tls, capture_state, registration_coordination = service["volumes"]
            self.assertEqual(deployment.core_ca_file, core_ca["source"])
            self.assertEqual(CORE_CA_TARGET, core_ca["target"])
            self.assertTrue(core_ca["read_only"])
            self.assertEqual(instance.capture_tls_dir, capture_tls["source"])
            self.assertEqual("/etc/motus-capture-tls", capture_tls["target"])
            self.assertTrue(capture_tls["read_only"])
            self.assertEqual(instance.capture_state_dir, capture_state["source"])
            self.assertEqual("/var/lib/motus-teleop-shadow", capture_state["target"])
            self.assertFalse(capture_state["read_only"])
            self.assertEqual(
                deployment.registration_coordination_dir,
                registration_coordination["source"],
            )
            self.assertEqual(
                REGISTRATION_COORDINATION_TARGET,
                registration_coordination["target"],
            )
            self.assertFalse(registration_coordination["read_only"])
            for volume in service["volumes"]:
                self.assertFalse(volume["bind"]["create_host_path"])

    def test_rendering_is_deterministic_and_never_reads_process_secrets(self):
        first = _instances_document()
        second = copy.deepcopy(first)
        second["instances"].reverse()
        secret_environment = {
            "MOTUS_DRIVER_TOKEN": "driver-secret-sentinel",
            "MOTUS_TELEOP_TICKET_SECRET": "ticket-secret-sentinel-at-least-32-bytes",
        }
        with mock.patch.dict(os.environ, secret_environment):
            first_render = render_compose(parse_instances_document(first), "teleop-shadow:local")
            second_render = render_compose(parse_instances_document(second), "teleop-shadow:local")
        self.assertEqual(first_render, second_render)
        for name, secret in secret_environment.items():
            self.assertNotIn(name, first_render)
            self.assertNotIn(secret, first_render)

    def test_every_identity_port_url_and_storage_domain_is_unique(self):
        for field in (
            "service", "container", "driver_id", "robot_id", "mcp_port",
            "capture_port", "capture_wss_url", "capture_tls_dir", "capture_state_dir",
        ):
            with self.subTest(field=field):
                document = _instances_document()
                document["instances"][1][field] = document["instances"][0][field]
                if field == "capture_port":
                    document["instances"][1]["capture_wss_url"] = (
                        "wss://192.0.2.110:15812/ws/teleop-capture"
                    )
                with self.assertRaisesRegex(RenderError, field):
                    parse_instances_document(document)

        document = _instances_document()
        document["instances"][1]["capture_port"] = document["instances"][0]["mcp_port"]
        document["instances"][1]["capture_wss_url"] = (
            "wss://192.0.2.110:15712/ws/teleop-capture"
        )
        with self.assertRaisesRegex(RenderError, "globally unique"):
            parse_instances_document(document)

    def test_tls_state_and_core_ca_ownership_trees_never_overlap(self):
        mutations = (
            # Same instance: writable state cannot be the TLS directory or
            # one of its ancestors/children.
            (1, "capture_state_dir", "/opt/phanthy-motus/data/teleop-capture-tls/lab-a"),
            (1, "capture_state_dir", "/opt/phanthy-motus/data/teleop-capture-tls"),
            (1, "capture_state_dir", "/opt/phanthy-motus/data/teleop-capture-tls/lab-a/state"),
            # Cross instance: a writable state tree cannot contain another
            # instance's TLS key material.
            (1, "capture_state_dir", "/opt/phanthy-motus/data/teleop-capture-tls/lab-b"),
            # No managed directory may contain the mounted Core CA file.
            (1, "capture_state_dir", "/opt/phanthy-motus/data"),
            (1, "capture_tls_dir", "/opt/phanthy-motus/data/certs"),
            # The shared registration directory owns only its lock/marker.
            (1, "capture_state_dir", "/opt/phanthy-motus/data/teleop-registration"),
            (1, "capture_tls_dir", "/opt/phanthy-motus/data/teleop-registration/tls"),
        )
        for index, field, value in mutations:
            with self.subTest(index=index, field=field, value=value):
                document = _instances_document()
                document["instances"][index][field] = value
                with self.assertRaisesRegex(RenderError, "overlaps"):
                    parse_instances_document(document)

        document = _instances_document()
        document["registration_coordination_dir"] = "/opt/phanthy-motus/data"
        with self.assertRaisesRegex(RenderError, "registration_coordination_dir"):
            parse_instances_document(document)

    def test_multi_instance_requires_coordination_but_single_instance_is_fast_path(self):
        document = _instances_document()
        document["registration_coordination_dir"] = None
        with self.assertRaisesRegex(RenderError, "required for multi-instance"):
            parse_instances_document(document)

        document["instances"] = document["instances"][:1]
        deployment = parse_instances_document(document)
        rendered = yaml.safe_load(render_compose(deployment, "teleop-shadow:local"))
        service = next(iter(rendered["services"].values()))
        self.assertFalse(
            any(
                entry.startswith("MOTUS_REGISTRATION_COORDINATION_FILE=")
                for entry in service["environment"]
            )
        )
        self.assertEqual(3, len(service["volumes"]))

    def test_old_secret_schema_and_unknown_fields_fail_with_migration_signal(self):
        document = _instances_document()
        document["schema_version"] = 3
        with self.assertRaisesRegex(RenderError, "schema_version must be exactly 4"):
            parse_instances_document(document)
        document = _instances_document()
        document["instances"][0]["driver_token_env"] = "MOTUS_DRIVER_TOKEN"
        with self.assertRaisesRegex(RenderError, "driver_token_env"):
            parse_instances_document(document)
        document = _instances_document()
        document["instances"][0].pop("capture_wss_url")
        with self.assertRaisesRegex(RenderError, "capture_wss_url"):
            parse_instances_document(document)

    def test_identifiers_urls_ports_paths_and_images_reject_injection(self):
        mutations = (
            ("service", "Bad/Service"),
            ("container", "../container"),
            ("driver_id", "driver\nINJECTED=1"),
            ("robot_id", "r" * 65),
            ("driver_name", "${SECRET}"),
            ("mcp_port", 15699),
            ("mcp_port", "15711"),
            ("capture_port", 80),
            ("capture_port", True),
            ("capture_wss_url", "ws://10.0.0.1:15812/ws/teleop-capture"),
            ("capture_wss_url", "wss://user:pass@10.0.0.1:15812/ws/teleop-capture"),
            ("capture_wss_url", "wss://10.0.0.1:15813/ws/teleop-capture"),
            ("capture_wss_url", "wss://10.0.0.1:15812/other"),
            ("capture_tls_dir", "/opt/phanthy-motus/../private"),
            ("capture_tls_dir", "/etc/shadow"),
            ("capture_state_dir", "relative/state"),
            ("capture_state_dir", "/opt/phanthy-motus/${SECRET}"),
        )
        for field, value in mutations:
            with self.subTest(field=field, value=value):
                document = _instances_document()
                document["instances"][0][field] = value
                with self.assertRaises(RenderError):
                    parse_instances_document(document)
        for image in ("", "../image", "image\nservices:", "${SECRET_IMAGE}"):
            with self.subTest(image=image), self.assertRaises(RenderError):
                validate_image(image)

    def test_core_ca_path_and_duplicate_yaml_keys_are_rejected(self):
        for path in (
            "relative/cert.pem", "/opt/phanthy-motus/../secret.pem",
            "/opt/phanthy-motus/${CORE_CA}", "/etc/shadow",
        ):
            document = _instances_document()
            document["core_ca_file"] = path
            with self.subTest(path=path), self.assertRaises(RenderError):
                parse_instances_document(document)
        text = """schema_version: 4
core_ca_file: /opt/phanthy-motus/data/certs/cert.pem
core_ca_file: /tmp/overridden.pem
registration_coordination_dir: /opt/phanthy-motus/data/teleop-registration
instances: []
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "instances.yml"
            path.write_text(text, encoding="utf-8")
            with self.assertRaisesRegex(RenderError, "duplicate key"):
                load_instances(path)

    def test_atomic_output_alias_and_cli_modes(self):
        deployment = parse_instances_document(_instances_document())
        rendered = render_compose(deployment, "teleop-shadow:local")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "teleop-shadow.compose.yml"
            output.write_text("partial", encoding="utf-8")
            atomic_write(output, rendered)
            self.assertEqual(rendered, output.read_text(encoding="utf-8"))
            self.assertEqual(0o644, output.stat().st_mode & 0o777)
            self.assertEqual([], list(root.glob(f".{output.name}.*.tmp")))

            source = root / "instances.yml"
            source.write_text(yaml.safe_dump(_instances_document(), sort_keys=False), encoding="utf-8")
            with self.assertRaisesRegex(RenderError, "must not replace"):
                require_distinct_input_output(source, source)
            hard_link = root / "hard-link.yml"
            os.link(source, hard_link)
            with self.assertRaisesRegex(RenderError, "must not replace"):
                require_distinct_input_output(source, hard_link)

            for flag in ("--stdout", "--dry-run"):
                destination = root / f"{flag[2:]}.yml"
                stdout = io.StringIO()
                stderr = io.StringIO()
                with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                    code = main([
                        "--instances", str(source), "--image", "teleop-shadow:local",
                        "--output", str(destination), flag,
                    ])
                self.assertEqual(0, code, stderr.getvalue())
                self.assertFalse(destination.exists())
                self.assertEqual(2, len(yaml.safe_load(stdout.getvalue())["services"]))

    def test_atomic_output_rejects_broken_symlink_and_cli_refuses_source_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "compose.yml"
            missing = root / "missing"
            output.symlink_to(missing)
            with self.assertRaisesRegex(RenderError, "regular file path"):
                atomic_write(output, "services: {}\n")
            source = root / "instances.yml"
            original = yaml.safe_dump(_instances_document(), sort_keys=False)
            source.write_text(original, encoding="utf-8")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                code = main([
                    "--instances", str(source), "--image", "teleop-shadow:local",
                    "--output", str(source),
                ])
            self.assertEqual(2, code)
            self.assertEqual(original, source.read_text(encoding="utf-8"))

    def test_bundled_example_is_directly_renderable(self):
        deployment = load_instances(ROOT / "deploy/instances.example.yml")
        rendered = render_compose(deployment, "teleop-shadow:local")
        self.assertEqual(2, len(yaml.safe_load(rendered)["services"]))
        self.assertNotIn("MOTUS_DRIVER_TOKEN", rendered)
        self.assertNotIn("MOTUS_TELEOP_TICKET_SECRET", rendered)

    @unittest.skipUnless(shutil.which("docker"), "Docker Compose is unavailable")
    def test_rendered_document_passes_real_docker_compose_preflight(self):
        version = subprocess.run(
            ["docker", "compose", "version"], capture_output=True, text=True, check=False
        )
        if version.returncode != 0:
            self.skipTest("Docker Compose plugin is unavailable")
        rendered = render_compose(
            parse_instances_document(_instances_document()), "teleop-shadow:local"
        )
        with tempfile.TemporaryDirectory() as directory:
            compose = Path(directory) / "compose.yml"
            compose.write_text(rendered, encoding="utf-8")
            result = subprocess.run(
                ["docker", "compose", "-f", str(compose), "config", "--quiet"],
                capture_output=True, text=True, check=False,
            )
            self.assertEqual(0, result.returncode, result.stderr)


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
            subprocess.run([
                "openssl", "req", "-x509", "-newkey", "rsa:2048",
                "-keyout", str(private_key), "-out", str(certificate),
                "-days", "1", "-nodes", "-subj", "/CN=motus-renderer-test",
            ], check=True, capture_output=True)
            with mock.patch.object(renderer, "CORE_CA_SOURCE_ROOT", root):
                self.assertEqual(str(certificate.resolve()), _validate_core_ca_file_on_host(certificate))
                private_inside = tls_dir / "private.pem"
                shutil.copyfile(private_key, private_inside)
                with self.assertRaisesRegex(RenderError, "private-key PEM"):
                    _validate_core_ca_file_on_host(private_inside)
                combined = tls_dir / "combined.pem"
                combined.write_bytes(certificate.read_bytes() + private_key.read_bytes())
                with self.assertRaisesRegex(RenderError, "private-key PEM"):
                    _validate_core_ca_file_on_host(combined)
                outside = temporary / "outside.pem"
                shutil.copyfile(certificate, outside)
                escaped = tls_dir / "escaped.pem"
                escaped.symlink_to(outside)
                with self.assertRaisesRegex(RenderError, "symlink"):
                    _validate_core_ca_file_on_host(escaped)

                in_tree_link = tls_dir / "linked.pem"
                in_tree_link.symlink_to(certificate)
                with self.assertRaisesRegex(RenderError, "symlink"):
                    _validate_core_ca_file_on_host(in_tree_link)

                real_parent = root / "real-ca-parent"
                real_parent.mkdir()
                ancestor_link = root / "linked-ca-parent"
                ancestor_link.symlink_to(real_parent, target_is_directory=True)
                linked_certificate = ancestor_link / "core-ca.pem"
                shutil.copyfile(certificate, real_parent / linked_certificate.name)
                with self.assertRaisesRegex(RenderError, "symlink ancestor"):
                    _validate_core_ca_file_on_host(linked_certificate)


class CanonicalManagedPathTests(unittest.TestCase):
    def _document(self, root: Path) -> dict:
        return {
            "schema_version": 4,
            "core_ca_file": str(root / "core" / "core-ca.pem"),
            "registration_coordination_dir": str(root / "registration"),
            "instances": [
                {
                    "service": "teleop-a",
                    "container": "teleop-a",
                    "driver_id": "teleop-a",
                    "driver_name": "Teleop A",
                    "robot_id": "robot-a",
                    "mcp_port": 15711,
                    "capture_port": 15811,
                    "capture_wss_url": "wss://192.0.2.11:15811/ws/teleop-capture",
                    "capture_tls_dir": str(root / "tls-a"),
                    "capture_state_dir": str(root / "state-a"),
                },
                {
                    "service": "teleop-b",
                    "container": "teleop-b",
                    "driver_id": "teleop-b",
                    "driver_name": "Teleop B",
                    "robot_id": "robot-b",
                    "mcp_port": 15712,
                    "capture_port": 15812,
                    "capture_wss_url": "wss://192.0.2.12:15812/ws/teleop-capture",
                    "capture_tls_dir": str(root / "tls-b"),
                    "capture_state_dir": str(root / "state-b"),
                },
            ],
        }

    def _tree(self, temporary: str) -> tuple[Path, dict]:
        root = Path(temporary).resolve() / "opt" / "phanthy-motus"
        for name in (
            "core",
            "registration",
            "tls-a",
            "state-a",
            "tls-b",
            "state-b",
        ):
            (root / name).mkdir(parents=True, exist_ok=True)
        (root / "core" / "core-ca.pem").write_text("test", encoding="ascii")
        return root, self._document(root)

    def _parse_without_ca_contents(self, root: Path, document: dict):
        with (
            mock.patch.object(renderer, "CORE_CA_SOURCE_ROOT", root),
            mock.patch(
                "render_instances._validate_core_ca_file_on_host",
                side_effect=lambda path: str(path.resolve(strict=True)),
            ),
        ):
            return parse_instances_document(document)

    def test_bind_sources_are_host_canonical_and_isolated(self):
        with tempfile.TemporaryDirectory() as directory:
            root, document = self._tree(directory)
            deployment = self._parse_without_ca_contents(root, document)
            self.assertEqual(str((root / "registration").resolve()), deployment.registration_coordination_dir)
            for instance in deployment.instances:
                self.assertEqual(str(Path(instance.capture_tls_dir).resolve()), instance.capture_tls_dir)
                self.assertEqual(str(Path(instance.capture_state_dir).resolve()), instance.capture_state_dir)
            compose = yaml.safe_load(render_compose(deployment, "teleop-shadow:local"))
            for service in compose["services"].values():
                for volume in service["volumes"]:
                    self.assertEqual(str(Path(volume["source"]).resolve()), volume["source"])

    def test_same_instance_symlink_alias_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root, document = self._tree(directory)
            (root / "state-a").rmdir()
            (root / "state-a").symlink_to(root / "tls-a", target_is_directory=True)
            with self.assertRaisesRegex(RenderError, "symlink"):
                self._parse_without_ca_contents(root, document)

    def test_cross_instance_symlink_alias_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root, document = self._tree(directory)
            (root / "state-b").rmdir()
            (root / "state-b").symlink_to(root / "state-a", target_is_directory=True)
            with self.assertRaisesRegex(RenderError, "symlink"):
                self._parse_without_ca_contents(root, document)

    def test_ancestor_symlink_is_rejected_for_managed_and_coordination_paths(self):
        for field, instance_index in (
            ("capture_tls_dir", 0),
            ("capture_state_dir", 0),
            ("registration_coordination_dir", None),
        ):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                root, document = self._tree(directory)
                real_parent = root / "real-parent"
                real_parent.mkdir()
                leaf = real_parent / "leaf"
                leaf.mkdir()
                linked_parent = root / "linked-parent"
                linked_parent.symlink_to(real_parent, target_is_directory=True)
                value = str(linked_parent / "leaf")
                if instance_index is None:
                    document[field] = value
                else:
                    document["instances"][instance_index][field] = value
                with self.assertRaisesRegex(RenderError, "symlink ancestor"):
                    self._parse_without_ca_contents(root, document)


if __name__ == "__main__":
    unittest.main()
