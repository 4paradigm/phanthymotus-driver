from __future__ import annotations

import json
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class DeploymentManifestTests(unittest.TestCase):
    def test_service_http_is_loopback_only_for_core_proxy(self):
        with (ROOT / "deploy/service.yml").open(encoding="utf-8") as handle:
            manifest = yaml.safe_load(handle)
        service = manifest["generic-teleop-shadow"]
        self.assertEqual("host", service["network_mode"])
        self.assertNotIn("ipc", service)
        self.assertNotIn("pid", service)
        self.assertIn("MOTUS_BIND_HOST=127.0.0.1", service["environment"])
        self.assertIn("MOTUS_MCP_URL=http://localhost:15711/mcp", service["environment"])
        self.assertIn(
            "MOTUS_DRIVER_TOKEN=${MOTUS_DRIVER_TOKEN:?set MOTUS_DRIVER_TOKEN before Docker Compose validation}",
            service["environment"],
        )
        self.assertIn(
            "MOTUS_TELEOP_TICKET_SECRET=${MOTUS_TELEOP_TICKET_SECRET:?set MOTUS_TELEOP_TICKET_SECRET before Docker Compose validation}",
            service["environment"],
        )

        self.assertIn(
            "/opt/phanthy-motus/data/certs:/etc/motus-core-certs:ro",
            service["volumes"],
        )
        self.assertIn(
            "MOTUS_AGENT_CORE_CA_FILE=/etc/motus-core-certs/cert.pem",
            service["environment"],
        )

    def test_driver_and_runtime_ports_match(self):
        with (ROOT / "driver.yaml").open(encoding="utf-8") as handle:
            driver = yaml.safe_load(handle)
        with (ROOT / "config.yaml").open(encoding="utf-8") as handle:
            config = yaml.safe_load(handle)
        self.assertEqual(15711, driver["port"])
        self.assertEqual(driver["port"], config["mcp_port"])

    def test_image_contains_final_dispatch_and_health_requires_readiness(self):
        dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("dispatch.py", dockerfile)
        self.assertIn("smoke_recording.py", dockerfile)
        self.assertIn("ENV MOTUS_MCP_PORT=15711", dockerfile)
        self.assertIn("os.environ['MOTUS_MCP_PORT']", dockerfile)
        self.assertIn("s['driver_id']==d", dockerfile)
        self.assertNotIn("127.0.0.1:15711/health", dockerfile)
        self.assertIn("s['ready'] is True", dockerfile)
        self.assertIn("s['rtc_enabled'] is True", dockerfile)
        command_line = next(
            line.strip() for line in dockerfile.splitlines() if line.strip().startswith("CMD [")
        )
        command = json.loads(command_line.removeprefix("CMD "))
        self.assertEqual(["python", "-c"], command[:2])
        compile(command[2], "<docker-healthcheck>", "exec")

    def test_readme_exposes_direct_multi_instance_render_and_start_commands(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn("render_instances.py", readme)
        self.assertIn("deploy/instances.example.yml", readme)
        self.assertIn("--dry-run", readme)
        self.assertIn("config --quiet", readme)
        self.assertIn("up -d", readme)
        self.assertIn("contain no secret values", readme)
        self.assertIn("schema_version: 2", readme)
        self.assertIn("MOTUS_TELEOP_SHADOW_LAB_A_DRIVER_TOKEN", readme)
        self.assertIn("MOTUS_TELEOP_SHADOW_LAB_A_TICKET_SECRET", readme)
        self.assertIn("schema v1", readme)
        self.assertIn("compatible single-instance fragment", readme)
        self.assertIn("lifecycle `stop`", readme)
        self.assertIn("24–4096 restricted ASCII Bearer characters", readme)
        self.assertIn("A-Z a-z 0-9 . _ ~ + / = -", readme)
        self.assertIn("MOTUS_DRIVER_TOKENS", readme)
        self.assertIn("rtc_enabled=true", readme)
        self.assertIn("invalid_signature", readme)
        self.assertIn("Release every active schema v1 session", readme)
        self.assertIn("robot_id == driver_id", readme)
        self.assertIn("private-key PEM block is rejected", readme)
        self.assertIn("one host trust zone", readme)
        self.assertIn("authority-domain", readme)
        self.assertIn("restart_required=true", readme)
        self.assertIn("teleop_ready=true", readme)
        self.assertIn("ss -ltnp", readme)

    def test_example_instances_are_direct_standalone_authority_domains(self):
        with (ROOT / "deploy/instances.example.yml").open(encoding="utf-8") as handle:
            example = yaml.safe_load(handle)
        self.assertEqual(2, example["schema_version"])
        self.assertEqual(2, len(example["instances"]))
        secret_environment_names = []
        for instance in example["instances"]:
            self.assertEqual(instance["driver_id"], instance["robot_id"])
            secret_environment_names.extend((
                instance["driver_token_env"],
                instance["ticket_secret_env"],
            ))
        self.assertEqual(len(secret_environment_names), len(set(secret_environment_names)))


if __name__ == "__main__":
    unittest.main()
