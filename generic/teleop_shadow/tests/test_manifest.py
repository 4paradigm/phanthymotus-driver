from __future__ import annotations

import json
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class DeploymentManifestTests(unittest.TestCase):
    def test_mcp_is_loopback_and_capture_uses_separate_tls_state_mounts(self):
        manifest = yaml.safe_load((ROOT / "deploy/service.yml").read_text(encoding="utf-8"))
        service = manifest["generic-teleop-shadow"]
        self.assertEqual("host", service["network_mode"])
        environment = service["environment"]
        self.assertIn("MOTUS_BIND_HOST=127.0.0.1", environment)
        self.assertIn("MOTUS_MCP_URL=http://localhost:15711/mcp", environment)
        self.assertIn("MOTUS_CAPTURE_BIND_HOST=0.0.0.0", environment)
        self.assertIn("MOTUS_CAPTURE_PORT=15712", environment)
        self.assertIn(
            "MOTUS_CAPTURE_WSS_URL=${MOTUS_CAPTURE_WSS_URL:?set the headset-reachable wss URL}",
            environment,
        )
        self.assertFalse(any(entry.startswith("MOTUS_DRIVER_TOKEN=") for entry in environment))
        self.assertFalse(any(entry.startswith("MOTUS_TELEOP_TICKET_SECRET=") for entry in environment))
        self.assertFalse(
            any(
                entry.startswith("MOTUS_REGISTRATION_COORDINATION_FILE=")
                for entry in environment
            )
        )
        self.assertIn(
            "/opt/phanthy-motus/data/certs/cert.pem:/etc/motus-core-ca/core-ca.pem:ro",
            service["volumes"],
        )
        self.assertNotIn(
            "/opt/phanthy-motus/data/certs:/etc/motus-core-certs:ro",
            service["volumes"],
        )
        self.assertIn(
            "/opt/phanthy-motus/data/teleop-capture-tls:/etc/motus-capture-tls:ro",
            service["volumes"],
        )
        self.assertNotIn(
            "/opt/phanthy-motus/data/certs:/etc/motus-capture-tls:ro",
            service["volumes"],
        )
        self.assertIn(
            "/opt/phanthy-motus/data/teleop-shadow:/var/lib/motus-teleop-shadow",
            service["volumes"],
        )

    def test_driver_and_runtime_ports_match_and_capture_port_is_distinct(self):
        driver = yaml.safe_load((ROOT / "driver.yaml").read_text(encoding="utf-8"))
        config = yaml.safe_load((ROOT / "config.yaml").read_text(encoding="utf-8"))
        self.assertEqual(driver["port"], config["mcp_port"])
        self.assertEqual("127.0.0.1", config["bind_host"])
        self.assertFalse(config["mcp_allow_non_loopback"])
        self.assertNotEqual(config["mcp_port"], config["capture"]["port"])
        self.assertEqual("0.0.0.0", config["capture"]["bind_host"])
        self.assertIsNone(config["registration"]["coordination_file"])

    def test_image_contains_capture_runtime_and_declares_both_ports(self):
        dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
        self.assertIn("capture.py", dockerfile)
        self.assertIn("dispatch.py", dockerfile)
        self.assertIn("EXPOSE 15711", dockerfile)
        self.assertIn("EXPOSE 15712", dockerfile)
        command_line = next(
            line.strip() for line in dockerfile.splitlines() if line.strip().startswith("CMD [")
        )
        command = json.loads(command_line.removeprefix("CMD "))
        self.assertEqual(["python", "-c"], command[:2])
        compile(command[2], "<docker-healthcheck>", "exec")

    def test_docs_state_stock_core_tls_and_persistent_pairing_boundaries(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        for statement in (
            "未修改 Core",
            "stock Core",
            "MOTUS_CAPTURE_WSS_URL",
            "SAN",
            "不要复用或挂载\n  Core 私钥",
            "10001:10001",
            "0600",
            "自动重连",
            "无需用户\n在头显里再次点击",
            "revoke_headset",
            "schema v4",
            "registration_coordination_dir",
            "新的 Unix 秒",
            "32768 bytes",
        ):
            self.assertIn(statement, readme)


if __name__ == "__main__":
    unittest.main()
