"""Offline deployment contracts: no Docker, ROS or robot required."""
from pathlib import Path
import unittest

import yaml

from scripts.check_service_yml import check, env_map


ROOT = Path(__file__).resolve().parents[1]
SERVICE = ROOT / "agilex/piper/deploy/service.yml"


class PiperDeploymentTests(unittest.TestCase):
    def setUp(self):
        self.service = yaml.safe_load(SERVICE.read_text())["agilex-piper"]
        self.environment = env_map(self.service["environment"])

    def test_pr_fragment_shares_host_can_and_dds_network(self):
        # run-pr-image.sh wraps this fragment; it does not inject host networking.
        self.assertEqual(self.service.get("network_mode"), "host")
        self.assertNotIn("ports", self.service)

    def test_exact_readonly_dds_profile_is_mounted_and_selected(self):
        profile = "/opt/phanthy-motus/dds-local.xml"
        self.assertIn(f"{profile}:{profile}:ro", self.service["volumes"])
        self.assertEqual(self.environment["FASTRTPS_DEFAULT_PROFILES_FILE"], profile)
        self.assertEqual(self.environment["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp")
        self.assertEqual(self.environment["ROS_DOMAIN_ID"], "42")
        self.assertNotIn("FASTDDS_BUILTIN_TRANSPORTS", self.environment)

    def test_repo_deployment_checks_and_motion_default(self):
        violations, _ = check(str(SERVICE), "agilex/piper")
        self.assertEqual(violations, [])
        self.assertEqual(self.environment["PIPER_MOTION_ENABLED"], "${PIPER_MOTION_ENABLED:-0}")


if __name__ == "__main__":
    unittest.main()
