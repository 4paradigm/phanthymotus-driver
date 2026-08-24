import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import textwrap
import unittest

import yaml


G1_DIR = Path(__file__).resolve().parents[1]
DEPLOY_SCRIPT = G1_DIR / "deploy" / "deploy-from-git.sh"
SERVICE_TEMPLATE = G1_DIR / "deploy" / "service.yml"


class DeployFromGitContractTest(unittest.TestCase):
    def test_service_template_preserves_the_g1_runtime_contract(self):
        image = "phanthy-g1-driver:test-runtime"
        rendered = "services:\n" + textwrap.indent(
            SERVICE_TEMPLATE.read_text().replace("__IMAGE__", image), "  "
        )
        service = yaml.safe_load(rendered)["services"]["unitree-g1"]

        self.assertEqual(service["image"], image)
        self.assertEqual(service["container_name"], "embodied-unitree-g1")
        self.assertEqual(service["network_mode"], "host")
        self.assertEqual(service["ipc"], "host")
        self.assertEqual(service["pid"], "host")
        self.assertTrue(service["privileged"])
        self.assertEqual(service["restart"], "unless-stopped")
        self.assertIn("/dev:/dev", service["volumes"])

    def test_deploy_script_uses_the_template_and_compose_service_identity(self):
        source = DEPLOY_SCRIPT.read_text()

        self.assertIn('service_template="$source_dir/unitree/g1/deploy/service.yml"', source)
        self.assertIn("https://codeload.github.com/", source)
        self.assertIn("SOURCE_MODE=github_archive", source)
        self.assertIn('timeout 45 git clone', source)
        self.assertIn('ps -q unitree-g1)', source)
        self.assertIn('docker inspect "$container_id"', source)
        self.assertIn('docker exec -w /work "$container_id"', source)
        self.assertIn('docker logs --tail 80 "$container_id"', source)
        self.assertIn('"camera_rgb_frame"', source)
        self.assertIn('"camera_depth_frame"', source)
        self.assertIn("phanthy.sensor.camera_rgb_frame.v1", source)
        self.assertIn("phanthy.sensor.camera_depth_frame.v1", source)
        self.assertIn("CAMERA_FRAME_CONTRACT=PASS", source)
        self.assertNotIn("docker inspect embodied-unitree-g1", source)

    def test_script_syntax_and_real_dry_run(self):
        subprocess.run(["bash", "-n", str(DEPLOY_SCRIPT)], check=True)

        with tempfile.TemporaryDirectory(prefix="g1-driver-deploy-test.") as tmp:
            repo = Path(tmp)
            deploy_dir = repo / "unitree" / "g1" / "deploy"
            deploy_dir.mkdir(parents=True)
            shutil.copy2(DEPLOY_SCRIPT, deploy_dir / DEPLOY_SCRIPT.name)
            shutil.copy2(SERVICE_TEMPLATE, deploy_dir / SERVICE_TEMPLATE.name)

            subprocess.run(["git", "-C", tmp, "init", "-q", "-b", "main"], check=True)
            subprocess.run(
                ["git", "-C", tmp, "config", "user.name", "codex-test"], check=True
            )
            subprocess.run(
                [
                    "git",
                    "-C",
                    tmp,
                    "config",
                    "user.email",
                    "codex-test@example.invalid",
                ],
                check=True,
            )
            subprocess.run(["git", "-C", tmp, "add", "unitree/g1/deploy"], check=True)
            subprocess.run(
                ["git", "-C", tmp, "commit", "-q", "-m", "test"], check=True
            )

            env = os.environ.copy()
            env.update(
                {
                    "DRY_RUN": "1",
                    "REPO_URL": "https://github.com/NBStarry/phanthymotus-driver.git",
                }
            )
            result = subprocess.run(
                [str(deploy_dir / DEPLOY_SCRIPT.name), "g1-bj-wifi"],
                check=True,
                capture_output=True,
                env=env,
                text=True,
            )

        self.assertIn("DRY_RUN=PASS target=g1-bj-wifi", result.stdout)
        self.assertIn(
            "repo=https://github.com/NBStarry/phanthymotus-driver.git",
            result.stdout,
        )
        self.assertIn(
            "archive=https://codeload.github.com/NBStarry/phanthymotus-driver/tar.gz/",
            result.stdout,
        )
        self.assertIn("remote_repo=~/hanzebei/phanthymotus-driver", result.stdout)


if __name__ == "__main__":
    unittest.main()
