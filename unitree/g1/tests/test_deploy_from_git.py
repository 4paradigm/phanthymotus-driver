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

        self.assertNotIn('"refs/heads/$source_ref:', source)
        self.assertEqual(source.count("'FETCH_HEAD^{commit}'"), 2)
        self.assertIn('service_template="$source_dir/unitree/g1/deploy/service.yml"', source)
        self.assertIn("https://codeload.github.com/", source)
        self.assertIn("SOURCE_MODE=github_archive", source)
        self.assertIn('archive_tree="$(git -C "$temporary_source_root/archive" write-tree)"', source)
        self.assertIn("Source archive tree mismatch", source)
        self.assertIn('timeout 45 git clone', source)
        self.assertIn(
            '"$repo_url" "$temporary_source_root/git" &&\n'
            '    git -C "$temporary_source_root/git" fetch',
            source,
        )
        self.assertIn('ps -q unitree-g1)', source)
        self.assertIn('docker inspect "$container_id"', source)
        self.assertIn('docker exec -w /work "$container_id"', source)
        self.assertIn('docker logs --tail 80 "$container_id"', source)
        self.assertIn('"camera_rgb_frame"', source)
        self.assertIn('"camera_depth_frame"', source)
        self.assertIn("phanthy.sensor.camera_rgb_frame.v1", source)
        self.assertIn("phanthy.sensor.camera_depth_frame.v1", source)
        self.assertIn("CAMERA_FRAME_CONTRACT=PASS", source)
        self.assertIn('DEPTH_COMPRESSION == "zlib"', source)
        self.assertIn('cp -a "$source_dir/unitree/g1/." "$build_context/"', source)
        self.assertIn('cp -a "$source_dir/common" "$build_context/common"', source)
        self.assertIn('-f "$build_context/Dockerfile"', source)
        self.assertIn('\n  "$build_context"\n', source)
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
            expected_tree = subprocess.check_output(
                ["git", "-C", tmp, "rev-parse", "HEAD^{tree}"], text=True
            ).strip()

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
        self.assertIn(f"tree={expected_tree}", result.stdout)
        self.assertIn(
            "repo=https://github.com/NBStarry/phanthymotus-driver.git",
            result.stdout,
        )
        self.assertIn(
            "archive=https://codeload.github.com/NBStarry/phanthymotus-driver/tar.gz/",
            result.stdout,
        )
        self.assertIn("remote_repo=~/hanzebei/phanthymotus-driver", result.stdout)

    def test_source_ref_fetch_accepts_branches_and_tags(self):
        with tempfile.TemporaryDirectory(prefix="g1-driver-ref-test.") as tmp:
            root = Path(tmp)
            source = root / "source"
            remote = root / "origin.git"
            target = root / "target"
            subprocess.run(["git", "init", "-q", "-b", "main", source], check=True)
            subprocess.run(
                [
                    "git", "-C", source,
                    "-c", "user.name=codex-test",
                    "-c", "user.email=codex-test@example.invalid",
                    "commit", "-q", "--allow-empty", "-m", "test",
                ],
                check=True,
            )
            subprocess.run(["git", "-C", source, "tag", "release-test"], check=True)
            subprocess.run(["git", "clone", "-q", "--bare", source, remote], check=True)
            subprocess.run(["git", "init", "-q", target], check=True)
            expected = subprocess.check_output(
                ["git", "-C", source, "rev-parse", "HEAD"], text=True
            ).strip()

            for source_ref in (
                "main",
                "refs/heads/main",
                "release-test",
                "refs/tags/release-test",
            ):
                with self.subTest(source_ref=source_ref):
                    subprocess.run(
                        [
                            "git", "-C", target, "fetch", "-q", "--no-tags",
                            remote, source_ref,
                        ],
                        check=True,
                    )
                    actual = subprocess.check_output(
                        ["git", "-C", target, "rev-parse", "FETCH_HEAD^{commit}"],
                        text=True,
                    ).strip()
                    self.assertEqual(actual, expected)

    def test_archive_tree_reconstruction_detects_content_changes(self):
        with tempfile.TemporaryDirectory(prefix="g1-driver-archive-test.") as tmp:
            root = Path(tmp)
            repo = root / "repo"
            repo.mkdir()
            subprocess.run(["git", "-C", repo, "init", "-q", "-b", "main"], check=True)
            (repo / "payload.txt").write_text("expected\n")
            subprocess.run(["git", "-C", repo, "add", "payload.txt"], check=True)
            subprocess.run(
                [
                    "git",
                    "-C",
                    repo,
                    "-c",
                    "user.name=codex-test",
                    "-c",
                    "user.email=codex-test@example.invalid",
                    "commit",
                    "-q",
                    "-m",
                    "test",
                ],
                check=True,
            )
            commit = subprocess.check_output(
                ["git", "-C", repo, "rev-parse", "HEAD"], text=True
            ).strip()
            expected_tree = subprocess.check_output(
                ["git", "-C", repo, "rev-parse", "HEAD^{tree}"], text=True
            ).strip()
            archive = root / "source.tar.gz"
            subprocess.run(
                [
                    "git",
                    "-C",
                    repo,
                    "archive",
                    "--format=tar.gz",
                    f"--prefix=repo-{commit}/",
                    f"--output={archive}",
                    commit,
                ],
                check=True,
            )
            extracted = root / "extracted"
            extracted.mkdir()
            subprocess.run(
                ["tar", "-xzf", archive, "--strip-components=1", "-C", extracted],
                check=True,
            )
            subprocess.run(
                ["git", "-C", extracted, "init", "-q", "--object-format=sha1"],
                check=True,
            )
            subprocess.run(["git", "-C", extracted, "add", "-f", "-A"], check=True)
            actual_tree = subprocess.check_output(
                ["git", "-C", extracted, "write-tree"], text=True
            ).strip()
            self.assertEqual(actual_tree, expected_tree)

            (extracted / "payload.txt").write_text("changed\n")
            subprocess.run(["git", "-C", extracted, "add", "-f", "-A"], check=True)
            changed_tree = subprocess.check_output(
                ["git", "-C", extracted, "write-tree"], text=True
            ).strip()
            self.assertNotEqual(changed_tree, expected_tree)


if __name__ == "__main__":
    unittest.main()
