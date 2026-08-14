import os
import subprocess
import tempfile
import unittest
from pathlib import Path


T800_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = T800_ROOT.parents[1]
BUILD_SCRIPT = REPO_ROOT / "build.sh"


class BuildScriptContractTests(unittest.TestCase):
    def _environment(self, tool_dir: Path, log_path: Path) -> dict[str, str]:
        env = os.environ.copy()
        for name in (
            "REGISTRY",
            "REGISTRY_USER",
            "REGISTRY_PASSWORD",
            "IMAGE_NAMESPACE",
            "RESOURCE_CENTER_API_KEY",
        ):
            env.pop(name, None)
        env["PATH"] = f"{tool_dir}:{env['PATH']}"
        env["DOCKER_LOG"] = str(log_path)
        return env

    def test_unknown_driver_is_a_nonzero_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            tool_dir = Path(tmp)
            log_path = tool_dir / "docker.log"
            result = subprocess.run(
                ["bash", str(BUILD_SCRIPT), "--mirror", "none", "engineai/not-a-driver"],
                cwd=REPO_ROOT,
                env=self._environment(tool_dir, log_path),
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                timeout=10,
            )
        self.assertEqual(2, result.returncode)
        self.assertIn("未知驱动参数", result.stderr)
        self.assertFalse(log_path.exists())

    def test_t800_manifest_aliases_select_the_same_driver_and_forward_ros_mirror(self):
        aliases = (
            "engineai/t800",
            "engineai-t800-driver",
            "engineai-t800",
            "t800",
            "engineai-t800-dev",
            "engineai/t800-dev",
            "t800-dev",
            str(T800_ROOT),
        )
        for alias in aliases:
            with self.subTest(alias=alias), tempfile.TemporaryDirectory() as tmp:
                tool_dir = Path(tmp)
                log_path = tool_dir / "docker.log"
                docker = tool_dir / "docker"
                docker.write_text(
                    "#!/bin/sh\n"
                    "printf '%s\\n' \"$*\" >> \"$DOCKER_LOG\"\n"
                    "case \"$*\" in *'alpine:3.20 uname -m'*) echo aarch64 ;; esac\n",
                    encoding="utf-8",
                )
                docker.chmod(0o755)
                result = subprocess.run(
                    ["bash", str(BUILD_SCRIPT), "--mirror", "none", alias],
                    cwd=REPO_ROOT,
                    env=self._environment(tool_dir, log_path),
                    stdin=subprocess.DEVNULL,
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                self.assertEqual(0, result.returncode, result.stderr + result.stdout)
                calls = log_path.read_text(encoding="utf-8")
                self.assertIn("buildx build", calls)
                self.assertIn("ROS_APT_MIRROR=http://packages.ros.org/ros2/ubuntu", calls)
                self.assertIn("UBUNTU_APT_MIRROR=http://ports.ubuntu.com/ubuntu-ports", calls)
                self.assertIn("/engineai/t800:", calls)
                self.assertNotIn("/engineai/t800-dev:", calls)
                self.assertIn("--platform linux/arm64", calls)
                if "dev" in alias:
                    self.assertIn("legacy T800 alias", result.stderr)

    def test_noninteractive_publish_sync_does_not_open_dev_tty(self):
        with tempfile.TemporaryDirectory() as tmp:
            tool_dir = Path(tmp)
            docker_log = tool_dir / "docker.log"
            curl_log = tool_dir / "curl.log"
            docker = tool_dir / "docker"
            docker.write_text(
                "#!/bin/sh\n"
                "printf '%s\\n' \"$*\" >> \"$DOCKER_LOG\"\n"
                "case \"$*\" in *'alpine:3.20 uname -m'*) echo aarch64 ;; esac\n",
                encoding="utf-8",
            )
            docker.chmod(0o755)
            git = tool_dir / "git"
            git.write_text(
                "#!/bin/sh\n"
                "case \"$*\" in *'rev-parse --short=7 HEAD'*) echo abcdef0 ;; esac\n",
                encoding="utf-8",
            )
            git.chmod(0o755)
            curl = tool_dir / "curl"
            curl.write_text(
                "#!/bin/sh\n"
                "printf '%s\\n' \"$*\" >> \"$CURL_LOG\"\n"
                "out=''\n"
                "while [ $# -gt 0 ]; do [ \"$1\" = -o ] && { shift; out=$1; }; shift; done\n"
                "[ -n \"$out\" ] && printf '{\"ok\":true}' > \"$out\"\n"
                "printf '%s' \"${FAKE_HTTP_CODE:-201}\"\n",
                encoding="utf-8",
            )
            curl.chmod(0o755)
            env = self._environment(tool_dir, docker_log)
            env.update({
                "REGISTRY": "registry.example.test",
                "REGISTRY_USER": "robot",
                "REGISTRY_PASSWORD": "secret",
                "IMAGE_NAMESPACE": "drivers",
                "RESOURCE_CENTER_API_KEY": "test-key",
                "RESOURCE_CENTER_URL": "https://resource.example.test",
                "CURL_LOG": str(curl_log),
            })
            result = subprocess.run(
                ["bash", str(BUILD_SCRIPT), "--mirror", "none", "engineai/t800"],
                cwd=REPO_ROOT,
                env=env,
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                timeout=15,
            )
            self.assertEqual(0, result.returncode, result.stderr + result.stdout)
            self.assertNotIn("/dev/tty", result.stderr)
            self.assertIn("/api/admin/register", curl_log.read_text(encoding="utf-8"))
            self.assertIn("✓ EngineAI T800", result.stdout)
            calls = docker_log.read_text(encoding="utf-8")
            self.assertIn("--output=type=docker", calls)
            self.assertNotIn("buildx build --push", calls)
            smoke_at = calls.index("run --rm --platform linux/arm64 --entrypoint /bin/bash")
            push_at = calls.index("push registry.example.test/drivers/engineai/t800:")
            self.assertLess(smoke_at, push_at)

            failed_env = dict(env, FAKE_HTTP_CODE="500")
            failed = subprocess.run(
                ["bash", str(BUILD_SCRIPT), "--mirror", "none", "engineai/t800"],
                cwd=REPO_ROOT,
                env=failed_env,
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                timeout=15,
            )
            self.assertEqual(1, failed.returncode)
            self.assertIn("注册失败 (HTTP 500)", failed.stderr)


if __name__ == "__main__":
    unittest.main()
