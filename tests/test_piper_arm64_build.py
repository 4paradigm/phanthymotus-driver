"""Native Jetson builds work without changing Docker or installing buildx."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Arm64BuildTests(unittest.TestCase):
    def build(self, architecture, *, platform='linux/arm64', extra=(), buildx=False):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            log = folder / 'commands'
            docker = folder / 'docker'
            docker.write_text('''#!/bin/bash
if [[ "$1 $2" == "buildx version" ]]; then exit "$TEST_BUILDX_STATUS"; fi
printf '%s\\n' "$*" >> "$TEST_LOG"
context="${@: -1}"
test -f "$context/device.py" && test -f "$context/common/vendor_runtime.py"
''')
            uname = folder / 'uname'
            uname.write_text('#!/bin/bash\nprintf "%s\\n" "$TEST_ARCH"\n')
            docker.chmod(0o755); uname.chmod(0o755)
            env = {**os.environ, 'PATH': str(folder) + os.pathsep + os.environ['PATH'],
                   'TEST_LOG': str(log), 'TEST_ARCH': architecture,
                   'PIPER_BUILD_PLATFORM': platform, 'TEST_BUILDX_STATUS': '0' if buildx else '1'}
            result = subprocess.run(['bash', str(ROOT/'agilex/piper/build-local.sh'), 'piper:test', *extra],
                                    env=env, text=True, capture_output=True)
            return result, log.read_text() if log.exists() else ''

    def test_native_jetson_uses_complete_context_without_buildx(self):
        result, commands = self.build('aarch64')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('build --platform linux/arm64 --tag piper:test', commands)
        self.assertIn('ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:', commands)
        self.assertNotIn('run', commands)

    def test_cross_platform_without_buildx_rejects_before_build(self):
        result, commands = self.build('x86_64')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Cross-platform build requires buildx', result.stderr)
        self.assertEqual(commands, '')

    def test_amd64_keeps_pinned_multiarch_base_with_legacy_builder(self):
        result, commands = self.build('x86_64', platform='linux/amd64')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--build-arg ROS_BASE_IMAGE=ros:humble-ros-base@sha256:', commands)

    def test_amd64_keeps_pinned_base_with_buildx_and_caller_can_override(self):
        override = 'ROS_BASE_IMAGE=example.invalid/ros@sha256:' + '1' * 64
        result, commands = self.build('x86_64', platform='linux/amd64', buildx=True,
                                      extra=('--build-arg', override))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('buildx build --platform linux/amd64', commands)
        self.assertLess(commands.index('ROS_BASE_IMAGE=ros:'), commands.index(override))


if __name__ == '__main__':unittest.main()
