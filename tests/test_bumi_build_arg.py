"""build.sh forwards opt-in build args to the Bumi image build.

The Dockerfile exposes INSTALL_POSE_DEPS as a build-time switch, but a switch
nobody can pass through the documented `./build.sh` entry point is dead code.
These tests run build.sh with a fake docker to assert the actual command.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BuildArgForwardingTests(unittest.TestCase):
    def run_build(self, *args):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            log = folder / 'commands'
            docker = folder / 'docker'
            uname = folder / 'uname'
            docker.write_text('#!/bin/bash\nprintf "%s\\n" "$*" >> "$TEST_LOG"\n')
            uname.write_text('#!/bin/bash\nprintf "aarch64\\n"\n')
            docker.chmod(0o755)
            uname.chmod(0o755)
            env = {
                **os.environ,
                'PATH': str(folder) + os.pathsep + os.environ['PATH'],
                'TEST_LOG': str(log),
                'REGISTRY': '', 'REGISTRY_USER': '', 'REGISTRY_PASSWORD': '',
                'IMAGE_NAMESPACE': '',
            }
            result = subprocess.run(
                ['bash', str(ROOT / 'build.sh'), '--mirror none', *args, 'noetix/bumi'],
                env=env, text=True, capture_output=True, cwd=str(ROOT))
            commands = log.read_text() if log.exists() else ''
            return result, commands

    def test_pose_deps_arg_reaches_docker_build(self):
        result, commands = self.run_build('--build-arg', 'INSTALL_POSE_DEPS=1')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--build-arg INSTALL_POSE_DEPS=1', commands)
        self.assertIn('buildx build', commands)

    def test_default_build_omits_pose_deps(self):
        result, commands = self.run_build()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('INSTALL_POSE_DEPS', commands)

    def test_unlisted_build_arg_is_rejected(self):
        # Guard against arbitrary ARG injection (e.g. overriding pinned bases).
        result, commands = self.run_build('--build-arg', 'PYPI_MIRROR=evil')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('build-arg', result.stdout + result.stderr)
        self.assertEqual(commands, '')

    def test_build_sh_keeps_its_executable_bit(self):
        """The documented `./build.sh ...` form must stay runnable.

        The blob tests above call `bash build.sh`, which works even when the
        executable bit is lost — as happened when a Git-data-API push wrote the
        blob with mode 100644. Assert the committed mode directly so that
        user-facing regression cannot slip through again.
        """
        result = subprocess.run(
            ['git', 'ls-files', '-s', 'build.sh'],
            text=True, capture_output=True, cwd=str(ROOT))
        self.assertEqual(result.returncode, 0, result.stderr)
        # git ls-files -s prints: <mode> <sha> <stage>\t<path>
        self.assertTrue(result.stdout.startswith('100755 '),
                        f"build.sh must be executable (100755), got: {result.stdout!r}")


if __name__ == '__main__':
    unittest.main()
