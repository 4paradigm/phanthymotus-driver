"""Native Jetson builds work without changing Docker or installing buildx."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Arm64BuildTests(unittest.TestCase):
    def build(self, architecture):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            log = folder / 'commands'
            docker = folder / 'docker'
            docker.write_text('''#!/bin/bash
if [[ "$1 $2" == "buildx version" ]]; then exit 1; fi
printf '%s\\n' "$*" >> "$TEST_LOG"
context="${@: -1}"
test -f "$context/device.py" && test -f "$context/common/vendor_runtime.py"
''')
            uname = folder / 'uname'
            uname.write_text('#!/bin/bash\nprintf "%s\\n" "$TEST_ARCH"\n')
            docker.chmod(0o755); uname.chmod(0o755)
            env = {**os.environ, 'PATH': str(folder) + os.pathsep + os.environ['PATH'],
                   'TEST_LOG': str(log), 'TEST_ARCH': architecture}
            result = subprocess.run(['bash', str(ROOT/'agilex/piper/build-arm64.sh'), 'piper:test'],
                                    env=env, text=True, capture_output=True)
            return result, log.read_text() if log.exists() else ''

    def test_native_jetson_uses_complete_context_without_buildx(self):
        result, commands = self.build('aarch64')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('build --platform linux/arm64 --tag piper:test', commands)
        self.assertNotIn('run', commands)

    def test_cross_platform_without_buildx_rejects_before_build(self):
        result, commands = self.build('x86_64')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Cross-platform build requires buildx', result.stderr)
        self.assertEqual(commands, '')


if __name__ == '__main__':unittest.main()
