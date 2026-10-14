"""Exercise the build entry point without Docker or registry access."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BuildArchitectureTests(unittest.TestCase):
    def run_build(self, architectures):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copy(ROOT / 'build.sh', root / 'build.sh')
            commands = root / 'commands.jsonl'
            binary = root / 'bin'
            binary.mkdir()
            for name in ('docker', 'curl'):
                tool = binary / name
                tool.write_text('''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
with open(os.environ['BUILD_TEST_LOG'], 'a') as stream:
    stream.write(json.dumps([Path(sys.argv[0]).name] + sys.argv[1:]) + '\\n')
if Path(sys.argv[0]).name == 'curl':
    Path(sys.argv[sys.argv.index('-o') + 1]).write_text('{}')
    print('200')
''')
                tool.chmod(0o755)
            targets = []
            for index, architecture in enumerate(architectures):
                target = f'vendor/driver{index}'
                targets.append(target)
                driver = root / target
                driver.mkdir(parents=True)
                metadata = f'name: test\nid: test{index}\nimage_name: test{index}\ncategory: driver\nhardware_provider: vendor\nhardware_model: driver{index}\nport: 15741\n'
                if architecture is not None:
                    metadata += f'cpu_arch: {architecture}\n'
                (driver / 'driver.yaml').write_text(metadata)
                (driver / 'Dockerfile').write_text('FROM scratch\n')
            env = dict(os.environ, PATH=str(binary) + os.pathsep + os.environ['PATH'],
                       BUILD_TEST_LOG=str(commands), REGISTRY='example.invalid',
                       REGISTRY_USER='test', REGISTRY_PASSWORD='test',
                       IMAGE_NAMESPACE='drivers', RESOURCE_CENTER_API_KEY='test',
                       RESOURCE_CENTER_URL='https://example.invalid')
            result = subprocess.run(['bash', str(root / 'build.sh'), '--mirror', 'none', *targets],
                                    env=env, text=True, capture_output=True,
                                    start_new_session=True, timeout=30)
            calls = [json.loads(line) for line in commands.read_text().splitlines()] if commands.exists() else []
            return result, calls

    def test_mixed_build_and_registration_architectures_match(self):
        result, calls = self.run_build([None, 'amd64'])
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        builds = [call for call in calls if call[1:3] == ['buildx', 'build']]
        self.assertEqual([call[call.index('--platform') + 1] for call in builds],
                         ['linux/arm64', 'linux/amd64'])
        payloads = [json.loads(call[call.index('-d') + 1]) for call in calls if call[0] == 'curl']
        self.assertEqual([payload['cpu_arch'] for payload in payloads], ['arm64', 'amd64'])

    def test_invalid_architecture_rejected_before_external_calls(self):
        result, calls = self.run_build(['invalid'])
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('cpu_arch', result.stdout)
        self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
