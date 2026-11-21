"""The standalone companion must protect logs before loading dependencies."""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('layout', ['checkout', 'image'])
@pytest.mark.parametrize('standalone', [True, False])
def test_companion_logging_before_numpy(tmp_path, layout, standalone):
    # Real logsafe and entrypoint, with hardware imports stopped at NumPy.
    directory = tmp_path / 'x-humanoid/tianyi2.0' if layout == 'checkout' else tmp_path
    directory.mkdir(parents=True, exist_ok=True)
    script = directory / 'estimated_body_tf.py'
    shutil.copyfile(ROOT / 'x-humanoid/tianyi2.0/estimated_body_tf.py', script)
    (tmp_path / 'common').mkdir()
    shutil.copyfile(ROOT / 'common/logsafe.py', tmp_path / 'common/logsafe.py')
    code = r'''
import importlib.abc
import runpy
import sys
class StopAtNumpy(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'numpy':
            print('out\x00\x1b[31mput\x1b[0m', flush=True)
            print('err\x00or', file=sys.stderr, flush=True)
            raise SystemExit(0)
sys.meta_path.insert(0, StopAtNumpy())
runpy.run_path(sys.argv[1], run_name=sys.argv[2])
'''
    result = subprocess.run(
        [sys.executable, '-I', '-c', code, str(script),
         '__main__' if standalone else 'imported_companion'],
        cwd=tmp_path.parent, capture_output=True, check=True,
    )
    if standalone:
        assert result.stdout == b'output\n'
        assert result.stderr == b'error\n'
    else:
        assert result.stdout == b'out\x00\x1b[31mput\x1b[0m\n'
        assert result.stderr == b'err\x00or\n'
