"""Exercise the real child entrypoint in an isolated interpreter."""
import subprocess
import sys


def test_projection_worker_installs_atomic_logging_before_reading_jobs():
    script = '''
from deep_robotics.s10_pro.front_depth import _project_worker
class Jobs:
    def get(self):
        print("worker\\x00\\x1b[31m diagnostic\\x1b[0m", flush=True)
        return None
_project_worker(Jobs(), None)
'''
    result = subprocess.run([sys.executable, '-c', script],
                            capture_output=True, check=True)
    assert result.stdout == b'worker diagnostic\n'
