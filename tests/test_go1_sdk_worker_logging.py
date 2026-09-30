"""A real spawn worker protects Python SDK logs before SDK import/start."""

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GO1 = ROOT / "unitree" / "go1"


def test_spawn_worker_scrubs_and_frames_sdk_logs(tmp_path):
    # A deterministic SDK substitute exercises import-time output and threads
    # without loading robot_interface or connecting to a robot.
    (tmp_path / "go1_sdk_client.py").write_text(r'''
import sys
import threading

print("\x1b[31mSDK import\x1b[0m\x00", flush=True)
print("SDK stderr\x00\x1b[31m\x1b[0m", file=sys.stderr, flush=True)

class Go1HighSdkClient:
    available = False
    def __init__(self, **kwargs):
        pass
    def start(self):
        threads = [threading.Thread(target=lambda i=i: print(
            f"SDK-line-{i}:" + "x" * 5000 + "\x00\x1b[31m", flush=True))
            for i in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
    def snapshot(self):
        return {"fresh": False}
    def stop(self):
        pass
''')
    runner = tmp_path / "run_worker.py"
    runner.write_text(f'''
import multiprocessing
import sys
sys.path.extend([{str(GO1)!r}, {str(ROOT)!r}])
from sdk_proxy import _sdk_worker

if __name__ == "__main__":
    from common.logsafe import PIPE_BUF
    ctx = multiprocessing.get_context("spawn")
    commands, results = ctx.Queue(), ctx.Queue()
    child = ctx.Process(target=_sdk_worker,
        args=(commands, results, "", "127.0.0.1", 8082, 8090))
    try:
        child.start()
        assert results.get(timeout=5) == {{"available": False}}
        commands.put({{"cmd": "snapshot"}})
        assert results.get(timeout=5) == {{"result": {{"fresh": False}}}}
        commands.put(None)
        child.join(timeout=5)
        assert not child.is_alive() and child.exitcode == 0
    finally:
        if child.is_alive():
            child.terminate()
            child.join(timeout=3)
        for queue in (commands, results):
            queue.close()
            queue.join_thread()
    print("PIPE_BUF=" + str(PIPE_BUF))
''')
    result = subprocess.run(
        [sys.executable, str(runner)], capture_output=True, timeout=20,
        env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert b"\x00" not in result.stdout + result.stderr
    assert b"\x1b" not in result.stdout + result.stderr
    lines = result.stdout.splitlines()
    assert b"SDK import" in lines
    assert b"SDK stderr" in result.stderr.splitlines()
    assert b"[SdkWorker] ready (STUB)" in lines
    assert b"[SdkWorker] stopped" in lines
    limit = int(next(line.split(b"=", 1)[1] for line in lines
                     if line.startswith(b"PIPE_BUF=")))
    records = [line for line in lines if line.startswith(b"SDK-line-")]
    assert len(records) == 8
    for index in range(8):
        assert sum(line.startswith(f"SDK-line-{index}:".encode()) for line in records) == 1
    assert all(len(line) + 1 <= limit for line in records)
