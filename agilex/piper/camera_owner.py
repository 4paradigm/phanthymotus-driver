"""Shared nonblocking camera ownership for live video and one-shot capture."""
import fcntl
import hashlib
from pathlib import Path


class CameraOwner:
    def __init__(self, data_dir, device):
        directory = Path(data_dir)
        directory.mkdir(parents=True, exist_ok=True)
        identity = hashlib.sha256(str(device).encode()).hexdigest()[:16]
        self.handle = (directory / ('camera-owner-' + identity + '.lock')).open('a+')
        try:
            fcntl.flock(self.handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.handle.close()
            raise RuntimeError('相机正在被实时监控或拍照占用，请先停止另一项采集') from None

    def close(self):
        self.handle.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
