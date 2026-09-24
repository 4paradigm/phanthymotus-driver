"""Capture one JPEG directly from a Go1 Nano RGB camera service."""

import os
import socket
import struct
from datetime import datetime
from pathlib import Path
from uuid import uuid4


POSITIONS = ("front", "chin", "left", "right", "belly")
DEFAULT_ENDPOINTS = {
    "front": ("192.168.123.13", 9201),
    "chin": ("192.168.123.13", 9202),
    "left": ("192.168.123.14", 9203),
    "right": ("192.168.123.14", 9204),
    "belly": ("192.168.123.15", 9205),
}


class CameraSnapshotPlugin:
    def __init__(self, plugin_config):
        self._endpoints = dict(DEFAULT_ENDPOINTS)
        for position, endpoint in (plugin_config.get("positions") or {}).items():
            if position in self._endpoints:
                self._endpoints[position] = (endpoint["board_ip"], endpoint["image_port"])
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/camera_snapshot"))

    def get_tool(self):
        return {
            "name": "camera_snapshot", "type": "actuator", "multiInstance": False,
            "description": "Capture and save a JPEG directly from a Go1 camera position.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "capture_photo", "info", "stop"]},
                    "position": {"type": "string", "enum": list(POSITIONS), "default": "front"},
                },
                "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "准备抓拍卡，无需启动 camera_rgb。"},
                    "capture_photo": {"params": ["position"], "description": "保存指定机位的新 JPEG。"},
                    "info": {"params": [], "description": "查看抓拍文件目录。"},
                    "stop": {"params": [], "description": "停用抓拍卡。"},
                },
            },
        }

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

    @staticmethod
    def _receive_exact(connection, size):
        chunks = bytearray()
        while len(chunks) < size:
            chunk = connection.recv(size - len(chunks))
            if not chunk:
                raise OSError("camera stream closed before a complete JPEG arrived")
            chunks.extend(chunk)
        return bytes(chunks)

    def _capture_jpeg(self, position):
        host, port = self._endpoints[position]
        # 中文说明：连接会让 Nano 按需开启相机；读完一帧即断开并释放机位。
        with socket.create_connection((host, port), timeout=8) as connection:
            connection.settimeout(20)
            length = struct.unpack(">I", self._receive_exact(connection, 4))[0]
            if not 0 < length <= 5_000_000:
                raise OSError(f"invalid JPEG frame length: {length}")
            return self._receive_exact(connection, length)

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            return {"state": "ready", "output_dir": str(self._output_dir),
                    "positions": list(POSITIONS)}
        if action != "capture_photo":
            return None

        position = args.get("position", "front")
        if position not in POSITIONS:
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": "unknown camera position"}
        try:
            jpeg = self._capture_jpeg(position)
        except OSError as exc:
            return {"ok": False, "code": "CAMERA_UNAVAILABLE",
                    "message": f"cannot capture from {position}: {exc}"}
        if not jpeg or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            return {"ok": False, "code": "CAMERA_UNAVAILABLE",
                    "message": f"invalid JPEG from {position}"}

        temporary_path = None
        try:
            self._output_dir.mkdir(parents=True, exist_ok=True)
            path = self._output_dir / (
                f"{position}_{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}.jpg")
            temporary_path = path.with_name(f".{path.name}.tmp")
            # 中文说明：先写入并同步临时文件，再公开 JPEG 路径，避免重启后留下空照片。
            with temporary_path.open("xb") as output:
                output.write(jpeg)
                output.flush()
                os.fsync(output.fileno())
            temporary_path.replace(path)
            return {"ok": True, "position": position, "media_type": "photo",
                    "file_path": str(path),
                    "captured_at": datetime.now().astimezone().isoformat(timespec="seconds")}
        except OSError as exc:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
            return {"ok": False, "code": "SAVE_FAILED", "message": str(exc)}
