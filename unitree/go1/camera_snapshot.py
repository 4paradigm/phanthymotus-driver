"""Capture one JPEG directly from a Go1 Nano RGB camera service."""

import json
import logging
import os
import socket
import ssl
import struct
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from uuid import uuid4


if __package__:
    from . import camera
else:
    import camera


POSITIONS = tuple(camera._VALID_POSITIONS)
log = logging.getLogger(__name__)


class CameraSnapshotPlugin:
    def __init__(self, plugin_config):
        self._endpoints = {
            position: (endpoint["board_ip"], int(endpoint["image_port"]))
            for position, endpoint in camera._resolve_positions_raw(plugin_config).items()
        }
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/camera_snapshot"))
        self._active = set()
        self._last_capture = None

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
                "x-completion": {"actions": ["capture_photo"], "timeout": 45},
                "x-resource": "camera",
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
        # 已受理的抓拍继续完成并上报 ACP，不提前释放仍在使用的机位。
        with camera._CAMERA_LOCK:
            return {"state": "idle", "capture_active": bool(self._active)}

    @staticmethod
    def _receive_exact(connection, size, deadline):
        chunks = bytearray()
        while len(chunks) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("camera frame deadline exceeded")
            connection.settimeout(remaining)
            chunk = connection.recv(size - len(chunks))
            if not chunk:
                raise OSError("camera stream closed before a complete JPEG arrived")
            chunks.extend(chunk)
        return bytes(chunks)

    def _capture_jpeg(self, position):
        host, port = self._endpoints[position]
        # 中文说明：连接会让 Nano 按需开启相机；读完一帧即断开并释放机位。
        with socket.create_connection((host, port), timeout=8) as connection:
            # 整帧共用 20 秒期限，避免慢速分片反复重置超时导致 ACP 永不完成。
            deadline = time.monotonic() + 20
            length = struct.unpack(">I", self._receive_exact(connection, 4, deadline))[0]
            if not 0 < length <= 5_000_000:
                raise OSError(f"invalid JPEG frame length: {length}")
            return self._receive_exact(connection, length, deadline)

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            with camera._CAMERA_LOCK:
                return {"state": "capturing" if self._active else "ready",
                        "output_dir": str(self._output_dir), "positions": list(POSITIONS),
                        "last_capture": self._last_capture}
        if action != "capture_photo":
            return None

        position = args.get("position", "front")
        if position not in POSITIONS:
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": "unknown camera position"}
        with camera._CAMERA_LOCK:
            if position in camera._SNAPSHOT_POSITIONS or camera.running_stream(position) is not None:
                return {"ok": False, "code": "RESOURCE_BUSY",
                        "message": f"camera position {position!r} is occupied; stop its stream first"}
            camera._SNAPSHOT_POSITIONS.add(position)
            self._active.add(position)
        action_id = f"camera_snapshot_{uuid4().hex}"
        # 先确定文件名，画布收到受理结果时即可显示目标路径；完成回调才确认文件存在。
        path = self._output_dir / (
            f"{position}_{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}.jpg")
        try:
            threading.Thread(target=self._capture_async, args=(position, action_id, path), daemon=True).start()
        except Exception as exc:
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard(position)
                self._active.discard(position)
            return {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}
        return {"ok": True, "state": "capturing", "action_id": action_id,
                "position": position, "file_path": str(path)}

    def _notify_complete(self, action_id, status, result):
        # 仅用标准库，沿用 Go2 vision_capture 的 ACP/TLS 约定，不增加镜像依赖。
        payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                              "tool": "camera_snapshot", "ts": time.time()}).encode()
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        # 最坏 3×3 秒请求 + 0.5/1 秒退避，留在 45 秒 ACP 超时预算内。
        for attempt in range(3):
            try:
                ctx = ssl.create_default_context()
                if url.startswith(("https://localhost:", "https://127.0.0.1:")):
                    ctx.check_hostname = False
                    ctx.verify_mode = ssl.CERT_NONE
                request = urllib.request.Request(url + "/api/acp/complete", data=payload,
                                                 headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(request, timeout=3, context=ctx):
                    pass
                return
            except Exception as exc:
                if attempt == 2:
                    log.warning("[camera_snapshot] ACP completion delivery failed for %s: %s", action_id, exc)
                else:
                    time.sleep(0.5 * 2 ** attempt)

    def _capture_async(self, position, action_id, path):
        try:
            result = self._capture_and_save(position, path)
        except Exception as exc:
            result = {"ok": False, "code": "CAPTURE_FAILED", "message": str(exc)}
        finally:
            with camera._CAMERA_LOCK:
                camera._SNAPSHOT_POSITIONS.discard(position)
                self._active.discard(position)
        status = "completed" if result.get("ok") else "error"
        with camera._CAMERA_LOCK:
            self._last_capture = {"action_id": action_id, "status": status, "result": result}
        self._notify_complete(action_id, status, result)

    def _capture_and_save(self, position, path):
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
