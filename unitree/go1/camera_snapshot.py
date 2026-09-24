"""Save a fresh JPEG from an already running Go1 RGB camera card."""

from datetime import datetime
from pathlib import Path
from uuid import uuid4


POSITIONS = ("front", "chin", "left", "right", "belly")


class CameraSnapshotPlugin:
    def __init__(self, plugin_config, camera_rgb):
        self._camera_rgb = camera_rgb
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/camera_snapshot"))

    def get_tool(self):
        return {
            "name": "camera_snapshot", "type": "actuator", "multiInstance": False,
            "description": "Save a fresh JPEG from a running Go1 RGB camera position.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "capture_photo", "info", "stop"]},
                    "position": {"type": "string", "enum": list(POSITIONS), "default": "front"},
                },
                "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": [], "description": "准备抓拍卡。"},
                    "capture_photo": {"params": ["position"], "description": "保存指定机位的新 JPEG。"},
                    "info": {"params": [], "description": "查看抓拍文件目录。"},
                    "stop": {"params": [], "description": "停用抓拍卡，不影响相机流。"},
                },
            },
        }

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

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
        stream = self._camera_rgb.running_stream(position) if self._camera_rgb else None
        if stream is None:
            return {"ok": False, "code": "CAMERA_UNAVAILABLE",
                    "message": f"start camera_rgb at position {position} before capture"}
        sequence = stream.frame_sequence()
        jpeg = stream.wait_for_frame(sequence, timeout_s=10)
        if not jpeg or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            return {"ok": False, "code": "CAMERA_UNAVAILABLE",
                    "message": f"no fresh JPEG from {position} within 10 seconds"}

        path = None
        try:
            self._output_dir.mkdir(parents=True, exist_ok=True)
            path = self._output_dir / (
                f"{position}_{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}.jpg")
            with path.open("xb") as output:
                output.write(jpeg)
            return {"ok": True, "position": position, "media_type": "photo",
                    "file_path": str(path),
                    "captured_at": datetime.now().astimezone().isoformat(timespec="seconds")}
        except OSError as exc:
            if path is not None:
                path.unlink(missing_ok=True)
            return {"ok": False, "code": "SAVE_FAILED", "message": str(exc)}
