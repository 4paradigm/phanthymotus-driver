"""Save A3 camera snapshots (chest D457 RGB) to persistent storage.

一期只做拍照。帧源是 A3Nodes 维护的最新帧缓存 `latest_frames`（mirror 回调
在容器内逐帧写入，relay 与非 relay 两条编码路径都会经过），因此本卡不新增
任何 DDS 订阅，也不独占相机——预览流与快照共享同一路流。录像依赖镜像加装
ffmpeg，二期连同 Dockerfile 一起提（go1 版有现成口径可抄）。
"""

import os
import re
import time
from datetime import datetime
from pathlib import Path
from uuid import uuid4

FRAME_KEY = "camera_chest_front_d457_rgb"
STREAM_NAME = "chest_front_d457_rgb"


class VisionCapturePlugin:
    PREFIX = "vision_capture"

    def __init__(self, nodes, plugin_config):
        self.nodes = nodes
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture"))
        self._last_capture = None

    # -- tool card --

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "把 A3 胸前相机当前画面（预览分辨率）拍成照片保存到机器人上，可列出、删除存档。",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["capture_photo", "list", "delete", "info"],
                    },
                    "image_name": {"type": "string", "description": "照片文件名，不含 .jpg；留空则自动命名。"},
                    "name": {"type": "string", "description": "删除时填写完整的 .jpg 文件名。"},
                },
                "required": ["action"], "additionalProperties": False,
                # capture_photo 同步返回文件路径，不申报 x-completion：那张表是给
                # 录像这类要等 ACP 完成回调的长动作用的（go1/go2/realman 同款）。
                "x-action-params": {
                    "capture_photo": {"params": ["image_name"], "description": "保存胸前相机当前画面为 JPEG。"},
                    "list": {"params": [], "description": "列出已保存的照片。"},
                    "delete": {"params": ["name"], "description": "按完整文件名删除照片。"},
                    "info": {"params": [], "description": "查看存档目录与最近一次拍照状态。"},
                },
            },
        }

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

    def shutdown(self):
        pass

    def dispatch(self, action, args):
        if action == "capture_photo":
            return self._capture_photo(args or {})
        if action == "list":
            return self._list_files()
        if action == "delete":
            return self._delete_file((args or {}).get("name"))
        if action == "info":
            return self._info()
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        return None

    # -- capture --

    def _latest_frame(self):
        entry = getattr(self.nodes, "latest_frames", {}).get(FRAME_KEY)
        if not entry:
            return None, None
        jpeg, ts = entry
        return jpeg, (time.monotonic() - ts) if ts is not None else None

    def _capture_photo(self, args):
        path = self._file_path(args.get("image_name"))
        if isinstance(path, str):
            # _file_path 校验失败时直接返回错误结构
            return {"ok": False, "code": "INVALID_ARGUMENT", "message": path}
        if path.exists():
            # rename 会静默覆盖同名文件，先拒绝而不是覆盖旧照片。
            return {"ok": False, "code": "FILE_EXISTS", "state": "error",
                    "message": f"file already exists: {path.name}"}
        jpeg, frame_age = self._latest_frame()
        if not jpeg:
            result = {"ok": False, "code": "CAMERA_UNAVAILABLE",
                      "message": "预览流暂无可用帧；确认相机流在跑后重试"}
        elif not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            result = {"ok": False, "code": "CAMERA_UNAVAILABLE",
                      "message": "缓存帧不是完整 JPEG，等待下一帧后重试"}
        else:
            result = self._save_jpeg(path, jpeg)
        result["state"] = "captured" if result.get("ok") else "error"
        if result.get("ok") and frame_age is not None:
            result["frame_age_s"] = round(max(frame_age, 0.0), 2)
        self._last_capture = {"status": result["state"], "result": result}
        return result

    def _save_jpeg(self, path, jpeg):
        temporary_path = None
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = path.with_name(f".{path.name}.tmp")
            # 先写临时文件并同步，再公开正式路径，避免重启后留下空照片。
            with temporary_path.open("xb") as output:
                output.write(jpeg)
                output.flush()
                os.fsync(output.fileno())
            temporary_path.replace(path)
            return {"ok": True, "media_type": "photo",
                    "file_path": str(path), "filename": path.name,
                    "channel_reply_path": self._channel_path(path),
                    "mime": "image/jpeg", "size": len(jpeg),
                    "captured_at": datetime.now().astimezone().isoformat(timespec="seconds")}
        except FileExistsError:
            return {"ok": False, "code": "FILE_EXISTS", "message": f"file already exists: {path.name}"}
        except OSError as exc:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
            return {"ok": False, "code": "SAVE_FAILED", "message": str(exc)}

    def _file_path(self, name):
        directory = self._output_dir / "photos"
        if name:
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,99}", name):
                return "文件名只能用 1-100 个字母、数字、'_' 或 '-'，不带扩展名"
            return directory / f"{name}.jpg"
        return directory / f"{STREAM_NAME}_{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}.jpg"

    def _channel_path(self, path):
        channel_root = Path(os.environ.get("PHANTHY_CHANNEL_OUTPUT_DIR", "/work/resource/vision_capture"))
        try:
            return str(channel_root / path.relative_to(self._output_dir))
        except ValueError:
            return str(path)

    # -- management --

    def _list_files(self):
        files = []
        directory = self._output_dir / "photos"
        if directory.exists():
            for path in directory.iterdir():
                # 点号开头的临时文件尚未发布，不应显示为可用照片。
                if path.is_file() and not path.name.startswith(".") and path.suffix.lower() == ".jpg":
                    try:
                        metadata = path.stat()
                    except FileNotFoundError:
                        # 并发删除已移除文件；跳过该项。
                        continue
                    files.append((metadata.st_mtime,
                                  {"filename": path.name, "path": str(path),
                                   "channel_reply_path": self._channel_path(path),
                                   "size": metadata.st_size, "mime": "image/jpeg"}))
        files.sort(key=lambda item: item[0], reverse=True)
        return {"ok": True, "state": "listed", "files": [item[1] for item in files]}

    def _delete_file(self, name):
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,99}\.jpg", name, re.I):
            return {"ok": False, "code": "INVALID_ARGUMENT",
                    "message": "name must be a complete .jpg filename"}
        path = self._output_dir / "photos" / name
        if not path.is_file():
            return {"ok": False, "code": "NOT_FOUND", "message": f"file not found: {name}"}
        try:
            path.unlink()
        except FileNotFoundError:
            # 另一条删除请求先完成时，返回正常的不存在结果。
            return {"ok": False, "code": "NOT_FOUND", "message": f"file not found: {name}"}
        return {"ok": True, "state": "deleted", "filename": name}

    def _info(self):
        jpeg, frame_age = self._latest_frame()
        info = {"state": "ready", "output_dir": str(self._output_dir),
                "photos_dir": str(self._output_dir / "photos"),
                "stream": STREAM_NAME,
                "frame_available": jpeg is not None,
                "last_capture": self._last_capture}
        if frame_age is not None:
            info["frame_age_s"] = round(max(frame_age, 0.0), 2)
        return info


def make_vision_capture(nodes, plugin_config):
    return VisionCapturePlugin(nodes, plugin_config)
