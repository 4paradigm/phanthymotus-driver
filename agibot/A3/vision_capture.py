"""Save A3 camera snapshots (chest D457 RGB) to persistent storage.

一期只做拍照。帧源是 A3Nodes 的 `LatestFrameCache`（latest_frames.py）：relay
转发与容器内重编码两条路径都在发布前写入，本卡只登记自己消费的流并按年龄取帧，
因此不新增任何 DDS 订阅，也不独占相机——预览流与快照共享同一路流。录像依赖镜像
加装 ffmpeg，二期连同 Dockerfile 一起提（go1 版有现成口径可抄）。

卡片的四个 action 返回同一个信封：`ok / state`，失败再加 `code / message`。
"""

import os
import re
from datetime import datetime
from pathlib import Path
from uuid import uuid4

FRAME_KEY = "camera_chest_front_d457_rgb"
STREAM_NAME = "chest_front_d457_rgb"
# 预览流正常刷新间隔是几十毫秒到几百毫秒（真机实测 frame_age 0.01–0.25 s）；
# 超过这个秒数说明流已经停了或卡住。存档不能拿旧画面当“现在”交差——调用方会
# 拿它当刚拍的证据，所以宁可直接报 CAMERA_UNAVAILABLE 让调用方重试。
DEFAULT_MAX_FRAME_AGE_S = 2.0


class VisionCapturePlugin:
    PREFIX = "vision_capture"

    def __init__(self, nodes, plugin_config):
        # 接线错误要炸在构建时：没有帧缓存就不可能拍照。
        self._frames = nodes.latest_frames
        self._frames.register(FRAME_KEY, jpeg=True)
        self._output_dir = Path(plugin_config.get(
            "output_dir", "/opt/phanthy-motus/data/vision_capture"))
        self._max_frame_age_s = float(plugin_config.get(
            "max_frame_age_s", DEFAULT_MAX_FRAME_AGE_S))
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
        return self._ok("ready")

    def stop(self):
        return self._ok("idle")

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

    # -- result envelope: every action returns ok/state, failures add code --

    @staticmethod
    def _ok(state, **payload):
        return {"ok": True, "state": state, **payload}

    @staticmethod
    def _error(code, message):
        return {"ok": False, "state": "error", "code": code, "message": message}

    # -- capture --

    def _capture_photo(self, args):
        path, error = self._resolve_name(args.get("image_name"))
        if error is not None:
            result = self._error("INVALID_ARGUMENT", error)
        elif path.exists():
            # rename 会静默覆盖同名文件，先拒绝而不是覆盖旧照片。
            result = self._error("FILE_EXISTS", f"file already exists: {path.name}")
        else:
            frame = self._frames.get(FRAME_KEY, self._max_frame_age_s)
            if frame is None:
                result = self._error("CAMERA_UNAVAILABLE", self._unavailable_message())
            else:
                result = self._save_jpeg(path, frame.data, frame.age_s)
        self._last_capture = {"status": result["state"], "result": result}
        return result

    def _unavailable_message(self):
        age = self._frames.age(FRAME_KEY)
        if age is None:
            return "预览流暂无可用帧；确认相机流在跑后重试"
        if age > self._max_frame_age_s:
            return (f"缓存帧已过期（{age:.1f} s，上限 "
                    f"{self._max_frame_age_s:g} s）；确认预览流在跑后重试")
        # 时钟回退时时间戳在未来：无法判定新鲜度，等下一帧。
        return "缓存帧时间戳异常（时钟回退）；等待下一帧后重试"

    def _save_jpeg(self, path, jpeg, frame_age_s):
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
            return self._ok(
                "captured", media_type="photo",
                file_path=str(path), filename=path.name,
                channel_reply_path=self._channel_path(path),
                mime="image/jpeg", size=len(jpeg),
                captured_at=datetime.now().astimezone().isoformat(timespec="seconds"),
                frame_age_s=round(max(frame_age_s, 0.0), 2))
        except FileExistsError:
            return self._error("FILE_EXISTS", f"file already exists: {path.name}")
        except OSError as exc:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)
            return self._error("SAVE_FAILED", str(exc))

    def _resolve_name(self, name):
        """Map a requested filename to a path, or to a validation message."""
        directory = self._output_dir / "photos"
        if name:
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,99}", name):
                return None, "文件名只能用 1-100 个字母、数字、'_' 或 '-'，不带扩展名"
            return directory / f"{name}.jpg", None
        return directory / f"{STREAM_NAME}_{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:8]}.jpg", None

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
        return self._ok("listed", files=[item[1] for item in files])

    def _delete_file(self, name):
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,99}\.jpg", name, re.I):
            return self._error("INVALID_ARGUMENT", "name must be a complete .jpg filename")
        path = self._output_dir / "photos" / name
        if not path.is_file():
            return self._error("NOT_FOUND", f"file not found: {name}")
        try:
            path.unlink()
        except FileNotFoundError:
            # 另一条删除请求先完成时，返回正常的不存在结果。
            return self._error("NOT_FOUND", f"file not found: {name}")
        return self._ok("deleted", filename=name)

    def _info(self):
        age = self._frames.age(FRAME_KEY)
        info = self._ok(
            "ready", output_dir=str(self._output_dir),
            photos_dir=str(self._output_dir / "photos"),
            stream=STREAM_NAME,
            frame_available=age is not None,
            frame_fresh=self._frames.get(FRAME_KEY, self._max_frame_age_s) is not None,
            max_frame_age_s=self._max_frame_age_s,
            last_capture=self._last_capture)
        if age is not None:
            info["frame_age_s"] = round(max(age, 0.0), 2)
        return info


def make_vision_capture(nodes, plugin_config):
    return VisionCapturePlugin(nodes, plugin_config)
