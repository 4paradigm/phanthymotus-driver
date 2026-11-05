"""Insertion-order USB pairing. Never probe the receiver with Modbus frames."""

from __future__ import annotations

import json
import os
from pathlib import Path
import secrets
import stat
import tempfile
import time

from common.vendor_runtime import action_schema, tool

DEFAULT_FILE = "/opt/phanthy-motus/data/linkerhand-l20-lite-usb.json"


def pairing_file():
    return Path(os.environ.get("L20_USB_PAIRING_FILE", DEFAULT_FILE))


def inventory():
    """Use physical USB paths: identical CH340s can collide under by-id."""
    devices = {}
    for path in sorted(Path("/dev/serial/by-path").glob("*")):
        try:
            if stat.S_ISCHR(path.stat().st_mode):
                devices.setdefault(str(path.resolve()), str(path))
        except OSError:
            continue
    return sorted(devices.values())


def validate_mapping(data):
    if not isinstance(data, dict) or data.get("version") != 1:
        raise ValueError("unsupported USB pairing configuration")
    receiver, hands = data.get("receiver"), data.get("hands")
    paths = [receiver, *(hands.values() if isinstance(hands, dict) else [])]
    if (not isinstance(hands, dict) or not hands or
            not set(hands) <= {"left", "right"} or
            any(not isinstance(p, str) or Path(p).parent != Path("/dev/serial/by-path")
                or Path(p).name in ("", ".", "..") for p in paths) or
            len(set(paths)) != len(paths)):
        raise ValueError("invalid or overlapping USB device roles")
    return data


def load_pairing():
    path = pairing_file()
    if not path.exists():
        return None
    return validate_mapping(json.loads(path.read_text()))


def restore_pairing():
    """Saved pairing takes precedence over deployment's initial fallback paths."""
    data = load_pairing()
    if data:
        os.environ["GLOVE_SERIAL_PORT"] = data["receiver"]
        os.environ["L20_SERIAL_PORT"] = next(iter(data["hands"].values()))
    return data


def ensure_not_pairing(hand):
    plugin = getattr(hand, "_usb_pairing", None)
    if plugin and plugin.active():
        raise RuntimeError("USB 配对进行中，请完成或取消配对后再操作设备")
    if os.environ.get("L20_USB_PAIRING_REQUIRED") == "1" and not hand.config.get("usb_paired_hands"):
        raise RuntimeError("请先通过卡片上的 USB 设备配对向导配置接收器和灵巧手")


class UsbPairingPlugin:
    PREFIX = "usb_pairing"

    def __init__(self, hand, glove, *, list_devices=inventory, probe=None, clock=time.monotonic):
        self.hand, self.glove = hand, glove
        self.list_devices, self.clock = list_devices, clock
        self.probe = probe or self._probe
        self.session = None
        hand._usb_pairing = self

    def start(self):
        pass

    def stop(self):
        self.session = None

    def active(self):
        if self.session and self.clock() - self.session["touched"] > 600:
            self.session = None
        return self.session is not None

    def get_tool(self):
        definition = tool(self.PREFIX, "actuator", "按插入顺序配对手套接收器和灵巧手，无需填写串口路径。",
            action_schema({
                "start": ([], "画布生命周期启动；不打开设备或开始配对"),
                "stop": ([], "画布生命周期停止；取消配对，不修改已保存配置"),
                "status": ([], "查看配对状态"),
                "begin": ([], "断开接收器和灵巧手 USB 后开始；会停止预览并清除会话标定"),
                "receiver": (["session_id"], "仅插入无线接收器后确认"),
                "hands": (["session_id"], "再插入已上电的灵巧手转接器后确认，只读识别左右手"),
                "apply": (["session_id"], "保存并应用配对，不启用运动"),
                "cancel": (["session_id"], "取消，保留原端口配置"),
            }, {"session_id": {"type": "string"}}))
        definition["title"] = "USB 设备配对"
        return definition

    def _probe(self, port):
        from modbus_device import L20LiteConnection
        # This is a newly inserted hand candidate, not the receiver captured
        # in this session. The old saved receiver path may now be this USB
        # socket; do not let the previous mapping override the new pairing.
        if not self.session or port == self.session.get("receiver"):
            raise ValueError("refusing a Modbus probe on the receiver")
        config = {**self.hand.config, "serial_port": port, "plugins": {
            **self.hand.config.get("plugins", {}), "glove_teleop": {"enabled": False}}}
        connection = L20LiteConnection(config)
        try:
            with self.hand._bus_lock:
                return connection.detect_sides()
        finally:
            connection.close()

    def _idle(self):
        controllers = [self.hand._legacy, *self.hand._controllers.values()]
        gesture = getattr(self.hand, "_gesture_plugin", None)
        if (self.hand._teleop_active or self.glove._armed or self.glove._arming or
                self.glove._pending_disarm or not self.glove._cleanup_done.is_set() or
                any(c._active is not None for c in controllers) or
                (gesture and gesture._active is not None)):
            raise RuntimeError("请先退出遥操并等待动作及卸力完成，再开始 USB 配对")

    def dispatch(self, action, args):
        # The same admission lock guards manual motion, gestures and arming.
        # Do not hold the bus lock while stopping preview: its cleanup worker
        # needs that lock even when there are no active clients.
        with self.hand._ownership_lock:
            if action == "start":
                return {"state": "ready"}
            if action == "stop":
                self.session = None
                return {"state": "idle"}
            self.active()
            if action in ("info", "status"):
                saved = load_pairing()
                return {"state": self.session["step"] if self.session else "idle",
                        "paired": saved is not None, "mapping": saved,
                        "missing": [p for p in ([saved["receiver"], *saved["hands"].values()] if saved else [])
                                    if p not in self.list_devices()]}
            if action == "begin":
                if self.active():
                    raise RuntimeError("已有配对会话，请先完成或取消，或等待 10 分钟超时")
                self._idle()
                self.glove._stop()
                for controller in [self.hand._legacy, *self.hand._controllers.values()]:
                    controller.connection.close()
                gesture = getattr(self.hand, "_gesture_plugin", None)
                if gesture:
                    for connection in gesture._connections.values():
                        connection.close()
                self.session = {"id": secrets.token_urlsafe(18), "step": "receiver",
                                "baseline": set(self.list_devices()), "touched": self.clock()}
                return {"state": "receiver", "session_id": self.session["id"]}
            session = self.session
            if action == "cancel" and session is None:
                return {"state": "cancelled"}
            if not session or args.get("session_id") != session["id"]:
                raise ValueError("配对会话已过期或不匹配，请重新开始")
            session["touched"] = self.clock()
            self._idle()
            if action == "cancel":
                self.session = None
                return {"state": "cancelled"}
            current = set(self.list_devices())
            if action == "receiver" and session["step"] == "receiver":
                added = current - session["baseline"]
                if len(added) != 1:
                    raise ValueError("请只插入一个无线接收器，再点击确认；若已插着，请取消后按步骤重试")
                session.update(receiver=added.pop(), step="hands")
            elif action == "hands" and session["step"] == "hands":
                receiver = session["receiver"]
                if receiver not in current:
                    raise ValueError("接收器已断开或换口，请取消并重新配对")
                added = current - session["baseline"] - {receiver}
                if not 1 <= len(added) <= 2:
                    raise ValueError("请插入一到两个灵巧手转接器，并保持接收器连接")
                found = {}
                for port in sorted(added):
                    sides = self.probe(port)
                    if len(sides) != 1 or sides[0] not in ("left", "right"):
                        raise ValueError("新增设备未识别为单只灵巧手，请检查电源、RS485 接线和设备地址")
                    if sides[0] in found:
                        raise ValueError("检测到重复的左右手地址，请检查设备配置")
                    found[sides[0]] = port
                session.update(hands=found, step="review")
            elif action == "apply" and session["step"] == "review":
                data = validate_mapping({"version": 1, "receiver": session["receiver"], "hands": session["hands"]})
                if not {data["receiver"], *data["hands"].values()} <= current:
                    raise ValueError("设备已断开或换口，请取消并重新配对")
                # Revalidate addresses immediately before saving; never probe receiver.
                for side, port in data["hands"].items():
                    if self.probe(port) != [side]:
                        raise ValueError("灵巧手识别发生变化，请重新配对")
                path = pairing_file()
                path.parent.mkdir(parents=True, exist_ok=True)
                fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=".usb-pairing-")
                try:
                    with os.fdopen(fd, "w") as handle:
                        json.dump(data, handle, ensure_ascii=False, indent=2)
                        handle.flush()
                        os.fsync(handle.fileno())
                    os.replace(temporary, path)
                finally:
                    if os.path.exists(temporary):
                        os.unlink(temporary)
                self._apply(data)
                self.session = None
                return {"state": "paired", "mapping": data, "armed": False,
                        "message": "配对已保存。保持 USB 插孔位置，重新开始预览并标定；不会自动启用运动。"}
            else:
                raise ValueError("配对步骤不匹配，请按向导顺序操作")
            return {"state": session["step"], "session_id": session["id"],
                    "receiver": session.get("receiver"), "hands": session.get("hands", {})}

    def _apply(self, data):
        os.environ["GLOVE_SERIAL_PORT"] = data["receiver"]
        os.environ["L20_SERIAL_PORT"] = next(iter(data["hands"].values()))
        self.hand.config["serial_port"] = os.environ["L20_SERIAL_PORT"]
        self.hand.config["usb_paired_hands"] = data["hands"]
        for controller in [self.hand._legacy, *self.hand._controllers.values()]:
            controller.connection.close()
            controller.connection._glove_port = data["receiver"]
        self.hand._legacy.connection.port = os.environ["L20_SERIAL_PORT"]
        self.hand._controllers.clear()
        gesture = getattr(self.hand, "_gesture_plugin", None)
        if gesture:
            for connection in gesture._connections.values():
                connection.close()
            gesture._connections.clear()
        self.glove.glove_port = data["receiver"]
