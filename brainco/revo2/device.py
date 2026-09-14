#!/usr/bin/env python3
"""BrainCo Revo 2 灵巧手 — hand (actuator) + hand_state (sensor) [+ hand_touch]。

Revo 2 通过 RS485/CAN FD 直连主机（不是机器人自带的 DDS 网络），所以这里只用
common.vendor_runtime 的 core domain 发布状态遥测，不使用 robot domain。

bc_stark_sdk (PyPI: bc-stark-sdk) 的 DeviceContext 全部是协程方法，MCP 的
dispatch() 是同步的，所以用 _AsyncBridge 起一个专用事件循环线程去 run_coroutine_threadsafe。
"""

from __future__ import annotations

import asyncio
import inspect
import json
import threading
import time

from common.vendor_runtime import action_schema, jsonable, tool

FINGER_NAMES = ["thumb", "thumb_aux", "index", "middle", "ring", "pinky"]

GESTURE_NAMES = {
    "open": "DefaultGestureOpen",
    "fist": "DefaultGestureFist",
    "pinch_two": "DefaultGesturePinchTwo",
    "pinch_three": "DefaultGesturePinchThree",
    "pinch_side": "DefaultGesturePinchSide",
    "point": "DefaultGesturePoint",
}

# SDK's unified position range: 0 = fully open, 1000 = fully closed (same
# across Modbus/CAN/CANFD/EtherCAT and all Revo2 variants).
POSITION_MIN = 0
POSITION_MAX = 1000


def _enum_name(value):
    """Best-effort readable name for a pyo3 enum value."""
    name = getattr(value, "name", None)
    if name:
        return name
    return str(value)


def _finger_id(sdk, name: str):
    idx = FINGER_NAMES.index(name)
    member_name = ["Thumb", "ThumbAux", "Index", "Middle", "Ring", "Pinky"][idx]
    return getattr(sdk.FingerId, member_name)


def _coerce_positions(values, *, expected: int = 6) -> list[int]:
    if values is None:
        raise ValueError("position vector is required")
    try:
        values = list(values)
    except TypeError as exc:
        raise ValueError("position vector must be an array") from exc
    if len(values) != expected:
        raise ValueError(f"position vector must contain {expected} values")
    result = []
    for v in values:
        try:
            v = float(v)
        except (TypeError, ValueError) as exc:
            raise ValueError("positions must be numbers") from exc
        result.append(int(max(POSITION_MIN, min(POSITION_MAX, round(v)))))
    return result


class _AsyncBridge:
    """One asyncio event loop in a background thread, used to await bc_stark_sdk coroutines."""

    def __init__(self):
        self._loop = None
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, name="revo2_async", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=5.0)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._ready.set()
        self._loop.run_forever()

    def run(self, coro, timeout: float = 5.0):
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=timeout)

    def maybe_await(self, value, timeout: float = 5.0):
        """Resolve `value`, whether the SDK call that produced it was sync or async.

        DeviceContext methods are confirmed coroutines by the SDK docs; a few
        module-level constructors (modbus_open/init_device_handler) aren't
        documented either way, so every call site routes through this instead
        of assuming.
        """
        if asyncio.iscoroutine(value) or inspect.isawaitable(value):
            return self.run(value, timeout=timeout)
        return value

    def close(self):
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)


class RevoNodes:
    """Owns the SDK connection, the core-domain state publisher, and connection retry."""

    def __init__(self, config, namespace, ros2):
        self.config = config
        self.namespace = namespace
        self.hands = config.get("hands") or [{"side": "right", "slave_id": 127}]
        self.variant = str(config.get("variant", "basic")).lower()
        self.bridge = _AsyncBridge()
        self.sdk = None
        self.ctx = None
        self._connect_lock = threading.Lock()
        self._last_connect_attempt = 0.0
        self._connect_retry_s = 5.0

        from rclpy.node import Node

        self.core = Node("revo2_driver_core", context=ros2.ctx_core)
        ros2.executor_core.add_node(self.core)
        from std_msgs.msg import String

        self.state_topic = f"/{namespace}/revo2/hand_state"
        self.state_pub = self.core.create_publisher(String, self.state_topic, 10)
        self.touch_topic = f"/{namespace}/revo2/hand_touch"
        self.touch_pub = self.core.create_publisher(String, self.touch_topic, 10)

        self._try_connect()

    def _try_connect(self) -> bool:
        with self._connect_lock:
            now = time.monotonic()
            if self.ctx is not None:
                return True
            if now - self._last_connect_attempt < self._connect_retry_s:
                return False
            self._last_connect_attempt = now
            try:
                import bc_stark_sdk.main_mod as sdk
            except ImportError as exc:
                print(f"[revo2] bc_stark_sdk not installed: {exc}", flush=True)
                return False
            self.sdk = sdk
            protocol = str(self.config.get("protocol", "canfd")).lower()
            conn = self.config.get("connection", {})
            try:
                if protocol == "modbus":
                    baudrate = getattr(sdk.Baudrate, conn.get("baudrate", "Baud460800"))
                    self.ctx = self.bridge.maybe_await(sdk.modbus_open(conn.get("port_name", "/dev/ttyUSB0"), baudrate))
                elif protocol == "canfd":
                    sdk.init_socketcan_canfd(conn.get("can_iface", "can0"))
                    self.ctx = self.bridge.maybe_await(sdk.init_device_handler(sdk.StarkProtocolType.CanFd))
                else:
                    print(f"[revo2] unsupported protocol: {protocol} (v1 supports modbus/canfd only)", flush=True)
                    return False
                print(f"[revo2] connected via {protocol}", flush=True)
                return True
            except Exception as exc:
                print(f"[revo2] connect failed ({protocol}): {exc}", flush=True)
                self.ctx = None
                return False

    def slave_id(self, side: str) -> int | None:
        for hand in self.hands:
            if hand.get("side") == side:
                return hand.get("slave_id")
        return None

    def default_slave_id(self) -> int:
        return self.hands[0]["slave_id"]

    def require_ctx(self):
        if self.ctx is None and not self._try_connect():
            raise RuntimeError("Revo2 not connected (check protocol/port/CAN interface in config.yaml)")
        return self.ctx

    def close(self):
        try:
            if self.ctx is not None and self.sdk is not None:
                protocol = str(self.config.get("protocol", "canfd")).lower()
                if protocol == "modbus":
                    self.bridge.maybe_await(self.sdk.modbus_close(self.ctx))
                else:
                    self.bridge.maybe_await(self.sdk.close_device_handler(self.ctx))
        except Exception as exc:
            print(f"[revo2] close failed: {exc}", flush=True)
        self.bridge.close()


class HandPlugin:
    """Direct finger/position/gesture/LED control — actuator tool `hand`."""

    def __init__(self, nodes: RevoNodes, plugin_config: dict):
        self.nodes = nodes
        self.open_positions = _coerce_positions(plugin_config.get("open_positions", [0] * 6))
        self.close_positions = _coerce_positions(plugin_config.get("close_positions", [1000] * 6))

    def get_tool(self) -> dict:
        return tool(
            "hand",
            "actuator",
            (
                "BrainCo Revo 2 灵巧手电机控制 — 位置范围 0~1000 (0=全开, 1000=全闭)。"
                "这是驱动真实电机运动的工具；执行任何张合/手势动作前，必须先向用户说明"
                "具体动作并等待明确确认。"
            ),
            action_schema(
                {
                    "get_state": ([], "读取所有手指当前位置/速度/电流/电机状态"),
                    "get_device_info": ([], "读取设备信息 (序列号/固件/型号/左右手)"),
                    "open": (["side"], "张开所有手指（使用配置的 open_positions）"),
                    "close": (["side"], "握紧所有手指（使用配置的 close_positions）"),
                    "set_position": (["finger", "position", "side"], "设置单个手指目标位置 (0~1000)"),
                    "set_positions": (["positions", "side"], "设置全部 6 个手指目标位置数组"),
                    "run_gesture": (["gesture", "side"], "触发预设手势"),
                    "calibrate": (["side"], "位置校准（上电后必须执行一次，期间所有手指会打开）"),
                    "reset_gesture": (["side"], "恢复默认手势配置"),
                    "set_led": (["color", "mode", "side"], "设置手背 LED 颜色/模式"),
                    "get_button_event": (["side"], "读取手背按键的最近一次按下/松开事件（原始事件，不含长按/双击语义）"),
                },
                {
                    "side": {"type": "string", "enum": ["left", "right"], "description": "目标手（未配置该手时使用默认手）"},
                    "finger": {"type": "string", "enum": FINGER_NAMES},
                    "position": {"type": "integer", "minimum": POSITION_MIN, "maximum": POSITION_MAX},
                    "positions": {"type": "array", "items": {"type": "integer"}, "minItems": 6, "maxItems": 6},
                    "gesture": {"type": "string", "enum": list(GESTURE_NAMES)},
                    "color": {"type": "string", "enum": ["Unchanged", "R", "G", "RG", "B", "RB", "GB", "RGB"]},
                    "mode": {"type": "string", "enum": ["Shutdown", "Keep", "Blink", "OneShot", "Blink0_5Hz", "Blink2Hz"]},
                },
            ),
        )

    def start(self):
        pass

    def stop(self):
        pass

    def _slave_id(self, args: dict) -> int:
        side = args.get("side")
        if side:
            slave_id = self.nodes.slave_id(side)
            if slave_id is None:
                raise ValueError(f"no hand configured for side={side}")
            return slave_id
        return self.nodes.default_slave_id()

    def dispatch(self, action: str, args: dict) -> dict | None:
        ctx = self.nodes.require_ctx()
        sdk = self.nodes.sdk
        bridge = self.nodes.bridge
        slave_id = self._slave_id(args)

        if action == "get_state":
            positions = bridge.maybe_await(ctx.get_finger_positions(slave_id))
            speeds = bridge.maybe_await(ctx.get_finger_speeds(slave_id))
            currents = bridge.maybe_await(ctx.get_finger_currents(slave_id))
            motor_state = bridge.maybe_await(ctx.get_motor_state(slave_id))
            return jsonable({
                "positions": dict(zip(FINGER_NAMES, positions)),
                "speeds": dict(zip(FINGER_NAMES, speeds)),
                "currents": dict(zip(FINGER_NAMES, currents)),
                "motor_state": _enum_name(motor_state),
            })

        if action == "get_device_info":
            info = bridge.maybe_await(ctx.get_device_info(slave_id))
            return {
                "serial_number": getattr(info, "serial_number", None),
                "firmware_version": getattr(info, "firmware_version", None),
                "hardware_version": getattr(info, "hardware_version", None),
                "hand_type": _enum_name(getattr(info, "hand_type", None)),
                "hardware_type": _enum_name(getattr(info, "hardware_type", None)),
            }

        if action == "open":
            bridge.maybe_await(ctx.set_finger_positions(slave_id, self.open_positions))
            return {"state": "opening", "positions": dict(zip(FINGER_NAMES, self.open_positions))}

        if action == "close":
            bridge.maybe_await(ctx.set_finger_positions(slave_id, self.close_positions))
            return {"state": "closing", "positions": dict(zip(FINGER_NAMES, self.close_positions))}

        if action == "set_position":
            finger = args.get("finger")
            if finger not in FINGER_NAMES:
                raise ValueError(f"finger must be one of {FINGER_NAMES}")
            position = int(max(POSITION_MIN, min(POSITION_MAX, round(float(args.get("position", 0))))))
            bridge.maybe_await(ctx.set_finger_position(slave_id, _finger_id(sdk, finger), position))
            return {"state": "moving", "finger": finger, "position": position}

        if action == "set_positions":
            positions = _coerce_positions(args.get("positions"))
            bridge.maybe_await(ctx.set_finger_positions(slave_id, positions))
            return {"state": "moving", "positions": dict(zip(FINGER_NAMES, positions))}

        if action == "run_gesture":
            gesture = args.get("gesture")
            if gesture not in GESTURE_NAMES:
                raise ValueError(f"gesture must be one of {list(GESTURE_NAMES)}")
            action_id = getattr(sdk.ActionSequenceId, GESTURE_NAMES[gesture])
            bridge.maybe_await(ctx.run_action_sequence(slave_id, action_id))
            return {"state": "running_gesture", "gesture": gesture}

        if action == "calibrate":
            bridge.maybe_await(ctx.calibrate_position(slave_id))
            return {"state": "calibrating"}

        if action == "reset_gesture":
            bridge.maybe_await(ctx.reset_default_gesture(slave_id))
            return {"state": "reset"}

        if action == "set_led":
            color = getattr(sdk.LedColor, args.get("color", "G"))
            mode = getattr(sdk.LedMode, args.get("mode", "Keep"))
            led_info = sdk.LedInfo(color, mode)
            bridge.maybe_await(ctx.set_led_info(slave_id, led_info))
            return {"state": "set", "color": args.get("color", "G"), "mode": args.get("mode", "Keep")}

        if action == "get_button_event":
            event = bridge.maybe_await(ctx.get_button_event(slave_id))
            return {
                "button_id": getattr(event, "button_id", None),
                "press_state": _enum_name(getattr(event, "press_state", None)),
                "timestamp": getattr(event, "timestamp", None),
            }

        return None


class HandStatePlugin:
    """Periodic finger-position/motor-state telemetry — sensor tool `hand_state`."""

    def __init__(self, nodes: RevoNodes, plugin_config: dict):
        self.nodes = nodes
        self.poll_interval_s = float(plugin_config.get("poll_interval_s", 0.15))
        self._stop_event = threading.Event()
        self._thread = None

    def get_tool(self) -> dict:
        return tool(
            "hand_state",
            "sensor",
            "BrainCo Revo 2 手指位置/速度/电流/电机状态遥测",
            topic_out=[{"topic": self.nodes.state_topic, "format": "data/json"}],
        )

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, name="revo2_state_poll", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        # HandStatePlugin owns closing the shared SDK connection — it's always
        # enabled by default and stops last-ish, mirroring lynx_m20's
        # M20StatePlugin.stop() -> nodes.close() pattern.
        self.nodes.close()

    def _poll_loop(self):
        from std_msgs.msg import String

        while not self._stop_event.is_set():
            if self.nodes.ctx is not None:
                try:
                    payload = {"hands": {}}
                    bridge = self.nodes.bridge
                    ctx = self.nodes.ctx
                    for hand in self.nodes.hands:
                        slave_id = hand["slave_id"]
                        positions = bridge.maybe_await(ctx.get_finger_positions(slave_id), timeout=1.0)
                        motor_state = bridge.maybe_await(ctx.get_motor_state(slave_id), timeout=1.0)
                        voltage = bridge.maybe_await(ctx.get_voltage(slave_id), timeout=1.0)
                        payload["hands"][hand["side"]] = {
                            "positions": dict(zip(FINGER_NAMES, positions)),
                            "motor_state": _enum_name(motor_state),
                            "voltage": voltage,
                        }
                    msg = String()
                    msg.data = json.dumps(jsonable(payload), ensure_ascii=False)
                    self.nodes.state_pub.publish(msg)
                except Exception as exc:
                    print(f"[revo2] state poll failed: {exc}", flush=True)
            else:
                self.nodes._try_connect()
            self._stop_event.wait(self.poll_interval_s)


TOUCH_READERS = {
    "capacitive": "get_touch_sensor_status",
    "pressure": "get_modulus_touch_data",
    "array_pressure": "get_array_pressure_touch_data",
    "force3d": "get_force3d_finger_array",
}


class HandTouchPlugin:
    """Fingertip tactile telemetry — sensor tool `hand_touch`. Only meaningful on variant=touch."""

    def __init__(self, nodes: RevoNodes, plugin_config: dict):
        self.nodes = nodes
        self.poll_interval_s = float(plugin_config.get("poll_interval_s", 0.1))
        self.reader_name = TOUCH_READERS.get(
            str(nodes.config.get("touch_vendor", "array_pressure")).lower(),
            "get_array_pressure_touch_data",
        )
        self._stop_event = threading.Event()
        self._thread = None

    def get_tool(self) -> dict:
        return tool(
            "hand_touch",
            "sensor",
            "BrainCo Revo 2 触觉版指尖触觉数据（仅 variant=touch 时有意义）",
            topic_out=[{"topic": self.nodes.touch_topic, "format": "data/json"}],
        )

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, name="revo2_touch_poll", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _poll_loop(self):
        from std_msgs.msg import String

        while not self._stop_event.is_set():
            if self.nodes.ctx is not None:
                try:
                    bridge = self.nodes.bridge
                    ctx = self.nodes.ctx
                    reader = getattr(ctx, self.reader_name)
                    payload = {"hands": {}}
                    for hand in self.nodes.hands:
                        data = bridge.maybe_await(reader(hand["slave_id"]), timeout=1.0)
                        payload["hands"][hand["side"]] = jsonable(data)
                    msg = String()
                    msg.data = json.dumps(payload, ensure_ascii=False)
                    self.nodes.touch_pub.publish(msg)
                except Exception as exc:
                    print(f"[revo2] touch poll failed: {exc}", flush=True)
            self._stop_event.wait(self.poll_interval_s)


class ModelPlugin:
    """Static spec sheet — resource tool `model`. No URDF/skeleton in v1."""

    def __init__(self, nodes: RevoNodes):
        self.nodes = nodes

    def get_tool(self) -> dict:
        return tool("model", "resource", "BrainCo Revo 2 静态规格 (自由度/尺寸/协议/变体表)")

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        from pathlib import Path

        info_path = Path(__file__).parent / "resource" / "revo2_info.json"
        info = json.loads(info_path.read_text(encoding="utf-8"))
        info["configured_variant"] = self.nodes.variant
        info["configured_hands"] = self.nodes.hands
        return info


def build_plugins(config, namespace, ros2):
    nodes = RevoNodes(config, namespace, ros2)
    plugins_cfg = config.get("plugins", {})
    plugins = [ModelPlugin(nodes)]
    if plugins_cfg.get("hand", {}).get("enabled", True):
        plugins.append(HandPlugin(nodes, plugins_cfg.get("hand", {})))
    if plugins_cfg.get("hand_state", {}).get("enabled", True):
        plugins.append(HandStatePlugin(nodes, plugins_cfg.get("hand_state", {})))
    if nodes.variant == "touch" and plugins_cfg.get("hand_touch", {}).get("enabled", True):
        plugins.append(HandTouchPlugin(nodes, plugins_cfg.get("hand_touch", {})))
    return plugins
