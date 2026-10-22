#!/usr/bin/env python3
"""noetix/bumi/estop.py — Bumi 软件急停锁存与 `estop` 卡片。

Bumi 的厂商 SDK 没有暴露硬件急停状态（本仓库里没有任何 estop 读取），天轶的
`estop` 卡读的是 `/power/board/key_status`，这里读不到。因此这张卡做的是**驱动级
软件锁存**，不是硬件急停的替代品：

    trigger → 置位 → 立刻下发零速、暂停 loco_servo、终止 loco.move 线程
    锁存期间 → 任何非零速度在 LocoPlugin._publish_cmd 被丢弃
    reset   → 释放（必须 confirm=true），运动恢复

两条边界要写在文档里，不要含糊：

* 机器人身上的物理急停按钮仍是最终手段，这张卡管不到它。
* 锁存写入 `/opt/phanthy-motus/data/bumi/estop_state.json`，容器重启后仍然
  保持锁存；只有 `reset(confirm=true)` 才解除。状态文件损坏时按锁存处理
  （fail-closed），不会因为读不出来就放开运动。

`loco` 与 `loco_servo` 的所有速度最终都经过 `LocoPlugin._publish_cmd`
（`loco_servo` 经 `loco.publish_velocity` 转发），所以闸门放在那里就能同时
管住调用式与流式两条路径。
"""

from __future__ import annotations

import json
import os
import threading
import time
from contextlib import contextmanager
from pathlib import Path

CARD_ESTOP = "estop"
_TOPIC_ESTOP = "/{ns}/state/estop"
_PROTECTION_MODE = 26
_PROTECTION_CHECK_INTERVAL_S = 1.0


def _now_ms() -> int:
    return int(time.time() * 1000)


class EmergencyStopLatch:
    """Process-wide, thread-safe latch. Trigger wins until an explicit reset.

    One instance is created by the bundle and shared by `loco`, `loco_servo`
    and the `estop` card, so there is no second copy of the state to disagree
    with.
    """

    def __init__(self, state_path: str | None = None):
        # Re-entrant: a publisher holds the gate across `refuses()` (which takes
        # it again) and the SDK write. `trigger` takes the same lock, so the
        # latch transition cannot land between a publisher's check and its
        # write; either the command completes before the latch, or it is
        # dropped.
        self._lock = threading.RLock()
        self._latched = False
        self._source = ""
        self._reason = ""
        self._since_ms = None
        self._blocked = 0
        self._on_latch = []
        self._state_path = Path(state_path) if state_path else None
        self._restore()

    # ── restart survival ─────────────────────────────────────────────────────
    #
    # `deploy/service.yml` restarts this container after a crash. A latch that
    # lives only in memory would release an E-stop nobody reset, so the state
    # is written beside the other Bumi runtime data and read back at startup.
    # The file is treated as fail-closed: if it exists but cannot be parsed,
    # the robot comes back latched rather than free to move.

    def _restore(self) -> None:
        if self._state_path is None:
            return
        try:
            raw = self._state_path.read_text()
        except FileNotFoundError:
            return
        except OSError as exc:
            print(f"[estop] could not read {self._state_path}: {exc}; "
                  "starting latched", flush=True)
            self._set_restored("estop_state_unreadable", str(exc))
            return
        try:
            data = json.loads(raw)
        except ValueError as exc:
            print(f"[estop] {self._state_path} is not valid JSON: {exc}; "
                  "starting latched", flush=True)
            self._set_restored("estop_state_unreadable", str(exc))
            return
        if not isinstance(data, dict) or not data.get("latched"):
            return
        since = data.get("since_ms")
        self._set_restored(
            str(data.get("source") or "restored"),
            str(data.get("reason") or ""),
            int(since) if isinstance(since, (int, float)) else None,
        )

    def _set_restored(self, source: str, reason: str, since_ms: int | None = None) -> None:
        self._latched = True
        self._source = source
        self._reason = reason
        self._since_ms = since_ms if since_ms is not None else _now_ms()

    def _persist_locked(self) -> None:
        """Write the latch to disk. Called with `_lock` held."""
        if self._state_path is None:
            return
        payload = {
            "latched": self._latched,
            "source": self._source,
            "reason": self._reason,
            "since_ms": self._since_ms,
        }
        try:
            self._state_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._state_path.with_name(self._state_path.name + ".tmp")
            temporary.write_text(json.dumps(payload))
            os.replace(temporary, self._state_path)
        except OSError as exc:
            # Not fatal for the running process, but it must be loud: the next
            # restart would come back unlatched.
            print(f"[estop] could not persist latch state to {self._state_path}: "
                  f"{exc}", flush=True)

    @property
    def latched(self) -> bool:
        with self._lock:
            return self._latched

    def add_on_latch(self, callback) -> None:
        """Register a callback invoked outside the lock on every new latch."""
        with self._lock:
            self._on_latch.append(callback)

    def trigger(self, source: str, reason: str = "") -> bool:
        """Set the latch. Returns True only on the idle -> latched transition.

        Callbacks run after the state is visible, so a callback that publishes
        a zero command cannot race a command that still sees "not latched".
        """
        with self._lock:
            if self._latched:
                return False
            self._latched = True
            self._source = str(source or "unknown")
            self._reason = str(reason or "")
            self._since_ms = _now_ms()
            self._persist_locked()
            callbacks = list(self._on_latch)
        for callback in callbacks:
            try:
                callback(self._source, self._reason)
            except Exception as exc:                          # noqa: BLE001
                print(f"[estop] on-latch callback failed: {exc}", flush=True)
        return True

    def reset(self) -> bool:
        """Release the latch. Returns True only on the latched -> idle edge."""
        with self._lock:
            if not self._latched:
                return False
            self._latched = False
            self._source = ""
            self._reason = ""
            self._since_ms = None
            self._persist_locked()
            return True

    @contextmanager
    def motion_gate(self):
        """Hold the latch state still across a check-and-publish pair.

        A publisher that only checked `latched` and then released the lock
        could be overtaken by `trigger` before it reached the SDK — an E-stop
        that still let one non-zero command through. Publishers therefore wrap
        both steps in this gate, and `trigger` takes the same lock.
        """
        with self._lock:
            yield

    def note_blocked(self) -> None:
        with self._lock:
            self._blocked += 1

    def refuses(self, x: float, y: float, z: float) -> bool:
        """Whether this non-zero velocity must be dropped while latched.

        Zero is always allowed: a stop command is not the thing an emergency
        stop is supposed to prevent, and `loco_servo`'s hold path publishes
        zeros at 50 Hz. Non-zero is dropped and counted.
        """
        if not self.latched:
            return False
        if abs(x) <= 1e-9 and abs(y) <= 1e-9 and abs(z) <= 1e-9:
            return False
        self.note_blocked()
        return True

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "latched": self._latched,
                "source": self._source,
                "reason": self._reason,
                "since_ms": self._since_ms,
                "blocked_commands": self._blocked,
            }


class EstopPlugin:
    """`estop` card: latch control, state publication and an optional auto-latch.

    `auto_trigger_on_protection` defaults to off. Turning it on makes workmode
    26 (protection) latch the driver, and a latched driver also refuses
    `stand_up` — which is the documented way out of protection. That is a
    deliberate operator decision, not something to enable by default.
    """

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 latch: EmergencyStopLatch, high_ctrl=None):
        config = plugin_config or {}
        self._latch = latch
        self._high_ctrl = high_ctrl
        self._ns = namespace
        self._topic = _TOPIC_ESTOP.format(ns=namespace)
        self._publish_hz = float(config.get("publish_hz", 10))
        if not 0 < self._publish_hz <= 50:
            raise ValueError("estop.publish_hz must be in (0, 50]")
        self._auto_protection = bool(
            config.get("auto_trigger_on_protection", False))
        self._last_protection_check = 0.0
        self._stopped = False
        self._node = None
        self._pub = None

        if executor is not None:
            try:
                from rclpy.node import Node
                from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                                       HistoryPolicy, DurabilityPolicy)
                from std_msgs.msg import String

                self._String = String
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    durability=DurabilityPolicy.VOLATILE,
                )
                self._node = Node("bumi_estop")
                self._pub = self._node.create_publisher(String, self._topic, qos)
                self._node.create_timer(1.0 / self._publish_hz, self._tick)
                executor.add_node(self._node)
                self._node.get_logger().info(
                    f"bumi estop → {self._topic} @ {self._publish_hz:g}Hz")
            except Exception as exc:                          # noqa: BLE001
                print(f"[estop] ROS2 发布不可用，仅 MCP 轮询: {exc}", flush=True)
                self._node = None
                self._pub = None

    # ── tool schema ──────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": CARD_ESTOP,
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Bumi 软件急停（锁存）：trigger 立即停住并拒绝所有速度指令，"
                "直到 reset（需 confirm）。不读取物理急停状态；锁存跨驱动重启保持，"
                "机器人上的物理急停按钮仍是最终手段。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["trigger", "reset", "info"],
                        "description": "trigger=急停并锁存；reset=解除锁存；info=查询状态",
                    },
                    "confirm": {
                        "type": "boolean",
                        "description": "reset 必须显式传 true，否则拒绝",
                    },
                    "reason": {
                        "type": "string",
                        "description": "记录到状态里的触发原因（可选）",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "trigger": {"params": ["reason"],
                                "description": "立即停住底盘并锁存，后续运动指令一律拒绝"},
                    "reset": {"params": ["confirm"],
                              "description": "解除锁存、恢复运动；必须传 confirm=true"},
                    "info": {"params": [], "description": "返回锁存状态与拦截计数"},
                },
            },
            "topic_out": ([{"topic": self._topic, "format": "data/json"}]
                          if self._node is not None else []),
        }

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> dict:
        self._stopped = False
        # Actuator lifecycle per the driver contract: `ready`, not `running`.
        # `running` is what a sensor card reports once it is publishing.
        return {"state": "ready"}

    def stop(self) -> None:
        # Stopping the card must not release the latch: a silent release when a
        # project is stopped is exactly the surprise an emergency stop exists
        # to prevent.
        self._stopped = True

    # ── state publication ────────────────────────────────────────────────────

    def _snapshot(self) -> dict:
        state = self._latch.snapshot()
        state.update({
            "topic": self._topic,
            "publish_hz": self._publish_hz,
            "auto_trigger_on_protection": self._auto_protection,
            "timestamp_ms": _now_ms(),
        })
        return state

    def _publish_now(self) -> None:
        if self._pub is None:
            return
        msg = self._String()
        msg.data = json.dumps(self._snapshot())
        self._pub.publish(msg)

    def _tick(self) -> None:
        if self._stopped:
            return
        self._check_protection_mode()
        self._publish_now()

    def _check_protection_mode(self) -> None:
        if not self._auto_protection or self._high_ctrl is None:
            return
        now = time.monotonic()
        if now - self._last_protection_check < _PROTECTION_CHECK_INTERVAL_S:
            return
        self._last_protection_check = now
        try:
            mode = int(self._high_ctrl.get_mode())
        except Exception:                                     # noqa: BLE001
            return
        if mode == _PROTECTION_MODE:
            self._latch.trigger("protection_mode", f"workmode={mode}")

    # ── dispatch ─────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict | None) -> dict | None:
        args = args or {}
        if action == "start":
            return self.start()
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD_ESTOP):
            return {
                "state": "running",
                "estop": self._snapshot(),
                "topic_out": ([{"topic": self._topic, "format": "data/json"}]
                              if self._node is not None else []),
            }
        if action == "trigger":
            newly = self._latch.trigger("estop_card", args.get("reason", ""))
            self._publish_now()
            return {"state": "latched", "newly_latched": newly,
                    "estop": self._snapshot()}
        if action == "reset":
            if args.get("confirm") is not True:
                return {
                    "state": "error",
                    "code": "CONFIRM_REQUIRED",
                    "error": "reset 会恢复运动能力，必须显式传 confirm=true",
                    "estop": self._snapshot(),
                }
            released = self._latch.reset()
            self._publish_now()
            return {"state": "released", "released": released,
                    "estop": self._snapshot()}
        return None
