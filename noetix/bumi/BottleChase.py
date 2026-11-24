#!/usr/bin/env python3
"""Visual approach card for the Bumi "grab the stool" game.

Consumes the nearest bottle target published by ``bottle_detect`` and walks
towards it slightly faster than normal walking, then slows down inside 1.2 m
and stops about 0.55 m away.  The card is one motion primitive: approach and
stop.  It does not detect bottles and does not speak; those are the neighbouring
cards' jobs.

Steering uses the target bearing (front 0, left positive) and the chassis
``vyaw`` convention (positive left).  The card reserves the chassis through
LocoPlugin's ``_body_action`` marker, preempts the streaming ``loco_servo``
velocity and publishes through LocoPlugin's shared rate limiter.
"""

from __future__ import annotations

import json
import math
import os
import ssl
import threading
import time
import urllib.request
import uuid

MAX_ACTION_S = 90.0
_MIN_VX = 0.45              # 0.25 m/s, the chassis forward stiction floor
_MAX_VYAW = 0.50
_MIN_TURN_VYAW = 0.40       # above the ~0.40 yaw stiction boundary
_TURN_IN_PLACE_BEARING_DEG = 55.0
_SMALL_BEARING_DEG = 6.0
_TARGET_STALE_S = 0.8
_MODE_CHECK_S = 0.5
_CONTROL_PERIOD_S = 0.05


class ChaseInputError(ValueError):
    """The chase request is invalid or unsafe to start."""


def clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def parse_target(payload) -> dict | None:
    """Extract a usable target from a ``bottle_detect`` JSON payload."""
    if not isinstance(payload, dict) or payload.get("state") != "fresh":
        return None
    bottle = payload.get("bottle")
    if not isinstance(bottle, dict):
        return None
    try:
        distance = float(bottle["distance_m"])
        bearing = float(bottle["bearing_deg"])
    except (KeyError, TypeError, ValueError):
        return None
    if not math.isfinite(distance) or not math.isfinite(bearing):
        return None
    if not 0.20 <= distance <= 6.0 or not -180.0 <= bearing <= 180.0:
        return None
    return {
        "distance_m": distance,
        "bearing_deg": bearing,
        "bbox": bottle.get("bbox"),
        "confidence": bottle.get("confidence"),
    }


def speed_for_distance(distance_m: float, stop_m: float, slow_m: float,
                       cruise: float) -> float:
    """Cruise, then taper inside the slow zone, with a stiction floor."""
    if distance_m <= stop_m:
        return 0.0
    if distance_m >= slow_m:
        return cruise
    span = max(1e-6, slow_m - stop_m)
    tapered = cruise * (distance_m - stop_m) / span
    return max(_MIN_VX, tapered)


def steer_for_bearing(bearing_deg: float) -> float:
    """Normalised vyaw for a target bearing; zero inside the dead band."""
    if abs(bearing_deg) < _SMALL_BEARING_DEG:
        return 0.0
    magnitude = clamp(abs(bearing_deg) * 0.03, _MIN_TURN_VYAW, _MAX_VYAW)
    return math.copysign(magnitude, bearing_deg)


def forward_for_bearing(speed: float, bearing_deg: float) -> float:
    """Forward speed, stopping forward motion for a very large bearing."""
    if abs(bearing_deg) > _TURN_IN_PLACE_BEARING_DEG:
        return 0.0
    factor = max(0.4, math.cos(math.radians(bearing_deg)))
    return max(_MIN_VX, speed * factor)


class BottleChasePlugin:
    """MCP card: ``bottle_chase`` — walk to the nearest detected bottle."""

    PREFIX = "bottle_chase"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 high_ctrl, loco_plugin=None):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._high_ctrl = high_ctrl
        self._loco = loco_plugin
        self._target_topic = f"/{namespace}/bottle/target"
        self._event_topic = f"/{namespace}/bottle/event"
        try:
            self._cruise = clamp(float(plugin_config.get("cruise_speed", 0.72)),
                                 0.45, 0.85)
        except (TypeError, ValueError):
            self._cruise = 0.72
        try:
            self._stop_distance = clamp(
                float(plugin_config.get("stop_distance_m", 0.55)), 0.35, 1.2)
        except (TypeError, ValueError):
            self._stop_distance = 0.55
        try:
            self._slow_distance = clamp(
                float(plugin_config.get("slow_distance_m", 1.2)), 0.6, 3.0)
        except (TypeError, ValueError):
            self._slow_distance = 1.2
        if self._slow_distance <= self._stop_distance + 0.1:
            self._slow_distance = self._stop_distance + 0.1
        self._target = None
        self._target_at = 0.0
        self._lock = threading.RLock()
        self._active = None
        self._thread = None
        self._last = None
        self._cmd = None
        self._sub = None

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)
        from std_msgs.msg import String

        self._String = String
        self._qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=10,
                               durability=DurabilityPolicy.VOLATILE)
        self._node = Node("bumi_bottle_chase")
        executor.add_node(self._node)
        self._event_pub = self._node.create_publisher(
            String, self._event_topic, self._qos)
        self._subscribe(self._target_topic)
        print(f"[bottle_chase] target←{self._target_topic} "
              f"event→{self._event_topic}", flush=True)

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "bottle_chase",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "朝 bottle_detect 发布的最近矿泉水瓶走去：速度比普通行走略快，"
                "进入 1.2m 后减速，在离瓶约 0.55m 处停车。只做靠近/停车这一个动作，"
                "不检测瓶子、不播报。要求机器人稳定站立、workmode=2、地面平坦防滑、"
                "前方至少 3m 无人无障碍；执行中不要靠近或推拉机器人。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["chase", "stop", "info", "start"],
                        "description": ("chase=开始追最近的水瓶；stop=立即停车；"
                                        "info=查看目标与状态；start=建立目标输入。"),
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "chase": {"params": [],
                              "description": "使用已连接的最近水瓶目标靠近并停车。"},
                    "stop": {"params": [], "description": "立即停车并发送零速度。"},
                    "info": {"params": [], "description": "返回目标新鲜度和最近结果。"},
                    "start": {"params": [],
                              "description": "订阅连接的目标话题，不移动。"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "stop"}},
                "x-completion": {"actions": ["chase"], "timeout": 100},
            },
            "topic_in": [{"format": "data/json",
                          "message_type": "std_msgs/msg/String"}],
            "topic_out": [{"topic": self._event_topic, "format": "data/json"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self, args: dict | None = None):
        args = args or {}
        topic = str(args.get("input_topic") or "").strip()
        self._subscribe(topic or self._target_topic)
        return {"state": "ready", "topic_in": [{
            "topic": self._subscription_topic,
            "format": "data/json",
            "message_type": "std_msgs/msg/String",
        }]}

    def stop(self):
        with self._lock:
            record = self._active
            thread = self._thread
        if record is None:
            return {"state": "idle", "stopped": False}
        record["stop_event"].set()
        if (thread is not None and thread.is_alive()
                and thread is not threading.current_thread()):
            thread.join(timeout=5)
        with self._lock:
            still_running = self._active is record
        if still_running:
            try:
                self._publish(0.0, 0.0)
            except Exception:
                pass
        return {"state": "idle", "stopped": True}

    def dispatch(self, action: str, args: dict):
        args = args or {}
        if action == "start":
            return self.start(args)
        if action == "chase":
            return self._start_chase(args)
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        return None

    # ── Targets ──────────────────────────────────────────────────────────────

    def _subscribe(self, topic: str) -> None:
        if not topic:
            return
        if self._sub is not None and topic == self._subscription_topic:
            return
        if self._sub is not None:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        self._sub = self._node.create_subscription(
            self._String, topic, self._on_target, self._qos)
        self._subscription_topic = topic

    def _on_target(self, msg) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        target = parse_target(payload)
        with self._lock:
            if target is None:
                self._target = None
                self._target_at = 0.0
            else:
                self._target = target
                self._target_at = time.monotonic()

    def _fresh_target(self) -> dict | None:
        with self._lock:
            target = dict(self._target) if self._target else None
            age = time.monotonic() - self._target_at if target else None
        if target is None or age is None or age > _TARGET_STALE_S:
            return None
        target["age_s"] = round(age, 2)
        return target

    # ── Start / status ───────────────────────────────────────────────────────

    def _start_chase(self, args: dict) -> dict:
        try:
            self._preflight()
        except ChaseInputError as exc:
            return {"state": "error", "error": str(exc)}

        with self._lock:
            if self._active is not None:
                return {"state": "error",
                        "error": "机器人正在追瓶，请等待或先 stop"}
            target = self._fresh_target()
            record = {
                "action_id": f"bumi_bottle_chase_{uuid.uuid4().hex}",
                "started_at": time.time(),
                "stop_event": threading.Event(),
                "start_distance_m": target["distance_m"] if target else None,
            }
            thread = threading.Thread(target=self._run_chase, args=(record,),
                                      daemon=True, name="bumi_bottle_chase")
            self._active = record
            self._thread = thread
            thread.start()
        return {
            "state": "running",
            "action_id": record["action_id"],
            "target": target,
            "cruise_speed": self._cruise,
            "stop_distance_m": self._stop_distance,
            "expected_duration_s": round(
                (target["distance_m"] if target else 2.0) / 0.40 + 8.0, 1),
        }

    def _info(self) -> dict:
        with self._lock:
            target = dict(self._target) if self._target else None
            age = (round(time.monotonic() - self._target_at, 2)
                   if target else None)
            active = None if self._active is None else {
                "action_id": self._active["action_id"],
                "started_at": self._active["started_at"],
            }
            last = dict(self._last) if self._last else None
        mode = None
        if self._high_ctrl is not None:
            try:
                mode = int(self._high_ctrl.get_mode())
            except Exception:
                mode = None
        return {
            "state": "running" if active else "idle",
            "workmode": mode,
            "target_topic": self._subscription_topic,
            "target": target,
            "target_age_s": age,
            "cruise_speed": self._cruise,
            "stop_distance_m": self._stop_distance,
            "active": active,
            "last": last,
        }

    # ── Worker ───────────────────────────────────────────────────────────────

    def _run_chase(self, record: dict) -> None:
        outcome = {"state": "error", "error": "chase did not start"}
        status = "error"
        reserved = False
        owns_chassis = False
        try:
            reserved = self._reserve_loco()
            self._prepare(reserved)
            owns_chassis = True
            outcome = self._drive(record)
            status = {"done": "completed", "stopped": "cancelled"}.get(
                outcome["state"], "error")
        except Exception as exc:
            outcome = {"state": "error",
                       "error": f"{type(exc).__name__}: {exc}"}
        finally:
            if reserved or owns_chassis:
                for _ in range(3):
                    try:
                        self._publish(0.0, 0.0)
                        break
                    except Exception:
                        time.sleep(0.05)
            if owns_chassis:
                self._release_chassis()
            if reserved:
                self._release_loco()
            result = {
                **outcome,
                "action": "bottle_chase",
                "stop_distance_m": self._stop_distance,
                "duration_s": round(time.time() - record["started_at"], 1),
            }
            with self._lock:
                if self._active is record:
                    self._active = None
                    self._thread = None
                self._last = result
            self._notify_completion(record["action_id"], status, result)

    def _drive(self, record: dict) -> dict:
        deadline = time.monotonic() + MAX_ACTION_S
        started = time.monotonic()
        next_mode_check = started + _MODE_CHECK_S
        filtered_distance = None
        filtered_bearing = None

        while True:
            if record["stop_event"].is_set():
                return {"state": "stopped"}
            now = time.monotonic()
            if now > deadline:
                raise RuntimeError("追瓶超时，已停车；请检查目标是否仍在视野内")
            target = self._fresh_target()
            if target is None:
                raise RuntimeError("丢失水瓶目标，已停车")
            if now >= next_mode_check:
                self._require_walking()
                next_mode_check = now + _MODE_CHECK_S

            distance = target["distance_m"]
            bearing = target["bearing_deg"]
            if filtered_distance is None:
                filtered_distance, filtered_bearing = distance, bearing
            else:
                filtered_distance = 0.6 * filtered_distance + 0.4 * distance
                filtered_bearing = 0.6 * filtered_bearing + 0.4 * bearing

            if filtered_distance <= self._stop_distance:
                self._publish(0.0, 0.0)
                self._publish_arrived(filtered_distance, filtered_bearing,
                                      record["action_id"])
                return {
                    "state": "done",
                    "distance_m": round(filtered_distance, 2),
                    "bearing_deg": round(filtered_bearing, 1),
                }

            speed = speed_for_distance(
                filtered_distance, self._stop_distance, self._slow_distance,
                self._cruise)
            vyaw = steer_for_bearing(filtered_bearing)
            vx = forward_for_bearing(speed, filtered_bearing)
            self._publish(vx, vyaw)
            time.sleep(_CONTROL_PERIOD_S)

    def _publish_arrived(self, distance: float, bearing: float,
                         action_id: str) -> None:
        payload = {
            "state": "arrived",
            "event": "claim_stool",
            "distance_m": round(distance, 2),
            "bearing_deg": round(bearing, 1),
            "action_id": action_id,
            "timestamp_ms": int(time.time() * 1000),
        }
        for _ in range(3):
            msg = self._String()
            msg.data = json.dumps(payload)
            self._event_pub.publish(msg)
            time.sleep(0.1)

    # ── Chassis arbitration ──────────────────────────────────────────────────

    def _prepare(self, reserved: bool = False) -> None:
        self._require_walking()
        if self._loco is not None:
            if not reserved and self._loco.is_moving():
                raise ChaseInputError("机器人正在执行其他移动动作，请先停止")
            self._loco._preempt_servo("bottle_chase.chase")
            self._loco._set_servo_velocity_owner(True)

    def _reserve_loco(self) -> bool:
        if self._loco is None:
            return False
        with self._loco._rotate_lock:
            if self._loco._body_action:
                raise ChaseInputError(
                    f"底盘已被 {self._loco._body_action} 占用，请稍后再试")
            if self._loco._rotate_state["state"] == "running":
                raise ChaseInputError("机器人正在执行目标移动或旋转，请先停止")
            self._loco._body_action = "bottle_chase.chase"
        return True

    def _release_loco(self) -> None:
        if self._loco is None:
            return
        with self._loco._rotate_lock:
            if self._loco._body_action == "bottle_chase.chase":
                self._loco._body_action = None

    def _release_chassis(self) -> None:
        if self._loco is not None:
            try:
                self._loco._set_servo_velocity_owner(False)
            except Exception as exc:
                print(f"[bottle_chase] release owner failed: {exc}", flush=True)

    def _require_walking(self) -> None:
        if self._high_ctrl is None:
            raise ChaseInputError("HighController 未初始化，无法追瓶")
        mode = int(self._high_ctrl.get_mode())
        if mode == 26:
            raise ChaseInputError("机器人处于保护模式，无法追瓶；请先重启机器人")
        if mode != 2:
            raise ChaseInputError(
                f"追瓶需要机器人处于行走模式（workmode=2），当前为 {mode}。"
                "请先让机器人稳定站立并进入行走模式。")

    def _preflight(self) -> None:
        self._require_walking()
        if self._loco is not None and self._loco.is_moving():
            raise ChaseInputError("机器人正在执行其他移动动作，请先停止")
        if self._fresh_target() is None:
            raise ChaseInputError(
                "没有新鲜的水瓶目标：请先启动 bottle_detect 并连接其输出，"
                "确认目标在相机视野内")

    def _publish(self, vx: float, vyaw: float) -> None:
        if self._loco is not None:
            self._loco.publish_velocity(float(vx), 0.0, float(vyaw))
            return
        if self._high_ctrl is None:
            raise RuntimeError("HighController 未初始化")
        self._high_ctrl.publish_cmd(float(vx), 0.0, float(vyaw),
                                    self._default_cmd(), 0)

    def _default_cmd(self):
        if self._cmd is None:
            from highcontrol_py import ControlCmd
            self._cmd = ControlCmd.DEFAULT
        return self._cmd

    @staticmethod
    def _notify_completion(action_id: str, status: str, result: dict) -> None:
        try:
            url = os.environ.get("AGENT_CORE_URL",
                                 "https://localhost:15678").rstrip("/")
            payload = json.dumps({
                "action_id": action_id,
                "status": status,
                "result": result,
                "tool": "bottle_chase",
                "ts": time.time(),
            }).encode()
            request = urllib.request.Request(
                f"{url}/api/acp/complete", data=payload,
                headers={"Content-Type": "application/json"}, method="POST")
            context = (ssl._create_unverified_context()
                       if url.startswith("https://") else None)
            with urllib.request.urlopen(request, timeout=5,
                                        context=context) as response:
                response.read()
        except Exception as exc:
            print(f"[bottle_chase] ACP callback failed for {action_id}: {exc}",
                  flush=True)
