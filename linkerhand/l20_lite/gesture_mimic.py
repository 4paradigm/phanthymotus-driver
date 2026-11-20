"""Run calibrated L20 Lite poses from the Pose card's hand gesture events."""
from __future__ import annotations

import json
import logging
import math
import re
import threading
import time
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from gesture_device import CONFIRM_MOTION_DESCRIPTION, GesturePlugin
from modbus_client import JOINT_COUNT
from modbus_device import HandPlugin, SPEED_RATIOS


LOG = logging.getLogger(__name__)
ACTIONS = {
    "open_palm": ("open", None), "fist": ("close", None),
    "pointing": ("number", 1), "victory": ("v_sign_gesture", None),
    "ok": ("ok_gesture", None), "thumbs_up": ("thumb_up_gesture", None),
}
SIDES = ("left", "right")
DEFAULT_SPEED_RATIO = "2.0x"
TOPIC = re.compile(r"(?:/[A-Za-z_][A-Za-z_0-9]*)+/poses/gesture")
STREAM_TIMEOUT = 1.0
OPERATOR_GRACE = 3.0
ACQUIRE_WINDOW = 0.05
EVENT_MAX_AGE = 1.0
CANCEL_TIMEOUT = 2.0


class MimicMotion:
    """Use the manual cards' calibrated paths with a dedicated cancellation signal."""

    def __init__(self, hand, gesture_config):
        self.hand = hand
        self.gestures = GesturePlugin(hand, gesture_config)
        self.cancel = self.gestures._stop
        self.controller = None
        self.port = None
        self.side = None
        self.speed_ratio = DEFAULT_SPEED_RATIO

    def claim(self, side, speed_ratio):
        if not self.hand._bus_lock.acquire(timeout=0.25):
            raise RuntimeError("HAND_BUSY")
        try:
            if self.hand._mimic_side is not None:
                raise RuntimeError("HAND_BUSY")
            if side not in self.gestures._calibration:
                raise RuntimeError("CALIBRATION_MISSING")
            self.hand._mimic_side = side
            self.side = side
            self.speed_ratio = speed_ratio
        finally:
            self.hand._bus_lock.release()

    def move(self, label):
        action, number = ACTIONS[label]
        with self.hand._bus_lock:
            self.gestures._check_stop()
            port = self.hand._discover().get(self.side)
            if not port or (self.port is not None and self.port != port):
                raise RuntimeError("HAND_UNAVAILABLE")
            if self.controller is None:
                connection = self.hand._connection(side=self.side, serial_port=port)
                self.controller = HandPlugin(
                    connection, {**self.hand.hand_config, "auto_detect_gestures": False},
                    motion_lock=self.hand._bus_lock,
                )
                self.controller._stop_requested = self.cancel
                self.port = port
            controller = self.controller
            if self.side not in controller.poses_by_side:
                raise RuntimeError("CALIBRATION_MISSING")
            client = controller.connection.get(self.side)
            client.probe_positions(self.side)
            self.gestures._check_stop()
            if action in ("open", "close"):
                speed, torque = controller._motion_options(action, {"speed_ratio": self.speed_ratio})
                result = controller._move(controller.poses_by_side[self.side][action], action,
                                          speed=speed, torque=torque, interruptible_midpoint=True)
                if result["state"] == "released_timeout":
                    raise TimeoutError("target position not reached")
                return
            poses = self.gestures._calibration[self.side]
            base_speed = self.gestures.number_speed if number is not None else self.gestures.speed
            speed = self.gestures._speed(self.speed_ratio, base_speed)
            target = poses["number"][number] if number is not None else poses[action]
            try:
                client.set_speeds(self.gestures._joint_speeds(speed))
                self.gestures._check_stop()
                client.set_torques([self.gestures.torque] * JOINT_COUNT)
                self.gestures._pose(client, target, speed, poses)
            except Exception:
                client.set_torques([0] * JOINT_COUNT)
                raise

    def release(self):
        with self.hand._bus_lock:
            if self.controller is not None:
                connection = self.controller.connection
                try:
                    client = connection.get(self.side)
                    client.probe_positions(self.side)
                    client.set_torques([0] * JOINT_COUNT)
                except (OSError, TimeoutError, ValueError):
                    # Linux may assign a new tty after unplug/replug. Keep the
                    # motion reservation until the uniquely detected bound hand
                    # acknowledges a release on a fresh connection.
                    connection.invalidate()
                    port = self.hand._discover().get(self.side)
                    if port is None:
                        raise RuntimeError("未找到重连后的绑定灵巧手")
                    recovered = self.hand._connection(side=self.side, serial_port=port)
                    try:
                        client = recovered.get(self.side)
                        client.probe_positions(self.side)
                        client.set_torques([0] * JOINT_COUNT)
                    finally:
                        recovered.close()
                finally:
                    connection.close()
            self.controller = self.port = self.side = None
            self.hand._mimic_side = None


class GestureMimicPlugin:
    PREFIX = "gesture_mimic"

    def __init__(self, motion, namespace, ros2):
        self.motion = motion
        self.ros2 = ros2
        if not namespace or namespace[0].isdigit():
            namespace = f"driver_{namespace or 'l20_lite'}"
        self.output = f"/{namespace}/gesture_mimic/status"
        self.lock = threading.RLock()
        self.instance = None
        self.bound_side = None
        self.speed_ratio = DEFAULT_SPEED_RATIO
        self.node = self.publisher = None
        self.subscriptions = []
        self.input_topic = None
        self.armed = False
        self.armed_at = 0.0
        self.operator = None
        self.operator_seen = self.last_frame = 0.0
        self.frame_stamp = 0.0
        self.people = {}
        self.candidates = {}
        self.active_gestures = {}
        self.gesture_seen = {}
        self.previous_labels = {}
        self.stable_labels = {}
        self.waiting_for_handover = False
        self.last_events = {}
        self.acquisition_until = None
        self.pending = None
        self.active = None
        self.holding = None
        self.cancel_since = None
        self.release_pending = False
        self.fault = None
        self.status = {"state": "not_armed", "reason_code": "NOT_ARMED",
                       "message": "请确认台架安全并启用手势模仿"}

    def get_tool(self):
        schema = action_schema({
            "info": ([], "查看订阅、操作者与执行状态"),
            "enable": (["confirm_motion"], "确认台架安全后启用自动模仿"),
            "stop": ([], "取消动作并释放力矩"),
        }, {"confirm_motion": {"type": "boolean", "const": True,
                               "description": CONFIRM_MOTION_DESCRIPTION}})
        result = tool(self.PREFIX, "actuator", "监听 Pose 手势事件，复现六种已校准手型。",
                      schema, topic_in=[{"format": "data/json", "desc": "Pose gesture"}],
                      topic_out=[{"topic": self.output, "format": "data/json"}])
        result["title"] = "L20 Lite 手势模仿"
        result["multiInstance"] = True
        result["configSchema"] = {"type": "object", "properties": {
            "bound_side": {"type": "string", "enum": list(SIDES),
                           "scope": "instance", "title": "绑定灵巧手"},
            "speed_ratio": {"type": "string", "enum": list(SPEED_RATIOS), "default": DEFAULT_SPEED_RATIO,
                            "scope": "instance", "title": "关节速度倍率"},
        }}
        return result

    def _report(self, state, message, code=None, *, label=None, motion_id=None):
        if label is None:
            label = self.holding
        action = ACTIONS[label] if label else None
        self.status = {
            "timestamp": time.time(), "state": state, "reason_code": code, "message": message,
            "source_track": self.operator[0] if self.operator else None,
            "source_hand": self.operator[1] if self.operator else None,
            "gesture": label, "target_action": action[0] if action else None,
            "target_number": action[1] if action else None, "motion_id": motion_id,
        }
        if self.publisher is not None:
            from std_msgs.msg import String

            self.publisher.publish(String(data=json.dumps(self.status, ensure_ascii=False)))

    def _info(self):
        return {**self.status, "armed": self.armed, "bound_side": self.bound_side,
                "speed_ratio": self.speed_ratio,
                "topic_in": [{"topic": self.input_topic, "format": "data/json"}] if self.input_topic else [],
                "topic_out": [{"topic": self.output, "format": "data/json"}]}

    def start(self):
        if self.ros2 is None or self.ros2.ctx_core is None:
            return
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        node = Node("l20_lite_gesture_mimic", context=self.ros2.ctx_core)
        try:
            publisher = node.create_publisher(
                String, self.output, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            node.create_timer(0.025, self._tick)
            self.ros2.executor_core.add_node(node)
        except Exception:
            node.destroy_node()
            raise
        self.node, self.publisher = node, publisher

    def stop(self):
        with self.lock:
            self._disarm()
            active = self.active
        if active is not None:
            active["thread"].join(timeout=CANCEL_TIMEOUT)
        with self.lock:
            if self.active is None and self.release_pending:
                self._release()
            if self.node is not None:
                self.ros2.executor_core.remove_node(self.node)
                self.node.destroy_node()
                self.node = self.publisher = None

    def _subscribe(self, topic):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        if self.node is None:
            raise RuntimeError("Pose 订阅尚未初始化")
        if self.input_topic == topic:
            return
        subscriptions = []
        try:
            subscriptions.append(self.node.create_subscription(
                String, topic, lambda msg: self._receive(topic, self._event, msg.data),
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)))
            subscriptions.append(self.node.create_subscription(
                String, topic.rsplit("/", 1)[0],
                lambda msg: self._receive(topic, self._frame, msg.data),
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)))
        except Exception:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
            raise
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions, self.input_topic = subscriptions, topic
        self.last_frame = self.frame_stamp = 0.0
        self.people.clear()

    def _receive(self, topic, handler, raw):
        with self.lock:
            if topic != self.input_topic:
                return
            try:
                handler(json.loads(raw))
            except (ValueError, TypeError, KeyError) as exc:
                LOG.debug("Ignored invalid Pose input: %s", exc)
                if self.armed and self.active is None and not self.fault:
                    self._report(self.status["state"], "已忽略无效的 Pose 消息", "INVALID_EVENT")

    @staticmethod
    def _fresh(stamp):
        return (type(stamp) in (int, float) and math.isfinite(stamp)
                and -0.25 <= time.time() - stamp <= EVENT_MAX_AGE)

    def _frame(self, data):
        if not isinstance(data, dict) or not isinstance(data.get("persons"), list):
            raise ValueError("expected Pose persons array")
        stamp = data.get("timestamp")
        if not self._fresh(stamp) or stamp <= self.frame_stamp:
            return
        self.people = {person["id"]: person for person in data["persons"]
                       if isinstance(person, dict) and type(person.get("id")) is int}
        self.frame_stamp, self.last_frame = stamp, time.monotonic()
        if self.operator:
            track, side = self.operator
            person = self.people.get(track, {})
            seen = person.get("hands_seen")
            if seen is None:
                # Older Pose messages may only include the named hands.
                seen = person.get("hand_gestures") or {}
            if isinstance(seen, (list, dict)) and side in seen:
                self.operator_seen = self.last_frame
        labels = {}
        for track, person in self.people.items():
            hands = person.get("hand_gestures")
            if isinstance(hands, dict):
                labels.update({(track, side): label for side, label in hands.items()
                               if side in SIDES and isinstance(label, str) and label in ACTIONS})
        self.stable_labels = {key: label for key, label in labels.items()
                              if self.previous_labels.get(key) == label}
        self.previous_labels = labels
        for key, gesture in list(self.active_gestures.items()):
            if labels.get(key) == gesture["gesture"]:
                self.gesture_seen[key] = self.last_frame
            elif self.last_frame - self.gesture_seen[key] >= OPERATOR_GRACE:
                del self.active_gestures[key]
                del self.gesture_seen[key]
        if self.pending and self.frame_stamp >= self.pending["t"] and not self._visible(self.pending):
            self.pending = None

    def _event(self, data):
        if not isinstance(data, dict):
            raise ValueError("expected Pose event object")
        kind, label = data.get("event"), data.get("gesture")
        if (kind not in ("gesture_start", "gesture_end") or not isinstance(label, str)
                or label not in ACTIONS or data.get("hand") not in SIDES):
            return
        if type(data.get("track")) is not int:
            raise ValueError("invalid track or hand")
        if not self.armed:
            return
        if not self._fresh(data.get("t")) or data["t"] < self.armed_at - 0.001:
            if self.active is None:
                self._report(self.status["state"], "已忽略过期手势事件", "STALE_EVENT")
            return
        key = (data["track"], data["hand"])
        previous = self.last_events.get(key)
        if previous and (data["t"] < previous[0] or (data["t"], kind, label) == previous):
            return
        self.last_events[key] = (data["t"], kind, label)
        if kind == "gesture_end":
            active_gesture = self.active_gestures.get(key)
            if active_gesture and active_gesture["gesture"] == label:
                self.active_gestures.pop(key)
                self.gesture_seen.pop(key)
            candidate = self.candidates.get(key)
            if candidate and candidate["gesture"] == label:
                self.candidates.pop(key)
            if self.pending and key == self.operator and self.pending["gesture"] == label:
                self.pending = None
            return
        self.active_gestures[key] = data
        self.gesture_seen[key] = time.monotonic()
        if self.waiting_for_handover:
            return
        if self.operator is not None and self.operator != key:
            return
        if self.operator is None:
            self.candidates[key] = data
            if self.acquisition_until is None:
                delay = (ACQUIRE_WINDOW if len(self.people) > 1 or self.frame_stamp < data["t"]
                         else 0)
                self.acquisition_until = time.monotonic() + delay
        else:
            self._target(data)

    def _visible(self, data):
        person = self.people.get(data["track"], {})
        labels = person.get("hand_gestures")
        return isinstance(labels, dict) and labels.get(data["hand"]) == data["gesture"]

    def _target(self, data):
        label = data["gesture"]
        if self.active and self.active["label"] == label and self.cancel_since is None:
            return
        if self.active is None and self.holding == label:
            return
        self.pending = data
        if self.active and self.cancel_since is None:
            self.cancel_since = time.monotonic()
            self.motion.cancel.set()
            self._report("switching", "正在取消上一动作", label=label, motion_id=self.active["id"])

    def _acquire(self):
        self.acquisition_until = None
        candidates = [event for event in self.candidates.values()
                      if self._fresh(event["t"]) and self._visible(event)]
        self.candidates.clear()
        if not candidates:
            return

        event = min(candidates, key=lambda item: (item["t"], self._distance(item), item["track"]))
        self._lock_on(event)

    def _distance(self, event):
        position = self.people[event["track"]].get("position")
        try:
            distance = math.hypot(float(position[0]), float(position[1]))
            return distance if math.isfinite(distance) else math.inf
        except (IndexError, KeyError, TypeError, ValueError):
            return math.inf

    def _lock_on(self, event):
        self.operator = (event["track"], event["hand"])
        self.operator_seen = self.last_frame
        self.waiting_for_handover = False
        self._target(event)
        if self.active is None and self.holding == event["gesture"]:
            self._report("holding", "已切换操作者，灵巧手保持当前手型")

    def _handover(self):
        candidates = [event for key, event in self.active_gestures.items()
                      if self.stable_labels.get(key) == event["gesture"]]
        if candidates:
            event = min(candidates, key=lambda item: (self._distance(item), item["t"], item["track"]))
            self._lock_on(event)

    def _run(self, active):
        error = None
        completed = False
        try:
            self.motion.move(active["label"])
            completed = True
        except InterruptedError:
            pass
        except TimeoutError as exc:
            error = ("MOTION_TIMEOUT", "灵巧手未在规定时间内到位", exc)
        except Exception as exc:
            code = str(exc) if str(exc) in ("HAND_UNAVAILABLE", "CALIBRATION_MISSING") else "MOTION_FAILED"
            messages = {"HAND_UNAVAILABLE": "未找到绑定的灵巧手",
                        "CALIBRATION_MISSING": "该侧灵巧手缺少校准",
                        "MOTION_FAILED": "灵巧手执行失败，请检查连接和台架"}
            error = (code, messages[code], exc)
        with self.lock:
            self.active, self.cancel_since = None, None
            if error:
                code, message, exc = error
                LOG.warning("L20 Lite motion %s failed: %s", active["id"], exc)
                if not self.fault:
                    self._fail(code, message)
            elif completed and self.armed:
                self.holding = active["label"]
                self._report("holding", "灵巧手已到位", motion_id=active["id"])
            elif self.armed:
                self.holding = None
                self._report("ready", "上一动作已取消", motion_id=active["id"])
            if self.release_pending:
                self._release()

    def _tick(self):
        with self.lock:
            now = time.monotonic()
            if self.release_pending and self.active is None:
                self._release()
            if not self.armed:
                return
            if now - self.last_frame > STREAM_TIMEOUT:
                self._fail("POSE_STREAM_LOST", "Pose 数据中断，正在停止自动模仿")
                return
            if self.cancel_since is not None and now - self.cancel_since > CANCEL_TIMEOUT:
                self._fail("CANCEL_TIMEOUT", "上一动作未确认停止，已阻止执行下一手势")
                return
            if self.operator and now - self.operator_seen >= OPERATOR_GRACE:
                self.operator, self.pending = None, None
                self.waiting_for_handover = True
                if self.active is None:
                    self._report("waiting", "锁定手已丢失，当前手型已保留，等待有效手势")
            if self.waiting_for_handover:
                self._handover()
            self.last_events = {key: previous for key, previous in self.last_events.items()
                                if self._fresh(previous[0])}
            if self.acquisition_until is not None and now >= self.acquisition_until:
                self._acquire()
            if self.active is not None or self.pending is None:
                return
            if not self._visible(self.pending):
                if self.frame_stamp >= self.pending["t"]:
                    self.pending = None
                return
            label = self.pending["gesture"]
            self.pending = None
            if label == self.holding:
                return
            self.holding = None
            self.motion.cancel.clear()
            active = {"label": label, "id": uuid4().hex}
            active["thread"] = threading.Thread(target=self._run, args=(active,), daemon=True)
            self.active = active
            self._report("executing", "正在执行手势", label=label, motion_id=active["id"])
            active["thread"].start()

    def _disarm(self):
        self.armed = False
        self.operator = self.pending = self.acquisition_until = None
        self.candidates.clear()
        self.active_gestures.clear()
        self.gesture_seen.clear()
        self.previous_labels.clear()
        self.stable_labels.clear()
        self.waiting_for_handover = False
        self.last_events.clear()
        self.holding = None
        self.motion.cancel.set()
        self.release_pending = self.motion.side is not None

    def _release(self):
        self.release_pending = False
        try:
            self.motion.release()
        except Exception as exc:
            LOG.warning("L20 Lite torque release failed: %s", exc)
            self.fault = "STOP_FAILED"
            self._report("fault", "未确认力矩释放，请检查灵巧手连接", self.fault)
        else:
            if not self.fault:
                self._report("stopped", "已停止自动模仿")

    def _fail(self, code, message):
        self.fault = code
        self._disarm()
        self._report("fault", message, code)

    def dispatch(self, action, args):
        with self.lock:
            if action == "info":
                return self._info()
            instance = args.get("instance_id")
            # The canvas details dialog can send an action without the card ID.
            # Only manual enable/stop may use the single configured instance.
            if instance is None and action in ("enable", "stop"):
                instance = self.instance
                if instance is None:
                    if action == "stop":
                        return self._info()
                    raise ValueError("请先在画布上配置手势模仿卡并连接 Pose")
            if not isinstance(instance, str) or not instance:
                raise ValueError("需要卡片 instance_id")
            changing_instance = self.instance not in (None, instance)
            if changing_instance and (
                action != "config" or self.armed or self.active is not None
                or self.release_pending or self.motion.side is not None
            ):
                raise ValueError("首版只支持一张模仿卡，请先停止当前实例")
            if action == "config":
                bound = args.get("bound_side")
                speed_ratio = args.get("speed_ratio", DEFAULT_SPEED_RATIO)
                if bound not in SIDES:
                    raise ValueError("请选择要控制的灵巧手")
                if not isinstance(speed_ratio, str) or speed_ratio not in SPEED_RATIOS:
                    raise ValueError("请选择有效的关节速度倍率")
                if self.armed and (bound, speed_ratio) != (self.bound_side, self.speed_ratio):
                    raise ValueError("请先停止手势模仿，再修改设备或速度配置")
                if changing_instance:
                    for subscription in self.subscriptions:
                        self.node.destroy_subscription(subscription)
                    self.subscriptions.clear()
                    self.input_topic = None
                    self.last_frame = self.frame_stamp = 0.0
                    self.people.clear()
                self.instance, self.bound_side, self.speed_ratio = instance, bound, speed_ratio
            elif action == "start":
                topic = args.get("input_topic") or (args.get("input_topics") or [""])[0]
                if not isinstance(topic, str) or not TOPIC.fullmatch(topic):
                    raise ValueError("请连接 Pose 卡的 gesture 输出")
                if self.armed and topic != self.input_topic:
                    raise ValueError("请先停止手势模仿，再换接 Pose 输入")
                self._subscribe(topic)
                self.instance = instance
            elif action == "enable":
                if args.get("confirm_motion") is not True:
                    raise ValueError("启用前必须确认允许自动运动")
                if self.armed:
                    return self._info()
                if self.fault == "STOP_FAILED":
                    raise RuntimeError("上次卸力未确认。请连接绑定的灵巧手，然后在本卡执行 stop 重试卸力")
                if self.active is not None or self.release_pending or self.motion.side is not None:
                    raise RuntimeError("上一会话尚未停止，请在本卡执行 stop 后重试")
                if self.bound_side not in SIDES:
                    raise ValueError("请选择要控制的灵巧手")
                if time.monotonic() - self.last_frame > STREAM_TIMEOUT:
                    raise ValueError("请先启动 Pose 并确认已收到连续识别数据")
                try:
                    self.motion.claim(self.bound_side, self.speed_ratio)
                except RuntimeError as exc:
                    messages = {"HAND_BUSY": "该灵巧手正在执行其他动作，请稍后启用",
                                "CALIBRATION_MISSING": "该侧灵巧手缺少校准"}
                    if str(exc) not in messages:
                        raise
                    self._report("not_armed", messages[str(exc)], str(exc))
                    return self._info()
                self.fault, self.armed = None, True
                self.armed_at = time.time()
                self._report("waiting", "等待有效手势")
            elif action == "stop":
                self.fault = None
                self._disarm()
                if self.active is not None:
                    self._report("stopping", "正在取消动作并释放力矩")
                elif self.release_pending:
                    self._release()
                else:
                    self._report("stopped", "已停止自动模仿")
            else:
                return None
            return self._info()
