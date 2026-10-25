"""LinkerHand L20 gesture card.

Adapts gesture_device.py from L20 Lite to the USB-CAN transport and 16-joint layout.
Clearance sequences avoid mechanical collision between the thumb and fingers.
Calibrated per-side pose tables can be customized via config.yaml.

Actions:
  ok_gesture        — Thumb and index tips meet; remaining fingers extended
  thumb_up_gesture  — Thumb extended upward; four fingers curled into fist
  v_sign_gesture    — Index and middle fingers extended in V; other fingers curled
  natural_gesture   — Natural relaxed hanging posture
  number            — Show only the selected 0–9 gesture, without counting
  count             — Counts sequentially between start_num and end_num (0–9)
  config            — Accept lifecycle configuration without binding a hand
"""

from __future__ import annotations

import json
import os
import ssl
import threading
import time
import urllib.request
from typing import Any, Dict, List, Optional, Tuple, Union
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from can_client import (
    ACTIVE_JOINT_COUNT,
    VECTOR_JOINT_COUNT,
    JOINT_NAMES_16,
    validate_position_vector,
    vector20_to_named16,
    named16_to_vector20,
)
from hand_device import (
    HandPlugin, _SingleHandController, _setting, validate_hold_torque,
    SPEED_RATIOS, verify_hand_connection, ensure_motion_available,
    validate_bound_side, claim_motion, release_motion,
)

ACTIONS = (
    "ok_gesture", "thumb_up_gesture", "v_sign_gesture", "natural_gesture",
    "number", "count",
)

# 20-element vector format for L20 poses:
#   [0:5]   finger_base pitch (thumb, index, middle, ring, pinky)
#   [5:10]  yaw angles        (thumb, index, middle, ring, pinky)
#   [10:15] thumb roll        (thumb_roll, 0, 0, 0, 0)
#   [15:20] fingertip flexion (thumb, index, middle, ring, pinky)

DEFAULT_POSES_20: dict[str, list[int]] = {
    # OK gesture: thumb and index pinch together, other 3 fingers straight open
    "ok_gesture": [
        214, 166, 255, 255, 255,   # pitch
         85, 164, 110,  50,   0,   # yaw
        199, 255, 255, 255, 255,   # thumb roll & reserved
         94,  71, 255, 255, 255,   # tip
    ],
    # Thumbs up: thumb extended straight up, 4 fingers curled tight
    "thumb_up_gesture": [
        100,   0,   0,   0,   0,   # pitch
        255, 145, 142, 132, 111,   # yaw: curled fingers naturally resting in palm
        210, 255, 255, 255, 255,   # thumb roll & reserved
        255,   0,   0,   0,   0,   # tip
    ],
    # V sign: index and middle straight, other fingers curled
    "v_sign_gesture": [
         96, 255, 255,   0,   0,   # pitch
         59, 190,  66, 127, 102,   # yaw: pinky naturally resting at 102
        197, 255, 255, 255, 255,   # thumb roll & reserved
        144, 255, 255,   0,   0,   # tip
    ],
    # Natural relaxed hanging posture
    "natural_gesture": [
        135, 255, 255, 255, 255,   # pitch
         70, 110, 100,  99, 100,   # yaw
        255, 255, 255, 255, 255,   # thumb roll & reserved
        255, 255, 255, 255, 255,   # tip
    ],
}

NUMBER_POSES_20: dict[int, list[int]] = {
    0: [110,   0,   0,   0,   0, 140, 193, 158, 128,  91, 255, 255, 255, 255, 255, 119,   0,   0,   0,   0],
    1: [110, 255,   0,   0,   0,  81, 125, 162, 127,  80, 177, 255, 255, 255, 255, 132, 255,   0,   0,   0],
    2: [ 96, 255, 255,   0,   0,  59, 190,  66, 127,  80, 197, 255, 255, 255, 255, 144, 255, 255,   0,   0],
    3: [ 96, 255, 255, 255,   0,  22, 200, 132,  55,  80, 233, 255, 255, 255, 255, 144, 255, 255, 255,   0],
    4: [ 96, 255, 255, 255, 255,  22, 200, 132,  55,   0, 233, 255, 255, 255, 255, 144, 255, 255, 255, 255],
    5: [255, 255, 255, 255, 255,  70, 200, 132,  55,   0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
    6: [255,   0,   0,   0, 255,  70, 200, 132,  55,   0, 255, 255, 255, 255, 255, 255,   0,   0,   0, 255],
    7: [173,  34,  15,   0,   0, 111, 132, 132,  55,   0, 125, 255, 255, 255, 255, 131, 168, 168,   0,   0],
    8: [173, 255,   0,   0,   0, 172, 132, 132,  55,   0, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0],
    9: [188, 255,   0,   0,   0,  60, 162, 162, 144, 100, 189, 255, 255, 255, 255,  65,   0,   0,   0,   0],
}

# Directions confirmed by real-hand testing to require the thumb to return last.
# Keep the reverse direction unchanged when only one direction was reported.
THREE_STAGE_NUMBER_TRANSITIONS = frozenset({
    (0, 4), (4, 0),
    (0, 3), (3, 0),
    (0, 7), (7, 0),
    (4, 1), (1, 4),
    (1, 3), (3, 1),
    (7, 1), (7, 2),
    (6, 3), (8, 3), (9, 3),
    (6, 4), (8, 4), (9, 4),
})


class GesturePlugin:
    PREFIX = "hand_gesture"

    def __init__(self, hand: HandPlugin | None, config: dict, global_config: dict | None = None):
        self.hand = hand
        self.config = config
        self.global_config = global_config or (hand.config if hand else {})
        self._lock = threading.RLock()
        self._controllers: dict[tuple[str, str], _SingleHandController] = {}
        self._active: dict[tuple[str, str], dict] = {}
        self._held: dict[tuple[str, str], dict] = {}
        self._task = threading.local()
        self._stop = threading.Event()
        self._delivery_failed = False
        self._last_result: dict | None = None
        self._current_number: dict[str, int | None] = {"left": None, "right": None}

        self.speed = _setting(config.get("motion_speed", 85), "motion_speed")
        self.count_speed = _setting(config.get("count_speed", 85), "count_speed")
        self.torque = _setting(config.get("motion_torque", 120), "motion_torque")
        self.thumb_speed_scaling = config.get("thumb_speed_scaling", True)
        if not isinstance(self.thumb_speed_scaling, bool):
            raise ValueError("thumb_speed_scaling must be boolean")

        self.timeout = float(config.get("motion_timeout_s", 2.5))
        self.tolerance = int(config.get("position_tolerance", 20))
        self.poll = float(config.get("poll_interval_s", 0.08))

        # Load per-side calibrated poses
        self._calibration: dict[str, dict] = {}
        for side in ("left", "right"):
            poses = config.get(f"{side}_poses")
            if poses is not None:
                self._calibration[side] = {
                    name: validate_position_vector(poses[name], f"{side}_poses/{name}")
                    for name in DEFAULT_POSES_20 if name in poses
                }
            else:
                self._calibration[side] = dict(DEFAULT_POSES_20)

            numbers = config.get(f"{side}_number_poses")
            if numbers is not None:
                self._calibration[side]["count"] = {
                    int(n): validate_position_vector(pose, f"{side}_number_poses/{n}")
                    for n, pose in numbers.items()
                }
            else:
                self._calibration[side]["count"] = dict(NUMBER_POSES_20)

    # ── tool schema ──────────────────────────────────────────────────

    def get_tool(self) -> dict:
        schema = action_schema(
            {
                "info": (["bound_side"], "查看所选手的手势动作状态"),
                "start": ([], "启动手势控制"),
                "stop": (["bound_side"], "停止所选手的手势动作并释放本卡片的保持力矩"),
                "ok_gesture": (["bound_side", "speed_ratio", "hold_torque", "confirm_motion"],
                               "OK 手势：拇指与食指捏合，其余三指伸展；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）"),
                "thumb_up_gesture": (["bound_side", "speed_ratio", "hold_torque", "confirm_motion"],
                                     "点赞手势：拇指伸展向上，四指收拢握拳；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）"),
                "v_sign_gesture": (["bound_side", "speed_ratio", "hold_torque", "confirm_motion"],
                                   "V 字手势：食指和中指伸展呈 V 字形，拇指收合；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）"),
                "natural_gesture": (["bound_side", "speed_ratio", "hold_torque", "confirm_motion"],
                                    "自然下垂常态手势：手指轻微弯曲放松；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）"),
                "number": (["bound_side", "number", "speed_ratio", "hold_torque", "confirm_motion"],
                           "数字手势：仅切换到指定的 0–9 数字手势，不逐个计数；保留已有避碰步骤；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）"),
                "count": (["bound_side", "start_num", "end_num", "hold_time_s", "speed_ratio",
                           "hold_torque", "confirm_motion"],
                          "连续计数：从起始数字计数到结束数字（支持 0–9 和倒数）；hold_torque 设置到位保持力矩（0–255，默认 0 断力防发热）"),
                "config": ([], "接收卡片生命周期配置；执行时选择左右手"),
            },
            {
                "bound_side": {"type": "string", "enum": ["", "left", "right"], "default": "",
                               "description": "本次操作的灵巧手：left 左手，right 右手；必须明确选择"},
                "instance_id": {"type": "string"},
                "number":      {"type": "integer", "enum": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], "default": 0, "description": "选择目标数字手势（0–9）"},
                "speed_ratio": {"type": "string", "enum": list(SPEED_RATIOS), "description": "速度倍率（1.0x 到 3.0x）"},
                "hold_torque": {
                    "type": "integer",
                    "minimum": 0,
                    "maximum": 255,
                    "default": 0,
                    "description": "到位后保持力矩（0–255 整数；0 表示断电/断力防发热，1–255 保持指定力矩；不填默认为 0）",
                },
                "start_num":   {"type": "integer", "minimum": 0, "maximum": 9, "default": 1},
                "end_num":     {"type": "integer", "minimum": 0, "maximum": 9, "default": 5},
                "hold_time_s": {"type": "number",  "minimum": 0.05, "maximum": 10.0, "default": 1.0},
                "confirm_motion": {"type": "boolean", "const": True, "description": "确认手附近安全后传入 true"},
            },
        )
        schema["allOf"] = [
            {"if": {"properties": {"action": {"enum": list(ACTIONS)}}},
             "then": {"required": ["bound_side", "confirm_motion"],
                      "properties": {"bound_side": {"enum": ["left", "right"]}}}},
            {"if": {"properties": {"action": {"const": "info"}}},
             "then": {"required": ["bound_side"],
                      "properties": {"bound_side": {"enum": ["left", "right"]}}}},
        ]
        schema["x-is-dangerous"] = True
        schema["x-completion"] = {"actions": list(ACTIONS), "timeout": 300}

        definition = tool(self.PREFIX, "actuator", "L20 预置手势与计数卡片。动作须显式传 confirm_motion=true。", schema)
        definition["title"] = "L20 手势"
        definition["multiInstance"] = True
        return definition

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            for active in self._active.values():
                active["stop_event"].set()
            held = list(self._held.values())
            self._held.clear()
        self._release_held(held)

    def _release_held(self, held: list[dict]) -> None:
        for record in held:
            ctrl = record["controller"]
            with ctrl._motion_lock:
                if vars(ctrl).get("_last_command_owner") == record["owner"] and not vars(ctrl).get("_grasp_owner"):
                    ctrl.release_torque()
                    ctrl._holding_torque = 0

    # ── helpers ──────────────────────────────────────────────────────

    def _get_client_and_ctrl(self, side: str):
        side = validate_bound_side(side)
        shared_config = self.hand.config if self.hand is not None else self.global_config
        channel = shared_config.get("can_channel", "can0")
        if self.hand is not None:
            if hasattr(self.hand, "get_controller_for_side"):
                ctrl = self.hand.get_controller_for_side(side, channel)
            else:
                raise RuntimeError("Invalid hand reference")
        else:
            from hand_device import get_shared_controller
            ctrl = get_shared_controller({**self.global_config, **self.config}, side, channel)

        with self._lock:
            self._controllers[(side, channel)] = ctrl
        return ctrl.get(), side, ctrl

    def _speed(self, ratio: str, base: int) -> int:
        if not isinstance(ratio, str) or ratio not in SPEED_RATIOS:
            raise ValueError(f"speed_ratio must be one of {', '.join(SPEED_RATIOS)}")
        return min(255, round(base * SPEED_RATIOS[ratio]))

    def _check_stop(self) -> None:
        if self._stop.is_set() or self._stop_event().is_set():
            raise InterruptedError("gesture motion stopped")

    def _wait(self, seconds: float) -> None:
        if self._stop_event().wait(seconds):
            raise InterruptedError("gesture motion stopped")
        self._check_stop()

    def _stop_event(self) -> threading.Event:
        return getattr(self._task, "stop_event", self._stop)

    def _drive_and_wait(self, client, target_vec: list[int], speed: int,
                        *, tolerance: int | None = None, timeout: float | None = None,
                        strict: bool = False, ignore_closed_yaw: bool = False,
                        finger_tolerance: int | None = None, phase: str | None = None,
                        joint_tolerances: dict[int, int] | None = None) -> list[int]:
        self._check_stop()
        client.set_joint_positions(target_vec)
        t_start = time.monotonic()
        effective_timeout = self.timeout if timeout is None else timeout
        deadline = t_start + effective_timeout
        limit = self.tolerance if tolerance is None else tolerance
        checked_indices = self._position_check_indices(target_vec, ignore_closed_yaw=ignore_closed_yaw)
        limits = {
            i: min(limit, finger_tolerance) if finger_tolerance is not None and i in (1, 2, 3, 4, 16, 17, 18, 19) else limit
            for i in checked_indices
        }
        for idx, joint_limit in (joint_tolerances or {}).items():
            if idx in limits:
                limits[idx] = min(limit, joint_limit)

        prev_vec: list[int] | None = None
        stable_count = 0

        while True:
            self._check_stop()
            measured_vec = client.get_positions_vector()
            diffs = {i: abs(measured_vec[i] - target_vec[i]) for i in checked_indices}

            # 1. Target reached within tolerance
            if diffs and all(d <= limits[i] for i, d in diffs.items()):
                return measured_vec

            # 2. Settled / arrival detection: after minimal travel time (0.35s),
            # if motors have stopped moving (delta <= 2 for 2 consecutive cycles)
            elapsed = time.monotonic() - t_start
            if not strict and prev_vec is not None and elapsed >= 0.35:
                delta = max(abs(a - b) for i, (a, b) in enumerate(zip(measured_vec, prev_vec)) if i not in (11, 12, 13, 14))
                if delta <= 2:
                    stable_count += 1
                    if stable_count >= 2:
                        return measured_vec
                else:
                    stable_count = 0
            prev_vec = measured_vec

            # 3. Timeout fallback
            if time.monotonic() >= deadline:
                if strict:
                    measured = vector20_to_named16(measured_vec)
                    target = vector20_to_named16(target_vec)
                    named_limits = vector20_to_named16([limits.get(i, -1) for i in range(VECTOR_JOINT_COUNT)])
                    pending = ", ".join(
                        f"{name}={measured[name]}/{value}"
                        for name, value in target.items()
                        if named_limits[name] >= 0 and abs(measured[name] - value) > named_limits[name]
                    )
                    raise TimeoutError(
                        f"避碰过渡姿态未到位，已中止后续动作（{phase or '目标到位'}；"
                        f"实测/目标: {pending}）"
                    )
                return measured_vec
            self._wait(min(self.poll, 0.06))

    def _position_check_indices(self, target_vec: list[int], *, ignore_closed_yaw: bool = False) -> list[int]:
        """Folded fingers may settle at a different yaw despite reaching flexion.

        Actual digit-0 feedback repeatedly reports index yaw around 159 against
        target 193. Do not use folded yaw to reject a digit or block thumb return.
        Thumb, finger pitch/tip and yaw of extended fingers remain checked.
        """
        ignored = {11, 12, 13, 14}
        if ignore_closed_yaw:
            for finger in range(1, 5):
                if target_vec[finger] <= self.tolerance and target_vec[finger + 15] <= self.tolerance:
                    ignored.add(finger + 5)
        return [i for i in range(VECTOR_JOINT_COUNT) if i not in ignored]

    def _clearance(self, client, target_vec: list[int], speed: int) -> None:
        """Clearance motion to avoid thumb-finger collision."""
        current_vec = client.get_positions_vector()
        # If thumb is currently deep inside and target requires moving outward, lift thumb pitch first
        if current_vec[0] < 120 and target_vec[0] > 180:
            lift = [max(0, min(255, v)) for v in current_vec]
            lift[11:15] = [0, 0, 0, 0]
            lift[0] = 220
            lift[15] = max(lift[15], 180)
            self._drive_and_wait(client, lift, speed, tolerance=25, timeout=0.8)
            self._wait(0.05)

    def _pose(self, client, target_vec: list[int], speed: int) -> list[int]:
        self._clearance(client, target_vec, speed)
        return self._drive_and_wait(client, target_vec, speed)

    def _clear_thumb_transition(self, client, target_vec: list[int], speed: int,
                                clearance_vec: list[int], *, thumb_last: bool = False,
                                release_thumb_tip_first: bool = False,
                                joint_tolerances: dict[int, int] | None = None) -> list[int]:
        """Clear the thumb, optionally waiting for the other fingers before return."""
        current_vec = client.get_positions_vector()
        if release_thumb_tip_first:
            # Leaving digit 7: do not sweep yaw/roll while the thumb tip is hooked.
            # Release pitch and tip together, holding yaw/roll and four fingers.
            prep_release_tip = list(current_vec)
            prep_release_tip[0] = 255
            prep_release_tip[15] = 255
            self._drive_and_wait(client, prep_release_tip, speed, strict=True, phase="拇指指尖与指根同时松开")
            self._wait(0.05)

        prep_free_thumb = list(current_vec)
        # Keep the other fingers at their measured positions. All four thumb
        # joints, including roll, follow this hand's calibrated digit-5 pose.
        for idx in (0, 5, 10, 15):
            prep_free_thumb[idx] = clearance_vec[idx]
        self._drive_and_wait(client, prep_free_thumb, speed, strict=True, phase="拇指让位")
        self._wait(0.05)

        if thumb_last:
            # Keep the thumb clear until all four fingers have reached their
            # targets, preventing them from hooking or scraping a returning thumb.
            prep_target = list(target_vec)
            for idx in (0, 5, 10, 15):
                prep_target[idx] = clearance_vec[idx]
            self._drive_and_wait(
                client, prep_target, speed, strict=True, ignore_closed_yaw=True,
                finger_tolerance=10, phase="四指到位，拇指保持让位",
                joint_tolerances=joint_tolerances,
            )
            self._wait(0.05)

        # After clearance, the thumb and other fingers move to the target together.
        # With thumb_last, the other fingers are already at their target positions.
        return self._drive_and_wait(
            client, target_vec, speed, strict=True, ignore_closed_yaw=thumb_last,
            phase="拇指收尾" if thumb_last else "目标手势",
        )

    def _transition_to_number(self, client, from_num: int | None, to_num: int, numbers: dict, speed: int) -> list[int]:
        target = numbers[to_num]

        if (from_num, to_num) in THREE_STAGE_NUMBER_TRANSITIONS:
            print(f"[Gesture] {from_num}→{to_num}: 拇指让位→四指到位→拇指收尾 (speed={speed})", flush=True)
            return self._clear_thumb_transition(
                client, target, speed, clearance_vec=numbers[5], thumb_last=True,
                release_thumb_tip_first=(from_num == 7 and to_num in (0, 1, 2)),
                # Recorded 0->7 stalls at pinky pitch 15 against target 0.
                # Keep its configured tolerance; other finger gates remain 10.
                joint_tolerances={4: self.tolerance} if (from_num, to_num) == (0, 7) else None,
            )

        if to_num != from_num and (from_num in (1, 2, 9) or to_num in (1, 2, 3, 4, 9)):
            return self._clear_thumb_transition(client, target, speed, clearance_vec=numbers[5])

        # 倒数场景 7 变 6（大拇指先松开外展让出空间，随后食指和中指收紧入掌）
        if from_num == 7 and to_num == 6:
            # 第一阶段：食指中指保持 7 的姿态，大拇指先动（松开外展）
            prep_free_thumb = list(numbers[7])
            prep_free_thumb[0] = target[0]   # thumb_pitch (255)
            prep_free_thumb[5] = target[5]   # thumb_yaw (70)
            prep_free_thumb[10] = target[10] # thumb_roll (255)
            prep_free_thumb[15] = target[15] # thumb_tip (255)
            self._drive_and_wait(client, prep_free_thumb, speed, tolerance=25, timeout=0.8)
            self._wait(0.05)

            # 第二阶段：大拇指已在外侧安全区域，食指和中指收紧入掌，小拇指张开到位
            return self._drive_and_wait(client, target, speed)

        else:
            return self._pose(client, target, speed)

    def _detect_current_number(self, client, numbers: dict) -> int | None:
        """Attempt to match current feedback vector to one of the 0..9 number poses."""
        try:
            curr = client.get_positions_vector()
            best_num = None
            min_max_diff = 999
            for num, pose in numbers.items():
                checked_indices = self._position_check_indices(pose, ignore_closed_yaw=True)
                diffs = [abs(curr[i] - pose[i]) for i in checked_indices]
                max_d = max(diffs)
                if max_d < min_max_diff:
                    min_max_diff = max_d
                    best_num = num
            if min_max_diff <= 28:
                return best_num
        except Exception:
            pass
        return None

    def _number(self, client, side: str, poses: dict, options: dict) -> dict:
        speed = options["speed"]
        numbers = poses["count"]
        target_num = options["number"]
        if target_num not in numbers:
            raise ValueError(f"number must be between 0 and 9, got {target_num}")

        # Read the actual posture: count, other cards or released torque may
        # have changed it since the previous number action.
        from_num = self._detect_current_number(client, numbers)
        # Apply target-side clearance even from a non-numeric posture.
        last_measured = self._transition_to_number(client, from_num, target_num, numbers, speed)

        with self._lock:
            self._current_number[side] = target_num

        return {
            "state": "completed",
            "action": "number",
            "number": target_num,
            "positions": vector20_to_named16(last_measured),
        }

    def _count(self, client, poses: dict, options: dict) -> dict:
        speed = options["speed"]
        numbers = poses["count"]
        start_num = options["start_num"]
        end_num = options["end_num"]
        hold_time = options["hold_time_s"]

        # 区分正数和倒数场景：
        if start_num <= end_num:
            # 1. 正数场景：数数前先平滑收回握拳态（数字 0）作为起始基准
            from_num = self._detect_current_number(client, numbers)
            if from_num in (1, 2, 4, 9) or (from_num, 0) in THREE_STAGE_NUMBER_TRANSITIONS:
                last_measured = self._transition_to_number(client, from_num, 0, numbers, speed)
            else:
                current_vec = client.get_positions_vector()
                prep_fist = list(numbers[0])
                prep_fist[0] = max(prep_fist[0], current_vec[0])
                prep_fist[5] = max(prep_fist[5], current_vec[5])
                prep_fist[15] = max(prep_fist[15], current_vec[15])
                self._drive_and_wait(client, prep_fist, speed, tolerance=25, timeout=1.0)
                self._wait(0.05)
                last_measured = self._drive_and_wait(client, numbers[0], speed, tolerance=25, timeout=1.0)
            self._wait(0.15)

            current_num = 0

            # 如果起始数字不是 0，先按防碰时序平滑过渡到 start_num
            if start_num > 0:
                for n in range(1, start_num + 1):
                    self._check_stop()
                    last_measured = self._transition_to_number(client, current_num, n, numbers, speed)
                    current_num = n
                    if n < start_num:
                        self._wait(min(0.15, hold_time))
        else:
            # 2. 倒数场景：直接从 start_num 开始倒数，无需先握拳再从 0 正数到 9
            from_num = self._detect_current_number(client, numbers)
            last_measured = self._transition_to_number(client, from_num, start_num, numbers, speed)
            current_num = start_num

        # 从 start_num 开始执行数数序列（正数或倒数）
        step = 1 if start_num <= end_num else -1
        sequence = list(range(start_num, end_num + step, step))

        for num in sequence:
            self._check_stop()
            if num != current_num:
                last_measured = self._transition_to_number(client, current_num, num, numbers, speed)
                current_num = num
            self._wait(hold_time)

        return {"state": "completed", "final_number": end_num, "positions": vector20_to_named16(last_measured)}

    # ── ACP callback ─────────────────────────────────────────────────

    def _acp_callback(self, action_id: str, status: str, result: dict) -> None:
        if self.hand:
            self.hand._acp_callback(action_id, status, result)

    # ── async action thread ──────────────────────────────────────────

    def _run_action(self, instance_id: str, active: dict) -> None:
        action = active["action"]
        options = active["options"]
        client, side, ctrl = active["client"], active["side"], active["controller"]
        poses = self._calibration.get(side, DEFAULT_POSES_20)
        owns_motion = False
        self._task.stop_event = active["stop_event"]

        try:
            with ctrl._motion_lock:
                ensure_motion_available(ctrl, active["owner"])
                self._check_stop()
                verify_hand_connection(client)
                owns_motion = True
                client.set_speeds([options["speed"]] * 5)
                client.set_currents([options["torque"]] * 5)

                if action == "number":
                    result = self._number(client, side, poses, options)
                elif action == "count":
                    options["side"] = side
                    result = self._count(client, poses, options)
                else:
                    with self._lock:
                        self._current_number[side] = None
                    target = poses.get(action, DEFAULT_POSES_20[action])
                    measured = self._pose(client, target, options["speed"])
                    result = {"state": "completed", "action": action, "positions": vector20_to_named16(measured)}

                hold_torque = options.get("hold_torque", 0)
                client.set_currents([hold_torque] * 5)
                ctrl._holding_torque = hold_torque
                result["torque"] = hold_torque
                result["hold_torque"] = hold_torque
                result["torque_released"] = (hold_torque == 0)
                with self._lock:
                    if hold_torque:
                        self._held[active["target"]] = active
                    else:
                        self._held.pop(active["target"], None)

            status = "completed"
        except InterruptedError as exc:
            status, result = "cancelled", {"state": "cancelled", "message": str(exc)}
            try:
                with ctrl._motion_lock:
                    if (owns_motion or active.get("inherited_hold")) and not vars(ctrl).get("_grasp_owner"):
                        client.set_currents([0] * 5)
                        ctrl._holding_torque = 0
            except Exception:
                pass
        except Exception as exc:
            status, result = "error", {"state": "error", "message": str(exc)}
            print(f"[Gesture] {active['action_id']} ({side}, speed={options['speed']}): {exc}", flush=True)
            try:
                with ctrl._motion_lock:
                    if (owns_motion or active.get("inherited_hold")) and not vars(ctrl).get("_grasp_owner"):
                        client.set_currents([0] * 5)
                        ctrl._holding_torque = 0
            except Exception:
                pass

        finally:
            release_motion(ctrl, active["owner"])
            del self._task.stop_event

        with self._lock:
            self._last_result = {"action_id": active["action_id"], "status": status, "result": result}
            if self._active.get(active["target"]) is active:
                self._active.pop(active["target"], None)
            if status != "completed" and self._held.get(active["target"]) is active:
                self._held.pop(active["target"], None)

        try:
            self._acp_callback(active["action_id"], status, result)
        except Exception as exc:
            with self._lock:
                self._delivery_failed = True
            print(f"[ACP] {active['action_id']}: {exc}", flush=True)

    def _start_action(self, instance_id: str, action: str, args: dict) -> dict:
        side = validate_bound_side(args.get("bound_side"))
        ratio = args.get("speed_ratio", "1.0x")
        speed = self._speed(ratio, self.count_speed if action in ("number", "count") else self.speed)
        hold_torque = validate_hold_torque(args.get("hold_torque"), default=0)

        options: dict[str, Any] = {
            "speed": speed,
            "torque": self.torque,
            "hold_torque": hold_torque,
        }

        if action == "number":
            raw_num = args.get("number", 0)
            # Canvas enum controls send their selected value as a string.
            if isinstance(raw_num, str) and len(raw_num) == 1 and raw_num in "0123456789":
                raw_num = int(raw_num)
            if isinstance(raw_num, bool) or not isinstance(raw_num, int) or not 0 <= raw_num <= 9:
                raise ValueError(f"number 必须是 0–9 之间的整数，收到: {raw_num!r}")
            options["number"] = raw_num
        elif action == "count":
            s_num = args.get("start_num", 1)
            if isinstance(s_num, bool) or not isinstance(s_num, int) or not 0 <= s_num <= 9:
                raise ValueError(f"start_num 必须是 0–9 之间的整数，收到: {s_num!r}")
            e_num = args.get("end_num", 5)
            if isinstance(e_num, bool) or not isinstance(e_num, int) or not 0 <= e_num <= 9:
                raise ValueError(f"end_num 必须是 0–9 之间的整数，收到: {e_num!r}")
            h_time = args.get("hold_time_s", 1.0)
            if isinstance(h_time, bool) or not isinstance(h_time, (int, float)) or not 0.05 <= h_time <= 10.0:
                raise ValueError(f"hold_time_s 必须是 0.05–10.0 之间的数值（秒），收到: {h_time!r}")
            options["start_num"] = s_num
            options["end_num"] = e_num
            options["hold_time_s"] = float(h_time)

        active = {
            "action": action,
            "action_id": f"gesture_{action}_{uuid4().hex}",
            "instance_id": instance_id,
            "options": options,
            "stop_event": threading.Event(),
        }
        active["owner"] = (id(self), active["action_id"])

        with self._lock:
            client, side, ctrl = self._get_client_and_ctrl(side)
            target = (side, ctrl.channel)
            if target in self._active:
                raise RuntimeError("该手已有手势动作正在执行，请先停止当前动作")
            shared_config = self.hand.config if self.hand is not None else self.global_config
            claim_motion(ctrl, active["owner"], config=shared_config)
            try:
                verify_hand_connection(client)
            except Exception:
                release_motion(ctrl, active["owner"], restore_previous=True)
                raise
            active.update(client=client, side=side, controller=ctrl, target=target)
            self._stop.clear()
            previous_hold = self._held.pop(target, None)
            active["inherited_hold"] = previous_hold is not None
            self._active[target] = active

        try:
            threading.Thread(
                target=self._run_action, args=(instance_id, active),
                name=active["action_id"], daemon=True,
            ).start()
        except Exception:
            with self._lock:
                self._active.pop(target, None)
                if previous_hold is not None:
                    self._held[target] = previous_hold
                release_motion(ctrl, active["owner"], restore_previous=True)
            raise

        return {"state": "running", "action_id": active["action_id"], "bound_side": side}

    # ── dispatch ─────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get("instance_id") or ""

        if action in ("config", "start"):
            return {"state": "configured" if action == "config" else "ready",
                    "message": "执行动作时请选择左手或右手"}

        if action == "info":
            side = validate_bound_side(args.get("bound_side"))
            shared_config = self.hand.config if self.hand is not None else self.global_config
            target = (side, shared_config.get("can_channel", "can0"))
            with self._lock:
                active_id = (self._active.get(target) or {}).get("action_id")
            return {
                "state": "running" if active_id else "ready",
                "bound_side": side,
                "active_action_id": active_id,
                "available_gestures": list(ACTIONS),
            }

        if action == "stop":
            side = args.get("bound_side")
            if side not in (None, ""):
                side = validate_bound_side(side)
                shared_config = self.hand.config if self.hand is not None else self.global_config
                target = (side, shared_config.get("can_channel", "can0"))
            else:
                target = None
            with self._lock:
                # Lifecycle cleanup can only address tasks whose execution actually
                # supplied this ID. Anonymous canvas requests are never guessed.
                matches = lambda key, record: key == target if target is not None else bool(instance_id) and record["instance_id"] == instance_id
                active = [record for key, record in self._active.items() if matches(key, record)]
                held = [(key, record) for key, record in self._held.items() if matches(key, record)]
                for record in active:
                    record["stop_event"].set()
                for key, record in held:
                    if self._held.get(key) is record:
                        self._held.pop(key, None)
            self._release_held([record for _, record in held])
            return {"state": "idle", "cancel_requested": bool(active),
                    "target_resolved": bool(active or held),
                    "active_action_id": active[0]["action_id"] if active else None}

        if action in ACTIONS:
            if args.get("confirm_motion") is not True:
                raise ValueError("confirm_motion must be true for every gesture request")
            return self._start_action(instance_id, action, args)

        return None
