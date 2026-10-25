"""L20 position-only side/top grasp and release card.

Preparation and release use position feedback. Closing sends the final position
command once; object obstruction is normal and does not mean action failure.
No tactile data or object-grasp confirmation is used.
"""
from __future__ import annotations

import threading
import time
from typing import Any
from uuid import uuid4

from common.vendor_runtime import action_schema, tool
from can_client import validate_position_vector, vector20_to_named16
from hand_device import (HandPlugin, _SingleHandController, _setting,
                         verify_hand_connection, ensure_motion_available, SPEED_RATIOS,
                         validate_bound_side, claim_motion, release_motion)

GRASP_ACTIONS = ("side_grasp", "top_grasp", "release")

DEFAULT_GRASP_PREP_POSE = [
    255, 255, 255, 255, 255, 119, 193, 148, 105, 42,
    61, 255, 255, 255, 255, 255, 255, 255, 255, 255,
]

DEFAULT_TOP_GRASP_PREP_POSE = [
    255, 255, 255, 255, 255, 70, 200, 132, 55, 0,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
]


class GraspFault(RuntimeError):
    def __init__(self, code: str, message: str, details: dict | None = None):
        super().__init__(message)
        self.code = code
        self.details = details or {}



def _validate_int(value: Any, name: str, min_val: int, max_val: int, default: int | None = None) -> int:
    if value is None or value == "":
        if default is not None:
            return default
        raise ValueError(f"{name} 不能为空")
    if isinstance(value, bool) or not isinstance(value, int) or not min_val <= value <= max_val:
        raise ValueError(f"{name} 必须是 {min_val}–{max_val} 之间的整数，收到: {value!r}")
    return value



def _validate_float(value: Any, name: str, min_val: float, max_val: float, default: float | None = None) -> float:
    if value is None or value == "":
        if default is not None:
            return default
        raise ValueError(f"{name} 不能为空")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not min_val <= value <= max_val:
        raise ValueError(f"{name} 必须是 {min_val}–{max_val} 之间的数值，收到: {value!r}")
    return float(value)



class GraspPlugin:
    PREFIX = "hand_grasp"

    def __init__(self, hand: HandPlugin | None, config: dict, global_config: dict | None = None):
        self.hand = hand
        self.config = config
        self.global_config = global_config or (hand.config if hand else {})
        self._lock = threading.RLock()
        # Motion and persistent grasp geometry belong to the physical hand,
        # independently of which canvas card submitted the command.
        self._controllers: dict[tuple[str, str], _SingleHandController] = {}
        self._active: dict[tuple[str, str], dict] = {}
        self._stop = threading.Event()
        self._delivery_failed = False
        self._last_result: dict | None = None
        self._results: dict[tuple[str, str], dict] = {}
        self._sessions: dict[tuple[str, str], dict] = {}
        self._states: dict[tuple[str, str], str] = {}
        self._worker_context = threading.local()
        # The multiplier baseline is fixed. Old pressure/hold configuration is unused.
        self.approach_speed = 85
        self.release_speed = _setting(config.get('release_speed', 80), 'release_speed')
        self.motion_current = _validate_int(config.get('motion_current'), 'motion_current', 1, 255, 230)
        self.poll_interval_s = _validate_float(config.get('poll_interval_s'), 'poll_interval_s', 0.001, 1.0, 0.05)
        self.pose_timeout_s = _validate_float(config.get('pose_timeout_s'), 'pose_timeout_s', 0.05, 15.0, 5.0)
        self.position_tolerance = _validate_int(config.get('position_tolerance'), 'position_tolerance', 1, 40, 20)

    def get_tool(self) -> dict:
        schema = action_schema({
            'side_grasp': (['bound_side', 'speed_ratio', 'confirm_motion'],
                           '侧向抓握：先形成 U 型包围，再将五指指根和指尖目标设为 0，保持至 release；不判断物体抓取结果'),
            'top_grasp': (['bound_side', 'speed_ratio', 'confirm_motion'],
                          '向下抓握：展开准备后收拢指根和四指指尖至 0；拇指尖保持伸直避碰，保持至 release'),
            'release': (['bound_side', 'confirm_motion'], '五指同步恢复对应准备姿态以放开物体，再完全张开并卸力'),
        }, {
            'bound_side': {'type': 'string', 'enum': ['', 'left', 'right'], 'default': '',
                           'description': '本次操作的灵巧手，必须选择左手或右手'},
            'instance_id': {'type': 'string'},
            'speed_ratio': {'type': 'string', 'enum': list(SPEED_RATIOS), 'default': '1.0x',
                            'description': '电机速度：1.0x=85，1.5x=128，2.0x=170，2.5x=212，3.0x=255'},
            'wait_for_result': {'type': 'boolean', 'const': False, 'default': False,
                                'description': '仅兼容 false；动作立即返回 action_id，最终结果通过 ACP 通知'},
            'confirm_motion': {'type': 'boolean', 'const': True, 'description': '确认周围安全后传入 true'},
        })
        schema['allOf'] = [{'if': {'properties': {'action': {'enum': list(GRASP_ACTIONS)}}},
                            'then': {'required': ['bound_side', 'confirm_motion'],
                                     'properties': {'bound_side': {'enum': ['left', 'right']}}}}]
        schema['x-is-dangerous'] = True
        schema['x-completion'] = {'actions': list(GRASP_ACTIONS), 'timeout': max(120, int(4 * self.pose_timeout_s + 30))}
        definition = tool(self.PREFIX, 'actuator', 'L20 纯位置抓握：侧向抓握、向下抓握和释放；不使用触觉反馈或判断物体抓取成功。', schema)
        definition['title'] = 'L20 位置抓握'
        definition['multiInstance'] = True
        return definition

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            for active in self._active.values():
                active['stop_event'].set()
            controllers = list(self._controllers.items())
            self._sessions.clear()
        for device_key, ctrl in controllers:
            self._unload_owned(ctrl, (id(self), device_key))

    def _get_client_and_ctrl(self, side: str, channel: str | None = None):
        side = validate_bound_side(side)
        channel = channel or self.global_config.get("can_channel", "can0")
        if self.hand is not None:
            if isinstance(self.hand, HandPlugin):
                ctrl = self.hand.get_controller_for_side(side, channel)
            elif hasattr(self.hand, "get_controller_for_side"):
                ctrl = self.hand.get_controller_for_side(side, channel)
            else:
                raise RuntimeError("Invalid hand reference")
        else:
            from hand_device import get_shared_controller
            hand_config = self.global_config.get('plugins', {}).get('hand', {})
            ctrl = get_shared_controller({**self.global_config, **hand_config, **self.config}, side, channel)

        with self._lock:
            self._controllers[(side, channel)] = ctrl
        return ctrl.get(), side, ctrl


    def _check_stop(self) -> None:
        event = getattr(self._worker_context, "stop_event", self._stop)
        if self._stop.is_set() or event.is_set():
            raise InterruptedError("grasp motion stopped")


    def _wait(self, seconds: float) -> None:
        event = getattr(self._worker_context, "stop_event", self._stop)
        if event.wait(seconds):
            raise InterruptedError("grasp motion stopped")
        self._check_stop()


    def _phase(self, value: str) -> None:
        active = getattr(self._worker_context, 'active', None)
        if active is not None:
            with self._lock:
                if self._active.get(active['device_key']) is active:
                    self._states[active['device_key']] = value


    def _move_pose(self, client, ctrl, target: list[int], phase: str) -> list[int]:
        self._phase(phase)
        self._check_stop()
        with ctrl._motion_lock:
            self._check_stop()
            client.set_joint_positions(target)
        deadline = time.monotonic() + self.pose_timeout_s
        while True:
            self._check_stop()
            with ctrl._motion_lock:
                measured = client.get_positions_vector()
            # Yaw of fully folded non-thumb fingers need not match an unloaded pose.
            indices = [i for i in range(20) if i not in (11, 12, 13, 14)]
            for finger in range(1, 5):
                if target[finger] <= self.position_tolerance and target[15 + finger] <= self.position_tolerance:
                    indices.remove(5 + finger)
            if all(abs(measured[i] - target[i]) <= self.position_tolerance for i in indices):
                return measured
            if time.monotonic() >= deadline:
                raise GraspFault('pose_timeout', f'{phase}未到位，已停止；未确认张开或到达准备姿态')
            self._wait(self.poll_interval_s)


    def _open_hand(self, client, ctrl) -> list[int]:
        with ctrl._motion_lock:
            current = client.get_positions_vector()
        target = validate_position_vector(ctrl._open_pose, 'open_pose')
        # Free pitch and tip together before sweeping the thumb out of the palm.
        free_tip = list(current)
        free_tip[0] = free_tip[15] = 255
        self._move_pose(client, ctrl, free_tip, '释放拇指指根与指尖')
        free_thumb = list(current)
        for i in (0, 5, 10, 15):
            free_thumb[i] = target[i]
        self._move_pose(client, ctrl, free_thumb, '拇指让位')
        return self._move_pose(client, ctrl, target, '四指张开')


    def _grasp_prep_pose(self, ctrl, mode: str = 'side') -> list[int]:
        if mode == 'top':
            name = f'{ctrl.side}_top_grasp_prep_pose'
            default_pose = self.config.get('top_grasp_prep_pose', DEFAULT_TOP_GRASP_PREP_POSE)
        else:
            name = f'{ctrl.side}_grasp_prep_pose'
            default_pose = self.config.get('grasp_prep_pose', DEFAULT_GRASP_PREP_POSE)
        return validate_position_vector(self.config.get(name, default_pose), name)


    def _grasp_poses(self, ctrl, mode: str = 'side') -> tuple[list[int], list[int]]:
        prep = self._grasp_prep_pose(ctrl, mode=mode)
        lower = list(prep)
        if mode == 'top':
            # 垂直向下抓握（手心向地）：
            # 大拇指：仅指根收紧（pitch 0 往 0 收拢），大拇指尖（tip 15）锁定保持 255 伸直，避免碰撞食指
            lower[0] = 0
            lower[15] = 255
            # 其余四指：指根 pitch 与指尖 tip 同步收拢（下限为 0）
            for joint in (*range(1, 5), *range(16, 20)):
                lower[joint] = 0
            # 五指侧摆（5..9）和拇指旋转（10）自始至终保持准备姿态不变
        else:
            # 水平侧向 U 型抓握：五指根弯与尖弯均可向 0 收拢
            for joint in (*range(5), *range(15, 20)):
                lower[joint] = 0
        return prep, lower


    def _do_grasp(self, client, ctrl, options: dict) -> dict:
        mode = options['grasp_mode']
        prep, target = self._grasp_poses(ctrl, mode)
        self._check_stop()
        with ctrl._motion_lock:
            self._check_stop()
            client.set_speeds([options['approach_speed']] * 5)
            client.set_currents([self.motion_current] * 5)
        self._open_hand(client, ctrl)
        self._move_pose(client, ctrl, prep, '抓握准备姿态')
        self._phase('closing')
        with ctrl._motion_lock:
            self._check_stop()
            client.set_joint_positions(target)
        # Do not wait for loaded joints to reach zero: obstruction is expected.
        self._check_stop()
        label = '向下抓握' if mode == 'top' else '侧向抓握'
        return {'message': f'{label}指令已下发，保持至 release',
                'state': 'completed', 'action': options['action'], 'grasp_mode': mode,
                'command_sent': True, 'holding': True, 'speed_ratio': options['speed_ratio'],
                'approach_speed': options['approach_speed'],
                'target_positions': vector20_to_named16(target)}

    def _do_release(self, client, ctrl, mode: str = 'side') -> dict:
        self._phase('releasing')
        prep = self._grasp_prep_pose(ctrl, mode=mode)
        target = validate_position_vector(ctrl._open_pose, 'open_pose')
        with ctrl._motion_lock:
            measured = client.get_positions_vector()
            self._check_stop()
            # Release is idempotent based on fresh feedback, not a cached last
            # action: other cards may have moved this shared hand since then.
            indices = [i for i in range(20) if i not in (11, 12, 13, 14)]
            if all(abs(measured[i] - target[i]) <= self.position_tolerance for i in indices):
                client.set_currents([0] * 5)
                return {'message': '已处于释放姿态，已卸力，无需重复运动',
                        'state': 'completed', 'action': 'release',
                        'positions': vector20_to_named16(measured), 'already_released': True,
                        'hand_opened': True, 'torque_released': True, 'holding': False}
            client.set_speeds([self.release_speed] * 5)
            client.set_currents([self.motion_current] * 5)
        # Reverse the grasp geometry using the same full calibrated preparation:
        # restore all fingers together before leaving the opposition.
        prep_msg = '五指同步恢复向下准备姿态并松开物体' if mode == 'top' else '五指同步恢复 U 型准备姿态并松开物体'
        self._move_pose(client, ctrl, prep, prep_msg)
        measured = self._move_pose(client, ctrl, target, '释放后恢复张开姿态')
        with ctrl._motion_lock:
            self._check_stop()
            client.set_currents([0] * 5)
        return {'message': '已成功释放：五指恢复张开姿态并卸力',
                'state': 'completed', 'action': 'release', 'positions': vector20_to_named16(measured),
                'hand_opened': True, 'torque_released': True, 'holding': False}


    def _unload_owned(self, ctrl, owner) -> bool:
        """Unload only our existing client; do not reconnect during cleanup."""
        with ctrl._motion_lock:
            with self._lock:
                if vars(ctrl).get('_grasp_owner') != owner:
                    return False
                try:
                    client = vars(ctrl).get('_client')
                    if client is None:
                        return False
                    client.set_currents([0] * 5)
                    ctrl._holding_torque = 0
                    return True
                except Exception:
                    return False
                finally:
                    del ctrl._grasp_owner

    def _acp_callback(self, action_id: str, status: str, result: dict) -> None:
        # Reuse the completion transport with this tool's identity, even when
        # the basic hand card is disabled. No controller access is required.
        HandPlugin._acp_callback(self, action_id, status, result)

    def _new_active(self, device_key, instance_id, action, options, client, side, ctrl, *, predecessor=None):
        return {'action': action, 'action_id': f'grasp_{action}_{uuid4().hex}',
                'instance_id': instance_id, 'device_key': device_key,
                'options': options, 'client': client, 'side': side,
                'controller': ctrl, 'owner': (id(self), device_key), 'predecessor': predecessor,
                'stop_event': threading.Event(), 'done_event': threading.Event(),
                'terminal': None}

    def _run_action(self, device_key: tuple[str, str], active: dict) -> None:
        client, ctrl = active['client'], active['controller']
        self._worker_context.active = active
        self._worker_context.stop_event = active['stop_event']
        try:
            predecessor = active.get('predecessor')
            while predecessor is not None and not predecessor['done_event'].wait(0.02):
                self._check_stop()
            self._check_stop()
            with ctrl._motion_lock:
                ensure_motion_available(ctrl, active['owner'])
                verify_hand_connection(client)
            if active['action'] == 'release':
                result = self._do_release(client, ctrl, active['options']['grasp_mode'])
            else:
                result = self._do_grasp(client, ctrl, active['options'])
            self._check_stop()
            status = 'completed'
        except InterruptedError as exc:
            status, result = 'cancelled', {'message': str(exc), 'state': 'cancelled', 'holding': False}
        except Exception as exc:
            label = '释放' if active['action'] == 'release' else '抓握指令'
            status, result = 'error', {'message': f'{label}失败：{exc}', 'state': 'error',
                                      'error_code': getattr(exc, 'code', 'device_error'),
                                      'holding': False, 'hand_opened': False, **getattr(exc, 'details', {})}

        # Atomically retain the motor position demand, or hand the lease to an
        # interrupting release. There is no background tactile/holding worker.
        with ctrl._motion_lock:
            with self._lock:
                current = self._active.get(device_key) is active
                successor = device_key in self._active and not current
                if status == 'completed' and (active['stop_event'].is_set() or self._stop.is_set() or not current):
                    status, result = 'cancelled', {'message': 'grasp motion stopped', 'state': 'cancelled', 'holding': False}
                keep = status == 'completed' and active['action'] != 'release'
                if keep:
                    self._sessions[device_key] = {**active['options'],
                                                  'source_instance_id': active['instance_id']}
                elif not successor:
                    self._sessions.pop(device_key, None)
                    result['torque_released'] = self._unload_owned(ctrl, active['owner'])
                if not successor:
                    release_motion(ctrl, active['owner'])
                result = {'bound_side': active['side'], 'can_channel': device_key[1],
                          **result, 'success': status == 'completed'}
                terminal = {'action_id': active['action_id'], 'status': status, 'result': result}
                if current:
                    self._active.pop(device_key, None)
                    self._states[device_key] = 'holding' if keep else 'error' if status == 'error' else 'ready'
                self._results[device_key] = self._last_result = terminal
                active['terminal'] = terminal
                active['done_event'].set()
        # Release the device lease before reporting the terminal result. Every
        # accepted worker reports completion, including errors and cancellation.
        try:
            self._acp_callback(active['action_id'], status, result)
        except Exception as exc:
            self._delivery_failed = True
            print(f"[ACP] {active['action_id']}: {exc}", flush=True)

    def _options(self, device_key: tuple[str, str], action: str, args: dict) -> dict:
        obsolete = {'target_force', 'max_force', 'hold_duration_s', 'slip_threshold', 'active_fingers', 'approach_speed'}
        if obsolete.intersection(args):
            raise ValueError('纯位置抓握已移除参数：' + ', '.join(sorted(obsolete.intersection(args))))
        if action == 'release':
            with self._lock:
                session = self._sessions.get(device_key, {})
                if device_key in self._active:
                    session = self._active[device_key]['options']
                mode = session.get('grasp_mode', 'side')
            return {'action': action, 'grasp_mode': mode}
        ratio = args.get('speed_ratio', '1.0x')
        if not isinstance(ratio, str) or ratio not in SPEED_RATIOS:
            raise ValueError(f"speed_ratio must be one of {', '.join(SPEED_RATIOS)}")
        return {'action': action, 'grasp_mode': 'top' if action == 'top_grasp' else 'side',
                'speed_ratio': ratio, 'approach_speed': min(255, round(85 * SPEED_RATIOS[ratio]))}

    def _action_reply(self, active: dict) -> dict:
        return {'message': '释放已开始' if active['action'] == 'release' else '抓握准备已开始，完成后回报指令下发结果',
                'state': 'running', 'action_id': active['action_id']}

    def _start_action(self, instance_id: str, action: str, args: dict) -> dict:
        side = validate_bound_side(args.get('bound_side'))
        channel = self.global_config.get('can_channel', 'can0')
        device_key = (side, channel)
        owner = (id(self), device_key)
        with self._lock:
            options = self._options(device_key, action, args)
            previous = self._active.get(device_key)
            if previous is not None:
                if action != 'release':
                    raise RuntimeError('当前正在抓握或释放，请先执行 release 后再重新抓取')
                active = previous if previous['action'] == 'release' else None
                client, side, ctrl = previous['client'], previous['side'], previous['controller']
            else:
                if action != 'release' and device_key in self._sessions:
                    raise RuntimeError('当前正在保持抓握，请先执行 release，再重新抓取，避免意外重新开手')
                client, side, ctrl = self._get_client_and_ctrl(side, channel)
                # Never block on another motion while holding the card lock.
                if not ctrl._motion_lock.acquire(blocking=False):
                    raise RuntimeError('该手正在执行其他动作，请等待结束后再抓握')
                try:
                    claim_motion(ctrl, owner, config=self.global_config)
                    verify_hand_connection(client)
                    ctrl._grasp_owner = owner
                except Exception:
                    release_motion(ctrl, owner, restore_previous=True)
                    raise
                finally:
                    ctrl._motion_lock.release()
                active = None
            if active is None:
                active = self._new_active(device_key, instance_id, action, options, client, side, ctrl, predecessor=previous)
                self._stop.clear()
                self._active[device_key] = active
                self._states[device_key] = 'releasing' if action == 'release' else 'preparing'
                if previous is not None:
                    previous['stop_event'].set()
                start_worker = True
            else:
                start_worker = False
        if start_worker:
            try:
                threading.Thread(target=self._run_action, args=(device_key, active), name=active['action_id'], daemon=True).start()
            except Exception as exc:
                with self._lock:
                    if self._active.get(device_key) is active:
                        self._active.pop(device_key, None)
                        self._sessions.pop(device_key, None)
                        self._states[device_key] = 'error'
                self._unload_owned(ctrl, active['owner'])
                release_motion(ctrl, active['owner'])
                active['terminal'] = {'status': 'error', 'result': {'state': 'error', 'message': str(exc), 'holding': False}}
                active['done_event'].set()
                raise
        return self._action_reply(active)

    def dispatch(self, action: str, args: dict) -> dict | None:
        instance_id = args.get('instance_id') or ''
        if action == 'config':
            # Legacy saved card settings are accepted without restoring a binding.
            # Device selection is always mandatory on each actual motion request.
            return {'state': 'configured', 'control_mode': 'position'}
        if action == 'start':
            return {'state': 'ready', 'control_mode': 'position', 'selection_required': True}
        if action == 'info':
            side = args.get('bound_side')
            if side is None or side == '':
                return {'state': 'ready', 'bound_side': None, 'active_action_id': None,
                        'last_result': None, 'control_mode': 'position', 'selection_required': True}
            side = validate_bound_side(side)
            device_key = (side, self.global_config.get('can_channel', 'can0'))
            with self._lock:
                active = self._active.get(device_key)
                return {'state': self._states.get(device_key, 'ready'), 'bound_side': side,
                        'active_action_id': active['action_id'] if active else None,
                        'last_result': self._results.get(device_key), 'control_mode': 'position'}
        if action == 'stop':
            with self._lock:
                side = args.get('bound_side')
                if side is not None and side != '':
                    side = validate_bound_side(side)
                    targets = [(side, self.global_config.get('can_channel', 'can0'))]
                elif instance_id:
                    # Only exact recorded sources are eligible. In particular,
                    # an ID-only lifecycle request cannot identify anonymous work.
                    targets = [key for key in self._controllers
                               if (self._active[key]['instance_id'] == instance_id if key in self._active
                                   else self._sessions.get(key, {}).get('source_instance_id') == instance_id)]
                else:
                    targets = []
                controllers = []
                for key in targets:
                    active = self._active.get(key)
                    if active:
                        active['stop_event'].set()
                    self._sessions.pop(key, None)
                    self._states.pop(key, None)
                    ctrl = self._controllers.get(key)
                    if ctrl is not None:
                        controllers.append((key, ctrl))
            unloaded = False
            for key, ctrl in controllers:
                unloaded = self._unload_owned(ctrl, (id(self), key)) or unloaded
            return {'state': 'idle', 'active_action_id': None, 'torque_released': unloaded,
                    'target_resolved': bool(targets)}
        if action in GRASP_ACTIONS:
            if args.get('confirm_motion') is not True:
                raise ValueError('confirm_motion must be true for every grasp motion request')
            wait = args.get('wait_for_result', False)
            if not isinstance(wait, bool):
                raise ValueError('wait_for_result 必须为布尔值')
            if wait:
                raise ValueError('抓握动作使用异步完成协议，不支持 wait_for_result=true；请通过 action_id 接收 ACP 完成通知')
            return self._start_action(instance_id, action, args)
        return None
