#!/usr/bin/env python3
"""Small, guarded PiPER driver for the tested single-arm CAN setup.

Joint motion is limited to 12 degrees per axis per command. The arm stays
enabled after a successful move; use ``disable`` only when it is supported.
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
import time
import threading


@dataclass(frozen=True)
class ArmState:
    joints_deg: tuple[float, ...]
    enabled: tuple[bool, ...]
    control_mode: str
    arm_status: str
    teach_status: str
    error_code: int
    feedback_time: float = 0.0
    motor_faults: tuple[str, ...] = ()


@dataclass(frozen=True)
class GripperState:
    width_mm: float
    effort_nm: float
    enabled: bool
    error_code: int
    feedback_time: float


class PiperDriver:
    _sdk_type = None
    def __init__(self, can_name="can0"):
        self.can_name = can_name
        self.arm = None
        self.cancelled = threading.Event()
        self._send_lock = threading.RLock()
        self.joint_limits_deg = None
        self.gripper_limits_mm = (0., 70.)
        self.can_transport = "socketcan"
        self.usb_index = 0
        self.last_transport_diagnostics = {}

    def connect(self):
        from piper_sdk import C_PiperInterface_V2

        class ObservedInterface(C_PiperInterface_V2):
            # SDK aggregate timestamps can hide one stale joint/motor frame.
            def __init__(self, *args, **kwargs):
                if not hasattr(self, "frame_times"):
                    self.frame_times = {}
                    self.frame_counts = {}
                    self.frame_lock = threading.Lock()
                super().__init__(*args, **kwargs)

            def ParseCANFrame(self, frame):
                result = super().ParseCANFrame(frame)
                if frame is not None:
                    with self.frame_lock:
                        self.frame_times[frame.arbitration_id] = time.time()
                        self.frame_counts[frame.arbitration_id] = self.frame_counts.get(frame.arbitration_id,0)+1
                return result

        if PiperDriver._sdk_type is None:
            PiperDriver._sdk_type = ObservedInterface
        self.arm = PiperDriver._sdk_type(self.can_name, judge_flag=False,can_auto_init=self.can_transport == "socketcan")
        receiver = getattr(self.arm, "_can_deal_th", None)
        if receiver is not None and receiver.is_alive():
            self.arm = None
            raise RuntimeError("CAN 接收线程仍在运行，请先关闭现有连接")
        if self.can_transport == "gs_usb":
            from agilex.piper.mac_can import port
            self.arm._arm_can = port(self.arm.ParseCANFrame,self.usb_index)
        elif self.can_transport != "socketcan":
            raise ValueError("can_transport 必须为 socketcan 或 gs_usb")
        with self.arm.frame_lock:
            self.arm.frame_times.clear()
            self.arm.frame_counts.clear()
        self.arm.ConnectPort(can_init=True, piper_init=False)
        try:
            deadline = time.monotonic() + 2
            while True:
                try:
                    self.get_state()
                    break
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.05)
        except Exception:
            self.close()
            raise
        return self

    def _raw_state(self):
        if self.arm is None:
            raise RuntimeError("请先 connect()")
        joints = self.arm.GetArmJointMsgs()
        status = self.arm.GetArmStatus()
        motors = self.arm.GetArmLowSpdInfoMsgs()
        now = time.time()
        required = (0x2A1, 0x2A5, 0x2A6, 0x2A7, *range(0x261, 0x267))
        frame_times = None
        if hasattr(self.arm, "frame_times"):
            with self.arm.frame_lock:
                frame_times = [self.arm.frame_times.get(i, 0.) for i in required]
            if any(abs(now-stamp) > .5 for stamp in frame_times):
                raise RuntimeError("某个关节或电机 CAN 帧超过 0.5 秒未更新")
        if any(abs(now - stamp) > 0.5 for stamp in (joints.time_stamp, status.time_stamp, motors.time_stamp)):
            raise RuntimeError("CAN 关节、状态或电机反馈超过 0.5 秒未更新")
        msg = status.arm_status
        raw = tuple(getattr(joints.joint_state, f"joint_{i}") for i in range(1, 7))
        if any(getattr(motors, f"motor_{i}").can_id == 0 for i in range(1, 7)):
            raise RuntimeError("尚未收到全部六个电机的状态反馈")
        enabled = tuple(self.arm.GetArmEnableStatus())
        faults = tuple(f"J{i}:{flag}" for i in range(1,7) for flag in
                       ("voltage_too_low", "motor_overheating", "driver_overcurrent", "driver_overheating",
                        "collision_status", "driver_error_status", "stall_status")
                       if getattr(getattr(motors,f"motor_{i}").foc_status,flag))
        state = ArmState(
            tuple(value / 1000 for value in raw), enabled,
            str(msg._ctrl_mode), str(msg._arm_status),
            str(msg._teach_status), int(msg._err_code),
            min(frame_times) if frame_times else min(joints.time_stamp, status.time_stamp, motors.time_stamp), faults
        )
        return raw, state, int(msg._arm_status)

    def get_state(self):
        """Return fresh feedback, without sending a movement command."""
        return self._raw_state()[1]

    def firmware(self, query=False):
        """Only explicit query sends an information request, never a control frame."""
        if query:
            self.arm.SearchPiperFirmwareVersion()
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                self._check_cancelled()
                version = self.arm.GetPiperFirmwareVersion()
                if isinstance(version, str) and len(version) == 8:
                    return version
                time.sleep(.05)
            return None
        version = self.arm.GetPiperFirmwareVersion()
        return version if isinstance(version, str) and len(version) == 8 else None

    def get_gripper(self):
        msg = self.arm.GetArmGripperMsgs()
        if abs(time.time() - msg.time_stamp) > .5:
            raise RuntimeError("夹爪反馈超过 0.5 秒未更新")
        state = msg.gripper_state
        return GripperState(state.grippers_angle / 1000, state.grippers_effort / 1000,
                            bool(state.foc_status.driver_enable_status),
                            int(state.status_code) & 0x3f, msg.time_stamp)

    def gripper_range(self):
        low, high = (v * 1000 for v in self.arm.GetSDKGripperRangeParam())
        low, high = max(low, self.gripper_limits_mm[0]), min(high, self.gripper_limits_mm[1])
        if not all(map(math.isfinite, (low, high))) or low >= high:
            raise RuntimeError("夹爪的 SDK 和部署限位没有有效交集")
        return low, high

    def factory_zero_status(self, tolerance_deg=2.0):
        """Compare fresh joint feedback with the SDK's six-joint 0° home pose.

        This does not calibrate joint zeros or prove that physical index marks
        are aligned. It never sends a CAN control command.
        """
        if not 0 < tolerance_deg <= 5:
            raise ValueError("零位容差必须大于 0° 且不超过 5°")
        state = self.get_state()
        distances = tuple(abs(value) for value in state.joints_deg)
        return {
            "joints_deg": state.joints_deg,
            "distance_to_zero_deg": distances,
            "within_tolerance": tuple(value <= tolerance_deg for value in distances),
            "feedback_near_zero": all(value <= tolerance_deg for value in distances),
            "tolerance_deg": tolerance_deg,
            "control_mode": state.control_mode,
            "arm_status": state.arm_status,
            "teach_status": state.teach_status,
            "error_code": state.error_code,
            "physical_marks_verified": False,
        }

    def guide_factory_zero(self, seconds=180, tolerance_deg=2.0):
        """Read-only guidance while a person moves the arm in teach mode.

        Success means feedback stayed near six numeric zeros for two seconds;
        the operator must separately verify the physical factory zero marks.
        """
        if not 5 <= seconds <= 600:
            raise ValueError("引导时间必须在 5–600 秒之间")
        self.factory_zero_status(tolerance_deg)
        _, state = self._healthy()
        if not state.control_mode.startswith("TEACHING_MODE"):
            raise RuntimeError(f"当前处于 {state.control_mode}，仅在拖动示教模式下引导回零")
        print("请现场扶稳机械臂，缓慢拖动到厂家零位；本程序只读取反馈，不发送运动命令。", flush=True)
        deadline = time.monotonic() + seconds
        near_since = None
        next_report = 0.0
        while time.monotonic() < deadline:
            _, state = self._healthy()
            if not state.control_mode.startswith("TEACHING_MODE"):
                raise RuntimeError(f"模式已变为 {state.control_mode}，已停止零位引导")
            report = self.factory_zero_status(tolerance_deg)
            now = time.monotonic()
            if now >= next_report:
                angles = ", ".join(f"J{i}={angle:+.1f}°" for i, angle in enumerate(state.joints_deg, 1))
                print(angles, flush=True)
                next_report = now + 1
            if report["feedback_near_zero"]:
                if near_since is None:
                    near_since = now
                elif now - near_since >= 2:
                    print("六关节反馈已连续 2 秒接近 0°；请再核对机身零位标记。", flush=True)
                    return report
            else:
                near_since = None
            time.sleep(0.1)
        raise RuntimeError("引导超时，未确认六关节反馈到达零位；机械臂控制模式未被更改")

    def _healthy(self):
        raw, state, status_code = self._raw_state()
        if status_code != 0 or state.error_code != 0 or state.motor_faults:
            raise RuntimeError(f"机械臂状态异常: {state}")
        return raw, state

    def cancel_motion(self):
        """Prevent further trajectory commands; caller waits for best-effort hold."""
        with self._send_lock:
            self.cancelled.set()

    def close(self):
        """Close CAN threads without changing motor enable state."""
        if self.arm is not None:
            if hasattr(self.arm,"frame_times"):
                with self.arm.frame_lock:
                    self.last_transport_diagnostics = {"receive_counts":{hex(k):v for k,v in self.arm.frame_counts.items()},
                        "tx_frames":list(getattr(self.arm._arm_can,"tx_frames",[]))}
            # The pinned SDK's DisconnectPort does not stop its monitor thread.
            monitor_stop = getattr(self.arm,"_can_monitor_stop_event",None)
            if monitor_stop is not None:
                monitor_stop.set()
            read_stop = getattr(self.arm,"_read_can_stop_event",None)
            if read_stop is not None:
                read_stop.set()
            receive_thread = getattr(self.arm,"_can_deal_th",None)
            # SDK 1.0.0 checks 'can_deal_th' without its leading underscore,
            # so its own DisconnectPort skips the join. Stop the reader before
            # shutting down the USB handle it is using.
            if receive_thread is not None:
                receive_thread.join(timeout=1)
            self.arm.DisconnectPort(thread_timeout=1)
            if receive_thread is not None and receive_thread.is_alive():
                receive_thread.join(timeout=1)
            monitor_thread = getattr(self.arm,"_can_monitor_th",None)
            if monitor_thread is not None:
                monitor_thread.join(timeout=1)
            if receive_thread is not None and receive_thread.is_alive():
                raise RuntimeError("CAN 接收线程尚未退出，拒绝创建第二个连接")
            self.arm = None

    def _check_cancelled(self):
        if self.cancelled.is_set():
            raise RuntimeError("Motion cancelled; motor torque is not released")

    def _command(self, raw_target, speed_pct, allow_standby=False, expected_enabled=None):
        with self._send_lock:
            self._check_cancelled()
            _, feedback = self._healthy()
            if not feedback.control_mode.startswith(("STANDBY", "CAN_CTRL") if allow_standby else ("CAN_CTRL",)):
                raise RuntimeError("Another controller owns the arm")
            if expected_enabled is not None and tuple(feedback.enabled) != tuple(expected_enabled):
                raise RuntimeError("使能状态已变化，已取消旧目标")
            if tuple(raw_target) != self._bounded_target(raw_target):
                raise RuntimeError("Target exceeds SDK limits; command refused")
            self.arm.MotionCtrl_2(0x01, 0x01, speed_pct, 0x00)
            self.arm.JointCtrl(*raw_target)

    def _hold_on_error(self, speed_pct):
        # Cancellation still allows one fresh-pose hold. Never disable or reset.
        try:
            with self._send_lock:
                actual, state = self._healthy()
                bounded = self._bounded_target(actual)
                if (state.control_mode.startswith("CAN_CTRL")
                        and all(state.enabled) and actual == bounded):
                    self.arm.MotionCtrl_2(0x01, 0x01, speed_pct, 0x00)
                    self.arm.JointCtrl(*bounded)
        except Exception:
            pass  # Feedback/CAN failure means holding cannot be guaranteed.

    def joint_bounds_deg(self):
        """Read the effective SDK/deployment intersection; no CAN command."""
        bounds = []
        for i in range(1, 7):
            low, high = self.arm.GetSDKJointLimitParam(f"j{i}")
            low, high = round(math.degrees(low) * 1000), round(math.degrees(high) * 1000)
            if self.joint_limits_deg is not None:
                deployed_low, deployed_high = self.joint_limits_deg[i-1]
                low, high = max(low, round(deployed_low * 1000)), min(high, round(deployed_high * 1000))
            if low >= high:
                raise RuntimeError(f"J{i} 的 SDK 限位无效")
            bounds.append((low / 1000, high / 1000))
        return tuple(bounds)

    def _bounded_target(self, raw):
        """Apply the same effective limits returned to the monitor."""
        target = []
        for value, (low_deg, high_deg) in zip(raw, self.joint_bounds_deg()):
            low, high = round(low_deg * 1000), round(high_deg * 1000)
            target.append(max(low, min(value, high)))
        return tuple(target)

    def prepare_control(self, execute=False, workspace_clear=False,
                        allow_limit_adjustment=False, speed_pct=5):
        """Plan, or explicitly enter CAN control and enable at the measured pose.

        Never reset, calibrate, command gripper motion, or take over teaching.
        A small SDK-boundary correction is disclosed and separately opted in.
        """
        if not 1 <= speed_pct <= 10:
            raise ValueError("初始化速度必须在 1–10")
        base, state = self._healthy()
        if not state.control_mode.startswith(("STANDBY", "CAN_CTRL")):
            raise RuntimeError(f"当前处于 {state.control_mode}，拒绝初始化接管")
        if any(state.enabled):
            raise RuntimeError("初始化要求六关节全部未使能")
        if not state.teach_status.startswith(("DISABLED", "STOP_RECORDING", "TERMINATE_EXECUTION")):
            raise RuntimeError("示教/轨迹状态未停止，拒绝初始化")
        target = self._bounded_target(base)
        adjustment = tuple((t-b)/1000 for t,b in zip(target,base))
        if max(map(abs, adjustment)) > 3:
            raise RuntimeError("当前角度超出 SDK 限位超过 3°，需先检查实物姿态")
        plan = {"start_deg": state.joints_deg,
                "target_deg": tuple(t/1000 for t in target),
                "boundary_adjustment_deg": adjustment,
                "speed_pct": speed_pct, "executed": False}
        if not execute:
            return plan
        if not workspace_clear:
            raise RuntimeError("执行初始化前需确认现场有人、底座固定、活动范围清空")
        if any(adjustment) and not allow_limit_adjustment:
            raise RuntimeError(f"初始化需要边界修正 {adjustment}°；先预览，再显式允许限位修正")
        print("初始化计划: " + json.dumps(plan, ensure_ascii=False), flush=True)

        def check_active():
            actual, feedback = self._healthy()
            if not feedback.control_mode.startswith("CAN_CTRL"):
                raise RuntimeError("初始化期间 CAN 控制模式丢失")
            if any(abs(a-b) > abs(t-b)+1000 for a,b,t in zip(actual,base,target)):
                raise RuntimeError("初始化期间关节超出预期移动范围")
            return actual, feedback

        try:
            # Confirm CAN mode and preload targets while all motors are disabled.
            for _ in range(10):
                actual, feedback = self._healthy()
                if any(feedback.enabled) or not feedback.control_mode.startswith(("STANDBY", "CAN_CTRL")):
                    raise RuntimeError("预置目标时状态被其它控制源更改")
                if any(abs(a-b) > 500 for a,b in zip(actual,base)):
                    raise RuntimeError("预置期间机械臂被移动，请保持放稳后重新预览")
                self._command(target, speed_pct, allow_standby=True, expected_enabled=(False,) * 6)
                time.sleep(0.05)
            check_active()
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                _, feedback = check_active()
                if all(feedback.enabled):
                    break
                self._command(target, speed_pct)
                with self._send_lock:
                    self._check_cancelled()
                    self.arm.EnablePiper()
                time.sleep(0.05)
            else:
                raise RuntimeError("初始化使能超时；可能部分关节已有保持力，请现场检查")
            deadline = time.monotonic() + 5
            near_since = None
            while time.monotonic() < deadline:
                actual, feedback = check_active()
                if not all(feedback.enabled):
                    raise RuntimeError("初始化保持阶段有电机失能")
                self._command(target, speed_pct)
                if all(abs(a-t) <= 500 for a,t in zip(actual,target)):
                    if near_since is None:
                        near_since = time.monotonic()
                    elif time.monotonic() - near_since >= 0.5:
                        return {**plan, "executed": True, "state": asdict(feedback)}
                else:
                    near_since = None
                time.sleep(0.05)
            raise RuntimeError("初始化未在 5 秒内稳定到目标 ±0.5°")
        except BaseException:
            self._hold_on_error(speed_pct)
            raise

    def hold_current(self, seconds=0.5, speed_pct=10):
        """Ask an enabled arm to hold its measured pose; motors remain enabled."""
        from agilex.piper.control import ready_to_move
        raw, state = self._healthy()
        ready_to_move(state)
        if not all(state.enabled):
            raise RuntimeError("六关节尚未全部使能，不能以保持指令代替使能")
        if not state.control_mode.startswith("CAN_CTRL"):
            raise RuntimeError(f"当前处于 {state.control_mode}，hold 会切换控制模式，已拒绝")
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self._command(raw, speed_pct, expected_enabled=(True,) * 6)
            time.sleep(0.05)
        return self.get_state()

    def disable(self, supported=False):
        """Release motor torque; requires a person to support the arm."""
        if not supported:
            raise RuntimeError("解除使能前必须由现场人员扶稳机械臂")
        self._healthy()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            with self._send_lock:
                self._check_cancelled()
                self.arm.DisablePiper()
            time.sleep(0.05)
            state = self.get_state()
            if not any(state.enabled):
                return state
        raise RuntimeError("3 秒内未确认六关节失能；请现场检查")

    def joint_plan(self, target, speed_deg_s=1., speed_pct=5):
        from agilex.piper.control import ready_to_move
        if (len(target) != 6 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                   or not math.isfinite(v) for v in target)):
            raise ValueError("目标必须是六个有限角度（度）")
        if isinstance(speed_deg_s, bool) or not math.isfinite(speed_deg_s) or not .5 <= speed_deg_s <= 5:
            raise ValueError("关节速度须为 0.5–5°/秒")
        if type(speed_pct) is not int or not 1 <= speed_pct <= 10:
            raise ValueError("SDK 速度上限须为 1–10%")
        raw, state = self._healthy()
        ready_to_move(state)
        raw_target = tuple(round(v * 1000) for v in target)
        if raw != self._bounded_target(raw) or raw_target != self._bounded_target(raw_target):
            raise ValueError("当前姿态或目标超过 SDK 与部署限位交集")
        delta = tuple((b-a)/1000 for a,b in zip(raw, raw_target))
        if max(map(abs, delta)) > 12:
            raise ValueError("每次各轴最多移动 12°，请分段预览")
        # Cubic interpolation has a peak slope of 1.5; scale duration accordingly.
        return {"start_deg": state.joints_deg, "target_deg": tuple(v/1000 for v in raw_target),
                "delta_deg": delta, "speed_deg_s": speed_deg_s, "speed_pct": speed_pct,
                "duration_seconds": 1.5 * max(map(abs, delta)) / speed_deg_s}

    def _move_joint_target(self, target, speed_deg_s=1., speed_pct=5):
        """Internal interpolation for move_joint; other axes hold their feedback."""
        from agilex.piper.control import ready_to_move
        plan = self.joint_plan(target, speed_deg_s, speed_pct)
        start = tuple(round(v * 1000) for v in plan["start_deg"])
        finish = tuple(round(v * 1000) for v in plan["target_deg"])
        duration = plan["duration_seconds"]
        elapsed = 0.
        last = time.monotonic()
        deadline = last + duration + 8
        near_since = None
        commanded = start
        try:
            while time.monotonic() < deadline:
                self._check_cancelled()
                actual, state = self._healthy()
                ready_to_move(state)
                if any(abs(a-c) > 3000 for a,c in zip(actual, commanded)):
                    raise RuntimeError("关节跟随偏差超过 3°，已取消旧目标")
                now = time.monotonic()
                # A paused Python process must never jump across its trajectory.
                elapsed += min(.1, max(0., now-last))
                last = now
                u = min(1., elapsed / duration) if duration else 1.
                blend = u*u*(3-2*u)
                commanded = tuple(round(a+(b-a)*blend) for a,b in zip(start,finish))
                self._command(commanded, speed_pct, expected_enabled=(True,) * 6)
                if u >= 1 and all(abs(a-b) <= 500 for a,b in zip(actual,finish)):
                    if near_since is None:
                        near_since = now
                    elif now-near_since >= .5:
                        return self.get_state()
                else:
                    near_since = None
                time.sleep(.05)
            raise RuntimeError("运动超时，未确认目标 ±0.5° 内稳定 0.5 秒")
        except BaseException:
            self._hold_on_error(speed_pct)
            raise

    def stationary(self):
        first = self.get_state()
        deadline = time.monotonic() + .5
        while time.monotonic() < deadline:
            self._check_cancelled()
            time.sleep(.05)
            state = self.get_state()
            if (state.control_mode != first.control_mode or state.enabled != first.enabled
                    or any(abs(a-b) > .1 for a,b in zip(first.joints_deg,state.joints_deg))):
                return False
        return True

    def _wait_confirmation(self, predicate, sent_at, label):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self._check_cancelled()
            state = self.get_state()
            from agilex.piper.control import healthy
            healthy(state, teaching=True)
            if state.feedback_time > sent_at and predicate(state):
                return {"confirmed": True, "state": asdict(state), "reminder": label}
            time.sleep(.05)
        return {"confirmed": False, "reminder": "未确认切换成功；请读取状态，不要重复点击。"}

    def switch_mode(self, target, expected, verified_teaching=False):
        from agilex.piper.control import signature, mode_name
        with self._send_lock:
            self._check_cancelled()
            state = self.get_state()
            if (signature(state) != signature(expected)
                    or any(abs(a-b) > .5 for a,b in zip(state.joints_deg, expected.joints_deg))):
                raise RuntimeError("执行前状态已变化，请重新预览")
            sent_at = time.time()
            if target in {"standby", "can"} and mode_name(state) != "teaching":
                if any(state.enabled):
                    raise RuntimeError("模式切换要求六轴全部失能")
                self.arm.MotionCtrl_2(0 if target == "standby" else 1, 1, 5, 0)
            elif verified_teaching:
                self.arm.MotionCtrl_1(grag_teach_ctrl=1 if target == "teaching" else 2)
            else:
                raise RuntimeError("此固件示教流程未验证，请使用机身按键")
        return self._wait_confirmation(lambda s: mode_name(s) == target and
                                       (target == "teaching" or mode_name(expected) == "teaching" or not any(s.enabled)), sent_at,
                                       "先检查状态，再按需初始化。")

    def stop_teaching(self, verified_teaching=False):
        # Never send stop-record (which can exit/home on old firmware) blindly.
        from agilex.piper.control import healthy
        state = self.get_state()
        healthy(state, teaching=True)
        from agilex.piper.control import teaching_stopped
        if teaching_stopped(state):
            return {"confirmed": True, "state": asdict(state), "reminder": "录制和回放已停止；未接管控制。"}
        if not verified_teaching or state.teach_status.startswith("START_RECORD"):
            return {"confirmed": False, "button_guidance": True,
                    "reminder": "扶稳后按该固件手册停止录制或回放，再读取状态；未接管控制。"}
        with self._send_lock:
            self._check_cancelled()
            sent_at = time.time()
            # Stop-record also exits teaching in this SDK. Only pause playback
            # here; recording requires the firmware's physical-button workflow.
            self.arm.MotionCtrl_1(grag_teach_ctrl=4)
        return self._wait_confirmation(lambda s: s.control_mode == state.control_mode and teaching_stopped(s),
                                       sent_at, "已停止示教活动；未接管电脑控制。")

    def _gripper_command(self, width, torque, enabled=True):
        from agilex.piper.control import ready_to_move
        with self._send_lock:
            self._check_cancelled()
            ready_to_move(self.get_state())
            grip = self.get_gripper()
            if grip.error_code:
                raise RuntimeError("夹爪有故障，请先检查")
            low, high = self.gripper_range()
            if (not math.isfinite(width) or not low <= width <= high
                    or not math.isfinite(torque) or not 0 < torque <= 1):
                raise ValueError("夹爪开度或力矩超过限位")
            self.arm.GripperCtrl(round(width*1000), round(torque*1000), 1 if enabled else 0, 0)

    def hold_gripper(self, torque=.5):
        grip = self.get_gripper()
        if grip.enabled:
            self._gripper_command(grip.width_mm, torque)
        return self.get_gripper()

    def move_gripper(self, width, torque=.5):
        self._gripper_command(width, torque)
        deadline = time.monotonic() + 5
        near_since = None
        try:
            while time.monotonic() < deadline:
                self._check_cancelled()
                grip = self.get_gripper()
                self._gripper_command(width, torque)
                if grip.enabled and abs(grip.width_mm-width) <= .5:
                    if near_since is None:
                        near_since = time.monotonic()
                    elif time.monotonic()-near_since >= .5:
                        return grip
                else:
                    near_since = None
                time.sleep(.05)
            raise RuntimeError("夹爪未确认到位；可能接触物体，请检查状态")
        except BaseException:
            try:
                with self._send_lock:
                    # One measured-width hold may follow cancellation.
                    from agilex.piper.control import ready_to_move
                    ready_to_move(self.get_state())
                    grip = self.get_gripper()
                    low, high = self.gripper_range()
                    if grip.enabled and not grip.error_code and low <= grip.width_mm <= high:
                        self.arm.GripperCtrl(round(grip.width_mm*1000), round(torque*1000), 1, 0)
            except Exception:
                pass
            raise

    def disable_gripper(self):
        from agilex.piper.control import healthy, mode_name
        grip = self.get_gripper()
        with self._send_lock:
            self._check_cancelled()
            state = self.get_state()
            healthy(state)
            if mode_name(state) not in {"standby", "can"} or grip.error_code:
                raise RuntimeError("请先检查模式与夹爪故障，再解除夹爪使能")
            sent_at = time.time()
            self.arm.GripperCtrl(round(grip.width_mm*1000), 500, 0, 0)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self._check_cancelled()
            grip = self.get_gripper()
            if grip.error_code:
                raise RuntimeError("夹爪失能期间收到故障反馈")
            if not grip.enabled and grip.feedback_time > sent_at:
                return grip
            time.sleep(.05)
        raise RuntimeError("未确认夹爪失能，请读取状态")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("status", "prepare", "hold", "disable",
                                            "home-check", "home-guide"))
    parser.add_argument("--can", default="can0")
    parser.add_argument("--speed", type=int, default=10)
    parser.add_argument("--execute", action="store_true", help="执行 prepare；默认仅预览")
    parser.add_argument("--workspace-clear", action="store_true", help="现场有人、底座固定、活动范围清空")
    parser.add_argument("--allow-limit-adjustment", action="store_true", help="允许预览显示的不超过 3° 的 SDK 边界修正")
    parser.add_argument("--arm-supported", action="store_true",
                        help="现场人员已扶稳机械臂，允许解除使能")
    parser.add_argument("--tolerance-deg", type=float, default=2.0,
                        help="零位检查的每个关节容差，默认 2°")
    parser.add_argument("--seconds", type=int, default=180,
                        help="回零引导最长时间，默认 180 秒")
    args = parser.parse_args()
    try:
        driver = PiperDriver(args.can).connect()
        if args.command == "status":
            state = driver.get_state()
        elif args.command == "prepare":
            state = driver.prepare_control(args.execute, args.workspace_clear,
                                           args.allow_limit_adjustment, args.speed)
        elif args.command == "hold":
            state = driver.hold_current(speed_pct=args.speed)
        elif args.command == "disable":
            state = driver.disable(supported=args.arm_supported)
        elif args.command == "home-check":
            state = driver.factory_zero_status(args.tolerance_deg)
        elif args.command == "home-guide":
            state = driver.guide_factory_zero(args.seconds, args.tolerance_deg)
    except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
        raise SystemExit(str(exc)) from exc
    print(json.dumps(asdict(state) if isinstance(state, ArmState) else state,
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
