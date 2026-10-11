#!/usr/bin/env python3
"""Small, guarded PiPER driver for the tested single-arm CAN setup.

Joint motion is limited to J1 and 12 degrees per command. The arm stays
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


class PiperDriver:
    def __init__(self, can_name="can0"):
        self.can_name = can_name
        self.arm = None
        self.cancelled = threading.Event()
        self._send_lock = threading.RLock()

    def connect(self):
        from piper_sdk import C_PiperInterface_V2

        self.arm = C_PiperInterface_V2(self.can_name, judge_flag=False)
        self.arm.ConnectPort(piper_init=False)
        time.sleep(0.4)
        try:
            self.get_state()
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
        if any(abs(now - stamp) > 0.5 for stamp in (joints.time_stamp, status.time_stamp, motors.time_stamp)):
            raise RuntimeError("CAN 关节、状态或电机反馈超过 0.5 秒未更新")
        msg = status.arm_status
        raw = tuple(getattr(joints.joint_state, f"joint_{i}") for i in range(1, 7))
        if any(getattr(motors, f"motor_{i}").can_id == 0 for i in range(1, 7)):
            raise RuntimeError("尚未收到全部六个电机的状态反馈")
        enabled = tuple(self.arm.GetArmEnableStatus())
        state = ArmState(
            tuple(value / 1000 for value in raw), enabled,
            str(msg._ctrl_mode), str(msg._arm_status),
            str(msg._teach_status), int(msg._err_code)
        )
        return raw, state, int(msg._arm_status)

    def get_state(self):
        """Return fresh feedback, without sending a movement command."""
        return self._raw_state()[1]

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
        if status_code != 0 or state.error_code != 0:
            raise RuntimeError(f"机械臂状态异常: {state}")
        return raw, state

    def cancel_motion(self):
        """Prevent further trajectory commands; caller waits for best-effort hold."""
        with self._send_lock:
            self.cancelled.set()

    def close(self):
        """Close CAN threads without changing motor enable state."""
        if self.arm is not None:
            self.arm.DisconnectPort()
            self.arm = None

    def _check_cancelled(self):
        if self.cancelled.is_set():
            raise RuntimeError("Motion cancelled; motor torque is not released")

    def _command(self, raw_target, speed_pct):
        with self._send_lock:
            self._check_cancelled()
            _, feedback = self._healthy()
            if not feedback.control_mode.startswith(("STANDBY", "CAN_CTRL")):
                raise RuntimeError("Another controller owns the arm")
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
                        and any(state.enabled)
                        and max(abs(a-b) for a, b in zip(actual, bounded)) <= 3000):
                    self.arm.MotionCtrl_2(0x01, 0x01, speed_pct, 0x00)
                    self.arm.JointCtrl(*bounded)
        except Exception:
            pass  # Feedback/CAN failure means holding cannot be guaranteed.

    def _bounded_target(self, raw):
        """Expose the same limits the installed SDK otherwise applies silently."""
        target = []
        for i, value in enumerate(raw, 1):
            low, high = self.arm.GetSDKJointLimitParam(f"j{i}")
            low, high = round(math.degrees(low) * 1000), round(math.degrees(high) * 1000)
            if low >= high:
                raise RuntimeError(f"J{i} 的 SDK 限位无效")
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
        if not state.teach_status.startswith("DISABLED"):
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
                self._command(target, speed_pct)
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
                if all(abs(a-t) <= 1000 for a,t in zip(actual,target)):
                    if near_since is None:
                        near_since = time.monotonic()
                    elif time.monotonic() - near_since >= 0.5:
                        return {**plan, "executed": True, "state": asdict(feedback)}
                else:
                    near_since = None
                time.sleep(0.05)
            raise RuntimeError("初始化未在 5 秒内稳定到目标 ±1°")
        except BaseException:
            self._hold_on_error(speed_pct)
            raise

    def hold_current(self, seconds=0.5, speed_pct=10):
        """Ask an enabled arm to hold its measured pose; motors remain enabled."""
        raw, state = self._healthy()
        if not all(state.enabled):
            raise RuntimeError("六关节尚未全部使能，不能以保持指令代替使能")
        if not state.control_mode.startswith("CAN_CTRL"):
            raise RuntimeError(f"当前处于 {state.control_mode}，hold 会切换控制模式，已拒绝")
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self._command(raw, speed_pct)
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

    def move_j1_to(self, target_deg, speed_pct=10, take_control=False):
        """Move J1 by at most 12 degrees, verify feedback, then hold target."""
        if not 1 <= speed_pct <= 20:
            raise ValueError("速度百分比必须在 1–20")
        if not math.isfinite(target_deg):
            raise ValueError("J1 target must be finite")
        base, state = self._healthy()
        if tuple(base) != self._bounded_target(base):
            raise RuntimeError("Current pose exceeds SDK limits; run prepare first")
        target_raw = round(target_deg * 1000)
        delta = target_raw - base[0]
        if abs(delta) < 100 or abs(delta) > 12000:
            raise ValueError("单次 J1 目标必须距当前角度 0.1–12°")
        if not -140000 <= target_raw <= 140000:
            raise ValueError("J1 目标超出本驱动的 ±140° 范围")
        if any(state.enabled) and not all(state.enabled):
            raise RuntimeError("只有部分关节使能，拒绝运动")
        if not state.control_mode.startswith("CAN_CTRL"):
            raise RuntimeError(f"当前处于 {state.control_mode}，拒绝由 J1 命令切换到 CAN 控制")
        if not all(state.enabled):
            raise RuntimeError("六关节未全部使能，请先运行 prepare 预览并执行初始化")
        if all(state.enabled) and not take_control:
            raise RuntimeError("机械臂已被使能；确认没有其它控制程序后使用 take_control=True")

        target = list(base)
        def command():
            actual, feedback = self._healthy()
            if not all(feedback.enabled) or not feedback.control_mode.startswith("CAN_CTRL"):
                raise RuntimeError("Motion lost enabled CAN control")
            if abs(actual[0] - base[0]) > abs(delta) + 2000:
                raise RuntimeError("J1 overshot the bounded motion envelope")
            if any(abs(actual[k] - base[k]) > 5000 for k in range(1, 6)):
                raise RuntimeError("Another joint moved outside the hold envelope")
            self._command(target, speed_pct)

        def ramp(start, finish, seconds):
            steps = round(seconds / 0.05)
            for i in range(1, steps + 1):
                target[0] = round(start + (finish - start) * i / steps)
                command()
                actual, feedback = self._healthy()
                if not all(feedback.enabled) or not feedback.control_mode.startswith("CAN_CTRL"):
                    raise RuntimeError("Motion lost enabled CAN control")
                if abs(actual[0] - base[0]) > abs(delta) + 2000:
                    raise RuntimeError("J1 实际角度越过目标范围")
                if any(abs(actual[k] - base[k]) > 5000 for k in range(1, 6)):
                    raise RuntimeError("其它关节偏移超过 5°")
                time.sleep(0.05)

        try:
            for _ in range(10):
                command()
                time.sleep(0.05)
            first = base[0] + (1000 if delta > 0 else -1000)
            if abs(delta) < 1000:
                first = target_raw
            ramp(base[0], first, 3)
            until = time.monotonic() + 2
            while time.monotonic() < until:
                command()
                time.sleep(0.05)
            actual, _ = self._healthy()
            if abs(actual[0] - base[0]) < max(100, abs(first - base[0]) * 0.3):
                raise RuntimeError("首段动作没有得到足够的关节反馈")

            if first != target_raw:
                ramp(first, target_raw, max(3, abs(target_raw - first) / 1000))
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                command()
                actual, feedback = self._healthy()
                if not all(feedback.enabled) or not feedback.control_mode.startswith("CAN_CTRL"):
                    raise RuntimeError("Motion lost enabled CAN control")
                if abs(actual[0] - target_raw) <= 500:
                    return self.get_state()
                time.sleep(0.05)
            raise RuntimeError("5 秒内未到达 J1 目标 ±0.5°")
        except BaseException:
            self._hold_on_error(speed_pct)
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("status", "prepare", "hold", "move-j1-to", "disable",
                                            "home-check", "home-guide"))
    parser.add_argument("--can", default="can0")
    parser.add_argument("--target-deg", type=float)
    parser.add_argument("--speed", type=int, default=10)
    parser.add_argument("--execute", action="store_true", help="执行 prepare；默认仅预览")
    parser.add_argument("--workspace-clear", action="store_true", help="现场有人、底座固定、活动范围清空")
    parser.add_argument("--allow-limit-adjustment", action="store_true", help="允许预览显示的不超过 3° 的 SDK 边界修正")
    parser.add_argument("--take-control", action="store_true",
                        help="确认当前使能无人控制后接管")
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
        else:
            if args.target_deg is None:
                parser.error("move-j1-to 需要 --target-deg")
            state = driver.move_j1_to(args.target_deg, args.speed, args.take_control)
    except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
        raise SystemExit(str(exc)) from exc
    print(json.dumps(asdict(state) if isinstance(state, ArmState) else state,
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
