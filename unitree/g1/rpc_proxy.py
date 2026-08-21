"""
rpc_proxy.py — Subprocess proxy for G1 LocoClient RPC calls.

The driver process has many threads (ROS2 executor, camera, mic, lidar, etc.)
causing GIL contention. CycloneDDS listener callbacks get starved, making RPC
responses arrive late or timeout. Running LocoClient in a subprocess avoids this.

Modeled after R1's rpc_proxy.py with G1-specific adaptations.
"""

import multiprocessing
import threading
import time


def _rpc_worker(cmd_queue: multiprocessing.Queue, result_queue: multiprocessing.Queue,
                network_iface: str):
    """Subprocess: holds dedicated LocoClient, processes commands sequentially."""
    # Spawned child: fresh interpreter, does not inherit the parent's sys.stdout.
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except ImportError:
        pass

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    ChannelFactoryInitialize(0, network_iface)

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    time.sleep(0.5)
    print("[G1 RpcWorker] ready", flush=True)

    while True:
        try:
            cmd = cmd_queue.get()
        except Exception:
            break
        if cmd is None:
            break

        method = cmd.get("method")
        args = cmd.get("args", [])
        kwargs = cmd.get("kwargs", {})

        try:
            # Special: FSM sequence execution (runs entirely in subprocess, no GIL)
            if method == "__run_fsm_sequence":
                steps_spec, interval, step_timeout, settle_delay = args
                completed = []
                for method_name, target_fsm, step_name in steps_spec:
                    fn = getattr(loco, method_name)
                    ret = fn()
                    if ret != 0:
                        result_queue.put({"result": {
                            "error": f"Step '{step_name}' failed: code={ret}",
                            "step": step_name, "completed": completed}})
                        break  # abort sequence on failure
                    # Poll FSM until target reached or timeout
                    elapsed = 0.0
                    ok = False
                    while elapsed < step_timeout:
                        time.sleep(interval)
                        elapsed += interval
                        code, fsm_id = loco.GetFsmId()
                        if code == 0 and fsm_id == target_fsm:
                            ok = True
                            break
                    if not ok:
                        _, current = loco.GetFsmId()
                        result_queue.put({"result": {
                            "error": f"Timeout '{step_name}' (expected={target_fsm}, got={current})",
                            "step": step_name, "fsm_id": current, "completed": completed}})
                        break  # abort sequence on timeout
                    completed.append(step_name)
                    # Wait for physical motion to settle before next step
                    time.sleep(settle_delay)
                else:
                    # Only reached if loop completed without break (all steps succeeded)
                    result_queue.put({"result": {"ret": 0, "steps": completed,
                                                 "fsm_id": steps_spec[-1][1]}})
                continue  # next cmd

            fn = getattr(loco, method)
            result = fn(*args, **kwargs)
            result_queue.put({"result": result})
        except Exception as e:
            result_queue.put({"error": str(e)})


class RpcProxy:
    """Proxy that forwards LocoClient RPC calls to a subprocess, avoiding GIL contention."""

    def __init__(self, network_iface: str = "eth0"):
        ctx = multiprocessing.get_context("spawn")
        self._cmd_q = ctx.Queue()
        self._result_q = ctx.Queue()
        self._proc = ctx.Process(
            target=_rpc_worker,
            args=(self._cmd_q, self._result_q, network_iface),
            daemon=True,
        )
        self._proc.start()
        self._lock = threading.Lock()

    def _call(self, method: str, *args, timeout: float = 15.0, **kwargs):
        with self._lock:
            self._cmd_q.put({"method": method, "args": args, "kwargs": kwargs})
            try:
                r = self._result_q.get(timeout=timeout)
            except Exception:
                return None
            if "error" in r:
                print(f"[G1 RpcProxy] {method} error: {r['error']}", flush=True)
                return None
            return r["result"]

    def _call_code(self, method: str, *args, **kwargs) -> int:
        """For methods that return a single int code."""
        result = self._call(method, *args, **kwargs)
        if result is None:
            return 3104
        return result

    def _call_tuple(self, method: str, *args, **kwargs):
        """For methods that return (code, data) tuple."""
        result = self._call(method, *args, **kwargs)
        if result is None:
            return 3104, None
        return result

    def stop(self):
        try:
            self._cmd_q.put(None)
            self._proc.join(timeout=3)
        except Exception:
            pass

    # ── LocoClient interface ──────────────────────────────────────────────────

    def RunFsmSequence(self, steps: list, interval: float = 1.0, step_timeout: float = 15.0,
                       settle_delay: float = 2.0):
        """Run FSM sequence entirely in subprocess (no GIL contention).
        steps = [(method_name, target_fsm_to_poll, step_name), ...]
        settle_delay = seconds to wait after FSM confirms state change.
        Returns dict with {ret, steps, fsm_id} on success or {error, step} on failure."""
        outer_timeout = len(steps) * (step_timeout + settle_delay + 5) + 10
        return self._call("__run_fsm_sequence", steps, interval, step_timeout, settle_delay,
                          timeout=outer_timeout)

    def GetFsmId(self):
        return self._call_tuple("GetFsmId")

    def GetFsmMode(self):
        return self._call_tuple("GetFsmMode")

    def GetBalanceMode(self):
        return self._call_tuple("GetBalanceMode")

    def GetSwingHeight(self):
        return self._call_tuple("GetSwingHeight")

    def GetStandHeight(self):
        return self._call_tuple("GetStandHeight")

    def GetPhase(self):
        return self._call_tuple("GetPhase")

    def SetFsmId(self, fsm_id: int):
        return self._call_code("SetFsmId", fsm_id)

    def SetBalanceMode(self, balance_mode: int):
        return self._call_code("SetBalanceMode", balance_mode)

    def SetStandHeight(self, stand_height: float):
        return self._call_code("SetStandHeight", stand_height)

    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        return self._call_code("SetVelocity", vx, vy, omega, duration)

    def SetTaskId(self, task_id: float):
        return self._call_code("SetTaskId", task_id)

    def Damp(self):
        return self._call_code("Damp")

    def Start(self):
        return self._call_code("Start")

    def Lie2StandUp(self):
        return self._call_code("Lie2StandUp")

    def StandUp2Squat(self):
        return self._call_code("StandUp2Squat")

    def Squat2StandUp(self):
        return self._call_code("Squat2StandUp")

    def Sit(self):
        return self._call_code("Sit")

    def ZeroTorque(self):
        return self._call_code("ZeroTorque")

    def StopMove(self):
        return self._call_code("StopMove")

    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
        return self._call_code("Move", vx, vy, vyaw, continous_move)

    def HighStand(self):
        return self._call_code("HighStand")

    def LowStand(self):
        return self._call_code("LowStand")

    def BalanceStand(self, balance_mode: int):
        return self._call_code("BalanceStand", balance_mode)

    def ContinuousGait(self, flag: bool):
        return self._call_code("ContinuousGait", flag)

    def WaveHand(self, turn_flag: bool = False):
        return self._call_code("WaveHand", turn_flag)

    def ShakeHand(self, stage: int = -1):
        return self._call_code("ShakeHand", stage)
