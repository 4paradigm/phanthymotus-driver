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
                for step in steps_spec:
                    # step = (method_name, targets, step_name[, timeout])
                    # `targets` is a single FSM id or an iterable of acceptable ids —
                    # several transitions settle into one of a few states (e.g. 躺起
                    # lands on 702 and is then pushed on to 500 by the controller).
                    method_name, targets, step_name = step[0], step[1], step[2]
                    this_timeout = step[3] if len(step) > 3 else step_timeout
                    targets = {targets} if isinstance(targets, int) else set(targets)
                    fn = getattr(loco, method_name)
                    ret = fn()
                    if ret != 0:
                        result_queue.put({"result": {
                            "error": f"Step '{step_name}' failed: code={ret}",
                            "step": step_name, "completed": completed}})
                        break  # abort sequence on failure
                    # Poll FSM until one of the target states is reached or timeout
                    elapsed = 0.0
                    ok = False
                    while elapsed < this_timeout:
                        time.sleep(interval)
                        elapsed += interval
                        code, fsm_id = loco.GetFsmId()
                        if code == 0 and fsm_id in targets:
                            ok = True
                            break
                    if not ok:
                        _, current = loco.GetFsmId()
                        result_queue.put({"result": {
                            "error": f"Timeout '{step_name}' after {this_timeout:.0f}s "
                                     f"(expected={sorted(targets)}, got={current})",
                            "step": step_name, "fsm_id": current, "completed": completed}})
                        break  # abort sequence on timeout
                    completed.append(step_name)
                    # Wait for physical motion to settle before next step
                    time.sleep(settle_delay)
                else:
                    # Only reached if loop completed without break (all steps succeeded)
                    _, final = loco.GetFsmId()
                    result_queue.put({"result": {"ret": 0, "steps": completed,
                                                 "fsm_id": final}})
                continue  # next cmd

            # Health check: lightweight GetFsmId round-trip proves both send and
            # recv DDS channels are alive. A publisher-context-invalid failure makes
            # the write return False before any request is sent, so this catches it
            # immediately — well before a real move/FSM call times out.
            if method == "health_check":
                result_queue.put({"result": _health_check(loco)})
                continue

            fn = getattr(loco, method)
            result = fn(*args, **kwargs)
            result_queue.put({"result": result})
        except Exception as e:
            result_queue.put({"error": str(e)})


def _health_check(loco) -> dict:
    """Lightweight DDS health probe via GetFsmId round-trip.

    Returns {'healthy': bool, ...}. A successful round-trip proves both the
    send channel (request went out) and recv channel (response came back)
    are alive.
    """
    try:
        code, fsm_id = loco.GetFsmId()
        return {"healthy": code == 0, "fsm_code": code, "fsm_id": fsm_id}
    except Exception as e:
        return {"healthy": False, "error": f"{type(e).__name__}: {e}"}


class RpcProxy:
    """Proxy that forwards LocoClient RPC calls to a subprocess, avoiding GIL contention.

    Auto-recovery: a watchdog thread probes the subprocess every 15s via a
    lightweight GetFsmId round-trip. If the process dies or three consecutive
    health probes fail, the worker is terminated and respawned with a fresh
    DDS context — this is the only reliable way to recover from a CycloneDDS
    "publisher context invalid" failure, because ChannelFactory is a Singleton
    that cannot be re-initialized within the same process.
    """

    # Watchdog tuning
    HEALTH_INTERVAL_S = 15.0   # seconds between health probes
    HEALTH_TIMEOUT_S = 5.0      # per-probe timeout
    UNHEALTHY_RESTART_S = 45.0  # restart after this long with no successful probe
    RESTART_COOLDOWN_S = 10.0   # min seconds between restarts (avoid restart loop)

    def __init__(self, network_iface: str = "eth0"):
        self._network_iface = network_iface
        self._restart_lock = threading.Lock()
        self._restart_count = 0
        self._last_healthy = time.time()
        self._last_restart = 0.0
        self._start_worker()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog, daemon=True, name="rpc_proxy_watchdog")
        self._watchdog_thread.start()
        print(f"[RpcProxy] worker started (pid={self._proc.pid}), watchdog active", flush=True)

    def _start_worker(self):
        """(Re)create queues + subprocess. Caller must hold _restart_lock or be in __init__."""
        ctx = multiprocessing.get_context("spawn")
        self._cmd_q = ctx.Queue()
        self._result_q = ctx.Queue()
        self._proc = ctx.Process(
            target=_rpc_worker,
            args=(self._cmd_q, self._result_q, self._network_iface),
            daemon=True,
        )
        self._proc.start()
        self._lock = threading.Lock()

    def _restart(self, reason: str):
        """Terminate and respawn the worker with a fresh DDS context. Thread-safe."""
        with self._restart_lock:
            now = time.time()
            if now - self._last_restart < self.RESTART_COOLDOWN_S:
                print(f"[RpcProxy] restart suppressed (cooldown, last={now-self._last_restart:.1f}s ago)", flush=True)
                return
            self._last_restart = now
            self._restart_count += 1
            print(f"[RpcProxy] RESTART worker #{self._restart_count} (reason={reason})", flush=True)
            if self._proc.is_alive():
                try:
                    self._proc.terminate()
                    self._proc.join(timeout=3)
                except Exception:
                    pass
                if self._proc.is_alive():
                    self._proc.kill()
                    self._proc.join(timeout=2)
            self._start_worker()
            self._last_healthy = time.time()
            print(f"[RpcProxy] worker respawned (pid={self._proc.pid})", flush=True)

    def _watchdog(self):
        """Background: check process liveness + DDS health, restart on failure."""
        while True:
            time.sleep(self.HEALTH_INTERVAL_S)
            # 1. Process liveness
            if not self._proc.is_alive():
                self._restart("process died")
                continue
            # 2. DDS health probe
            try:
                result = self._call("health_check", timeout=self.HEALTH_TIMEOUT_S)
                if result and isinstance(result, dict) and result.get("healthy"):
                    self._last_healthy = time.time()
                    continue
                print(f"[RpcProxy] watchdog probe unhealthy: {result}", flush=True)
            except Exception as e:
                print(f"[RpcProxy] watchdog probe error: {e}", flush=True)
            # 3. Restart if unhealthy for too long
            if time.time() - self._last_healthy > self.UNHEALTHY_RESTART_S:
                self._restart(f"no healthy probe for {self.UNHEALTHY_RESTART_S:.0f}s")

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

    def RunFsmSequence(self, steps: list, interval: float = 1.0, step_timeout: float = 30.0,
                       settle_delay: float = 2.0):
        """Run FSM sequence entirely in subprocess (no GIL contention).
        steps = [(method_name, targets, step_name[, timeout]), ...]
        `targets` is an FSM id or an iterable of acceptable ids; the optional 4th
        element overrides step_timeout for that step.
        settle_delay = seconds to wait after FSM confirms state change.
        Returns dict with {ret, steps, fsm_id} on success or {error, step} on failure."""
        budget = sum((s[3] if len(s) > 3 else step_timeout) + settle_delay + 5 for s in steps)
        outer_timeout = budget + 10
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
