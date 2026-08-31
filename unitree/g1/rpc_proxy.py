"""
rpc_proxy.py — Subprocess proxy for G1 LocoClient RPC calls.

The driver process has many threads (ROS2 executor, camera, mic, lidar, etc.)
causing GIL contention. CycloneDDS listener callbacks get starved, making RPC
responses arrive late or timeout. Running LocoClient in a subprocess avoids this.

Modeled after R1's rpc_proxy.py with G1-specific adaptations.
"""

import math
import multiprocessing
import queue
import threading
import time


PROPOSAL_CONTINUOUS_DURATION_SECONDS = 864000.0


class ProposalRpcExecutor:
    """Own one fail-closed velocity-proposal lease inside the RPC worker.

    G1 firmware does not reliably execute sub-second ``duration`` values.  A
    proposal therefore uses the same continuous velocity command as the known
    working ``Move(..., True)`` path, while this worker owns the short deadline
    and sends ``StopMove`` when it expires.
    """

    def __init__(self, loco, monotonic=time.monotonic, wall_time=time.time):
        self._loco = loco
        self._monotonic = monotonic
        self._wall_time = wall_time
        self.active = False
        self.deadline_monotonic = 0.0
        self.nav_id = None
        self.sequence = None

    def _clear(self) -> None:
        self.active = False
        self.deadline_monotonic = 0.0
        self.nav_id = None
        self.sequence = None

    def seconds_until_deadline(self):
        if not self.active:
            return None
        return max(0.0, self.deadline_monotonic - self._monotonic())

    def stop(self, reason: str, force: bool = False) -> dict:
        """Stop and retire the current lease; ``force`` also stops when idle."""
        if not self.active and not force:
            return {
                "ret": 0,
                "error": None,
                "reason": reason,
                "stop_issued": False,
            }
        started = self._monotonic()
        ret = None
        error = None
        try:
            ret = self._loco.StopMove()
        except Exception as exc:
            error = str(exc)
        completed = self._monotonic()
        self._clear()
        return {
            "ret": ret,
            "error": error,
            "reason": reason,
            "stop_issued": True,
            "started_monotonic": started,
            "completed_monotonic": completed,
            "completed_unix_ms": round(self._wall_time() * 1000),
            "duration_ms": max(0, round((completed - started) * 1000)),
        }

    def expire_if_due(self) -> dict | None:
        if not self.active or self._monotonic() < self.deadline_monotonic:
            return None
        return self.stop("proposal_ttl_expired")

    def apply(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        deadline_monotonic: float,
        nav_id: str,
        sequence: int,
        request_id: int | None = None,
    ) -> dict:
        """Apply a continuous velocity only while the Driver deadline is live."""
        started = self._monotonic()
        deadline = float(deadline_monotonic)
        request = {
            "vx": float(vx),
            "vy": float(vy),
            "vyaw": float(vyaw),
            "deadline_monotonic": deadline,
            "nav_id": nav_id,
            "sequence": int(sequence),
        }
        base = {
            "request_id": request_id,
            "rpc_method": "SetVelocity(continuous)",
            "request": request,
            "ret": None,
            "error": None,
            "applied": False,
            "started_monotonic": started,
        }
        if not math.isfinite(deadline) or deadline <= started:
            stopped = self.stop("proposal_ttl_expired_before_rpc", force=True)
            return {
                **base,
                "error": "proposal_ttl_expired_before_rpc",
                "stop_ret": stopped.get("ret"),
                "stop_error": stopped.get("error"),
                "completed_monotonic": self._monotonic(),
                "completed_unix_ms": round(self._wall_time() * 1000),
            }

        ret = None
        error = None
        try:
            # LocoClient.Move(..., True) maps to this exact long-duration RPC,
            # but its Python wrapper does not return the SetVelocity code.
            ret = self._loco.SetVelocity(
                float(vx),
                float(vy),
                float(vyaw),
                PROPOSAL_CONTINUOUS_DURATION_SECONDS,
            )
        except Exception as exc:
            error = str(exc)
        completed = self._monotonic()
        result = {
            **base,
            "ret": ret,
            "error": error,
            "completed_monotonic": completed,
            "completed_unix_ms": round(self._wall_time() * 1000),
            "duration_ms": max(0, round((completed - started) * 1000)),
        }

        if error is not None or ret != 0:
            stopped = self.stop("proposal_velocity_rpc_failed", force=True)
            result.update({
                "stop_ret": stopped.get("ret"),
                "stop_error": stopped.get("error"),
            })
            return result

        if completed >= deadline:
            stopped = self.stop("proposal_ttl_expired_after_rpc", force=True)
            result.update({
                "error": "proposal_ttl_expired_after_rpc",
                "stop_ret": stopped.get("ret"),
                "stop_error": stopped.get("error"),
            })
            return result

        self.active = True
        self.deadline_monotonic = deadline
        self.nav_id = nav_id
        self.sequence = int(sequence)
        result["applied"] = True
        return result


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

    proposal_executor = ProposalRpcExecutor(loco)

    time.sleep(0.5)
    print("[G1 RpcWorker] ready", flush=True)

    read_only_methods = {
        "GetFsmId",
        "GetFsmMode",
        "GetBalanceMode",
        "GetSwingHeight",
        "GetStandHeight",
        "GetPhase",
    }

    while True:
        proposal_executor.expire_if_due()
        try:
            wait_timeout = proposal_executor.seconds_until_deadline()
            if wait_timeout is None:
                cmd = cmd_queue.get()
            else:
                cmd = cmd_queue.get(timeout=wait_timeout)
        except queue.Empty:
            proposal_executor.expire_if_due()
            continue
        except Exception:
            break
        if cmd is None:
            proposal_executor.stop("rpc_worker_shutdown")
            break

        method = cmd.get("method")
        args = cmd.get("args", [])
        kwargs = cmd.get("kwargs", {})
        rpc_request_id = cmd.get("request_id")

        try:
            if method == "__apply_velocity_proposal":
                result_queue.put({
                    "request_id": rpc_request_id,
                    "result": proposal_executor.apply(*args, **kwargs),
                })
                continue

            if method == "StopMove":
                stopped = proposal_executor.stop("explicit_stop_move", force=True)
                result_queue.put({
                    "request_id": rpc_request_id,
                    "result": stopped.get("ret"),
                })
                continue

            # Read-only state queries may run while a proposal is active.  Any
            # other Loco operation first retires the proposal authority.
            if proposal_executor.active and method not in read_only_methods:
                proposal_executor.stop("parent_rpc_override", force=True)

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
                        result_queue.put({"request_id": rpc_request_id, "result": {
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
                        result_queue.put({"request_id": rpc_request_id, "result": {
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
                    result_queue.put({
                        "request_id": rpc_request_id,
                        "result": {
                            "ret": 0,
                            "steps": completed,
                            "fsm_id": final,
                        },
                    })
                continue  # next cmd

            fn = getattr(loco, method)
            result = fn(*args, **kwargs)
            result_queue.put({"request_id": rpc_request_id, "result": result})
        except Exception as e:
            result_queue.put({"request_id": rpc_request_id, "error": str(e)})
        finally:
            proposal_executor.expire_if_due()


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
        self._request_id = 0

    def _call(self, method: str, *args, timeout: float = 15.0, **kwargs):
        with self._lock:
            self._request_id += 1
            request_id = self._request_id
            self._cmd_q.put({
                "request_id": request_id,
                "method": method,
                "args": args,
                "kwargs": kwargs,
            })
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                try:
                    response = self._result_q.get(timeout=remaining)
                except Exception:
                    return None
                if response.get("request_id") != request_id:
                    continue
                if "error" in response:
                    print(
                        f"[G1 RpcProxy] {method} error: {response['error']}",
                        flush=True,
                    )
                    return None
                return response["result"]

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

    def ApplyVelocityProposal(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        deadline_monotonic: float,
        nav_id: str,
        sequence: int,
        request_id: int | None = None,
    ) -> dict:
        result = self._call(
            "__apply_velocity_proposal",
            vx,
            vy,
            vyaw,
            deadline_monotonic,
            nav_id,
            sequence,
            request_id,
            timeout=1.0,
        )
        if isinstance(result, dict):
            return result
        return {
            "request_id": request_id,
            "rpc_method": "SetVelocity(continuous)",
            "request": {
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
                "deadline_monotonic": deadline_monotonic,
                "nav_id": nav_id,
                "sequence": sequence,
            },
            "ret": None,
            "error": "parent_proposal_rpc_unavailable",
            "applied": False,
            "completed_monotonic": time.monotonic(),
            "completed_unix_ms": round(time.time() * 1000),
        }

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
