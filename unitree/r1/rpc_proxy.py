"""
rpc_proxy.py — Subprocess proxy for all CycloneDDS RPC clients.

The driver process has many threads (ROS2 executor, camera capture, mic capture, etc.)
causing severe GIL contention. CycloneDDS listener callbacks (which need the GIL) get
starved, so RPC responses arrive >5s late or timeout entirely.

Running RPC calls in a subprocess with minimal threads avoids this entirely.
Proven to work in <1s by standalone test (docker exec).
"""

import multiprocessing
import threading
import time


def _rpc_worker(cmd_queue: multiprocessing.Queue, result_queue: multiprocessing.Queue,
                network_iface: str):
    """Subprocess: holds dedicated RPC clients, processes commands sequentially."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.h2.loco.h2_loco_client import LocoClient
    from unitree_sdk2py.r1.arm.r1_arm_client import ArmClient
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

    ChannelFactoryInitialize(0, network_iface)

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    arm = ArmClient()
    arm.SetTimeout(10.0)
    arm.Init()

    audio = AudioClient()
    audio.SetTimeout(10.0)
    audio.Init()

    time.sleep(0.5)
    print("[RpcWorker] ready", flush=True)

    clients = {"loco": loco, "arm": arm, "audio": audio}

    while True:
        try:
            cmd = cmd_queue.get()
        except Exception:
            break
        if cmd is None:
            break

        client_name = cmd.get("client")  # "loco", "arm", or "audio"
        method = cmd.get("method")
        args = cmd.get("args", [])
        kwargs = cmd.get("kwargs", {})

        try:
            client = clients.get(client_name, loco)

            # Special: FSM sequence execution (runs entirely in subprocess, no GIL)
            if method == "__run_fsm_sequence":
                steps_spec, interval, step_timeout = args
                completed = []
                for method_name, target_fsm, step_name in steps_spec:
                    fn = getattr(client, method_name)
                    ret = fn()
                    if ret != 0:
                        result_queue.put({"result": {
                            "error": f"Step '{step_name}' failed: code={ret}",
                            "step": step_name, "completed": completed}})
                        continue  # next cmd — sequence done
                    # Poll FSM until target reached or timeout
                    elapsed = 0.0
                    ok = False
                    while elapsed < step_timeout:
                        time.sleep(interval)
                        elapsed += interval
                        code, fsm_id = client.GetFsmId()
                        if code == 0 and fsm_id == target_fsm:
                            ok = True
                            break
                    if not ok:
                        _, current = client.GetFsmId()
                        result_queue.put({"result": {
                            "error": f"Timeout '{step_name}' (expected={target_fsm}, got={current})",
                            "step": step_name, "fsm_id": current, "completed": completed}})
                        continue  # next cmd — sequence done
                    completed.append(step_name)
                result_queue.put({"result": {"ret": 0, "steps": completed,
                                             "fsm_id": steps_spec[-1][1]}})
                continue  # next cmd

            fn = getattr(client, method)
            result = fn(*args, **kwargs)
            result_queue.put({"result": result})
        except Exception as e:
            result_queue.put({"error": str(e)})


class RpcProxy:
    """Proxy that forwards RPC calls to a subprocess, avoiding GIL contention."""

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

    def _call(self, client: str, method: str, *args, timeout: float = 15.0, **kwargs):
        with self._lock:
            self._cmd_q.put({"client": client, "method": method, "args": args, "kwargs": kwargs})
            try:
                r = self._result_q.get(timeout=timeout)
            except Exception:
                return None  # caller handles based on method type
            if "error" in r:
                print(f"[RpcProxy] {client}.{method} error: {r['error']}", flush=True)
                return None  # caller handles based on method type
            return r["result"]

    def _call_code(self, client: str, method: str, *args, **kwargs) -> int:
        """For methods that return a single int code."""
        result = self._call(client, method, *args, **kwargs)
        if result is None:
            return 3104
        return result

    def _call_tuple(self, client: str, method: str, *args, **kwargs):
        """For methods that return (code, data) tuple."""
        result = self._call(client, method, *args, **kwargs)
        if result is None:
            return 3104, None
        return result

    def stop(self):
        try:
            self._cmd_q.put(None)
            self._proc.join(timeout=3)
        except Exception:
            pass

    # ── LocoClient interface (sport service — legs) ───────────────────────────

    def RunFsmSequence(self, steps: list, interval: float = 1.0, step_timeout: float = 15.0):
        """Run FSM sequence entirely in subprocess (no GIL contention in main process).
        steps = [(method_name, target_fsm_id, step_name), ...]
        Returns dict with {ret, steps, fsm_id} on success or {error, step} on failure."""
        outer_timeout = len(steps) * (step_timeout + 5) + 10
        return self._call("loco", "__run_fsm_sequence", steps, interval, step_timeout,
                          timeout=outer_timeout)

    def GetFsmId(self):
        return self._call_tuple("loco", "GetFsmId")

    def SetFsmId(self, fsm_id: int):
        return self._call_code("loco", "SetFsmId", fsm_id)

    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        return self._call_code("loco", "SetVelocity", vx, vy, omega, duration)

    def Damp(self):
        return self._call_code("loco", "Damp")

    def Stance(self):
        return self._call_code("loco", "Stance")

    def Start(self):
        return self._call_code("loco", "Start")

    def Lie2StandUp(self):
        return self._call_code("loco", "Lie2StandUp")

    def StandUp2Lie(self):
        return self._call_code("loco", "StandUp2Lie")

    def ZeroTorque(self):
        return self._call_code("loco", "ZeroTorque")

    def StopMove(self):
        return self._call_code("loco", "StopMove")

    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
        return self._call_code("loco", "Move", vx, vy, vyaw, continous_move)

    # ── ArmClient interface (arm service — hands) ─────────────────────────────

    def ArmEnable(self):
        return self._call_tuple("arm", "Enable")

    def ArmRelease(self):
        return self._call_tuple("arm", "Release")

    def ArmListActions(self):
        return self._call_tuple("arm", "ListActions")

    def ArmExecuteById(self, action_id: int):
        return self._call_tuple("arm", "ExecuteById", action_id)

    def ArmExecuteByName(self, action_name: str):
        return self._call_tuple("arm", "ExecuteByName", action_name)

    def ArmStop(self):
        return self._call_tuple("arm", "Stop")

    def ArmGetStatus(self):
        return self._call_tuple("arm", "GetStatus")

    # ── AudioClient interface ─────────────────────────────────────────────────

    def TtsMaker(self, text: str, speaker_id: int):
        return self._call_code("audio", "TtsMaker", text, speaker_id)

    def GetVolume(self):
        return self._call_tuple("audio", "GetVolume")

    def SetVolume(self, volume: int):
        return self._call_code("audio", "SetVolume", volume)

    def LedControl(self, R: int, G: int, B: int):
        return self._call_code("audio", "LedControl", R, G, B)

    def PlayStream(self, app_name: str, stream_id: str, pcm_data: bytes):
        return self._call_tuple("audio", "PlayStream", app_name, stream_id, pcm_data)

    def PlayStop(self, app_name: str):
        return self._call_code("audio", "PlayStop", app_name)
