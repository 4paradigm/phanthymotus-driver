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
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

    ChannelFactoryInitialize(0, network_iface)

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    audio = AudioClient()
    audio.SetTimeout(10.0)
    audio.Init()

    time.sleep(0.5)
    print("[RpcWorker] ready", flush=True)

    while True:
        try:
            cmd = cmd_queue.get()
        except Exception:
            break
        if cmd is None:
            break

        client_name = cmd.get("client")  # "loco" or "audio"
        method = cmd.get("method")
        args = cmd.get("args", [])
        kwargs = cmd.get("kwargs", {})

        try:
            client = loco if client_name == "loco" else audio
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

    # ── LocoClient interface ──────────────────────────────────────────────────

    def GetFsmId(self):
        return self._call_tuple("loco", "GetFsmId")

    def GetFsmMode(self):
        return self._call_tuple("loco", "GetFsmMode")

    def GetBalanceMode(self):
        return self._call_tuple("loco", "GetBalanceMode")

    def GetSwingHeight(self):
        return self._call_tuple("loco", "GetSwingHeight")

    def GetStandHeight(self):
        return self._call_tuple("loco", "GetStandHeight")

    def GetArmSdkStatus(self):
        return self._call_tuple("loco", "GetArmSdkStatus")

    def GetAvailableFsmIds(self):
        result = self._call("loco", "GetAvailableFsmIds")
        if result is None:
            return 3104, None, None
        return result

    def SetFsmId(self, fsm_id: int):
        return self._call_code("loco", "SetFsmId", fsm_id)

    def SetBalanceMode(self, balance_mode: int):
        return self._call_code("loco", "SetBalanceMode", balance_mode)

    def SetSwingHeight(self, swing_height: float):
        return self._call_code("loco", "SetSwingHeight", swing_height)

    def SetStandHeight(self, stand_height: float):
        return self._call_code("loco", "SetStandHeight", stand_height)

    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        return self._call_code("loco", "SetVelocity", vx, vy, omega, duration)

    def SetTaskId(self, task_id: float):
        return self._call_code("loco", "SetTaskId", task_id)

    def SetSpeedMode(self, speed_mode: int):
        return self._call_code("loco", "SetSpeedMode", speed_mode)

    def SetArmSdkStatus(self, arm_sdk_status: bool):
        return self._call_code("loco", "SetArmSdkStatus", arm_sdk_status)

    def Damp(self):
        return self._call_code("loco", "Damp")

    def Start(self):
        return self._call_code("loco", "Start")

    def Squat(self):
        return self._call_code("loco", "Squat")

    def Sit(self):
        return self._call_code("loco", "Sit")

    def StandUp(self):
        return self._call_code("loco", "StandUp")

    def ZeroTorque(self):
        return self._call_code("loco", "ZeroTorque")

    def StopMove(self):
        return self._call_code("loco", "StopMove")

    def HighStand(self):
        return self._call_code("loco", "HighStand")

    def LowStand(self):
        return self._call_code("loco", "LowStand")

    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = None):
        return self._call_code("loco", "Move", vx, vy, vyaw, continous_move)

    def BalanceStand(self):
        return self._call_code("loco", "BalanceStand")

    def ContinuousGait(self, flag: bool):
        return self._call_code("loco", "ContinuousGait", flag)

    def SwitchMoveMode(self, flag: bool):
        return self._call_code("loco", "SwitchMoveMode", flag)

    def WaveHand(self, turn_flag: bool = False):
        return self._call_code("loco", "WaveHand", turn_flag)

    def ShakeHand(self, stage: int = -1):
        return self._call_code("loco", "ShakeHand", stage)

    def EnableArmSDK(self):
        return self._call_code("loco", "EnableArmSDK")

    def DisableArmSDK(self):
        return self._call_code("loco", "DisableArmSDK")

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
