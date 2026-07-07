"""
rpc_proxy.py — Subprocess proxy for all CycloneDDS RPC clients (Go2).

The driver process has many threads (ROS2 executor, camera capture, mic capture, etc.)
causing severe GIL contention. CycloneDDS listener callbacks (which need the GIL) get
starved, so RPC responses arrive >5s late or timeout entirely.

Running RPC calls in a subprocess with minimal threads avoids this entirely.
"""

import multiprocessing
import threading
import time


def _rpc_worker(cmd_queue: multiprocessing.Queue, result_queue: multiprocessing.Queue,
                network_iface: str):
    """Subprocess: holds dedicated RPC clients, processes commands sequentially."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
    from unitree_sdk2py.go2.vui.vui_client import VuiClient
    from unitree_sdk2py.go2.video.video_client import VideoClient
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.a2.audio.audio_client import AudioClient

    ChannelFactoryInitialize(0, network_iface)

    sport = SportClient()
    sport.SetTimeout(10.0)
    sport.Init()

    obstacles_avoid = ObstaclesAvoidClient()
    obstacles_avoid.SetTimeout(10.0)
    obstacles_avoid.Init()

    vui = VuiClient()
    vui.SetTimeout(10.0)
    vui.Init()

    video = VideoClient()
    video.SetTimeout(10.0)
    video.Init()

    motion_switcher = MotionSwitcherClient()
    motion_switcher.SetTimeout(5.0)
    motion_switcher.Init()

    audio = AudioClient()
    audio.SetTimeout(10.0)
    audio.Init()

    # AudioHub megaphone publisher (in subprocess to avoid GIL issues in main process)
    from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
        Request_, RequestHeader_, RequestIdentity_, RequestLease_, RequestPolicy_,
    )
    audiohub_pub = ChannelPublisher("rt/api/audiohub/request", Request_)
    audiohub_pub.Init()

    def _audiohub_send(api_id, parameter_dict):
        import json as _json
        identity = RequestIdentity_(id=int(time.time() * 1000) % 2147483648, api_id=api_id)
        lease = RequestLease_(id=0)
        policy = RequestPolicy_(priority=0, noreply=False)
        header = RequestHeader_(identity=identity, lease=lease, policy=policy)
        req = Request_(header=header, parameter=_json.dumps(parameter_dict), binary=[])
        audiohub_pub.Write(req)

    clients = {
        "sport": sport,
        "obstacles_avoid": obstacles_avoid,
        "vui": vui,
        "video": video,
        "motion_switcher": motion_switcher,
        "audio": audio,
    }

    time.sleep(0.5)
    print("[RpcWorker] ready (Go2: sport, obstacles_avoid, vui, video, motion_switcher, audio, audiohub)", flush=True)

    while True:
        try:
            cmd = cmd_queue.get()
        except Exception:
            break
        if cmd is None:
            break

        client_name = cmd.get("client")
        method = cmd.get("method")
        args = cmd.get("args", [])
        kwargs = cmd.get("kwargs", {})

        try:
            # Special handling for audiohub commands
            if client_name == "audiohub":
                api_id = args[0] if args else kwargs.get("api_id", 0)
                param = args[1] if len(args) > 1 else kwargs.get("param", {})
                if api_id == 4003 and isinstance(param, bytes):
                    # Megaphone upload: param is raw WAV data, handle chunking here
                    import base64 as _b64
                    wav_data = param
                    b64_data = _b64.b64encode(wav_data).decode('utf-8')
                    chunk_size = 4096
                    chunks = [b64_data[i:i+chunk_size] for i in range(0, len(b64_data), chunk_size)]
                    total = len(chunks)
                    for i, chunk in enumerate(chunks, 1):
                        _audiohub_send(4003, {
                            "current_block_size": len(chunk),
                            "block_content": chunk,
                            "current_block_index": i,
                            "total_block_number": total,
                        })
                        time.sleep(0.01)
                    result_queue.put({"result": 0})
                else:
                    _audiohub_send(api_id, param)
                    result_queue.put({"result": 0})
                continue

            client = clients.get(client_name)
            if client is None:
                result_queue.put({"error": f"unknown client: {client_name}"})
                continue
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
                return None
            if "error" in r:
                print(f"[RpcProxy] {client}.{method} error: {r['error']}", flush=True)
                return None
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

    def _call_binary(self, client: str, method: str, *args, **kwargs):
        """For methods that return binary data (e.g. video frames)."""
        result = self._call(client, method, *args, **kwargs)
        return result

    def stop(self):
        try:
            self._cmd_q.put(None)
            self._proc.join(timeout=3)
        except Exception:
            pass

    # ── SportClient interface ─────────────────────────────────────────────────

    def Damp(self):
        return self._call_code("sport", "Damp")

    def BalanceStand(self):
        return self._call_code("sport", "BalanceStand")

    def StopMove(self):
        return self._call_code("sport", "StopMove")

    def StandUp(self):
        return self._call_code("sport", "StandUp")

    def StandDown(self):
        return self._call_code("sport", "StandDown")

    def RecoveryStand(self):
        return self._call_code("sport", "RecoveryStand")

    def Euler(self, roll: float, pitch: float, yaw: float):
        return self._call_code("sport", "Euler", roll, pitch, yaw)

    def Move(self, vx: float, vy: float, vyaw: float):
        return self._call_code("sport", "Move", vx, vy, vyaw)

    def Sit(self):
        return self._call_code("sport", "Sit")

    def RiseSit(self):
        return self._call_code("sport", "RiseSit")

    def SpeedLevel(self, level: int):
        return self._call_code("sport", "SpeedLevel", level)

    def Hello(self):
        return self._call_code("sport", "Hello")

    def Stretch(self):
        return self._call_code("sport", "Stretch")

    def Content(self):
        return self._call_code("sport", "Content")

    def Heart(self):
        return self._call_code("sport", "Heart")

    def Dance1(self):
        return self._call_code("sport", "Dance1")

    def Dance2(self):
        return self._call_code("sport", "Dance2")

    def SwitchJoystick(self, on: bool):
        return self._call_code("sport", "SwitchJoystick", on)

    def Pose(self, flag: bool):
        return self._call_code("sport", "Pose", flag)

    def Scrape(self):
        return self._call_code("sport", "Scrape")

    def FrontFlip(self):
        return self._call_code("sport", "FrontFlip")

    def FrontJump(self):
        return self._call_code("sport", "FrontJump")

    def FrontPounce(self):
        return self._call_code("sport", "FrontPounce")

    def LeftFlip(self):
        return self._call_code("sport", "LeftFlip")

    def BackFlip(self):
        return self._call_code("sport", "BackFlip")

    def HandStand(self, flag: bool):
        return self._call_code("sport", "HandStand", flag)

    def FreeWalk(self):
        return self._call_code("sport", "FreeWalk")

    def FreeBound(self, flag: bool):
        return self._call_code("sport", "FreeBound", flag)

    def FreeJump(self, flag: bool):
        return self._call_code("sport", "FreeJump", flag)

    def FreeAvoid(self, flag: bool):
        return self._call_code("sport", "FreeAvoid", flag)

    def ClassicWalk(self, flag: bool):
        return self._call_code("sport", "ClassicWalk", flag)

    def WalkUpright(self, flag: bool):
        return self._call_code("sport", "WalkUpright", flag)

    def CrossStep(self, flag: bool):
        return self._call_code("sport", "CrossStep", flag)

    def StaticWalk(self):
        return self._call_code("sport", "StaticWalk")

    def TrotRun(self):
        return self._call_code("sport", "TrotRun")

    def EconomicGait(self):
        return self._call_code("sport", "EconomicGait")

    def AutoRecoverySet(self, enabled: bool):
        return self._call_code("sport", "AutoRecoverySet", enabled)

    def AutoRecoveryGet(self):
        return self._call_tuple("sport", "AutoRecoveryGet")

    def SwitchAvoidMode(self):
        return self._call_code("sport", "SwitchAvoidMode")

    # ── ObstaclesAvoidClient interface ────────────────────────────────────────

    def OA_SwitchSet(self, on: bool):
        return self._call_code("obstacles_avoid", "SwitchSet", on)

    def OA_SwitchGet(self):
        return self._call_tuple("obstacles_avoid", "SwitchGet")

    def OA_Move(self, vx: float, vy: float, vyaw: float):
        return self._call_code("obstacles_avoid", "Move", vx, vy, vyaw)

    def OA_MoveToAbsolutePosition(self, x: float, y: float, yaw: float):
        return self._call_code("obstacles_avoid", "MoveToAbsolutePosition", x, y, yaw)

    def OA_MoveToIncrementPosition(self, x: float, y: float, yaw: float):
        return self._call_code("obstacles_avoid", "MoveToIncrementPosition", x, y, yaw)

    def OA_UseRemoteCommandFromApi(self, flag: bool):
        return self._call_code("obstacles_avoid", "UseRemoteCommandFromApi", flag)

    # ── VuiClient interface ───────────────────────────────────────────────────

    def Vui_SetVolume(self, level: int):
        return self._call_code("vui", "SetVolume", level)

    def Vui_GetVolume(self):
        return self._call_tuple("vui", "GetVolume")

    def Vui_SetBrightness(self, level: int):
        return self._call_code("vui", "SetBrightness", level)

    def Vui_GetBrightness(self):
        return self._call_tuple("vui", "GetBrightness")

    def Vui_SetSwitch(self, enable: int):
        return self._call_code("vui", "SetSwitch", enable)

    def Vui_GetSwitch(self):
        return self._call_tuple("vui", "GetSwitch")

    # ── VideoClient interface ─────────────────────────────────────────────────

    def Video_GetImageSample(self):
        return self._call_binary("video", "GetImageSample")

    # ── MotionSwitcherClient interface ────────────────────────────────────────

    def MSC_CheckMode(self):
        return self._call_tuple("motion_switcher", "CheckMode")

    def MSC_SelectMode(self, name: str):
        return self._call_tuple("motion_switcher", "SelectMode", name)

    def MSC_ReleaseMode(self):
        return self._call_tuple("motion_switcher", "ReleaseMode")

    # ── AudioClient interface ─────────────────────────────────────────────────

    def Audio_PlayStream(self, app_name: str, stream_id: str, pcm_data: bytes):
        return self._call("audio", "PlayStream", app_name, stream_id, pcm_data, timeout=5.0)

    def Audio_PlayStop(self, app_name: str):
        return self._call_code("audio", "PlayStop", app_name)

    # ── AudioHub interface (megaphone) ────────────────────────────────────────

    def AudioHub_Send(self, api_id: int, param):
        """Send an audiohub command via DDS Request_ in the subprocess.
        For api_id=4003, param can be raw WAV bytes (subprocess handles chunking).
        """
        return self._call("audiohub", "_", api_id, param, timeout=10.0)
