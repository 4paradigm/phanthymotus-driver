"""Dedicated process for Unitree As2W RPC calls."""
import multiprocessing
import threading

def _install_logsafe():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def _worker(commands, results, interface):
    _install_logsafe()
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.as2.sport.sport_client import SportClient
        from unitree_sdk2py.a2.audio.audio_client import AudioClient
        from unitree_sdk2py.go2.video.video_client import VideoClient
        ChannelFactoryInitialize(0, interface or None)
        client = SportClient()
        client.SetTimeout(10.0)
        client.Init()
        try:
            audio = AudioClient()
            audio.SetTimeout(10.0)
            audio.Init()
        except Exception as exc:
            # Audio is optional at the robot firmware level. Keep sport RPC
            # available when the voice service is not enabled.
            audio = None
            print(f"[as2w-rpc] AudioClient unavailable: {exc}", flush=True)
        try:
            video = VideoClient()
            video.SetTimeout(5.0)
            video.Init()
        except Exception as exc:
            video = None
            print(f"[as2w-rpc] VideoClient unavailable: {exc}", flush=True)
        results.put({"ready": True})
    except Exception as exc:
        results.put({"startup_error": str(exc)})
        return
    while True:
        command = commands.get()
        if command is None:
            return
        try:
            if command[0] == "GetState":
                state = {}
                code = client.GetState(state)
                results.put({"result": (code, state)})
            elif command[0].startswith("Audio_"):
                if audio is None:
                    raise RuntimeError("AS2 voice service unavailable")
                results.put({"result": getattr(audio, command[0][len("Audio_"):])(*command[1])})
            elif command[0].startswith("Video_"):
                if video is None:
                    raise RuntimeError("AS2 video service unavailable")
                results.put({"result": getattr(video, command[0][len("Video_"):])(*command[1])})
            else:
                results.put({"result": getattr(client, command[0])(*command[1])})
        except Exception as exc:
            results.put({"error": str(exc)})


class RpcProxy:
    def __init__(self, network_interface=""):
        context = multiprocessing.get_context("spawn")
        self._commands, self._results = context.Queue(), context.Queue()
        self._process = context.Process(target=_worker, args=(self._commands, self._results, network_interface), daemon=True)
        self._process.start()
        self._lock = threading.Lock()
        self._startup_error = None
        try:
            result = self._results.get(timeout=5)
            if result.get("startup_error"):
                self._startup_error = result["startup_error"]
        except Exception:
            self._startup_error = "SportClient worker did not become ready"

    def call(self, method, *args):
        if self._startup_error:
            return (3104, {}) if method == "GetState" else 3104
        with self._lock:
            self._commands.put((method, args))
            try:
                result = self._results.get(timeout=15)
            except Exception:
                return 3104
        if "error" in result:
            print(f"[as2w-rpc] {method}: {result['error']}", flush=True)
            return 3104
        return result["result"]

    def __getattr__(self, name):
        return lambda *args: self.call(name, *args)

    def stop(self):
        try:
            self._commands.put(None)
            self._process.join(timeout=3)
        except Exception:
            pass

    def Audio_PlayStream(self, app_name, stream_id, pcm_data):
        return self.call("Audio_PlayStream", app_name, stream_id, pcm_data)

    def Audio_PlayStop(self, app_name):
        return self.call("Audio_PlayStop", app_name)

    def Audio_GetVolume(self):
        return self.call("Audio_GetVolume")

    def Audio_SetVolume(self, volume):
        return self.call("Audio_SetVolume", volume)

    def Audio_LedControl(self, red, green, blue):
        return self.call("Audio_LedControl", red, green, blue)

    def Video_GetImageSample(self):
        return self.call("Video_GetImageSample")
