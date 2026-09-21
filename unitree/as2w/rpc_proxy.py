"""Independent RPC workers for AS2 sport, audio and video services."""
import multiprocessing
import threading
import time


def _install_logsafe():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def _serve(commands, results, interface, kind):
    _install_logsafe()
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, interface or None)
        if kind == "sport":
            from unitree_sdk2py.as2.sport.sport_client import SportClient
            client = SportClient()
            client.SetTimeout(5.0)
            client.Init()
        elif kind in ("audio", "audio_led"):
            from unitree_sdk2py.a2.audio.audio_client import AudioClient
            client = AudioClient()
            client.SetTimeout(2.0)
            client.Init()
        else:
            from unitree_sdk2py.go2.video.video_client import VideoClient
            client = VideoClient()
            client.SetTimeout(3.0)
            client.Init()
        results.put({"ready": True})
    except Exception as exc:
        results.put({"startup_error": str(exc)[:240]})
        return

    while True:
        command = commands.get()
        if command is None:
            return
        request_id, method, args = command
        try:
            if kind == "sport" and method == "GetState":
                state = {}
                code = client.GetState(state)
                result = (code, state)
            else:
                result = getattr(client, method)(*args)
            results.put({"request_id": request_id, "result": result})
        except Exception as exc:
            results.put({"request_id": request_id, "error": str(exc)[:240]})


class _RpcChannel:
    def __init__(self, interface, kind, timeout):
        context = multiprocessing.get_context("spawn")
        self._commands = context.Queue()
        self._results = context.Queue()
        self._process = context.Process(
            target=_serve, args=(self._commands, self._results, interface, kind),
            daemon=True,
        )
        self._process.start()
        self._lock = threading.Lock()
        self._timeout = timeout
        self._startup_error = None
        self._last_error = {}
        self._next_request_id = 0
        try:
            result = self._results.get(timeout=8)
            self._startup_error = result.get("startup_error")
        except Exception:
            self._startup_error = f"{kind} RPC worker did not become ready"

    def call(self, method, *args):
        fallback = (3104, {}) if method == "GetState" else 3104
        if self._startup_error:
            return fallback
        with self._lock:
            self._next_request_id += 1
            request_id = self._next_request_id
            self._commands.put((request_id, method, args))
            deadline = time.monotonic() + self._timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._report_error(method, "timeout")
                    return fallback
                try:
                    result = self._results.get(timeout=remaining)
                except Exception:
                    self._report_error(method, "timeout")
                    return fallback
                # A timed-out request may finish after the caller has moved
                # on. Never let that late result satisfy a newer call.
                if result.get("request_id") != request_id:
                    continue
                break
        if "error" in result:
            self._report_error(method, result["error"])
            return fallback
        return result["result"]

    def _report_error(self, method, message):
        now = time.monotonic()
        previous = self._last_error.get(method, 0.0)
        if now - previous >= 10.0:
            print(f"[as2w-rpc] {method} failed: {message[:160]}", flush=True)
            self._last_error[method] = now

    def stop(self):
        try:
            self._commands.put(None)
            self._process.join(timeout=1.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
        except Exception:
            pass


class RpcProxy:
    def __init__(self, network_interface=""):
        self._sport = _RpcChannel(network_interface, "sport", 7.0)
        self._audio = _RpcChannel(network_interface, "audio", 4.0)
        # Keep periodic LED refreshes from queueing behind speaker PCM
        # blocks.  Both clients use the same firmware service, but separate
        # workers prevent a slow LED call from stopping live playback (and
        # vice versa).
        self._audio_led = _RpcChannel(network_interface, "audio_led", 4.0)
        # A snapshot is polled as a live stream. A five-second RPC timeout
        # turns one unavailable frame into a multi-second camera freeze;
        # keep the failure bounded so the next snapshot can be attempted.
        self._video = _RpcChannel(network_interface, "video", 1.0)

    def call(self, method, *args):
        return self._sport.call(method, *args)

    def __getattr__(self, name):
        return lambda *args: self.call(name, *args)

    def stop(self):
        self._video.stop()
        self._audio_led.stop()
        self._audio.stop()
        self._sport.stop()

    def Audio_PlayStream(self, app_name, stream_id, pcm_data):
        return self._audio.call("PlayStream", app_name, stream_id, pcm_data)

    def Audio_PlayStop(self, app_name):
        return self._audio.call("PlayStop", app_name)

    def Audio_GetVolume(self):
        result = self._audio.call("GetVolume")
        if isinstance(result, tuple) and len(result) == 2:
            return result
        return result, None

    def Audio_SetVolume(self, volume):
        return self._audio.call("SetVolume", volume)

    def Audio_LedControl(self, red, green, blue):
        return self._audio_led.call("LedControl", red, green, blue)

    def Video_GetImageSample(self):
        return self._video.call("GetImageSample")
