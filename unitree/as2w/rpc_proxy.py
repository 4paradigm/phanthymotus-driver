"""Independent RPC workers for AS2 sport and RGB LED services."""
import multiprocessing
import threading
import time


def _install_logsafe():
    try:
        from common import logsafe
        logsafe.install(check_fd=False)
    except (ImportError, TypeError):
        pass


def _serve(commands, results, interface, kind, sdk_timeout=None):
    _install_logsafe()
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, interface or None)
        if kind == "sport":
            from unitree_sdk2py.as2.sport.sport_client import SportClient
            client = SportClient()
            client.SetTimeout(5.0 if sdk_timeout is None else sdk_timeout)
        else:
            from unitree_sdk2py.a2.audio.audio_client import AudioClient
            client = AudioClient()
            client.SetTimeout(2.0)
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
    def __init__(self, interface, kind, timeout, sdk_timeout=None,
                 fail_closed=False):
        context = multiprocessing.get_context("spawn")
        self._commands = context.Queue()
        self._results = context.Queue()
        self._process = context.Process(
            target=_serve,
            args=(self._commands, self._results, interface, kind, sdk_timeout),
            daemon=True,
        )
        self._process.start()
        self._lock = threading.Lock()
        self._timeout = timeout
        self._fail_closed = fail_closed
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
        bounded = getattr(self, "_fail_closed", False)
        deadline = time.monotonic() + self._timeout
        acquired = self._lock.acquire(timeout=self._timeout) if bounded else self._lock.acquire()
        if not acquired:
            return fallback
        try:
            if self._startup_error:
                return fallback
            self._next_request_id += 1
            request_id = self._next_request_id
            self._commands.put((request_id, method, args))
            if not bounded:
                deadline = time.monotonic() + self._timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._report_error(method, "timeout")
                    self._timeout_fault()
                    return fallback
                try:
                    result = self._results.get(timeout=remaining)
                except Exception:
                    self._report_error(method, "timeout")
                    self._timeout_fault()
                    return fallback
                # A timed-out request may finish after the caller has moved
                # on. Never let that late result satisfy a newer call.
                if result.get("request_id") != request_id:
                    continue
                break
        finally:
            self._lock.release()
        if "error" in result:
            self._report_error(method, result["error"])
            return fallback
        return result["result"]

    @property
    def ready(self):
        return self._startup_error is None and self._process.is_alive()

    def _timeout_fault(self):
        if not self._fail_closed:
            return
        # A timed-out Move must never remain queued and execute after StopMove.
        # This prevents further local sends; it does NOT retract an RPC already
        # delivered to the robot. Firmware stop semantics need a hardware test.
        self._startup_error = "control RPC timed out; restart driver to recover"
        if self._process.is_alive():
            self._process.terminate()

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


class _ServoClient:
    """Short RPC lanes isolated from legacy queries and long posture actions.

    The servo scheduler is the only writer and waits for its in-flight Move
    before issuing StopMove. A separate stop lane remains usable if that writer
    process timed out and was terminated. None of these timeouts is a physical
    stopping-time guarantee.
    """
    max_call_seconds = 0.35

    def __init__(self, owner, network_interface, channel_factory=_RpcChannel):
        self._owner = owner
        kwargs = {"timeout": self.max_call_seconds, "sdk_timeout": 0.15,
                  "fail_closed": True}
        self._motion = channel_factory(network_interface, "sport", **kwargs)
        self._state = channel_factory(network_interface, "sport", **kwargs)
        self._emergency = channel_factory(network_interface, "sport", **kwargs)
        self._active = 0
        self.conflict_check = None

    @property
    def control_ready(self):
        return self._motion.ready and self._state.ready and self._emergency.ready

    def acquire_control(self):
        if self.conflict_check is not None:
            problem = self.conflict_check()
            if problem:
                return {"ok": False, "error": str(problem)}
        with self._owner._ownership_lock:
            if not self.control_ready:
                return {"ok": False, "error": "short-timeout control RPC unavailable"}
            if self._owner._legacy_active or self._owner._legacy_motion_active:
                return {"ok": False, "error": "legacy motion active; call loco.stop_move first"}
            if self._owner._owner not in ("legacy", "servo"):
                return {"ok": False, "error": "chassis is unavailable"}
            self._owner._owner = "servo"
            return {"ok": True}

    def release_control(self):
        with self._owner._ownership_lock:
            if self._active:
                raise RuntimeError("cannot release chassis during a control RPC")
            if self._owner._owner == "servo":
                self._owner._owner = "legacy"

    def _write(self, method, *args):
        with self._owner._ownership_lock:
            if self._owner._owner != "servo":
                return 3104
            self._active += 1
        try:
            channel = self._motion
            if method == "StopMove" and not channel.ready:
                channel = self._emergency
            return channel.call(method, *args)
        finally:
            with self._owner._ownership_lock:
                self._active -= 1

    def Move(self, vx, vy, wz):
        return self._write("Move", vx, vy, wz)

    def StopMove(self):
        return self._write("StopMove")

    def GetState(self):
        return self._state.call("GetState")

    def close(self):
        self._motion.stop()
        self._state.stop()
        self._emergency.stop()


class RpcProxy:
    def __init__(self, network_interface=""):
        self._network_interface = network_interface
        self._ownership_lock = threading.Lock()
        self._owner = "legacy"
        self._legacy_active = 0
        self._legacy_motion_active = False
        self._servo_client = None
        self._sport = _RpcChannel(network_interface, "sport", 7.0)
        # The speaker card has its own verified backend. Keep LED refreshes in
        # a separate audio client so they cannot delay either locomotion RPCs
        # or speaker PCM delivery.
        self._audio_led = _RpcChannel(network_interface, "audio_led", 4.0)

    def call(self, method, *args):
        if method == "GetState":
            return self._sport.call(method, *args)
        with self._ownership_lock:
            if self._owner != "legacy":
                # Includes late finalizers from old loco workers. They must not
                # overwrite a newly acquired servo command with Move OR Stop.
                return 3104
            self._legacy_active += 1
            self._legacy_motion_active = True
        try:
            result = self._sport.call(method, *args)
            with self._ownership_lock:
                if result == 0 and method in ("StopMove", "Damp"):
                    self._legacy_motion_active = False
                else:
                    self._legacy_motion_active = True
            return result
        finally:
            with self._ownership_lock:
                self._legacy_active -= 1

    def create_control_client(self):
        if self._servo_client is None:
            self._servo_client = _ServoClient(self, self._network_interface)
        return self._servo_client

    def legacy_motion_active(self):
        with self._ownership_lock:
            return bool(self._legacy_active or self._legacy_motion_active)

    def __getattr__(self, name):
        return lambda *args: self.call(name, *args)

    def Audio_LedControl(self, red, green, blue):
        return self._audio_led.call("LedControl", red, green, blue)

    def stop(self):
        if self._servo_client is not None:
            self._servo_client.close()
        self._audio_led.stop()
        self._sport.stop()
