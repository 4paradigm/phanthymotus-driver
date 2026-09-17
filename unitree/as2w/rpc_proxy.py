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
        ChannelFactoryInitialize(0, interface or None)
        client = SportClient()
        client.SetTimeout(10.0)
        client.Init()
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
