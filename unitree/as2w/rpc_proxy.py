"""Dedicated process for Unitree AS2W RPC calls."""
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
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.as2.sport.sport_client import SportClient
    ChannelFactoryInitialize(0, interface)
    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()
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

    def call(self, method, *args):
        with self._lock:
            self._commands.put((method, args))
            try:
                result = self._results.get(timeout=15)
            except Exception:
                return 3104
        if "error" in result:
            print(f"[as2-rpc] {method}: {result['error']}", flush=True)
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
