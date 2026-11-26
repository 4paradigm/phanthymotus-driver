"""Expose virtual cards in the L20 registry and route them only to the preview worker."""
import http.client
import json
from pathlib import Path
import subprocess
import threading

from mujoco_card.worker import build_virtual_cards


class PreviewCards:
    PREFIX = "l20_mujoco"

    def __init__(self, config):
        self.port = int(config["plugins"][self.PREFIX]["worker_port"])
        self.routes = {}
        self.tools = []
        for public, card in build_virtual_cards(config, None).items():
            definition = card.get_tool()
            self.routes[public] = definition["name"]
            self.tools.append({**definition, "name": public})
        self.stopping = threading.Event()
        self.thread = None

    def get_tools(self):
        return self.tools

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stopping.clear()
        self.thread = threading.Thread(target=self._run, name="l20-preview", daemon=True)
        self.thread.start()

    def _run(self):
        launcher = Path(__file__).with_name("sim-launch")
        while not self.stopping.is_set():
            with subprocess.Popen(["/bin/bash", str(launcher)]) as worker:
                while worker.poll() is None:
                    if self.stopping.wait(0.25):
                        worker.terminate()
                        return
                print(f"[l20] preview exited ({worker.returncode}); retrying in 5s", flush=True)
            self.stopping.wait(5)

    def stop(self):
        self.stopping.set()
        if self.thread:
            self.thread.join()

    def dispatch(self, action, args):
        public = args.get("_tool_name")
        if public not in self.routes:
            return None
        arguments = {k: v for k, v in args.items() if k != "_tool_name"}
        payload = {"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                   "params": {"name": self.routes[public], "arguments": {**arguments, "action": action}}}
        # Direct loopback HTTP avoids inherited HTTP proxies. There is no hardware fallback.
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=8)
        try:
            conn.request("POST", "/mcp", json.dumps(payload).encode(), {"Content-Type": "application/json"})
            reply = json.load(conn.getresponse())
        except (OSError, http.client.HTTPException, ValueError) as exc:
            raise RuntimeError("L20 虚拟手服务暂不可用") from exc
        finally:
            conn.close()
        if "error" in reply:
            raise ValueError(reply["error"]["message"])
        return json.loads(reply["result"]["content"][0]["text"])
