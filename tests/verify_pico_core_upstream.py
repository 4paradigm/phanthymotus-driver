"""Opt-in localhost integration with an unchanged ordinary Core source export.

CORE_SNAPSHOT points to `git archive upstream/main agent-core` output. This
runner uses actual Core APIs, SQLite, JS and actual Driver MCP/TLS. Only the
Driver DDS transport is a sink; this is not a ROS or physical-device test.
"""

import argparse
import hashlib
import importlib.util
import json
import os
import re
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("core_snapshot", type=Path)
    parser.add_argument("--core-sha", required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    core = args.core_snapshot.resolve() / "agent-core"
    sys.path[:0] = [str(root), str(root / "tests"), str(core / "src")]
    with tempfile.TemporaryDirectory(prefix="pico-core-fixture-") as private:
        os.environ["DB_PATH"] = str(Path(private) / "core.db")
        import fastapi
        import uvicorn
        from fastapi.responses import HTMLResponse
        from fastapi.staticfiles import StaticFiles
        from api import canvas, config as core_config, mcp_manage, solutions
        import config
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pico/4ultra'))
        from ext_vr.plugin import ExtVrPlugin
        from test_pico_device import frame

        cfg = load("pico_identity", root / "pico/4ultra/identity.py").prepare_config(
            {
                "public_host": "127.0.0.1",
                "state_dir": private,
                "port": port(),
                "bind_host": "127.0.0.1",
                "discovery_enabled": False,
            }
        )

        class Sink:
            def publish(self, value):
                pass

            def close(self):
                pass

        plugin = ExtVrPlugin(cfg, "pico", transport_factory=lambda *a: Sink())
        handler = load("pico_main", root / "pico/4ultra/main.py").make_handler(plugin)
        mcp = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        thread = threading.Thread(target=mcp.serve_forever, daemon=True)
        thread.start()
        app = fastapi.FastAPI()
        app.include_router(canvas.router, prefix="/api")
        app.include_router(core_config.router, prefix="/api")
        app.include_router(mcp_manage.router, prefix="/api")
        app.mount("/js", StaticFiles(directory=core / "web/js"))
        files = [
            "src/api/canvas.py",
            "src/api/mcp_manage.py",
            "src/api/config.py",
            "src/api/solutions.py",
            "src/tool_config.py",
            "web/js/sidebar.js",
            "web/js/canvas.js",
            "web/js/renderers/activity.js",
        ]
        state = {
            "mcp_url": f"http://127.0.0.1:{mcp.server_port}/mcp",
            "core_sha": args.core_sha,
            "core_source_sha256": {
                p: hashlib.sha256((core / p).read_bytes()).hexdigest() for p in files
            },
        }

        @app.get("/fixture")
        async def fixture():
            return {**state, "tool": plugin.get_tool()}

        @app.get("/fixture/driver-state")
        async def driver_state():
            return plugin.dispatch("info", {"instance_id": "vr-card-1"})

        @app.post("/fixture/layout")
        async def layout(body: dict):
            config.main["canvas_layout"] = {
                "cards": [
                    {
                        "id": "vr-card-1",
                        "mcpId": body["mcp_id"],
                        "toolName": "teleop_device",
                    }
                ],
                "connections": [],
            }
            state["mcp_id"] = body["mcp_id"]
            return {"ok": True}

        @app.get("/fixture/pack")
        async def pack():
            value, redacted = solutions._pack_canvas({state["mcp_id"]: "d0"}, set())
            return {"canvas": value, "redacted": redacted}

        @app.get("/fixture/input")
        async def input_frame():
            item = plugin.instances["vr-card-1"]
            runtime = item["runtime"]
            binding, epoch = runtime.bind_capture("browser-fixture")
            return runtime.submit_rtc_frame(
                frame(), authority=binding, rtc_generation=epoch
            )

        @app.get("/", response_class=HTMLResponse)
        async def html():
            # Minimal DOM fixture; sidebar, modal, renderer and fetch handlers
            # are imported unchanged from ordinary Core. This is not the whole app.
            import_map = re.search(
                r'<script type="importmap">.*?</script>',
                (core / "web/index.html").read_text(),
                re.S,
            ).group(0)
            return (
                import_map
                + """<!doctype html><meta charset=utf-8><title>PICO ordinary Core fixture</title>
            <style>.hidden{display:none}label,input,select{display:block;margin:10px}</style>
            <div id=canvas-editor-bar></div><div id=tool-config-overlay class=hidden>
            <h2 id=tool-config-title></h2><div id=tool-config-body></div>
            <button id=tool-config-save>Save</button><button id=tool-config-close>Close</button>
            <button id=tool-config-cancel>Cancel</button></div><div id=monitor></div>"""
            )

        http_port = port()
        server = uvicorn.Server(
            uvicorn.Config(app, host="127.0.0.1", port=http_port, log_level="warning")
        )
        server_thread = threading.Thread(target=server.run, daemon=True)
        server_thread.start()
        try:
            deadline = time.monotonic() + 10
            while not server.started and time.monotonic() < deadline:
                time.sleep(0.02)
            assert server.started
            args.evidence.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(
                [
                    "node",
                    str(root / "tests/pico_core_ui.mjs"),
                    f"http://127.0.0.1:{http_port}",
                    str(args.evidence),
                ],
                check=True,
            )
        finally:
            server.should_exit = True
            server_thread.join(5)
            mcp.shutdown()
            mcp.server_close()
            thread.join(3)
            plugin.close()


if __name__ == "__main__":
    main()
