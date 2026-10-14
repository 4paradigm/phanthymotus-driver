"""Offline regression: a stale G1 config cannot expose the removed card."""

import ast
from io import BytesIO
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1] / "unitree" / "g1"


def load_bundle_and_handler():
    tree = ast.parse((ROOT / "main.py").read_text())
    selected = [node for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef))
                and node.name in ("G1DeviceBundle", "make_handler")]
    module = ast.Module(body=[ast.ImportFrom(module="__future__",
                                             names=[ast.alias(name="annotations")], level=0),
                              *selected], type_ignores=[])
    namespace = {"BaseHTTPRequestHandler": object, "json": json}
    exec(compile(ast.fix_missing_locations(module), str(ROOT / "main.py"), "exec"), namespace)
    return namespace


class TestG1MotionSwitcherRemoval(unittest.TestCase):
    def test_stale_config_does_not_register_card_in_mcp_tools_list(self):
        class TtsPlugin:
            def __init__(self, *args):
                pass

            def get_tool(self):
                return {"name": "tts", "type": "actuator"}

        namespace = load_bundle_and_handler()
        cfg = {"plugins": {"tts": {"enabled": True},
                           "motion_switcher": {"enabled": True},
                           "smart_motion": {"enabled": False}}}
        with patch.dict(sys.modules, {"device": SimpleNamespace(NativeTtsPlugin=TtsPlugin)}):
            bundle = namespace["G1DeviceBundle"](cfg, "test", None, None, None, None, None)
        namespace["_bundle"] = bundle

        handler_cls = namespace["make_handler"]()
        handler = handler_cls.__new__(handler_cls)
        request = {"jsonrpc": "2.0", "id": 1, "method": "tools/list"}
        payload = json.dumps(request).encode()
        handler.headers = {"Content-Length": str(len(payload))}
        handler.rfile = BytesIO(payload)
        responses = []
        handler._send = lambda status, body: responses.append((status, json.loads(body)))
        handler.do_POST()

        self.assertEqual(responses[0][0], 200)
        self.assertEqual([tool["name"] for tool in responses[0][1]["result"]["tools"]], ["tts"])


if __name__ == "__main__":
    unittest.main()
