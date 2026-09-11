"""Every Go2 multiprocessing child entry point must install logsafe.

README_dev.md's checklist requires `logsafe.install(check_fd=False)` at the
top of every multiprocessing child entry point: spawned children get a fresh
interpreter and do not inherit the parent's protected sys.stdout, so their
concurrent print() calls can still tear docker log records or emit control
bytes. The real workers need hardware or a DDS ring, so the contract is
asserted statically on the source.
"""

import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]

# module path -> child entry functions spawned by the bundle
CHILD_ENTRIES = {
    "unitree/go2/rpc_proxy.py": ["_rpc_worker"],
    "unitree/go2/device.py": ["_speaker_worker", "_run_camera_process"],
    "unitree/go2/realsense.py": ["_capture"],
    "unitree/go2/spatial.py": ["_slam_rpc_worker"],
    "unitree/go2/controlled_spatial.py": ["_slam_rpc_worker"],
    "unitree/go2/ext_devices.py": ["_run_ext_camera_process"],
}


def installs_logsafe(func_node):
    for node in ast.walk(func_node):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "install"):
            return any(kw.arg == "check_fd" for kw in node.keywords)
    return False


class LogsafeChildInstallTest(unittest.TestCase):
    def test_every_multiprocessing_child_entry_installs_logsafe(self):
        for rel_path, func_names in CHILD_ENTRIES.items():
            for func_name in func_names:
                with self.subTest(entry=f"{rel_path}:{func_name}"):
                    tree = ast.parse((ROOT / rel_path).read_text())
                    fn = next((node for node in ast.walk(tree)
                               if isinstance(node, ast.FunctionDef) and node.name == func_name), None)
                    self.assertIsNotNone(fn, f"{func_name} not found in {rel_path}")
                    self.assertTrue(
                        installs_logsafe(fn),
                        f"{rel_path}:{func_name} must call logsafe.install(check_fd=False); "
                        "a spawned child does not inherit the parent's protected sys.stdout")


if __name__ == "__main__":
    unittest.main()
