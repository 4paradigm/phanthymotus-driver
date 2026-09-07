"""SmartMotion 响应路由与日志启动约束回归测试。"""

import ast
import importlib.util
import queue
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    "safety_harness_proxy_under_test",
    Path(__file__).resolve().parents[1] / "safety_harness.py",
)
HARNESS = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = HARNESS
SPEC.loader.exec_module(HARNESS)


class DispatchFinished(BaseException):
    pass


class SafetyHarnessProxyTests(unittest.TestCase):
    def setUp(self):
        self.proxy = HARNESS.SmartMotionProxy.__new__(HARNESS.SmartMotionProxy)
        self.proxy._cmd_queue = queue.Queue()
        self.proxy._result_queue = queue.Queue()
        self.proxy._pending = {}
        self.proxy._dispatch_lock = threading.Lock()
        self.proxy._req_lock = threading.Lock()
        self.proxy._req_counter = 0

    def dispatch(self, *responses):
        with patch.object(self.proxy._result_queue, "get",
                          side_effect=[*responses, DispatchFinished()]):
            with self.assertRaises(DispatchFinished):
                self.proxy._dispatch_results()

    def test_timed_out_navigation_result_does_not_reach_next_request(self):
        result = self.proxy._call("wait_nav_done", timeout=0.001)
        self.assertIn("error", result)
        first = self.proxy._cmd_queue.get_nowait()
        self.assertNotIn(first["_req_id"], self.proxy._pending)

        results = []
        caller = threading.Thread(target=lambda: results.append(self.proxy.get_state()))
        caller.start()
        try:
            second = self.proxy._cmd_queue.get(timeout=1)
            self.dispatch(
                {"_req_id": first["_req_id"], "status": "superseded"},
                {"_req_id": second["_req_id"], "state": "nav_paused"},
            )
        finally:
            caller.join(timeout=16)
        self.assertFalse(caller.is_alive())
        self.assertEqual(results, [{"state": "nav_paused"}])
        self.assertEqual(self.proxy._pending, {})

    def test_invalid_and_unknown_responses_are_discarded(self):
        result_q = queue.Queue(maxsize=1)
        self.proxy._pending["current"] = result_q
        self.dispatch(None, "invalid", {}, {"_req_id": []},
                      {"_req_id": "unknown", "status": "arrived"})
        self.assertTrue(result_q.empty())
        self.dispatch({"_req_id": "current", "status": "ok"})
        self.assertEqual(result_q.get_nowait(), {"status": "ok"})

    def test_duplicate_response_cannot_block_dispatch(self):
        first = queue.Queue(maxsize=1)
        second = queue.Queue(maxsize=1)
        self.proxy._pending.update(first=first, second=second)
        with patch.object(first, "put", wraps=first.put) as put:
            self.dispatch(
                {"_req_id": "first", "status": "original"},
                {"_req_id": "first", "status": "duplicate"},
                {"_req_id": "second", "status": "ok"},
            )
        self.assertTrue(all(call.kwargs.get("block") is False
                            for call in put.call_args_list))
        self.assertEqual(first.get_nowait(), {"status": "original"})
        self.assertEqual(second.get_nowait(), {"status": "ok"})

    def test_paused_waiter_outlives_proxy_deadline(self):
        tree = ast.parse(Path(HARNESS.__file__).read_text())
        entry = next(node for node in tree.body
                     if isinstance(node, ast.FunctionDef)
                     and node.name == "_run_smart_motion_process")
        handler = next(node for node in entry.body
                       if isinstance(node, ast.FunctionDef)
                       and node.name == "handle_wait_nav_done")
        # 提取真实等待函数，将闭包变量映射到测试命名空间，避免依赖机器人 SDK。
        handler.body = [ast.copy_location(ast.Global(names=node.names), node)
                        if isinstance(node, ast.Nonlocal) else node
                        for node in handler.body]
        namespace = dict(vars(HARNESS))
        lock = threading.Condition()
        namespace.update(
            state=HARNESS.MotionState.NAV_PAUSED,
            nav_cmd={"navigation_id": "nav", "generation": 1,
                     "target_pose": {"x": 10, "y": 0}},
            nav_current_pose={"x": 0, "y": 0},
            nav_arrived_flag=False, nav_arrived_error=None,
            slam_info_lock=lock, shutdown_event=threading.Event(),
            poll_interval=0.1,
        )
        exec(compile(ast.Module(body=[handler], type_ignores=[]),
                     HARNESS.__file__, "exec"), namespace)
        waited = []

        def replace_navigation(timeout):
            waited.append(timeout)
            namespace["nav_cmd"] = None

        with patch.object(lock, "wait", side_effect=replace_navigation), \
                patch.object(HARNESS.time, "time", side_effect=[0, 0, 91, 92]):
            result = namespace["handle_wait_nav_done"](60, "nav")
        self.assertEqual(waited, [0.5])
        self.assertEqual(result, {"status": "superseded"})

    def test_missing_logsafe_aborts_startup_before_other_imports(self):
        import builtins
        real_import = builtins.__import__
        imports = []

        def import_module(name, *args, **kwargs):
            imports.append(name)
            if name == "common":
                raise ImportError("missing logsafe")
            return real_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=import_module):
            with self.assertRaisesRegex(ImportError, "missing logsafe"):
                HARNESS._run_smart_motion_process("g1", {}, "eth0", None, None)
        self.assertEqual(imports, ["common"])

    def test_logsafe_install_is_required_before_initialization(self):
        from common import logsafe
        with patch.object(logsafe, "install", side_effect=RuntimeError("install failed")) as install:
            with self.assertRaisesRegex(RuntimeError, "install failed"):
                HARNESS._run_smart_motion_process("g1", {}, "eth0", None, None)
        install.assert_called_once_with(check_fd=False)


if __name__ == "__main__":
    unittest.main()
