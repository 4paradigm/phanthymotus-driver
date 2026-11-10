"""visitor_badge 卡片对着模拟桥接服务的契约测试（不需要 ROS、打印机或范式+）。"""

from __future__ import annotations

import json
import sys
import threading
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))  # phanthymotus-driver/，提供 common
sys.path.insert(0, str(HERE))

from visitor_badge import VisitorBadgePlugin  # noqa: E402
from visitor_badge_mock_bridge import MockBridge  # noqa: E402

TOKEN = "test-token"


class VisitorBadgeTest(unittest.TestCase):
    def setUp(self):
        self.bridge = MockBridge(TOKEN, print_seconds=0.2)
        self.server = self.bridge.serve("127.0.0.1", 0)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.card = VisitorBadgePlugin({"poll_interval_s": 0.05})
        cfg = self.card.dispatch("config", {"bridge_url": self.url, "token": TOKEN, "print_wait_s": 5})
        self.assertTrue(cfg["adapter_ok"])

    def tearDown(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()

    def call(self, action, **args):
        result = self.card.dispatch(action, dict(args, _tool_name="visitor_badge"))
        json.dumps(result, ensure_ascii=False)  # 必须能被 MCP handler 序列化
        return result

    def test_tool_definition(self):
        definition = self.card.get_tool()
        self.assertEqual(definition["name"], "visitor_badge")
        self.assertEqual(definition["type"], "actuator")
        actions = definition["inputSchema"]["properties"]["action"]["enum"]
        for action in ("start", "stop", "info", "config", "check", "find", "print", "job"):
            self.assertIn(action, actions)
        props = definition["configSchema"]["properties"]
        self.assertEqual(props["token"]["format"], "password")
        self.assertTrue(props["bridge_url"]["x-sensitive"])
        self.assertEqual(definition["configSchema"]["required"], ["bridge_url"])

    def test_lifecycle(self):
        self.assertEqual(self.call("start")["state"], "ready")
        self.assertEqual(self.call("stop")["state"], "idle")
        self.assertTrue(self.call("info")["configured"])

    def test_unconfigured_card_says_so(self):
        card = VisitorBadgePlugin({})
        self.assertFalse(card.dispatch("config", {})["adapter_ok"])
        self.assertEqual(card.dispatch("find", {"keyword": "王磊"})["code"], "not_configured")

    def test_config_rejects_bad_url(self):
        result = self.card.dispatch("config", {"bridge_url": "ftp://x"})
        self.assertFalse(result["adapter_ok"])
        # 原配置不受影响
        self.assertEqual(self.call("check")["status"], "ok")

    def test_check(self):
        result = self.call("check")
        self.assertEqual(result["status"], "ok")
        self.assertTrue(result["printer"]["ready"])
        self.bridge.printer_ready = False
        result = self.call("check")
        self.assertEqual(result["status"], "not_ready")
        self.assertIn("缺纸", result["problems"][0])

    def test_find_single_multiple_and_none(self):
        one = self.call("find", keyword="王磊")
        self.assertEqual(one["status"], "found")
        self.assertEqual(one["visitors"][0]["visitor_id"], "V1001")
        self.assertEqual(one["visitors"][0]["host"], "张三")
        self.assertEqual(self.call("find", keyword="李娜")["status"], "multiple")
        self.assertEqual(self.call("find", keyword="5678")["visitors"][0]["visitor_id"], "V1002")
        self.assertEqual(self.call("find", keyword="赵六")["status"], "not_found")
        self.assertEqual(self.call("find")["code"], "bad_args")

    def test_print_waits_for_completion(self):
        result = self.call("print", visitor_id="V1001")
        self.assertEqual(result["status"], "printed")
        self.assertEqual(self.bridge.print_log, ["V1001"])
        # find 能看到已打印
        self.assertTrue(self.call("find", keyword="王磊")["visitors"][0]["printed"])

    def test_print_no_wait_then_job(self):
        result = self.call("print", visitor_id="V1002", wait=False)
        self.assertEqual(result["status"], "printing")
        import time
        time.sleep(0.3)
        self.assertEqual(self.call("job", job_id=result["job_id"])["status"], "printed")

    def test_no_duplicate_print_without_reprint(self):
        self.call("print", visitor_id="V1001")
        self.assertEqual(self.call("print", visitor_id="V1001")["status"], "already_printed")
        self.assertEqual(self.call("print", visitor_id="V1001", reprint=True)["status"], "printed")
        self.assertEqual(self.bridge.print_log, ["V1001", "V1001"])

    def test_print_requires_known_visitor_id(self):
        self.assertEqual(self.call("print")["code"], "bad_args")
        self.assertEqual(self.call("print", visitor_id="../etc")["code"], "bad_args")
        result = self.call("print", visitor_id="V9999")
        self.assertEqual(result["status"], "error")
        self.assertIn("没有这个访客", result["error"])
        self.assertEqual(self.bridge.print_log, [])

    def test_printer_not_ready_is_reported(self):
        self.bridge.printer_ready = False
        result = self.call("print", visitor_id="V1001")
        self.assertEqual(result["code"], "printer_not_ready")
        self.assertIn("缺纸", result["error"])

    def test_wrong_token(self):
        self.card.dispatch("config", {"token": "wrong"})
        self.assertEqual(self.call("check")["code"], "unauthorized")

    def test_bridge_down(self):
        self.server.shutdown()
        self.server.server_close()
        self.card.dispatch("config", {"timeout_s": 2})
        result = self.call("check")
        self.assertEqual(result["code"], "unreachable")
        self.assertIn("前台电脑", result["error"])
        self.server = None

    def test_unknown_action(self):
        self.assertEqual(self.call("explode")["code"], "bad_action")


if __name__ == "__main__":
    unittest.main()
