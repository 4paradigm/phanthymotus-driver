"""
test_tianyi_bridge_log_rate.py — 桥接的进度日志按时间计，不按消息数计。

`TopicHandler.publish` 原来每 100 条消息打一行。听着不多，但要看速率：一个 16 kHz 的
音频 topic 和关节流各自产出约 750 行，全部桥接 topic 合计占该容器日志的 **78%**
（2809 行里 2200 行）。真正需要被读到的插件错误就淹在里面。

这行日志要回答的是「数据还在流吗」。5 分钟一拍同样能回答，而且无论 topic 是 10 Hz 的
关节状态还是 16 kHz 的音频，代价都是同样的几行。第一条消息仍然立刻打印 —— 「这个桥
到底通没通」是另一个问题，它需要立刻得到答案。

Run: cd phanthymotus-driver && python3 -m pytest tests/test_tianyi_bridge_log_rate.py
"""

import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "x-humanoid" / "tianyi2.0" / "socket_bridge.py"


class _FakeHandler:
    """publish() 的日志判据，逐字复制，不带 ROS。"""

    def __init__(self, interval):
        self.topic = "/t"
        self.msg_count = 0
        self._last_progress_ts = 0.0
        self._interval = interval
        self.lines = []

    def publish_at(self, now):
        self.msg_count += 1
        if self.msg_count == 1 or now - self._last_progress_ts >= self._interval:
            self._last_progress_ts = now
            self.lines.append(self.msg_count)


class ProgressCadenceTests(unittest.TestCase):
    def test_the_first_message_is_reported_immediately(self):
        """「这个桥通没通」不能等 5 分钟。"""
        h = _FakeHandler(300.0)
        h.publish_at(1000.0)
        self.assertEqual(h.lines, [1])

    def test_a_fast_topic_does_not_flood(self):
        """16 kHz 音频：一小时按 100 条一行会是几万行。"""
        h = _FakeHandler(300.0)
        now = 0.0
        for _ in range(3600 * 100):        # 一小时、100 Hz
            now += 0.01
            h.publish_at(now)
        # 首条 + 每 5 分钟一条 ≈ 13
        self.assertLessEqual(len(h.lines), 14, f"一小时打了 {len(h.lines)} 行")
        self.assertGreaterEqual(len(h.lines), 12, "完全不报就看不出数据是否还在流")

    def test_a_slow_topic_costs_the_same(self):
        """按条数计时，慢 topic 反而几乎不报 —— 按时间计则两者一致。"""
        h = _FakeHandler(300.0)
        now = 0.0
        for _ in range(3600):              # 一小时、1 Hz
            now += 1.0
            h.publish_at(now)
        self.assertLessEqual(len(h.lines), 14)
        self.assertGreaterEqual(len(h.lines), 12)

    def test_the_number_reported_is_the_running_total(self):
        """限的是打印频率，不是计数本身 —— 那个数仍要是累计量。"""
        h = _FakeHandler(300.0)
        now = 0.0
        for _ in range(1000):              # 1000 条、1 Hz
            now += 1.0
            h.publish_at(now)
        self.assertEqual(h.msg_count, 1000)
        # 首条立刻打印并把计时器对齐到那一刻，所以之后的刻度是 1+300k，不是 300k。
        self.assertEqual(h.lines, [1, 301, 601, 901])


class SourceTests(unittest.TestCase):
    def setUp(self):
        self.src = BRIDGE.read_text(encoding="utf-8")

    def test_no_count_based_trigger_remains_in_either_direction(self):
        """两个方向都要改。

        出站那个先改了，入站这个被落下 —— 而它是同一个 bug：按条数触发在最快的
        topic 上最吵，在真停了才要紧的慢 topic 上几乎不吭声。入站今天恰好都是低速
        topic，所以它没在刷屏，但哪天有个高速 topic 接进来就会。
        """
        import re
        leftovers = re.findall(r"msg_count % \d+", self.src)
        self.assertEqual(leftovers, [], f"还有按条数的触发: {leftovers}")

    def test_both_directions_report_on_the_clock(self):
        outbound = ast.unparse(self._method("TopicHandler", "publish"))
        inbound = ast.unparse(self._method("SubscriptionHandler", "_forward"))
        for name, body in (("publish", outbound), ("_forward", inbound)):
            self.assertIn("PROGRESS_INTERVAL_S", body, f"{name} 不是按时间计")
            self.assertIn("msg_count == 1", body, f"{name} 丢了首条立即上报")

    @staticmethod
    def _method(cls_name, fn_name):
        tree = ast.parse(BRIDGE.read_text(encoding="utf-8"))
        cls = next(n for n in tree.body
                   if isinstance(n, ast.ClassDef) and n.name == cls_name)
        return next(n for n in cls.body
                    if isinstance(n, ast.FunctionDef) and n.name == fn_name)

    def test_the_interval_is_a_named_constant(self):
        self.assertIn("PROGRESS_INTERVAL_S", self.src)
        tree = ast.parse(self.src)
        val = next(n.value.value for n in tree.body
                   if isinstance(n, ast.Assign)
                   and getattr(n.targets[0], "id", "") == "PROGRESS_INTERVAL_S")
        self.assertGreaterEqual(val, 60.0, "太短就退化回刷屏")

    def test_errors_are_not_rate_limited(self):
        """限流只针对进度行；发布失败每次都要看得见。"""
        publish = self.src[self.src.index("def publish(self"):]
        publish = publish[:publish.index("\n    def ")] if "\n    def " in publish else publish
        err = publish[publish.index("except Exception"):]
        self.assertIn("ERROR publishing", err)
        self.assertNotIn("_last_progress_ts", err)


if __name__ == "__main__":
    unittest.main()
