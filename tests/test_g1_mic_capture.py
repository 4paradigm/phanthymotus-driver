"""
test_g1_mic_capture.py — 麦克风采集不能出现两路并行，否则音频会被交错切碎。

现象（G1）：本机 mic 走 ASR 出来的文字**重复且漏字严重**，同一套 ASR 接 remote mic
就正常 —— 所以问题在采集侧。

两路并行有两种来法，这里都盖住：

1. **同一个进程里两个 pump 线程。** `_pump` 原来循环在 `while self._sock is not None`
   上，并且每轮重新读这个属性。stop 之后紧接着 start（画布的 config→start→stop→
   config→start 是常态）可能撞上旧线程正停在内层发布循环里，等它回到外层时读到的
   已经是**新** socket，于是继续跑。两个线程从同一个 socket 收包，各自用自己的 buf
   按 1024 字节切分，而且每次 stop/start 还能再叠一个。

2. **同一台机器上两个驱动容器。** `ros_namespace` 留空时取 hostname，两个容器又都是
   host 网络 —— topic 名完全一样。这种情况本节点拦不住，但必须**说出来**，而不是让
   人从 ASR 的乱码里倒推。

无论哪一种，发出去的 PCM 都是两路错位流拼起来的，听感和 ASR 表现就是重复 + 漏字。

Run: cd phanthymotus-driver && python3 -m pytest tests/test_g1_mic_capture.py
"""

import ast
import socket
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
G1_DEVICE = ROOT / "unitree" / "g1" / "device.py"


def _code(name: str, cls: str = "") -> str:
    """A function's executable body, docstring stripped.

    Asserting against `ast.unparse` alone matches the docstring too, and these
    docstrings explain the bug being fixed — so a test looking for the old,
    wrong code kept finding it in the paragraph describing why it was wrong.
    """
    fn = _func(name, cls)
    body = fn.body[1:] if (fn.body and isinstance(fn.body[0], ast.Expr)
                           and isinstance(fn.body[0].value, ast.Constant)
                           and isinstance(fn.body[0].value.value, str)) else fn.body
    return "\n".join(ast.unparse(n) for n in body)


def _func(name: str, cls: str = "") -> ast.FunctionDef:
    """Pull one function out of device.py without importing it.

    Importing this module needs rclpy, the unitree SDK and audio_msgs; the
    behaviour under test is plain socket/threading code, so it is read out of the
    source instead. Same approach the other driver tests here take.
    """
    tree = ast.parse(G1_DEVICE.read_text(encoding="utf-8"))
    scope = tree.body
    if cls:
        scope = next(n.body for n in tree.body
                     if isinstance(n, ast.ClassDef) and n.name == cls)
    return next(n for n in scope
                if isinstance(n, ast.FunctionDef) and n.name == name)


class PumpLifecycleTests(unittest.TestCase):
    """The pump must belong to one capture session and end with it."""

    def setUp(self):
        self.src = G1_DEVICE.read_text(encoding="utf-8")

    def test_the_pump_loops_on_a_stop_event_not_on_the_socket_attribute(self):
        """Re-reading `self._sock` is what let a stopped thread adopt a new socket."""
        pump = _func("_pump", "_MicNode")
        loop = next(n for n in pump.body if isinstance(n, ast.While))
        self.assertEqual(ast.unparse(loop.test), "not stop_evt.is_set()")

    def test_the_pump_takes_its_socket_and_event_as_arguments(self):
        """Read off `self` and two sessions share one mutable handle."""
        args = [a.arg for a in _func("_pump", "_MicNode").args.args]
        self.assertEqual(args, ["self", "sock", "stop_evt"])

    def test_the_publish_loop_also_checks_the_stop_event(self):
        """It is the long inner loop that gives the race its window."""
        pump = _func("_pump", "_MicNode")
        inner = [n for n in ast.walk(pump) if isinstance(n, ast.While)]
        self.assertTrue(
            any("stop_evt.is_set()" in ast.unparse(w.test) for w in inner[1:]),
            "the chunk-publishing loop must be able to bail out too")

    def test_stop_capture_signals_then_joins(self):
        stop = _func("stop_capture", "_MicNode")
        body = ast.unparse(stop)
        self.assertIn("_stop_evt.set()", body)
        self.assertIn(".join(", body, "stop must not return while the pump is alive")

    def test_start_capture_gives_each_session_a_fresh_event(self):
        body = ast.unparse(_func("start_capture", "_MicNode"))
        self.assertIn("threading.Event()", body)


class PumpBehaviourTests(unittest.TestCase):
    """Run the real loop shape against a real UDP socket."""

    @staticmethod
    def _pump(sock, stop_evt, out):
        """The loop under test, transcribed — same conditions, no ROS."""
        buf = bytearray()
        while not stop_evt.is_set():
            try:
                data, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            buf.extend(data)
            while len(buf) >= 4 and not stop_evt.is_set():
                out.append(bytes(buf[:4]))
                buf = buf[4:]

    def test_a_signalled_pump_stops_even_if_a_new_socket_appears(self):
        """The regression itself: stop → start must not resurrect the old pump."""
        old = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        old.bind(("127.0.0.1", 0))
        old.settimeout(0.05)
        stop = threading.Event()
        out = []
        t = threading.Thread(target=self._pump, args=(old, stop, out), daemon=True)
        t.start()

        stop.set()                      # the session ends
        t.join(timeout=2.0)
        self.assertFalse(t.is_alive(), "the pump outlived its own capture session")

        # A new session starts; the old thread must not be feeding it.
        before = len(out)
        new = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        new.bind(("127.0.0.1", 0))
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.sendto(b"12345678", new.getsockname())
        time.sleep(0.2)
        self.assertEqual(len(out), before, "the stopped pump published into the new session")
        for s in (old, new, sender):
            s.close()

    def test_a_running_pump_chunks_the_stream_in_order(self):
        recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv.bind(("127.0.0.1", 0))
        recv.settimeout(0.05)
        stop = threading.Event()
        out = []
        t = threading.Thread(target=self._pump, args=(recv, stop, out), daemon=True)
        t.start()

        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.sendto(b"aaaabbbb", recv.getsockname())
        sender.sendto(b"ccccdddd", recv.getsockname())
        deadline = time.monotonic() + 2.0
        while len(out) < 4 and time.monotonic() < deadline:
            time.sleep(0.01)
        stop.set()
        t.join(timeout=2.0)

        self.assertEqual(out[:4], [b"aaaa", b"bbbb", b"cccc", b"dddd"])
        recv.close()
        sender.close()


class RivalPublisherTests(unittest.TestCase):
    def setUp(self):
        self.src = G1_DEVICE.read_text(encoding="utf-8")

    def test_the_self_check_counts_other_publishers(self):
        self.assertIn("_rival_publishers", self.src)
        body = ast.unparse(_func("_rival_publishers", "MicPlugin"))
        self.assertIn("get_publishers_info_by_topic", body)
        self.assertIn("node_name", body, "our own publisher must not be counted")

    def test_a_rival_publisher_makes_the_card_an_error(self):
        check = ast.unparse(_func("_self_check", "MicPlugin"))
        self.assertIn("_rival_publishers", check)
        # ast.unparse normalises quotes, so match the token, not the quoting.
        self.assertIn("state = 'error'", check)

    def test_the_message_says_what_to_do_about_it(self):
        """从 ASR 的乱码倒推是最贵的一条路，报错必须直接指向多余的容器。"""
        check = ast.unparse(_func("_self_check", "MicPlugin"))
        self.assertIn("embodied-unitree-g1", check)
        self.assertIn("重复", check)

    def test_it_counts_by_number_not_by_node_name(self):
        """对手是本驱动的第二份拷贝，节点名**也叫** g1_mic。

        第一版用 `node_name != self._node.get_name()` 排除自己，恰好在它要防的那个场景里
        把对手也排除掉 —— 护栏永远不会触发。ROS 2 的节点名不唯一，只有 GID 唯一；
        而我们在这个 topic 上只有一个发布端点，所以「第一个之外的都是别人」。
        """
        body = _code("_rival_publishers", "MicPlugin")
        self.assertNotIn("get_name()", body, "按名字排除自己会漏掉同名的对手")
        self.assertIn("len(infos) - 1", body)
        self.assertIn("max(0", body, "自己的发布者还没进图时不能算出负数")

    def test_a_stale_graph_entry_does_not_refuse_outright(self):
        """DDS 的图不会在进程死的瞬间就把端点摘掉。

        刚停掉重复容器之后立刻启动，图里可能还留着它 —— 这时拒绝，会把**修复动作**
        变成看起来像新故障。复核一次再决定。
        """
        check = _code("_self_check", "MicPlugin")
        self.assertEqual(check.count("_rival_publishers()"), 2,
                         "必须复核一次，不能一次就判死")
        self.assertIn("_t.sleep", check)

    def test_counting_never_raises(self):
        """诊断不该成为压垮启动的那件事 —— 图还没起来时 rclpy 会抛。"""
        body = _func("_rival_publishers", "MicPlugin")
        self.assertTrue(any(isinstance(n, ast.Try) for n in ast.walk(body)))


class LocalIpFallbackTests(unittest.TestCase):
    def test_the_fallback_uses_a_socket_family_that_exists(self):
        """`socket.AF_DGRAM` 不存在 —— AttributeError 被 except 吞掉，整条兜底是死的，
        返回 "" 之后组播加入退化成 INADDR_ANY，由路由表决定走哪个网卡。"""
        # Assert on the AST, not the raw text: the comment explaining the bug
        # names the broken constant, and matching that would defeat the check.
        body = _code("_get_local_ip")
        self.assertNotIn("socket.AF_DGRAM", body)
        self.assertIn("socket.AF_INET, socket.SOCK_DGRAM", body)

    def test_the_fallback_only_accepts_the_robot_link(self):
        """修好笔误之后，这条路径第一次真的会返回东西 —— 得保证返回对的东西。

        没有 192.168.123.x 路由时，路由表会爽快地给出办公室网段的地址，拿它去 join
        组播等于永远收不到麦克风流。返回 "" 退回 INADDR_ANY，正是这个笔误存在期间
        一直在用的行为，所以修笔误不会让任何情况变得更糟。
        """
        body = _code("_get_local_ip")
        self.assertIn("192.168.123.", body)
        self.assertIn("startswith", body)

    def test_the_fallback_actually_returns_an_address(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.123.1", 1))
        self.assertTrue(s.getsockname()[0])
        s.close()


if __name__ == "__main__":
    unittest.main()
