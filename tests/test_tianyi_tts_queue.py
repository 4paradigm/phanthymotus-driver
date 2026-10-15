"""天轶 TTS：speak 必须排队，不能互相覆盖；判超时之前必须先停 lyre。

2026-10-15 展厅导览实测到的两个故障，根因都在 `TtsPlugin` 对 lyre 的一个错误假设：
`/audio_play/play_text` 是**进队列**，不是开始播。

1. `_speak` 原先是 last-writer-wins —— 每条新 speak 把上一条的 cancel_event 置位。
   自动播报（on_notify）在 LLM 的讲解发出后 16 ms 到达，于是讲解那条被报成
   cancelled，agent-core 的 ACP barrier 认为嘴空了，3 秒后导航发车；而 lyre 把那段
   269 字的讲解完整播了 47 秒。机器人边走边讲，也就是「还没播完就走了」。
   顺带：那条路径想对 lyre 发 stop，但请求写成 `type(client.srv_type.Request)()`
   —— 元类，调用必抛 TypeError，又被 bare except 吞掉，所以 stop 从来没发出去过。

2. 段超时按字数估（280 字 → 122 秒，真实时长 48.1 秒），超时后判 error 并**继续播
   下一段**，而 lyre 可能还压着上一段。那次云端合成 connect 超时（ws://wsapi.xfyun.cn，
   websocketpp 默认 5 s，不可配不重试）把死任务留在 lyre 队头 150 秒，后面两段就这么
   在它后面排着，最终 3 分钟静默。

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_tianyi_tts_queue.py -q
"""

from __future__ import annotations

import ast
import queue
import sys
import threading
import time
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "x-humanoid" / "tianyi2.0"


def load_tts_class():
    """只把 TtsPlugin 这一个类取出来执行 —— device.py 整体导入要 rclpy。"""
    tree = ast.parse((BUNDLE / "device.py").read_text(encoding="utf-8"))
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == "TtsPlugin")
    module = ast.Module(
        body=[ast.ImportFrom(module="__future__",
                             names=[ast.alias(name="annotations")], level=0), cls],
        type_ignores=[])
    ns = {"threading": threading, "Node": object}
    exec(compile(ast.fix_missing_locations(module), str(BUNDLE / "device.py"), "exec"), ns)
    return ns["TtsPlugin"]


TtsPlugin = load_tts_class()


# ── lyre 替身 ────────────────────────────────────────────────────────────────

class _Req:
    text = ""
    force = False
    last = False


class _PlayText:
    Request = _Req


@pytest.fixture(autouse=True)
def fake_lyre_msgs(monkeypatch):
    """`_play_job` 里是 `from lyre_msgs.srv import PlayText`（刻意延迟导入，笔记本上
    没有这个包）。给它一个替身，这样这套用例完全不需要 ROS。

    必须随用例装卸，不能在模块顶层 `sys.modules.setdefault` —— 那是进程级的，会让
    后面任何「import 得到 lyre_msgs 就不跳过」的用例改变行为，而且只在同一次 pytest
    进程里才复现。
    """
    srv = types.ModuleType("lyre_msgs.srv")
    srv.PlayText = _PlayText
    pkg = types.ModuleType("lyre_msgs")
    pkg.srv = srv
    monkeypatch.setitem(sys.modules, "lyre_msgs", pkg)
    monkeypatch.setitem(sys.modules, "lyre_msgs.srv", srv)
    yield


class _Resp:
    def __init__(self, sid):
        self.sid = sid


class _Future:
    def __init__(self, resp):
        self._resp = resp

    def done(self):
        return True

    def result(self):
        return self._resp


class FakeLyre:
    """记录每次 play_text，并按脚本回放 progress / event。

    `script` 是 per-submission 的列表，每项 None 表示「永远不回任何事件」（合成失败
    或队头堵塞），否则是 (duration, delay_progress, delay_event)。
    """

    def __init__(self, plugin, script):
        self.plugin = plugin
        self.script = list(script)
        self.submitted = []        # [(sid, text, force, last)]
        self.timers = []
        self.srv_type = types.SimpleNamespace(Request=_Req)

    def call_async(self, req):
        n = len(self.submitted)
        sid = f"sid{n}"
        self.submitted.append((sid, req.text, req.force, req.last))
        plan = self.script[n] if n < len(self.script) else (0.2, 0.05, 0.2)
        if plan is not None:
            duration, d_prog, d_ev = plan
            t1 = threading.Timer(d_prog, self._progress, args=(sid, duration))
            t2 = threading.Timer(d_ev, self._event, args=(sid, 1))
            for t in (t1, t2):
                t.daemon = True
                t.start()
                self.timers.append(t)
        return _Future(_Resp(sid))

    def _progress(self, sid, duration):
        self.plugin._on_play_progress(types.SimpleNamespace(sid=sid, duration=duration))

    def _event(self, sid, code):
        self.plugin._on_play_event(types.SimpleNamespace(sid=sid, event=code))


class FakeStopClient:
    def __init__(self):
        self.calls = 0
        self.srv_type = types.SimpleNamespace(Request=_Req)


def make_plugin(script, grace=2.0, slack=0.4):
    p = TtsPlugin.__new__(TtsPlugin)
    p._jobs = queue.Queue(maxsize=TtsPlugin._QUEUE_MAX)
    p._queue_lock = threading.RLock()
    p._worker = None
    p._current_job = None
    p._pending_play = {}
    p._pending_play_status = {}
    p._pending_play_duration = {}
    p._pending_play_progress_at = {}
    p._play_event_buffer = {}
    p._startup_error = None
    p._PLAY_START_GRACE_S = grace
    p._PLAY_SLACK_S = slack

    lyre = FakeLyre(p, script)
    p._play_client = lyre
    p._stop_client = FakeStopClient()
    p.lyre = lyre

    p.acp = []
    p._acp_lock = threading.Lock()

    def _record(job, status):
        with p._acp_lock:
            p.acp.append((job["action_id"], status))

    p._acp_callback = _record

    stops = []

    def _stop(client, action_name):
        stops.append(action_name)
        client.calls += 1
        # 真实实现走的就是这一行；旧代码写成 type(...Request)() 会抛 TypeError。
        client.srv_type.Request()
        return {"state": action_name}

    p._call_empty_service = types.MethodType(
        lambda self, client, action_name: _stop(client, action_name), p)
    p.stops = stops
    return p


def wait_for(pred, timeout=6.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


# ── 1. 排队，不覆盖 ──────────────────────────────────────────────────────────

def test_two_speaks_both_play_in_order_and_each_gets_one_acp():
    """第二条 speak 不许把第一条顶掉。

    这是 13:06 那次「还没播完就走了」的最小复现：第二条（自动播报）到达时，第一条
    （讲解）必须继续，并且在它真的播完之后才报 completed。
    """
    p = make_plugin(script=[(0.3, 0.05, 0.35), (0.2, 0.05, 0.25)])
    first = p._speak("第一段讲解。", force=False)
    second = p._speak("自动播报。", force=False)

    assert first["state"] == "speaking"
    assert second["state"] == "queued" and second["queued_ahead"] >= 1

    assert wait_for(lambda: len(p.acp) == 2), p.acp
    assert p.acp == [(first["action_id"], "completed"),
                     (second["action_id"], "completed")], p.acp
    # 两条都真的发给了 lyre，而且是先后两次，不是被合并或丢弃
    assert [s[1] for s in p.lyre.submitted] == ["第一段讲解。", "自动播报。"]
    # 没有任何人去停 lyre —— 排队不是打断
    assert p.stops == [], p.stops


def test_second_speak_is_submitted_only_after_the_first_finished():
    """单消费者：前一条没播完，后一条不许进 lyre 的队列。

    否则就还是今天那个形状：两段在 lyre 里排队，而我们这边以为各自独立。
    """
    p = make_plugin(script=[(0.6, 0.05, 0.6), (0.1, 0.05, 0.15)])
    p._speak("长的一段。", force=False)
    p._speak("短的一段。", force=False)

    assert wait_for(lambda: len(p.lyre.submitted) == 1)
    time.sleep(0.3)
    assert len(p.lyre.submitted) == 1, "第二条在第一条播完前就提交了"
    assert wait_for(lambda: len(p.acp) == 2), p.acp


# ── 2. interrupt 才是打断 ────────────────────────────────────────────────────

def test_interrupt_stops_lyre_and_cancels_everything_queued():
    p = make_plugin(script=[(5.0, 0.05, 5.0), None, None], grace=5.0)
    a = p._speak("正在播的。", force=False)
    b = p._speak("排队的一。", force=False)
    c = p._speak("排队的二。", force=False)

    assert wait_for(lambda: len(p.lyre.submitted) == 1)
    p.dispatch("interrupt", {})

    assert wait_for(lambda: len(p.acp) == 3), p.acp
    by_id = dict(p.acp)
    assert by_id[a["action_id"]] == "cancelled"
    assert by_id[b["action_id"]] == "cancelled"
    assert by_id[c["action_id"]] == "cancelled"
    # 排队的两条一次都没进 lyre
    assert len(p.lyre.submitted) == 1
    assert p._stop_client.calls >= 1, "interrupt 没有停 lyre"


def test_force_speak_interrupts_then_plays():
    """force 的既有语义是「打断当前播放」，保留。"""
    p = make_plugin(script=[(5.0, 0.05, 5.0), (0.1, 0.05, 0.15)], grace=5.0)
    a = p._speak("正在播的。", force=False)
    assert wait_for(lambda: len(p.lyre.submitted) == 1)
    b = p._speak("插队的。", force=True)

    assert wait_for(lambda: len(p.acp) == 2), p.acp
    by_id = dict(p.acp)
    assert by_id[a["action_id"]] == "cancelled"
    assert by_id[b["action_id"]] == "completed"
    assert p._stop_client.calls >= 1


# ── 3. 超时：先停 lyre，且不续播后面的段 ─────────────────────────────────────

def test_no_playprogress_within_grace_stops_lyre_and_fails_the_job():
    """合成失败/队头堵塞 → 20s（这里缩短）内没有任何 PlayProgress。

    旧代码按字数估一个时长继续等，到点判 error 就走，而 lyre 之后才把音频放出来。
    现在必须先把 lyre 停掉再报错。
    """
    p = make_plugin(script=[None], grace=0.6)
    job = p._speak("一段不会被 lyre 回事件的话。", force=False)

    assert wait_for(lambda: len(p.acp) == 1), p.acp
    assert p.acp == [(job["action_id"], "error")]
    assert p._stop_client.calls >= 1, "判超时之前没有停 lyre"


def test_failed_segment_does_not_queue_the_next_segment():
    """多段里第一段失败，不许接着把第二段塞进 lyre。

    13:18–13:21 的 3 分钟静默就是这么攒出来的。
    """
    long_text = "甲" * 300 + "。" + "乙" * 200 + "。"
    p = make_plugin(script=[None, (0.1, 0.05, 0.15)], grace=0.6)
    job = p._speak(long_text, force=False)
    assert job["segments"] == 2, "用例前提：这段文本应被切成两段"

    assert wait_for(lambda: len(p.acp) == 1), p.acp
    assert p.acp == [(job["action_id"], "error")]
    assert len(p.lyre.submitted) == 1, "第一段失败后仍然提交了第二段"


def test_playprogress_duration_beats_the_character_estimate():
    """真实时长比字数估算长时，必须按 PlayProgress 的 duration 等，而不是提前判错。

    反过来也成立：字数估算宽了 2.5 倍，提前判错是「话被切断」，等太久是「静默」。
    """
    # 7 个字：字数兜底会给出 7/2.5+10 ≈ 12.8s；这里 grace 只有 0.5s，
    # 只有 progress 续约（duration=1.2s）才能等到 1.0s 的 COMPLETED。
    p = make_plugin(script=[(1.2, 0.1, 1.0)], grace=0.5, slack=0.4)
    job = p._speak("七个字的一句话", force=False)

    assert wait_for(lambda: len(p.acp) == 1, timeout=5), p.acp
    assert p.acp == [(job["action_id"], "completed")]
    assert p._stop_client.calls == 0, "不该停 lyre"


def test_overrunning_playback_stops_lyre_instead_of_claiming_success():
    """开播了但 duration + 余量到点还没 COMPLETED → 停 lyre 并报错，不能假成功。"""
    p = make_plugin(script=[(0.3, 0.05, 30.0)], grace=5.0, slack=0.2)
    job = p._speak("开了播但不报完成。", force=False)

    assert wait_for(lambda: len(p.acp) == 1, timeout=5), p.acp
    assert p.acp == [(job["action_id"], "error")]
    assert p._stop_client.calls >= 1


# ── 4. stop 请求的构造 ──────────────────────────────────────────────────────

def test_stop_request_is_constructed_from_the_class_not_its_metaclass():
    """`type(Request)()` 会抛 TypeError，被 bare except 吞掉后 stop 静默失效。

    在真机上实测：`type.__new__() takes exactly 3 arguments (0 given)`。
    """
    src = (BUNDLE / "device.py").read_text(encoding="utf-8")
    assert "type(self._stop_client.srv_type.Request)()" not in src
    # 正确写法在 _call_empty_service 里，且它是唯一的构造点
    assert "req = client.srv_type.Request()" in src


def test_no_cancel_event_single_slot_left_in_the_plugin():
    """last-writer-wins 的那个单槽位必须整个消失，否则两种语义会并存。"""
    src = (BUNDLE / "device.py").read_text(encoding="utf-8")
    tts = src[src.index("class TtsPlugin"):src.index("class VoicePlayActuatorPlugin")]
    code = [ln for ln in tts.split("\n")
            if "self._cancel_event" in ln and not ln.lstrip().startswith("#")]
    assert code == [], code
