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


NEVER = object()        # COMPLETED 永远不来


def plan(duration, *, progress_from=0.05, progress_every=0.05,
         progress_until=None, event_at=None, event=1):
    """一条播放计划。

    真实的 lyre 每 0.5s 发一条 progress（position 单调推进），播完时最后一条
    progress 与 COMPLETED 相隔 0.14s。用例里把这个节奏按比例缩小。

    `progress_until=None` 表示 progress 一直发到 `event_at`；给一个更小的值就是
    **播放中途卡死**（progress 停了但 COMPLETED 永远不来）—— 10-15 14:34 的形状。

    `event_at=NEVER` 表示 COMPLETED 永远不来。必须是显式哨兵而不是 None：None 在这里
    的含义是「沿用 duration」，拿它表达「永不」会静默变成「按时完成」，用例于是以
    completed 通过，看着像在测超时其实什么都没测到。
    """
    if event_at is None:
        event_at = duration
    if progress_until is None:
        progress_until = duration if event_at is NEVER else event_at
    return {"duration": duration, "progress_from": progress_from,
            "progress_every": progress_every, "progress_until": progress_until,
            "event_at": None if event_at is NEVER else event_at, "event": event}


DEAD = None             # lyre 收下了但永远不回任何事件（合成失败 / 队头堵塞）


class FakeLyre:
    """记录每次 play_text，并按计划回放 progress 流 / event。"""

    def __init__(self, plugin, script):
        self.plugin = plugin
        self.script = list(script)
        self.submitted = []        # [(sid, text, force, last)]
        self.timers = []
        self.progress_counts = {}
        self.srv_type = types.SimpleNamespace(Request=_Req)

    def call_async(self, req):
        n = len(self.submitted)
        sid = f"sid{n}"
        self.submitted.append((sid, req.text, req.force, req.last))
        p = self.script[n] if n < len(self.script) else plan(0.2)
        if p is not None:
            t = p["progress_from"]
            pos = 0.0
            while t <= p["progress_until"] + 1e-9:
                self._at(t, self._progress, sid, pos, p["duration"])
                pos += p["progress_every"]
                t += p["progress_every"]
            if p["event_at"] is not None and p["event"] is not None:
                self._at(p["event_at"], self._event, sid, p["event"])
        return _Future(_Resp(sid))

    def _at(self, delay, fn, *args):
        timer = threading.Timer(delay, fn, args=args)
        timer.daemon = True
        timer.start()
        self.timers.append(timer)

    def _progress(self, sid, position, duration):
        self.progress_counts[sid] = self.progress_counts.get(sid, 0) + 1
        self.plugin._on_play_progress(types.SimpleNamespace(
            sid=sid, position=position, duration=duration))

    def _event(self, sid, code):
        self.plugin._on_play_event(types.SimpleNamespace(sid=sid, event=code))


class FakeStopClient:
    def __init__(self):
        self.calls = 0
        self.srv_type = types.SimpleNamespace(Request=_Req)


def make_plugin(script, grace=2.0, stall=0.4):
    p = TtsPlugin.__new__(TtsPlugin)
    p._jobs = queue.Queue(maxsize=TtsPlugin._QUEUE_MAX)
    p._queue_lock = threading.RLock()
    p._worker = None
    p._current_job = None
    p._pending_play = {}
    p._pending_play_status = {}
    p._pending_play_duration = {}
    p._pending_play_progress_at = {}
    p._pending_play_position = {}
    p._play_event_buffer = {}
    p._startup_error = None
    p._PLAY_START_GRACE_S = grace
    p._PROGRESS_STALL_S = stall

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
    p = make_plugin(script=[plan(0.35), plan(0.25)])
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
    p = make_plugin(script=[plan(0.6), plan(0.15)])
    p._speak("长的一段。", force=False)
    p._speak("短的一段。", force=False)

    assert wait_for(lambda: len(p.lyre.submitted) == 1)
    time.sleep(0.3)
    assert len(p.lyre.submitted) == 1, "第二条在第一条播完前就提交了"
    assert wait_for(lambda: len(p.acp) == 2), p.acp


# ── 2. interrupt 才是打断 ────────────────────────────────────────────────────

def test_interrupt_stops_lyre_and_cancels_everything_queued():
    p = make_plugin(script=[plan(5.0), DEAD, DEAD], grace=5.0)
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
    p = make_plugin(script=[plan(5.0), plan(0.15)], grace=5.0)
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
    p = make_plugin(script=[DEAD], grace=0.6)
    job = p._speak("一段不会被 lyre 回事件的话。", force=False)

    assert wait_for(lambda: len(p.acp) == 1), p.acp
    assert p.acp == [(job["action_id"], "error")]
    assert p._stop_client.calls >= 1, "判超时之前没有停 lyre"


def test_failed_segment_does_not_queue_the_next_segment():
    """多段里第一段失败，不许接着把第二段塞进 lyre。

    13:18–13:21 的 3 分钟静默就是这么攒出来的。
    """
    long_text = "甲" * 300 + "。" + "乙" * 200 + "。"
    p = make_plugin(script=[DEAD, plan(0.15)], grace=0.6)
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
    p = make_plugin(script=[plan(1.2, event_at=1.0)], grace=0.5, stall=0.4)
    job = p._speak("七个字的一句话", force=False)

    assert wait_for(lambda: len(p.acp) == 1, timeout=5), p.acp
    assert p.acp == [(job["action_id"], "completed")]
    assert p._stop_client.calls == 0, "不该停 lyre"


def test_mid_playback_stall_is_caught_by_the_progress_watchdog():
    """开播之后 progress 断流 = 播放器卡死 → 停 lyre 并报错，不能假成功。

    10-15 14:34 的真机形状：lyre 播到 31.6/48.6s 卡死，position 在 32 秒墙钟里只走了
    0.5s。当时的规则是「首条 progress + duration + 8s」一次算定，所以要等整个
    duration 走完才察觉 —— 声音早停了 32 秒。按 progress 续期之后，停滞阈值到点就判。
    """
    p = make_plugin(script=[plan(2.0, progress_until=0.3, event_at=NEVER)], grace=5.0, stall=0.4)
    job = p._speak("开了播但不报完成。", force=False)

    assert wait_for(lambda: len(p.acp) == 1, timeout=5), p.acp
    assert p.acp == [(job["action_id"], "error")]
    assert p._stop_client.calls >= 1


def test_playback_longer_than_reported_duration_is_not_killed():
    """真实播放时长超过 lyre 报的 duration 时，只要 progress 还在推进就必须继续等。

    这是「按 progress 续期」相对「首条 progress + duration + 余量」的核心差别。后者
    在这个场景下会把话从中间切断 —— 而被误杀的代价是用户听到半句话，比多等几秒糟得多。
    """
    # duration 报 0.4s，progress 一直发到 1.2s，COMPLETED 也在 1.2s。
    # 一次算定的旧规则（0.4 + 余量）会在 1.2s 之前就判超时。
    p = make_plugin(script=[plan(0.4, progress_until=1.2, event_at=1.2)],
                    grace=0.5, stall=0.4)
    job = p._speak("一句被 lyre 低报了时长的话。", force=False)

    assert wait_for(lambda: len(p.acp) == 1, timeout=6), p.acp
    assert p.acp == [(job["action_id"], "completed")]
    assert p.stops == [], "把还在推进的播放停掉了"


def test_stall_is_detected_in_about_one_stall_window_not_one_duration():
    """卡死的发现时间应该是「一个停滞窗口」量级，而不是「一整个 duration」。

    14:34 那次 duration=48.6s，卡死在 31.6s 处，旧规则用了 32 秒才发现。续期之后，
    发现时间与 duration 无关。
    """
    import time as _t
    # duration 报 10s（相当于真机的 48.6s），progress 在 0.3s 就断（相当于卡死）
    p = make_plugin(script=[plan(10.0, progress_until=0.3, event_at=NEVER)],
                    grace=5.0, stall=0.5)
    t0 = _t.time()
    job = p._speak("一句播到一半就卡死的话。", force=False)
    assert wait_for(lambda: len(p.acp) == 1, timeout=6), p.acp
    elapsed = _t.time() - t0

    assert p.acp == [(job["action_id"], "error")]
    assert p.stops, "卡死后没有停 lyre"
    # 0.3s 断流 + 0.5s 停滞窗口 ≈ 0.8s；给足轮询余量也该远小于 duration
    assert elapsed < 3.0, f"用了 {elapsed:.1f}s 才发现，接近一整个 duration"


def test_chatty_but_never_completing_player_still_fails_eventually():
    """一直发 progress 却永远不 COMPLETED 的病态播放器不能把队列挂死。"""
    p = make_plugin(script=[plan(0.3, progress_until=30.0, event_at=NEVER)],
                    grace=0.4, stall=5.0)
    job = p._speak("一句永远不报完成的话。", force=False)

    # 上限 = duration * _PLAY_DURATION_CAP_FACTOR + grace = 0.3*3 + 0.4 ≈ 1.3s
    assert wait_for(lambda: len(p.acp) == 1, timeout=8), p.acp
    assert p.acp == [(job["action_id"], "error")]
    assert p.stops


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
