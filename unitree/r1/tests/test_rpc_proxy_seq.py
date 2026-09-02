"""
RpcProxy: one lock, one command/result queue pair, shared by loco + arm + audio.

`RunFsmSequence` holds that lock for the whole posture change. While switch_mode
was synchronous the ACP barrier guaranteed nothing else ran during that window;
now that dispatch returns immediately the turn keeps going, so TTS / speaker / LED
calls really do land on the proxy mid-sequence. That makes a latent bug reachable:

    try:
        r = self._result_q.get(timeout=timeout)
    except Exception:
        return None          # the command is STILL in flight

The late reply used to stay in the queue and be handed to the *next* caller,
desyncing every subsequent call by one. `GetFsmId()` would then return whatever
the previous command produced -- and every posture guard keys off that reading.

These tests drive the real RpcProxy._call against a fake queue pair; no subprocess,
no robot.

Run:  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest unitree/r1/tests -q
"""

import importlib.util
import queue
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

spec = importlib.util.spec_from_file_location("r1_rpc_proxy_under_test", ROOT / "rpc_proxy.py")
RP = importlib.util.module_from_spec(spec)
sys.modules["r1_rpc_proxy_under_test"] = RP
spec.loader.exec_module(RP)


class FakeProxy(RP.RpcProxy):
    """RpcProxy without the spawned subprocess -- plain queues stand in for it."""

    def __init__(self):
        self._cmd_q = queue.Queue()
        self._result_q = queue.Queue()
        self._proc = None
        self._lock = threading.Lock()
        self._seq = 0

    def take_cmd(self, timeout=1.0):
        return self._cmd_q.get(timeout=timeout)

    def reply(self, seq, result):
        self._result_q.put({"seq": seq, "result": result})


def test_every_command_carries_a_sequence_number():
    p = FakeProxy()
    t = threading.Thread(target=lambda: p._call("loco", "GetFsmId"), daemon=True)
    t.start()
    cmd = p.take_cmd()
    assert cmd["seq"] == 1
    assert cmd["client"] == "loco" and cmd["method"] == "GetFsmId"
    p.reply(cmd["seq"], (0, 811))
    t.join(timeout=2)


def test_normal_call_returns_its_own_result():
    p = FakeProxy()
    out = []
    t = threading.Thread(target=lambda: out.append(p._call("loco", "GetFsmId")), daemon=True)
    t.start()
    cmd = p.take_cmd()
    p.reply(cmd["seq"], (0, 811))
    t.join(timeout=2)
    assert out == [(0, 811)]


def test_a_late_reply_does_not_poison_the_next_call(capsys):
    """The regression. Call A times out; its reply arrives afterwards. Call B must
    get B's result, not A's -- otherwise GetFsmId() returns an FSM sequence dict and
    every posture guard is reading garbage."""
    p = FakeProxy()

    # Call A: times out with nothing in the queue.
    assert p._call("loco", "__run_fsm_sequence", timeout=0.15) is None
    cmd_a = p.take_cmd()

    # A's reply lands late.
    p.reply(cmd_a["seq"], {"ret": 0, "steps": ["standup2lie"], "fsm_id": 1})

    # Call B must skip it.
    out = []
    t = threading.Thread(target=lambda: out.append(p._call("loco", "GetFsmId")), daemon=True)
    t.start()
    cmd_b = p.take_cmd()
    assert cmd_b["seq"] != cmd_a["seq"]
    p.reply(cmd_b["seq"], (0, 811))
    t.join(timeout=2)

    assert out == [(0, 811)], f'call B got a stale reply: {out}'
    log = capsys.readouterr().out
    assert 'TIMEOUT' in log
    assert 'discarding stale reply' in log


def test_timeout_is_logged_loudly(capsys):
    p = FakeProxy()
    assert p._call("loco", "Damp", timeout=0.1) is None
    out = capsys.readouterr().out
    assert 'loco.Damp TIMEOUT' in out
    assert 'discarded' in out


def test_several_stale_replies_are_all_skipped():
    p = FakeProxy()
    for _ in range(3):
        assert p._call("audio", "TtsMaker", timeout=0.05) is None
    stale = [p.take_cmd() for _ in range(3)]
    for c in stale:
        p.reply(c["seq"], "stale")

    out = []
    t = threading.Thread(target=lambda: out.append(p._call("loco", "GetFsmId")), daemon=True)
    t.start()
    cmd = p.take_cmd()
    p.reply(cmd["seq"], (0, 4))
    t.join(timeout=2)
    assert out == [(0, 4)]


def test_worker_errors_still_surface_as_none(capsys):
    p = FakeProxy()
    out = []
    t = threading.Thread(target=lambda: out.append(p._call("loco", "Damp")), daemon=True)
    t.start()
    cmd = p.take_cmd()
    p._result_q.put({"seq": cmd["seq"], "error": "boom"})
    t.join(timeout=2)
    assert out == [None]
    assert 'loco.Damp error: boom' in capsys.readouterr().out


def test_a_long_call_blocks_others_and_says_so(capsys):
    """This is what changed: while switch_mode was synchronous nothing else reached
    the proxy during a sequence. Now TTS can, and it waits behind the whole posture
    change -- so the wait has to be visible."""
    p = FakeProxy()
    started = threading.Event()

    def slow():
        started.set()
        p._call("loco", "__run_fsm_sequence", timeout=5)

    t = threading.Thread(target=slow, daemon=True)
    t.start()
    started.wait(1)
    cmd_slow = p.take_cmd()

    out = []
    t2 = threading.Thread(target=lambda: out.append(p._call("audio", "TtsMaker")), daemon=True)
    t2.start()

    time.sleep(0.7)                       # t2 is parked on the lock
    assert out == [], 'the second caller should still be blocked'
    p.reply(cmd_slow["seq"], {"ret": 0})
    t.join(timeout=2)

    cmd_fast = p.take_cmd(timeout=2)
    p.reply(cmd_fast["seq"], 0)
    t2.join(timeout=2)

    assert out == [0]
    assert 'waited' in capsys.readouterr().out


def test_call_code_and_call_tuple_report_failure_codes():
    """Callers unpack these; None would raise instead of being handled."""
    p = FakeProxy()
    assert p._call_code("loco", "Damp", timeout=0.05) == 3104
    assert p._call_tuple("loco", "GetFsmId", timeout=0.05) == (3104, None)
