"""No real network: Event-gated ACP transport around the actual safety path."""
import ast
from copy import deepcopy
import json
from pathlib import Path
import threading
import types
import urllib.error

import pytest

from test_bumi_traffic_police_gesture import call, robot, wait_until
from test_traffic_light import load_bumi


class Sender:
    def __init__(self):
        self.entered, self.release = threading.Event(), threading.Event()
        self.calls, self.snapshots = [], []
        self.error = None
        self.on_enter = None

    def __call__(self, *args):
        self.calls.append(args)
        if self.on_enter:
            self.on_enter()
        self.entered.set()
        if not self.release.wait(2):
            raise TimeoutError("test transport was not released")
        self.snapshots.append(deepcopy(args))
        if self.error:
            raise self.error
        return {"state": "sent", "error": None}


@pytest.fixture
def sender(robot):
    sender = Sender()
    delivery = robot.plugin._traffic_gesture._delivery
    delivery.sender = sender
    yield sender
    sender.release.set()
    robot.plugin._traffic_gesture.shutdown()
    if delivery.worker:
        delivery.worker.join(2)
        assert not delivery.worker.is_alive()


def record(robot, action_id):
    records = robot.plugin._traffic_gesture.info()["acp_notifications"]["records"]
    return next((item for item in records if item["action_id"] == action_id), None)


def terminal(robot, action_id):
    wait_until(lambda: record(robot, action_id) and record(robot, action_id)["state"] in ("sent", "failed"))
    return record(robot, action_id)


def run_in_thread(fn):
    done, results = threading.Event(), []
    def run():
        try:
            results.append(fn())
        except Exception as exc:
            results.append(exc)
        finally:
            done.set()
    worker = threading.Thread(target=run)
    worker.start()
    return worker, done, results


@pytest.mark.parametrize("action", ["cancel", "stop"])
def test_slow_acp_does_not_block_local_cancel_info_or_next_gesture(robot, sender, action):
    card = robot.plugin._traffic_gesture
    first = call(robot.plugin, "go", execute=True)
    def check_locks_and_committed_state():
        assert card.operation_lock.acquire(blocking=False)
        card.operation_lock.release()
        assert card.lock.acquire(blocking=False)
        card.lock.release()
        assert card.active is None and robot.plugin._body_action is None
    sender.on_enter = check_locks_and_committed_state
    worker, done, results = run_in_thread(lambda: call(robot.plugin, action))
    try:
        assert done.wait(1)  # sender remains blocked until this test releases it
        assert sender.entered.wait(1) and not sender.release.is_set()
        assert results[0]["state"] == "idle" and results[0]["playback_exit_confirmed"]
        assert not results[0]["physical_completion_confirmed"]
        reader, read_done, info = run_in_thread(card.info)
        assert read_done.wait(1)
        reader.join(1)
        assert info[0]["gesture_state"] == "idle" and not info[0]["busy"]
        assert record(robot, first["action_id"])["state"] == "sending"
        sender.on_enter = None
        second = call(robot.plugin, "stop", execute=True)
        assert second["state"] == "running" and second["action_id"] != first["action_id"]
        assert robot.plugin._body_action == "traffic_police_gesture.stop"
    finally:
        sender.release.set()
        worker.join(1)


@pytest.mark.parametrize("failure", [TimeoutError("socket timeout"), ConnectionError("connection refused"),
    urllib.error.HTTPError("https://fake.invalid/acp", 503, "unavailable", {}, None)])
def test_network_failure_is_separate_from_local_safety_and_never_retried(robot, sender, failure):
    started = call(robot.plugin, "stop", execute=True)
    sender.error = failure
    result = call(robot.plugin, "cancel")
    local = deepcopy(robot.plugin._traffic_gesture.info()["last_result"])
    sender.release.set()
    delivery = terminal(robot, started["action_id"])
    assert delivery["state"] == "failed" and type(failure).__name__ in delivery["error"]
    info = robot.plugin._traffic_gesture.info()
    assert info["state"] == "ready" and info["gesture_state"] == "idle" and not info["busy"]
    assert info["last_result"] == local and info["last_error"] is None
    assert result["playback_exit_confirmed"] and not result["physical_completion_confirmed"]
    before = list(robot.events)
    for _ in range(5):
        assert call(robot.plugin, "cancel")["state"] == "idle"
    assert robot.events == before and len(sender.calls) == 1


def test_old_inflight_and_queued_snapshots_cannot_pollute_a_new_action(robot, sender):
    card = robot.plugin._traffic_gesture
    first = call(robot.plugin, "stop", execute=True)
    ended_a = call(robot.plugin, "cancel")
    assert sender.entered.wait(1)
    second = call(robot.plugin, "go", execute=True)
    ended_b = call(robot.plugin, "cancel")
    expected_b = deepcopy(ended_b)
    assert card.info()["acp_notifications"]["queued"] == 1
    # Neither caller-owned nor current diagnostic dictionaries may alias work.
    ended_a["gesture"] = "mutated A"
    ended_b["playback_result"]["state"] = "mutated B"
    card.last_result["gesture"] = "current diagnostic"
    third = call(robot.plugin, "stop", execute=True)
    current_result = deepcopy(card.last_result)
    sender.release.set()
    assert terminal(robot, first["action_id"])["state"] == "sent"
    assert terminal(robot, second["action_id"])["state"] == "sent"
    info = card.info()
    assert info["action_id"] == third["action_id"] and info["gesture_state"] == "running"
    assert info["last_result"] == current_result and robot.plugin._body_action == "traffic_police_gesture.stop"
    sent = {args[0]: args for args in sender.snapshots}
    assert sent[first["action_id"]][2]["gesture"] == "stop"
    assert sent[second["action_id"]][2] == expected_b
    assert sent[second["action_id"]][1] == "cancelled"
    assert all(not args[2]["physical_completion_confirmed"] for args in sent.values())


def test_real_watchdog_cancel_race_schedules_only_one_terminal(robot, sender, monkeypatch):
    card = robot.plugin._traffic_gesture
    module = __import__(type(card).__module__)
    ticks = iter([0., 10., 10.])
    monkeypatch.setattr(module, "time", types.SimpleNamespace(monotonic=lambda: next(ticks, 10.)))
    barrier, watchdog_ready = threading.Barrier(2), threading.Event()
    original_cancel = card.cancel
    def racing_cancel(*args, **kwargs):
        if kwargs.get("reason") in ("playback_timeout", "user_request"):
            if kwargs["reason"] == "playback_timeout":
                watchdog_ready.set()
            barrier.wait(2)
        return original_cancel(*args, **kwargs)
    monkeypatch.setattr(card, "cancel", racing_cancel)
    started = call(robot.plugin, "go", execute=True)
    assert watchdog_ready.wait(1)
    user, done, results = run_in_thread(lambda: card.cancel(reason="user_request"))
    assert done.wait(1)
    user.join(1)
    assert not isinstance(results[0], Exception)
    assert sender.entered.wait(1)
    card.worker.join(1)
    assert not card.worker.is_alive()
    for _ in range(4):
        card.cancel()
    assert len(sender.calls) == 1 and sender.calls[0][0] == started["action_id"]
    assert sender.calls[0][1] in ("cancelled", "error")
    assert robot.plugin._body_action is None
    assert [event[0] for event in robot.events].count("WALK") == 1


def test_stop_verification_finishes_before_return_or_notification(robot, sender, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = robot.plugin._observation._settled
    def verify():
        entered.set()
        assert release.wait(2)
        return original()
    monkeypatch.setattr(robot.plugin._observation, "_settled", verify)
    started = call(robot.plugin, "stop", execute=True)
    worker, done, results = run_in_thread(lambda: call(robot.plugin, "cancel"))
    try:
        assert entered.wait(1)
        assert not done.is_set() and not sender.entered.is_set()
        assert robot.plugin._body_action == "traffic_police_gesture.stop"
        release.set()
        assert done.wait(1) and results[0]["playback_exit_confirmed"]
        assert sender.entered.wait(1) and sender.calls[0][0] == started["action_id"]
    finally:
        release.set()
        worker.join(1)


def test_successful_http_cannot_clear_local_locked_state(robot, sender, monkeypatch):
    monkeypatch.setattr(robot.plugin._observation, "_settled", lambda: (False, "not stationary"))
    started = call(robot.plugin, "go", execute=True)
    result = call(robot.plugin, "cancel")
    assert result["state"] == "locked" and not result["playback_exit_confirmed"]
    sender.release.set()
    assert terminal(robot, started["action_id"])["state"] == "sent"
    info = robot.plugin._traffic_gesture.info()
    assert info["state"] == "error" and info["gesture_state"] == "locked" and info["busy"]
    assert not info["physical_completion_confirmed"]
    assert robot.plugin._body_action == "traffic_police_gesture.go"
    assert call(robot.plugin, "stop", execute=True)["state"] == "error"
    assert len(sender.calls) == 1 and sender.calls[0][1] == "error"


def test_locked_retry_before_delivery_scheduling_cannot_replace_the_first_terminal(robot, sender, monkeypatch):
    card = robot.plugin._traffic_gesture
    safe = [False]
    original_settled = robot.plugin._observation._settled
    monkeypatch.setattr(robot.plugin._observation, "_settled",
                        lambda: original_settled() if safe[0] else (False, "first exit unconfirmed"))
    committed, release = threading.Event(), threading.Event()
    original_notify = card._notify_once
    first = [True]
    def delayed(operation):
        if first[0]:
            first[0] = False
            committed.set()
            assert release.wait(2)
        original_notify(operation)
    monkeypatch.setattr(card, "_notify_once", delayed)
    started = call(robot.plugin, "go", execute=True)
    worker, done, results = run_in_thread(lambda: call(robot.plugin, "cancel"))
    try:
        assert committed.wait(1) and not done.is_set()
        safe[0] = True
        recovered = call(robot.plugin, "cancel")
        assert recovered["state"] == "idle" and recovered["playback_exit_confirmed"]
        assert sender.entered.wait(1)
        assert sender.calls[0][0] == started["action_id"] and sender.calls[0][1] == "error"
        assert sender.calls[0][2]["state"] == "locked"
        assert not sender.calls[0][2]["playback_exit_confirmed"]
        assert card.info()["gesture_state"] == "idle" and robot.plugin._body_action is None
    finally:
        release.set()
        worker.join(1)
    assert done.is_set() and results[0]["state"] == "locked" and len(sender.calls) == 1


def test_awaiting_verification_still_requires_local_cancel(robot, sender):
    started = call(robot.plugin, "go", execute=True)
    robot.ctrl.mode = 2
    wait_until(lambda: robot.plugin._traffic_gesture.info()["gesture_state"] == "awaiting_verification")
    assert not sender.entered.is_set()
    assert call(robot.plugin, "stop", execute=True)["reason"] == "body_action_busy"
    result = call(robot.plugin, "cancel")
    sender.release.set()
    assert terminal(robot, started["action_id"])["state"] == "sent"
    assert result["playback_exit_confirmed"] and not result["physical_completion_confirmed"]
    assert sender.calls[0][1] == "cancelled"


@pytest.mark.parametrize("bundle_cleanup", [False, True])
def test_shutdown_does_not_join_network_and_preserves_bundle_failure_isolation(robot, sender, bundle_cleanup, monkeypatch):
    card = robot.plugin._traffic_gesture
    call(robot.plugin, "go", execute=True)
    if bundle_cleanup:
        main = load_bumi(monkeypatch)
        bundle = object.__new__(main.BumiDeviceBundle)
        class CameraFailure:
            def stop(self):
                raise RuntimeError("camera failed")
        bundle._plugins = [robot.plugin, CameraFailure()]
        stop = bundle.stop_all
    else:
        stop = card.shutdown
    worker, done, results = run_in_thread(stop)
    try:
        assert done.wait(1) and sender.entered.wait(1)
        worker.join(1)
        if bundle_cleanup:
            assert isinstance(results[0], RuntimeError) and "camera failed" in str(results[0])
        else:
            assert results[0]["playback_exit_confirmed"]
        assert card.info()["closed"] and card.info()["acp_notifications"]["closed"]
        assert robot.plugin._body_action is None and not card.worker.is_alive()
        assert card._delivery.worker.is_alive() and not sender.release.is_set()
        assert call(robot.plugin, "stop", execute=True)["reason"] == "shutting_down"
    finally:
        sender.release.set()
        worker.join(1)


def test_one_sender_and_bounded_queue_history_do_not_block_safety_on_overflow(robot, sender):
    card = robot.plugin._traffic_gesture
    card._delivery.MAX_PENDING, card._delivery.MAX_RECORDS = 2, 5
    call(robot.plugin, "go", execute=True)
    call(robot.plugin, "cancel")
    assert sender.entered.wait(1)
    worker = card._delivery.worker
    for index in range(9):
        assert call(robot.plugin, "stop" if index % 2 else "go", execute=True)["state"] == "running"
        result = call(robot.plugin, "cancel")
        assert result["playback_exit_confirmed"] and robot.plugin._body_action is None
    info = card.info()["acp_notifications"]
    assert card._delivery.worker is worker and info["queued"] == 2
    assert len(info["records"]) == 5 and any(r["state"] == "failed" for r in info["records"])
    assert len(sender.calls) == 1
    sender.release.set()
    wait_until(lambda: card.info()["acp_notifications"]["in_flight"] is None
               and card.info()["acp_notifications"]["queued"] == 0 and len(sender.calls) == 3)
    assert len(sender.calls) == 3  # one in-flight plus two queued, no retry


@pytest.mark.parametrize("failure", [None, "connect", "read", "http"])
def test_real_acp_helper_transport_receipt_without_any_network(robot, monkeypatch, failure):
    path = Path(__file__).resolve().parents[1]/"noetix/bumi/device.py"
    node = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == "_acp_notify")
    namespace = dict(robot.module.__dict__)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), namespace)
    requests = []
    class Response:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def read(self):
            if failure == "read": raise TimeoutError("response stalled")
            return b'{"ok":true}'
    def urlopen(request, *, timeout, context):
        requests.append((request, timeout, context))
        if failure == "connect": raise ConnectionError("refused")
        if failure == "http": raise urllib.error.HTTPError(request.full_url, 503, "unavailable", {}, None)
        return Response()
    monkeypatch.setattr(namespace["urllib"].request, "urlopen", urlopen)
    monkeypatch.setenv("AGENT_CORE_URL", "https://fake-core.invalid")
    robot.plugin._traffic_gesture._delivery.sender = namespace["_acp_notify"]
    started = call(robot.plugin, "go", execute=True)
    result = call(robot.plugin, "cancel")
    delivered = terminal(robot, started["action_id"])
    assert delivered["state"] == ("failed" if failure else "sent")
    assert result["playback_exit_confirmed"] and not result["physical_completion_confirmed"]
    request, timeout, context = requests[0]
    assert timeout == 5 and request.full_url == "https://fake-core.invalid/api/acp/complete"
    payload = json.loads(request.data)
    assert payload["action_id"] == started["action_id"] and payload["status"] == "cancelled"
    assert payload["result"] == result and payload["tool"] == "traffic_police_gesture"
    robot.plugin._traffic_gesture.shutdown()
    robot.plugin._traffic_gesture._delivery.worker.join(1)


def test_sender_start_failure_is_observable_without_changing_local_exit(robot, monkeypatch):
    card = robot.plugin._traffic_gesture
    started = call(robot.plugin, "go", execute=True)
    module = __import__(type(card).__module__)
    def cannot_start(**kwargs):
        raise RuntimeError("no notification worker")
    with monkeypatch.context() as patch:
        patch.setattr(module, "threading", types.SimpleNamespace(Thread=cannot_start, current_thread=threading.current_thread))
        result = call(robot.plugin, "cancel")
    assert result["playback_exit_confirmed"] and robot.plugin._body_action is None
    delivery = record(robot, started["action_id"])
    assert delivery["state"] == "failed" and "no notification worker" in delivery["error"]
    assert card._delivery.worker is None and card.info()["acp_notifications"]["queued"] == 0
