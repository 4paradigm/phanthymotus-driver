"""Exercise the isolated Tianyi controller with real IPC and a fake chassis."""

import errno
import importlib.util
import json
import os
from pathlib import Path
import socket
import sys
import threading
import time
import types

import pytest

from test_tianyi_sdk_worker import native_worker


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "x-humanoid/tianyi2.0"


class Node:
    def __init__(self, name, context=None):
        self.name, self.context = name, context
        self.subscriptions = []
        self.destroyed = False

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions.append((kind, topic, callback, qos))
        return object()

    def destroy_node(self):
        self.destroyed = True


class Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        self.nodes.remove(node)


def command(seq=1, values=None, **extra):
    stamp = int(time.time() * 1000)
    return {"schema": "motus.control/1", "mode": "twist", "dof": 6,
            "values": [0.1, 0, 0, 0, 0, 0.1] if values is None else values,
            "source": "navi", "seq": seq, "priority": 0,
            "stamp_ms": stamp, "obs_stamp_ms": stamp, "ttl_ms": 200, **extra}


def send(card, payload):
    card._on_message(types.SimpleNamespace(data=json.dumps(payload)))


def wait_info(card, predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    info = None
    while time.monotonic() < deadline:
        info = card.dispatch("info", {})
        if predicate(info):
            return info
        time.sleep(0.005)
    pytest.fail(f"controller did not reach expected state: {info!r}")


def stats(info):
    return info["backend"]["native"]["stats"] or {}


def process_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    stat = Path(f"/proc/{pid}/stat")
    try:
        # A killed control process cannot reap its native child on Linux.
        # A zombie has exited and cannot execute or refresh a motion lease.
        if stat.read_text().rsplit(")", 1)[1].split()[0] == "Z":
            return False
    except FileNotFoundError:
        pass
    return True


@pytest.fixture
def process_setup(monkeypatch, native_worker):
    for name in ("rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    sys.modules["rclpy.node"].Node = Node
    qos = sys.modules["rclpy.qos"]
    qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sys.modules["std_msgs.msg"].String = types.SimpleNamespace
    sys.modules["std_msgs.msg"].Bool = types.SimpleNamespace
    monkeypatch.syspath_prepend(str(ROOT))
    monkeypatch.syspath_prepend(str(DRIVER))
    spec = importlib.util.spec_from_file_location(
        "tianyi_loco_servo_process_test", DRIVER / "loco_servo_process.py")
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    cards = []
    ros = types.SimpleNamespace(ctx_core=object(), ctx_tianyi=object(),
                                executor_core=Executor(), executor_tianyi=Executor())

    def create(*, start=True, **config):
        config = {"dry_run": False, "sdk_binary": str(native_worker),
                  "sdk_host": "ok", **config}
        card = module.LocoServoPlugin(config, "ubuntu", ros)
        cards.append(card)
        if start:
            result = card.dispatch("start", {})
            assert result["state"] == "running", result
            time.sleep(0.005)  # Commands must follow the start timestamp barrier.
        return card

    yield types.SimpleNamespace(module=module, ros=ros, create=create)

    for card in reversed(cards):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                if card.stop().get("state") == "idle":
                    break
            except (OSError, RuntimeError):
                break
            time.sleep(0.03)
        child = card._process
        if child is not None and child.poll() is None:
            child.terminate()
            child.wait(timeout=2)
        for name in ("_control_socket", "_data_socket"):
            channel = getattr(card, name, None)
            if channel is not None:
                channel.close()


def test_process_start_is_inert_and_parent_keeps_ros_subscription(process_setup):
    card = process_setup.create(start=False)
    card.start()
    assert card._process is None
    assert not process_setup.ros.executor_core.nodes
    info = card.dispatch("start", {})
    assert info["dispatch"]["mode"] == "process_worker"
    assert info["dispatch"]["worker_pid"] == card._process.pid != os.getpid()
    assert info["dispatch"]["worker_alive"]
    assert info["dispatch"]["controller_mode"] == "dedicated_worker"
    node = process_setup.ros.executor_core.nodes[0]
    assert node.context is process_setup.ros.ctx_core
    assert node.subscriptions[0][1] == "/actucore/navi/cmd"
    assert info["backend"]["state"] == "ready"
    assert info["counts"]["accepted"] == 0


def test_continuous_10hz_commands_keep_one_native_action(process_setup):
    card = process_setup.create()
    started = time.monotonic()
    for seq in range(1, 9):
        time.sleep(max(0, started + (seq - 1) * 0.1 - time.monotonic()))
        send(card, command(seq))
    info = wait_info(card, lambda item: item["counts"]["accepted"] == 8)
    assert stats(info)["set_attempts"] == 8
    assert stats(info)["action_starts"] == 1
    assert stats(info)["confirmed_stops"] == 0
    assert info["counts"]["holds"] == info["counts"]["dropped"] == 0


def test_parent_waiting_for_info_does_not_block_child_command_execution(process_setup, monkeypatch):
    card = process_setup.create()
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    entered, release = threading.Event(), threading.Event()
    reader = card._read_reply
    failures = []

    def delayed_reply():
        if threading.current_thread().name == "blocked-parent-info":
            entered.set()
            assert release.wait(2), "test did not release parent RPC reader"
        return reader()

    monkeypatch.setattr(card, "_read_reply", delayed_reply)

    def request_info():
        try:
            card.dispatch("info", {})
        except BaseException as exc:
            failures.append(exc)

    thread = threading.Thread(target=request_info, name="blocked-parent-info")
    thread.start()
    try:
        assert entered.wait(1)
        for seq in range(2, 6):
            send(card, command(seq))
            time.sleep(0.08)
    finally:
        release.set()
        thread.join(timeout=2)
    assert not thread.is_alive() and not failures
    info = wait_info(card, lambda item: item["counts"]["accepted"] == 5)
    assert stats(info)["action_starts"] == 1
    assert stats(info)["confirmed_stops"] == 0


def test_zero_pause_resume_and_handover_keep_confirmed_stop_semantics(process_setup):
    card = process_setup.create()
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    send(card, command(2, [0] * 6))
    info = wait_info(card, lambda item: item["counts"]["accepted"] == 2)
    assert info["output"] == [0] * 6 and stats(info)["confirmed_stops"] == 1
    assert card.dispatch("pause", {})["state"] == "paused"
    send(card, command(3))
    assert card.dispatch("resume", {})["state"] == "running"
    time.sleep(0.02)
    info = card.dispatch("info", {})
    assert info["counts"]["accepted"] == 2 and info["output"] == [0] * 6
    send(card, command(4))
    wait_info(card, lambda item: item["counts"]["accepted"] == 3)
    assert card.pause_for_explicit_command("nav.move_to") is True
    before = card.dispatch("info", {})
    send(card, command(5, [0] * 6))
    time.sleep(0.04)
    after = card.dispatch("info", {})
    assert after["yielded_to_explicit_command"]
    assert after["backend"]["native"]["sequence"] == before["backend"]["native"]["sequence"]


def test_old_epoch_packet_cannot_replay_after_resume(process_setup, monkeypatch):
    card = process_setup.create()
    packets = []
    original_flush = card._outbox.flush

    class RecordingSocket:
        def __init__(self, channel):
            self.channel = channel

        def send(self, packet):
            sent = self.channel.send(packet)
            packets.append(packet)
            return sent

    monkeypatch.setattr(card._outbox, "flush", lambda channel: original_flush(RecordingSocket(channel)))
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    assert packets
    old_packet = json.loads(packets[0])
    assert old_packet["kind"] == "motion"
    assert card.dispatch("pause", {})["state"] == "paused"
    assert card.dispatch("resume", {})["state"] == "running"
    time.sleep(0.005)
    # Make all command-level checks pass: the old transport epoch must reject it.
    old_packet["data"] = json.dumps(command(100))
    old_packet["received_at"] = [time.monotonic(), int(time.time() * 1000)]
    card._data_socket.send(json.dumps(old_packet).encode())
    time.sleep(0.06)
    assert card.dispatch("info", {})["counts"]["accepted"] == 1
    send(card, command(2))
    wait_info(card, lambda item: item["counts"]["accepted"] == 2)


@pytest.mark.parametrize("extra,reason", [
    ({"schema": "wrong"}, "schema"),
    ({"stamp_ms": "expired"}, "expired"),
    ({"obs_stamp_ms": 0}, "observation"),
])
def test_process_preserves_schema_ttl_and_observation_checks(process_setup, extra, reason):
    card = process_setup.create()
    if extra.get("stamp_ms") == "expired":
        time.sleep(0.24)
        extra = {"stamp_ms": int(time.time() * 1000) - 210}
    send(card, command(**extra))
    info = wait_info(card, lambda item: item["counts"]["received"] == 1)
    assert info["counts"]["accepted"] == 0
    assert reason in info["last"]["reason"]
    assert stats(info).get("set_attempts", 0) == 0


def test_process_preserves_sequence_arbitration(process_setup):
    card = process_setup.create()
    original = command()
    send(card, original)
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    send(card, original)
    info = wait_info(card, lambda item: item["counts"]["dropped"] == 1)
    assert "seq" in info["last"]["reason"]
    assert stats(info)["set_attempts"] == 1


def test_process_preserves_priority_between_sources(process_setup):
    card = process_setup.create()
    send(card, command(source="owner", priority=10))
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    send(card, command(source="other", priority=0))
    info = wait_info(card, lambda item: item["counts"]["dropped"] == 1)
    assert "outranked" in info["last"]["reason"]
    assert stats(info)["set_attempts"] == 1


def test_standing_age_uses_original_callback_time_not_child_receipt(process_setup):
    card = process_setup.create(require_standing=True, standing_topic="/verified/standing")
    card._on_standing(types.SimpleNamespace(data=True))
    wait_info(card, lambda item: not item["posture_problem"])
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    stale_at = (time.monotonic() - 2, int(time.time() * 1000) - 2000)
    card._outbox.put_standing(True, card._subscription_generation, received_at=stale_at)
    card._outbox.flush(card._data_socket)
    wait_info(card, lambda item: "stale" in item["posture_problem"])
    send(card, command(2))
    info = wait_info(card, lambda item: item["counts"]["received"] == 2)
    assert info["counts"]["accepted"] == 1 and info["output"] == [0] * 6


def test_standing_false_received_while_paused_revokes_motion_before_resume(process_setup):
    card = process_setup.create(require_standing=True, standing_topic="/verified/standing")
    card._on_standing(types.SimpleNamespace(data=True))
    wait_info(card, lambda item: not item["posture_problem"])
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    assert card.dispatch("pause", {})["state"] == "paused"
    card._on_standing(types.SimpleNamespace(data=False))
    assert card.dispatch("resume", {})["state"] == "running"
    time.sleep(0.005)
    send(card, command(2))
    info = wait_info(card, lambda item: item["counts"]["received"] == 2)
    assert info["counts"]["accepted"] == 1
    assert info["posture_problem"] and info["output"] == [0] * 6


def test_old_standing_callback_cannot_reauthorize_motion_after_resume(process_setup):
    card = process_setup.create(require_standing=True, standing_topic="/verified/standing")
    old_callback = process_setup.ros.executor_tianyi.nodes[0].subscriptions[0][2]
    old_callback(types.SimpleNamespace(data=True))
    wait_info(card, lambda item: not item["posture_problem"])
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    assert card.dispatch("pause", {})["state"] == "paused"
    assert card.dispatch("resume", {})["state"] == "running"
    time.sleep(0.005)
    old_callback(types.SimpleNamespace(data=True))
    send(card, command(2))
    refused = wait_info(card, lambda item: item["counts"]["received"] == 2)
    assert refused["counts"]["accepted"] == 1 and refused["posture_problem"]
    new_callback = process_setup.ros.executor_tianyi.nodes[0].subscriptions[0][2]
    assert new_callback is not old_callback
    new_callback(types.SimpleNamespace(data=True))
    wait_info(card, lambda item: not item["posture_problem"])
    send(card, command(3))
    wait_info(card, lambda item: item["counts"]["accepted"] == 2)


def test_old_motion_callback_is_rejected_by_subscription_generation(process_setup):
    card = process_setup.create()
    old_callback = process_setup.ros.executor_core.nodes[0].subscriptions[0][2]
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    assert card.dispatch("pause", {})["state"] == "paused"
    assert card.dispatch("resume", {})["state"] == "running"
    time.sleep(0.005)
    old_callback(types.SimpleNamespace(data=json.dumps(command(100))))
    time.sleep(0.06)
    assert card.dispatch("info", {})["counts"]["received"] == 1
    new_callback = process_setup.ros.executor_core.nodes[0].subscriptions[0][2]
    new_callback(types.SimpleNamespace(data=json.dumps(command(2))))
    wait_info(card, lambda item: item["counts"]["accepted"] == 2)


@pytest.mark.parametrize("action,args", [("start", {}), ("config", {})])
def test_idempotent_mutation_confirms_stop_before_clearing_pending_zero(
        process_setup, monkeypatch, action, args):
    card = process_setup.create()
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    original_flush = card._outbox.flush
    monkeypatch.setattr(card._outbox, "flush", lambda channel: 0)
    send(card, command(2, [0] * 6))
    send(card, command(3))
    assert card._outbox.info()["pending_zero"]
    result = card.dispatch(action, args)
    assert result["state"] == "running"
    assert result["output"] == [0] * 6
    assert stats(result)["confirmed_stops"] == 1
    assert stats(result)["stop_reason"] == "explicit_stop"
    assert result["counts"]["accepted"] == 1
    monkeypatch.setattr(card._outbox, "flush", original_flush)
    time.sleep(0.005)
    send(card, command(4))
    resumed = wait_info(card, lambda item: item["counts"]["accepted"] == 2)
    assert stats(resumed)["action_starts"] == 2


@pytest.mark.parametrize("config", [
    {"min_vx": float("nan")}, {"min_vx": float("inf")},
    {"min_vx": -0.01}, {"min_vx": 0.31},
    {"dry_run": 1}, {"rotate_only": "false"}, {"require_standing": None},
])
def test_invalid_config_does_not_revoke_active_process_or_advance_epoch(process_setup, config):
    card = process_setup.create()
    send(card, command())
    before = wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    process, epoch = card._process, card._epoch
    result = card.dispatch("config", config)
    assert result["state"] == "error"
    assert card._process is process and process.poll() is None
    assert card._epoch == epoch and card._outbox.info()["active"]
    current = card.dispatch("info", {})
    assert current["state"] == "running"
    assert current["backend"]["action_id"] == before["backend"]["action_id"]
    assert stats(current)["confirmed_stops"] == 0
    send(card, command(2))
    after = wait_info(card, lambda item: item["counts"]["accepted"] == 2)
    assert stats(after)["action_starts"] == 1 and stats(after)["confirmed_stops"] == 0


def test_killing_only_control_child_does_not_confirm_sdk_group_handover(process_setup, monkeypatch):
    card = process_setup.create()
    send(card, command())
    wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    child = card._process
    killpg = process_setup.module.os.killpg

    def denied_group(group, sig):
        if group == child.pid:
            raise PermissionError(errno.EPERM, "group termination not permitted")
        return killpg(group, sig)

    with monkeypatch.context() as patch:
        patch.setattr(process_setup.module.os, "killpg", denied_group)
        card._mark_fault("test unconfirmed native process group")
        child.wait(timeout=2)  # The individual child.kill fallback did work.
        card._unsafe_until = time.monotonic() - 1
        assert card.pause_for_explicit_command("nav.move_to") is False
        assert card.stop()["state"] == "error"
        assert card._process is child
    # Restore real group termination before the fixture performs final cleanup.


def test_dry_run_child_never_launches_the_sdk_binary(process_setup, tmp_path):
    card = process_setup.create(dry_run=True, sdk_binary=str(tmp_path / "not-a-real-sdk"))
    send(card, command())
    info = wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    assert info["counts"]["dry_run_commands"] == 1
    assert not info["backend"]["connected"]
    assert info["backend"]["process_id"] is None


@pytest.mark.parametrize("mode", ["cancel_failure", "ownership_failure"])
def test_backend_fault_cannot_acknowledge_unconfirmed_handover(process_setup, mode):
    card = process_setup.create(sdk_host=mode)
    send(card, command())
    if mode == "cancel_failure":
        wait_info(card, lambda item: item["counts"]["accepted"] == 1)
        assert card.dispatch("pause", {})["state"] == "error"
    else:
        wait_info(card, lambda item: item["state"] == "paused")
    assert card.pause_for_explicit_command("nav.move_to") is False
    assert not card.dispatch("info", {})["yielded_to_explicit_command"]


def test_control_channel_eof_stops_and_reaps_native_worker(process_setup):
    card = process_setup.create()
    send(card, command())
    info = wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    native_pid = info["backend"]["process_id"]
    child = card._process
    card._control_socket.shutdown(socket.SHUT_RDWR)
    card._control_socket.close()
    assert child.wait(timeout=3) == 0
    assert not process_alive(native_pid)


def test_dead_control_worker_blocks_handover_and_never_restarts_itself(process_setup):
    card = process_setup.create()
    send(card, command())
    info = wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    native_pid, child = info["backend"]["process_id"], card._process
    child.kill()
    child.wait(timeout=2)
    assert card.pause_for_explicit_command("nav.move_to") is False
    assert card.dispatch("resume", {})["state"] == "error"
    assert card._process is child and child.poll() is not None
    deadline = time.monotonic() + 2
    while process_alive(native_pid) and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not process_alive(native_pid)


def test_stop_confirms_idle_and_reaps_both_processes(process_setup):
    card = process_setup.create()
    send(card, command())
    info = wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    native_pid, child = info["backend"]["process_id"], card._process
    assert card.stop()["state"] == "idle"
    assert child.poll() is not None
    assert not process_alive(native_pid)
    assert not process_setup.ros.executor_core.nodes


def test_outbox_backpressure_keeps_latest_motion_and_priority_zero(process_setup):
    outbox = process_setup.module._ControlOutbox()
    outbox.activate(1, "test-session", 3, True)
    outbox.put_motion(json.dumps(command()), 1)
    outbox.put_motion(json.dumps(command(2, [0] * 6)), 1)
    outbox.put_motion(json.dumps(command(3)), 1)

    class Channel:
        blocked = True

        def __init__(self):
            self.packets = []

        def send(self, packet):
            if self.blocked:
                raise BlockingIOError(errno.EAGAIN, "outbox full")
            self.packets.append(json.loads(packet))
            return len(packet)

    channel = Channel()
    assert outbox.flush(channel) == 0
    info = outbox.info()
    assert info["pending_zero"] and info["pending_motion"] and info["pending"] == 2
    channel.blocked = False
    assert outbox.flush(channel) == 2
    assert [json.loads(packet["data"])["seq"] for packet in channel.packets] == [2, 3]
    assert all(packet["session"] == "test-session" and packet["epoch"] == 3
               for packet in channel.packets)
    assert outbox.info()["pending"] == 0

