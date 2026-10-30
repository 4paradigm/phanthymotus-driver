"""Visual-navigation sessions reserve the bundle's base without SDK takeover."""

import sys

import pytest

from test_tianyi_loco_servo import command, load_bundle, send, setup
from test_tianyi_loco_servo_process import process_setup, stats
from test_tianyi_sdk_worker import native_worker


MOTIONS = [
    ("nav", "move_to"), ("nav", "move_by"),
    ("nav", "rotate"), ("nav", "rotate_to"),
    ("home", "go_home"), ("chassis_raw", "move"),
    ("chassis_raw", "rotate"),
]
STOPS = [("nav", "cancel"), ("home", "cancel"), ("chassis_raw", "brake")]


class ExplicitCard:
    def __init__(self, name, calls):
        self.name, self.calls = name, calls

    def start(self):
        pass

    def stop(self):
        pass

    def get_tool(self):
        return {"name": self.name, "type": "actuator"}

    def dispatch(self, action, args):
        self.calls.append((self.name, action))
        return {"state": "stopped" if (self.name, action) in STOPS else "ready",
                "api_result": {}}


def make_bundle(module, ros, monkeypatch, **config):
    monkeypatch.setitem(sys.modules, "loco_servo_process", module)
    bundle = load_bundle()(
        {"plugins": {"loco_servo": {"enabled": True, "dry_run": False, **config}}},
        "ubuntu", ros, None)
    calls = []
    bundle._plugins.extend(ExplicitCard(name, calls) for name in
                           ("nav", "home", "chassis_raw"))
    bundle.start_all()
    return bundle, calls


@pytest.mark.parametrize("dry_run", [False, True])
def test_listening_servo_reserves_every_ordinary_motion_without_touching_sdk(
        setup, monkeypatch, dry_run):
    bundle, calls = make_bundle(setup.module, setup.ros, monkeypatch, dry_run=dry_run)
    servo = bundle._loco_servo
    try:
        assert bundle.dispatch("loco_servo", {"action": "start"})["state"] == "running"
        assert servo.reserves_chassis()
        assert servo.dispatch("info", {})["chassis_reserved"] is True
        assert servo._counts["received"] == 0
        backend = servo._backend
        before = (list(backend.messages), backend.stop_calls, dict(servo._counts))
        for tool, action in MOTIONS:
            rejected = bundle.dispatch(tool, {"action": action})
            assert rejected["state"] == "error" and "loco_servo" in rejected["error"]
            assert servo.reserves_chassis() and not servo._paused and not servo._yielded
            assert bundle._base_owner == ""
        assert calls == []
        assert (backend.messages, backend.stop_calls, servo._counts) == before
    finally:
        servo.stop()


def test_priority_survives_watchdog_zero_and_queries_but_stop_releases_it(setup, monkeypatch):
    bundle, calls = make_bundle(setup.module, setup.ros, monkeypatch)
    servo = bundle._loco_servo
    try:
        bundle.dispatch("loco_servo", {"action": "start"})
        setup.clock.advance()
        send(servo, command(setup))
        setup.clock.advance(0.31)
        servo._tick()
        assert servo._last_output == [0] * 6
        assert servo.reserves_chassis()
        before = list(servo._backend.messages)
        assert bundle.dispatch("nav", {"action": "get_pose"})["state"] == "ready"
        assert calls == [("nav", "get_pose")]
        assert servo._backend.messages == before and servo.reserves_chassis()
        assert bundle.dispatch("home", {"action": "go_home"})["state"] == "error"
        assert bundle.dispatch("loco_servo", {"action": "stop"})["state"] == "idle"
        assert not servo.reserves_chassis()
        assert bundle.dispatch("home", {"action": "go_home"})["state"] == "ready"
        assert bundle._base_owner == "home"
        assert bundle.dispatch("loco_servo", {"action": "start"})["state"] == "error"
        assert bundle.dispatch("home", {"action": "cancel"})["state"] == "stopped"
        assert bundle.dispatch("loco_servo", {"action": "start"})["state"] == "running"
        assert servo.reserves_chassis()
    finally:
        servo.stop()


@pytest.mark.parametrize("tool,action", STOPS)
def test_cancel_and_brake_pass_priority_and_require_explicit_resume(setup, monkeypatch, tool, action):
    bundle, calls = make_bundle(setup.module, setup.ros, monkeypatch)
    servo = bundle._loco_servo
    try:
        bundle.dispatch("loco_servo", {"action": "start"})
        setup.clock.advance()
        send(servo, command(setup))
        assert bundle.dispatch(tool, {"action": action})["state"] == "stopped"
        assert calls == [(tool, action)]
        assert servo._backend.messages[-1] == [0] * 6
        assert servo._paused and servo._yielded and not servo.reserves_chassis()
        assert bundle._base_owner == ""
        assert bundle.dispatch("loco_servo", {"action": "resume"})["state"] == "running"
        assert servo.reserves_chassis()
    finally:
        servo.stop()


def test_failed_start_has_no_reservation_and_stop_commands_keep_confirmation_guard(setup, monkeypatch):
    bundle, calls = make_bundle(setup.module, setup.ros, monkeypatch)
    servo = bundle._loco_servo
    try:
        assert bundle.dispatch("loco_servo", {"action": "start", "input_topic": "bad"})["state"] == "error"
        assert not servo.reserves_chassis()
        assert servo.dispatch("info", {})["chassis_reserved"] is False
        bundle.dispatch("loco_servo", {"action": "start"})
        setup.clock.advance()
        send(servo, command(setup))
        servo._backend.fail = True
        assert bundle.dispatch("chassis_raw", {"action": "brake"})["state"] == "error"
        assert not servo.reserves_chassis()
        assert calls == [] and bundle._base_owner == ""
    finally:
        if servo._backend is not None:
            servo._backend.fail = False
        servo.stop()


def test_real_process_facade_reserves_bundle_without_extra_rpc_and_releases_on_pause(
        process_setup, native_worker, monkeypatch):
    bundle, calls = make_bundle(process_setup.module, process_setup.ros, monkeypatch,
                                sdk_binary=str(native_worker), sdk_host="ok")
    servo = bundle._loco_servo
    try:
        info = bundle.dispatch("loco_servo", {"action": "start"})
        assert info["state"] == "running" and info["chassis_reserved"] is True
        assert info["dispatch"]["mode"] == "process_worker"
        # Querying the local reservation must not wait on the controller RPC;
        # admission remains reserved even before the first command arrives.
        original_rpc = servo._rpc

        def unexpected_rpc(*args, **kwargs):
            pytest.fail("priority arbitration must not issue a controller RPC")

        with monkeypatch.context() as patch:
            patch.setattr(servo, "_rpc", unexpected_rpc)
            assert servo.reserves_chassis()
            assert bundle.dispatch("nav", {"action": "move_to"})["state"] == "error"
        assert servo._rpc == original_rpc and calls == []
        info = servo.dispatch("info", {})
        assert info["counts"]["received"] == 0
        assert (stats(info).get("action_starts", 0)) == 0
        assert bundle.dispatch("loco_servo", {"action": "pause"})["state"] == "paused"
        assert not servo.reserves_chassis()
        assert bundle.dispatch("nav", {"action": "move_to"})["state"] == "ready"
        assert calls == [("nav", "move_to")] and bundle._base_owner == "nav"
        assert bundle.dispatch("loco_servo", {"action": "resume"})["state"] == "error"
        assert bundle.dispatch("nav", {"action": "cancel"})["state"] == "stopped"
        info = bundle.dispatch("loco_servo", {"action": "resume"})
        assert info["state"] == "running" and info["chassis_reserved"] is True
        assert servo.reserves_chassis()
    finally:
        servo.stop()
