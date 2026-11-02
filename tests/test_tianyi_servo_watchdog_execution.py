"""Admission and 300ms zero / 1000ms cancel deadlines stay independent."""
import json
import types
import pytest
from test_tianyi_loco_servo import StaleVelocityCommand, command, send, setup, start
from test_tianyi_loco_servo_process import (
    command as process_command, process_setup, send as process_send, stats, wait_info,
)
from test_tianyi_sdk_worker import native_worker


def test_accepted_command_outlives_entry_ttl_then_soft_zeros_before_cancel(setup):
    plugin = start(setup)
    original = command(setup, ttl_ms=100)
    received = setup.clock.monotonic()
    send(plugin, original)
    backend = plugin._backend
    writes = list(backend.messages)
    cancel = plugin._cancel_deadline_ms
    assert backend.deadlines[-1] == pytest.approx(original["stamp_ms"] + 300, abs=1)
    assert cancel == pytest.approx(original["stamp_ms"] + 1000, abs=1)
    setup.clock.advance(0.21)
    plugin._tick()
    assert backend.messages == writes
    setup.clock.advance(0.10)
    plugin._tick()
    assert plugin._last_output == [0] * 6
    assert backend.soft_stop_calls == [cancel]
    assert backend.stop_calls == 0
    assert plugin._last_valid_at == received
    assert plugin._cancel_deadline_ms == cancel
    assert plugin._soft_held
    setup.clock.advance(0.30)
    plugin._tick()
    assert backend.soft_stop_calls == [cancel]
    assert backend.stop_calls == 0
    setup.clock.advance(0.40)
    plugin._tick()
    assert backend.stop_calls == 1
    assert plugin._last_valid_at is None and plugin._cancel_deadline_ms == 0
    assert len(backend.deadlines) == 1


def test_fresh_10hz_commands_update_without_intermediate_zero_or_cancel(setup):
    plugin = start(setup)
    for sequence in range(1, 12):
        payload = command(setup, seq=sequence)
        send(plugin, payload)
        assert plugin._backend.deadlines[-1] == pytest.approx(payload["stamp_ms"] + 300, abs=1)
        assert plugin._backend.cancel_deadlines[-1] == pytest.approx(payload["stamp_ms"] + 1000, abs=1)
        setup.clock.advance(0.1)
        plugin._tick()
    assert plugin._counts["accepted"] == 11
    assert plugin._counts["holds"] == plugin._counts["soft_holds"] == 0
    assert all(any(values) for values in plugin._backend.messages)
    assert plugin._backend.stop_calls == 0


def test_valid_input_during_soft_hold_resumes_without_cancelling_action(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    setup.clock.advance(0.31)
    plugin._tick()
    assert plugin._soft_held
    send(plugin, command(setup, seq=2))
    assert not plugin._soft_held
    assert any(plugin._last_output)
    assert plugin._backend.stop_calls == 0
    assert len(plugin._backend.soft_stop_calls) == 1
    assert plugin._cancel_deadline_ms == pytest.approx(setup.clock.time() * 1000 + 1000, abs=1)


def test_ipc_wait_spends_both_original_receive_budgets(setup):
    plugin = start(setup)
    payload = command(setup, ttl_ms=1000)
    original_receipt = setup.clock.monotonic()
    plugin._on_message(types.SimpleNamespace(data=json.dumps(payload)))
    setup.clock.advance(0.12)
    plugin._tick()
    timing = plugin._info()["last_command_timing"]
    assert timing["local_wait_ms"] == pytest.approx(120)
    assert timing["receive_watchdog_ms"] == 300 and timing["cancel_after_ms"] == 1000
    assert timing["remaining_before_apply_ms"] == pytest.approx(180, abs=1)
    assert timing["cancel_remaining_before_apply_ms"] == pytest.approx(880, abs=1)
    assert plugin._last_valid_at == original_receipt
    setup.clock.advance(0.19)
    plugin._tick()
    assert plugin._soft_held and not plugin._backend.stop_calls
    setup.clock.advance(0.70)
    plugin._tick()
    assert plugin._backend.stop_calls == 1


def test_frame_waiting_beyond_receive_budget_cannot_renew_live_session(setup):
    plugin = start(setup)
    send(plugin, command(setup))
    original_cancel = plugin._cancel_deadline_ms
    setup.clock.advance(0.10)
    payload = command(setup, seq=2, ttl_ms=1000)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(payload)))
    setup.clock.advance(0.31)
    plugin._tick()
    assert plugin._counts["accepted"] == 1 and plugin._counts["dropped"] == 1
    assert plugin._soft_held and plugin._cancel_deadline_ms == original_cancel
    assert plugin._backend.stop_calls == 0
    assert len(plugin._backend.deadlines) == 1


def test_sdk_acknowledgement_does_not_restart_either_watchdog(setup, monkeypatch):
    plugin = start(setup)
    original_receipt = setup.clock.monotonic()
    write = plugin._backend.write
    def slow_write(values, deadline, **options):
        setup.clock.advance(0.23)
        write(values, deadline, **options)
    monkeypatch.setattr(plugin._backend, "write", slow_write)
    send(plugin, command(setup))
    assert plugin._counts["accepted"] == 1 and plugin._last_valid_at == original_receipt
    assert plugin._info()["last_command_timing"]["remaining_after_apply_ms"] == pytest.approx(70, abs=1)
    assert plugin._cancel_deadline_ms - setup.clock.time() * 1000 == pytest.approx(770, abs=1)
    setup.clock.advance(0.08)
    plugin._tick()
    assert plugin._soft_held and not plugin._backend.stop_calls
    setup.clock.advance(0.70)
    plugin._tick()
    assert plugin._backend.stop_calls == 1


@pytest.mark.parametrize("invalid", ["expired_ttl", "stale_observation", "replay", "bad_schema", "outranked"])
def test_unaccepted_frames_neither_stop_immediately_nor_renew_deadlines(setup, invalid):
    plugin = start(setup)
    send(plugin, command(setup, source="owner", priority=10))
    previous = (plugin._last_valid_at, plugin._lease_expiry_ms, plugin._cancel_deadline_ms)
    setup.clock.advance(0.15)
    payload = command(setup, seq=2, source="owner", priority=10)
    if invalid == "expired_ttl":
        payload["stamp_ms"] -= 220
    elif invalid == "stale_observation":
        payload["obs_stamp_ms"] -= 501
    elif invalid == "replay":
        payload["seq"] = 1
    elif invalid == "bad_schema":
        payload["schema"] = "bad"
    else:
        payload.update(source="other", priority=0)
    send(plugin, payload)
    assert plugin._counts["accepted"] == 1
    assert (plugin._last_valid_at, plugin._lease_expiry_ms, plugin._cancel_deadline_ms) == previous
    assert not plugin._backend.stop_calls and not plugin._backend.soft_stop_calls
    assert any(plugin._last_output)
    setup.clock.advance(0.16)
    plugin._tick()
    assert plugin._soft_held and not plugin._backend.stop_calls
    setup.clock.advance(0.70)
    plugin._tick()
    assert plugin._backend.stop_calls == 1


def test_native_stale_reply_preserves_old_deadlines_and_motion(setup, monkeypatch):
    plugin = start(setup)
    send(plugin, command(setup))
    previous = (plugin._last_valid_at, plugin._lease_expiry_ms, plugin._cancel_deadline_ms)
    setup.clock.advance(0.10)
    def stale(*args, **kwargs):
        raise StaleVelocityCommand("stale FOLLOW deadline")
    monkeypatch.setattr(plugin._backend, "write", stale)
    send(plugin, command(setup, seq=2))
    assert plugin._counts["accepted"] == 1 and plugin._counts["dropped"] == 1
    assert previous == (plugin._last_valid_at, plugin._lease_expiry_ms, plugin._cancel_deadline_ms)
    assert any(plugin._last_output)
    assert not plugin._backend.stop_calls and not plugin._backend.soft_stop_calls


def test_observation_limit_is_500ms_admission_only(setup):
    plugin = start(setup)
    assert plugin._info()["control_interface"]["rate"]["max_obs_age_ms"] == 500
    now = int(setup.clock.time() * 1000)
    send(plugin, command(setup, obs_stamp_ms=now - 450))
    assert plugin._counts["accepted"] == 1
    assert plugin._backend.deadlines[-1] == now + 300  # Receipt lease, not obs + 500.
    cancel = plugin._cancel_deadline_ms
    setup.clock.advance(0.06)
    plugin._tick()
    assert not plugin._soft_held and any(plugin._last_output)
    # Too old to admit: dropped, and the current motion is not stopped by it.
    send(plugin, command(setup, seq=2, obs_stamp_ms=int(setup.clock.time() * 1000) - 501))
    assert plugin._counts["accepted"] == 1 and plugin._counts["dropped"] == 1
    assert any(plugin._last_output) and not plugin._backend.soft_stop_calls
    setup.clock.advance(0.25)  # 310ms since the accepted receipt.
    plugin._tick()
    assert plugin._soft_held and plugin._cancel_deadline_ms == cancel
    assert len(plugin._backend.soft_stop_calls) == 1 and not plugin._backend.stop_calls


def test_repeated_new_packets_cannot_extend_one_old_observation(setup):
    plugin = start(setup)
    began = setup.clock.monotonic()
    observation = int(setup.clock.time() * 1000)
    for seq, elapsed in enumerate((0.0, 0.2, 0.4), start=1):
        setup.clock.advance(began + elapsed - setup.clock.monotonic())
        send(plugin, command(setup, seq=seq, obs_stamp_ms=observation))
    cancel = plugin._cancel_deadline_ms
    assert plugin._lease_expiry_ms == observation + 400 + 300  # Last receipt + 300.
    setup.clock.advance(0.101)  # Observation now 501ms old: no longer admissible.
    send(plugin, command(setup, seq=4, obs_stamp_ms=observation))
    assert plugin._counts["accepted"] == 3 and plugin._counts["dropped"] == 1
    assert plugin._lease_expiry_ms == observation + 700  # The dropped packet renewed nothing.
    setup.clock.advance(0.20)
    plugin._tick()
    assert plugin._soft_held and plugin._cancel_deadline_ms == cancel
    assert not plugin._backend.stop_calls
    setup.clock.advance(0.71)
    plugin._tick()
    assert plugin._backend.stop_calls == 1


@pytest.mark.parametrize("stop_kind", ["zero", "pause", "stop", "handover"])
def test_explicit_stops_cancel_without_waiting_for_either_watchdog(setup, stop_kind):
    plugin = start(setup)
    send(plugin, command(setup))
    setup.clock.advance(0.01)
    if stop_kind == "zero":
        send(plugin, command(setup, [0] * 6, seq=2))
    elif stop_kind == "handover":
        assert plugin.pause_for_explicit_command("nav.move_to")
    else:
        plugin.dispatch(stop_kind, {})
    assert plugin._backend is None or plugin._backend.stop_calls >= 1
    assert not any(plugin._last_output) and plugin._cancel_deadline_ms == 0


def test_real_process_soft_zeros_resumes_same_action_then_cancels_on_silence(process_setup):
    card = process_setup.create()
    process_send(card, process_command(ttl_ms=100))
    first = wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    assert first["control_interface"]["rate"]["watchdog_ms"] == 300
    assert first["control_interface"]["rate"]["max_obs_age_ms"] == 500
    assert first["backend"]["capabilities"]["staged_stop"]
    assert first["backend"]["last_write"]["lease_remaining_ms"] <= 300
    assert first["backend"]["last_write"]["cancel_remaining_ms"] > 700
    soft = wait_info(card, lambda item: item["counts"]["soft_holds"] == 1)
    assert soft["output"] == [0] * 6
    assert soft["backend"]["action_id"] == first["backend"]["action_id"]
    assert stats(soft)["confirmed_stops"] == 0
    process_send(card, process_command(seq=2))
    resumed = wait_info(card, lambda item: item["counts"]["accepted"] == 2)
    assert stats(resumed)["action_starts"] == 1 and any(resumed["output"])
    stopped = wait_info(card, lambda item: item["counts"]["holds"] > 0)
    assert stopped["output"] == [0] * 6 and stats(stopped)["confirmed_stops"] >= 1
    assert stopped["backend"]["action_id"] == 0
