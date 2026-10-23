"""Admission TTL and execution silence limits are distinct, without replay."""

import json
import time
import types

import pytest

from test_tianyi_loco_servo import command, send, setup, start
from test_tianyi_loco_servo_process import (
    command as process_command,
    process_setup,
    send as process_send,
    stats,
    wait_info,
)
from test_tianyi_sdk_worker import native_worker


def test_accepted_command_survives_ttl_and_old_watchdog_but_respects_observation_age(setup):
    plugin = start(setup)
    original = command(setup)
    received = setup.clock.monotonic()
    send(plugin, original)
    writes = list(plugin._backend.messages)

    assert plugin._backend.deadlines[-1] == original["obs_stamp_ms"] + 500
    assert plugin._last_valid_at == received
    setup.clock.advance(0.21)
    plugin._tick()
    assert plugin._backend.messages == writes
    assert plugin._last["verdict"] == "APPLIED"
    setup.clock.advance(0.07)
    plugin._tick()
    assert plugin._backend.messages == writes

    setup.clock.advance(0.03)
    plugin._tick()
    assert plugin._backend.messages == writes  # No stop at the old 300ms limit.
    assert plugin._last_valid_at == received
    setup.clock.advance(0.20)
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6
    assert "observation deadline expired" in plugin._last["reason"]
    assert len(plugin._backend.deadlines) == 1
    assert plugin._last_valid_at is None


def test_fresh_10hz_commands_extend_receive_deadline_without_restarting_output(setup):
    plugin = start(setup)
    for sequence in range(1, 12):
        payload = command(setup, seq=sequence)
        send(plugin, payload)
        assert plugin._backend.deadlines[-1] == pytest.approx(payload["obs_stamp_ms"] + 500, abs=1)
        setup.clock.advance(0.1)
        plugin._tick()
    assert plugin._counts["accepted"] == 11
    assert plugin._counts["holds"] == 0
    assert all(any(values) for values in plugin._backend.messages)


def test_ipc_or_inbox_wait_spends_original_receive_budget(setup):
    plugin = start(setup)
    payload = command(setup, ttl_ms=1000)
    original_receipt = setup.clock.monotonic()
    plugin._on_message(types.SimpleNamespace(data=json.dumps(payload)))
    setup.clock.advance(0.12)
    plugin._tick()
    timing = plugin._info()["last_command_timing"]
    assert timing["local_wait_ms"] == pytest.approx(120)
    assert timing["execution_deadline_basis"] == "receive_watchdog_and_observation"
    assert timing["receive_watchdog_ms"] == 1000
    assert timing["remaining_before_apply_ms"] == pytest.approx(380, abs=1)
    assert plugin._backend.deadlines[-1] == pytest.approx(payload["obs_stamp_ms"] + 500, abs=1)
    assert plugin._last_valid_at == original_receipt

    setup.clock.advance(0.39)
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6


def test_frame_waiting_longer_than_observation_limit_cannot_start_motion(setup):
    plugin = start(setup)
    payload = command(setup, ttl_ms=1000)
    plugin._on_message(types.SimpleNamespace(data=json.dumps(payload)))
    setup.clock.advance(0.51)
    plugin._tick()
    assert not any(any(values) for values in plugin._backend.messages)
    assert plugin._counts["accepted"] == 0
    assert plugin._counts["dropped"] == 1
    assert plugin._last_valid_at is None


def test_sdk_acknowledgement_cannot_start_a_new_receive_watchdog(setup, monkeypatch):
    plugin = start(setup)
    original_receipt = setup.clock.monotonic()
    write = plugin._backend.write

    def slow_write(values, deadline):
        setup.clock.advance(0.23)
        write(values, deadline)

    monkeypatch.setattr(plugin._backend, "write", slow_write)
    send(plugin, command(setup))
    assert plugin._counts["accepted"] == 1
    assert plugin._last_valid_at == original_receipt
    assert plugin._info()["last_command_timing"]["remaining_after_apply_ms"] == pytest.approx(270, abs=1)
    setup.clock.advance(0.28)
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6


@pytest.mark.parametrize("invalid", ["expired_ttl", "stale_observation", "replay", "bad_schema"])
def test_unaccepted_frames_do_not_refresh_receive_watchdog(setup, invalid):
    plugin = start(setup)
    original = command(setup)
    send(plugin, original)
    original_receipt = plugin._last_valid_at
    original_deadline = plugin._lease_expiry_ms
    setup.clock.advance(0.15)
    if invalid == "expired_ttl":
        payload = command(setup, seq=2, stamp_ms=int(setup.clock.time() * 1000) - 220)
    elif invalid == "stale_observation":
        payload = command(setup, seq=2, obs_stamp_ms=int(setup.clock.time() * 1000) - 501)
    elif invalid == "replay":
        payload = command(setup, seq=1)
    else:
        payload = command(setup, seq=2, schema="bad")
    send(plugin, payload)
    assert plugin._counts["accepted"] == 1
    assert plugin._last_valid_at == original_receipt
    assert plugin._lease_expiry_ms == original_deadline
    assert len(plugin._backend.deadlines) == 1
    assert any(plugin._backend.messages[-1])
    setup.clock.advance(0.36)
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6


def test_fresh_receipt_does_not_extend_observation_freshness_limit(setup):
    plugin = start(setup)
    now = int(setup.clock.time() * 1000)
    payload = command(setup, obs_stamp_ms=now - 350)
    send(plugin, payload)
    assert plugin._counts["accepted"] == 1
    assert plugin._backend.deadlines[-1] == payload["obs_stamp_ms"] + 500
    setup.clock.advance(0.14)
    plugin._tick()
    assert any(plugin._backend.messages[-1])
    setup.clock.advance(0.02)
    plugin._tick()
    assert plugin._backend.messages[-1] == [0] * 6


@pytest.mark.parametrize("stop_kind", ["zero", "pause"])
def test_zero_and_pause_still_stop_before_receive_watchdog(setup, stop_kind):
    plugin = start(setup)
    send(plugin, command(setup))
    setup.clock.advance(0.01)
    if stop_kind == "zero":
        send(plugin, command(setup, [0] * 6, seq=2))
    else:
        plugin.dispatch("pause", {})
    assert plugin._backend.messages[-1] == [0] * 6
    assert not any(plugin._last_output)


def test_real_control_process_passes_receive_budget_to_native_and_stops_on_silence(process_setup):
    card = process_setup.create()
    # A short, still-valid producer TTL makes the two limits easy to separate
    # without depending on the last few milliseconds of the 500ms observation limit.
    original = process_command(ttl_ms=100)
    process_send(card, original)
    first = wait_info(card, lambda item: item["counts"]["accepted"] == 1)
    assert first["control_interface"]["rate"]["watchdog_ms"] == 1000
    assert first["control_interface"]["rate"]["max_obs_age_ms"] == 500
    assert first["backend"]["last_write"]["lease_remaining_ms"] > 300
    original_expiry = original["stamp_ms"] + original["ttl_ms"]
    while int(time.time() * 1000) < original_expiry + 40:
        time.sleep(0.005)
    while int(time.time() * 1000) < original["stamp_ms"] + 330:
        time.sleep(0.005)
    during = card.dispatch("info", {})
    assert any(during["output"])  # Old 300ms silence limit no longer stops motion.
    assert during["counts"]["holds"] == 0
    assert stats(during)["confirmed_stops"] == 0

    process_send(card, process_command(seq=2))
    second = wait_info(card, lambda item: item["counts"]["accepted"] == 2)
    assert stats(second)["action_starts"] == 1
    assert stats(second)["confirmed_stops"] == 0
    stopped = wait_info(card, lambda item: item["counts"]["holds"] > 0)
    assert stopped["output"] == [0] * 6
    assert stopped["counts"]["accepted"] == 2
    assert stats(stopped)["confirmed_stops"] >= 1
