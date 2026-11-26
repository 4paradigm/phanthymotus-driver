"""Synthetic sensor traces and lifecycle tests for parameterized contact control."""
from __future__ import annotations

import time
import json
from types import SimpleNamespace
import pytest

from test_linkerhand_l20_adaptive_grasp import test_setup
from adaptive_grasp_device import PLACEMENT_NOTICE, DEFAULT_CLOSE_LIMIT_POSE, DEFAULT_PREP_POSE, FINGER_JOINTS
from grasp_feedback import Contact, FeedbackGrasp, GraspSettings, contact_patches, empty_baseline, settings_from

PROFILES = {"output_55": GraspSettings(55,40,5,28,95,3), "output_40": GraspSettings(40,35,3,18,110,2)}


def patches(peak=10, total=60, row=5.0):
    out = {f: Contact(0, 0, 0, 0) for f in FINGER_JOINTS}
    out.update(thumb=Contact(peak, total, row, 2), index=Contact(peak, total, row, 2))
    return out


def frame(readings):
    result = {f: [[0] * 6 for _ in range(12)] for f in FINGER_JOINTS}
    for f, (row, value) in readings.items():
        result[f][row][2] = value
    return result


@pytest.mark.parametrize("name", PROFILES)
def test_static_and_slip_then_cap(name):
    p = PROFILES[name]
    c = FeedbackGrasp(p, [200] * 20, [0] * 20)
    for i in range(150):
        c.update(patches(), c.target.copy(), i / 100, False)
    assert c.search_stop == "fingers_settled"
    before_transfer = c.target.copy()
    assert set(c.anchors) == {"thumb", "index"}
    assert not c.update(patches(total=45, row=5), c.target, 2.0, True)
    assert not c.update(patches(total=60, row=7), c.target, 2.1, True)
    for n in range(p.max_corrections):
        now = 3.0 + n
        slipping = patches(total=45, row=7)
        assert not c.update(slipping, c.target, now, True)
        assert c.update(slipping, c.target, now + 0.1, True)
        assert c.corrections == n + 1
        assert not c.update(patches(), c.target, now + 0.6, True)
    # The budget test uses a continuous sample stream; a long gap must re-prime the baseline.
    next_sample = 3.0 + p.max_corrections
    assert not c.update(patches(total=40, row=7), c.target, next_sample, True)
    assert not c.update(patches(total=40, row=7), c.target, next_sample + 0.1, True)
    assert c.paused and c.corrections == p.max_corrections
    assert c.target[0] == 200 - p.max_corrections and c.target[2] == before_transfer[2]


def test_overpressure_and_loss_never_chase():
    c = FeedbackGrasp(PROFILES["output_55"], [250] * 20, [0] * 20)
    for i in range(5):
        c.update(patches(), c.target, i, True)
    c.target[0] -= 3
    previous = list(c.target)
    assert c.update(patches(peak=100), c.target, 6, True)
    assert all(a >= b for a, b in zip(c.target, previous)) and c.paused
    c = FeedbackGrasp(PROFILES["output_40"], [250] * 20, [0] * 20)
    for _ in range(3):
        c.update(patches(), c.target, 0, False)
    missing = patches(); missing["thumb"] = Contact(0, 0, 0, 0)
    for _ in range(3):
        assert not c.update(missing, c.target, 2, True)
    assert not c.paused and c.stable_fingers == {"index"}
    empty = patches(peak=0, total=0)
    for i in range(2):
        assert not c.update(empty, c.target, 3 + i, True)
    assert c.waiting_for_signal and not c.corrections


def test_baseline_and_centroid():
    zero = frame({"thumb": (4, 2)})
    b = empty_baseline([zero, zero, zero])
    sample = frame({"thumb": (4, 12), "index": (7, 9)})
    observed = contact_patches(sample, b)
    assert observed["thumb"] == Contact(10, 10, 4, 2)
    assert observed["index"] == Contact(9, 9, 7, 2)


def wait_for(read, predicate, seconds=3):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        state = read()
        if predicate(state):
            return state
        time.sleep(0.01)
    pytest.fail(f"Timed out: {state}")


def operator_message(plugin, text, ts=None):
    msg = {"text":text, "ts":time.time() if ts is None else ts}
    plugin._placement_input(SimpleNamespace(data=json.dumps(msg,ensure_ascii=False)))


def start_right_countdown(plugin,args,monkeypatch):
    monkeypatch.setattr(plugin, "_input_available", lambda: True)
    result = plugin.dispatch("get_state", {"bound_side":"right"})["last_result"]
    return plugin.dispatch("close_until_contact", dict(args, prepare_id=result["action_id"],
                                                      placement_notice=PLACEMENT_NOTICE))


@pytest.mark.parametrize("fingers", [("thumb", "index"), ("middle",)])
def test_prepare_transfer_monitor_and_release(test_setup, monkeypatch, fingers):
    plugin, client, _, _, _, events, _ = test_setup
    args = {"bound_side": "left", "confirm_motion": True}
    state = lambda: plugin.dispatch("get_state", {"bound_side": "left"})
    readings = {}
    monkeypatch.setattr(client, "get_fresh_matrix_touch", lambda **kw: frame(readings))
    with pytest.raises(ValueError, match="prepare"):
        plugin.dispatch("close_until_contact", args)
    with pytest.raises(ValueError, match="confirm_motion"):
        plugin.dispatch("prepare", {"bound_side":"left"})
    plugin.dispatch("prepare", dict(args, output_limit=40, approach_travel=16, max_corrections=2))
    prepared = wait_for(state, lambda s: s["last_result"] is not None)
    assert prepared["last_result"]["result"]["state"] == "prepared"
    readings.update({finger: (4, 12) for finger in fingers})
    plugin.dispatch("close_until_contact", args)
    wrapping = wait_for(state, lambda s:s["tactile_status"].get("phase")=="wrapping")
    assert wrapping["holding"] and wrapping["tactile_status"]["grasp_confirmed"]
    assert "ring" in wrapping["tactile_status"]["remaining_fingers"] and not any(e[0] == wrapping["action_id"] for e in events)
    ready = wait_for(state, lambda s:s["tactile_status"].get("phase")=="monitoring"
                     and s.get("holding"))
    assert ready["tactile_status"]["output_limit_raw"] == PROFILES["output_40"].output_limit
    assert ready["tactile_status"]["remaining_fingers"] == []
    assert set(ready["tactile_status"]["travel_limited_fingers"]) == set(FINGER_JOINTS) - set(fingers)
    assert ready["tactile_status"]["contact_fingers"] == list(fingers)
    assert ready["tactile_status"]["grasp_confirmed"] is True
    assert ready["tactile_status"]["opposed_now"] is ("thumb" in fingers)
    assert ready["tactile_status"]["hold_elapsed_s"] == 0
    readings.update({finger: (7, 8) for finger in fingers})
    wait_for(state, lambda s:s["tactile_status"].get("corrections")==1)
    sample = wait_for(state, lambda s:s["tactile_status"].get("phase")=="monitoring" and s["tactile_status"].get("hold_elapsed_s", 0)>=5, seconds=7)
    assert sample["tactile_status"]["grasp_confirmed"] is True
    plugin.dispatch("release", args)
    final = wait_for(state, lambda s: s["last_result"]["result"].get("hand_opened"))
    assert final["last_result"]["result"]["torque_released"]
    assert final["tactile_status"]["contact_fingers"] == []
    assert final["tactile_status"]["grasp_confirmed"] is False
    completed = [e for e in events if e[0] == ready["action_id"]]
    assert len(completed) == 1
    assert completed[0][2]["grasp_confirmed"] and completed[0][2]["contact_fingers"] == list(fingers)


def test_first_ring_contact_releases_position_lag_before_opposition():
    c = FeedbackGrasp(PROFILES["output_40"], [220] * 20, [0] * 20)
    empty = {f: Contact(0,0,0,0) for f in FINGER_JOINTS}
    for _ in range(8):
        c.update(empty, c.target, 0, False)
    actual = c.target.copy()
    actual[3] += 3
    observed = dict(empty, ring=Contact(17, 30, 3, 2))
    c.update(observed, actual, 1, False)
    assert c.target[3] > actual[3]
    observed["ring"] = Contact(21, 40, 3, 2)
    old = c.target.copy()
    c.update(observed, actual, 1.1, False)
    assert not c.paused and c.peak_limited_fingers == ("ring",)
    assert c.target[3] >= old[3] and c.target[0] < old[0]
    assert c.update(observed, c.target.copy(), 1.2, False)
    observed.update(thumb=Contact(5,7,1,2),index=Contact(6,8,2,2),ring=Contact(5,6,1,3))
    for _ in range(3):
        c.update(observed, c.target, 1.5, False)
    assert set(c.anchors)=={"thumb","index","ring"} and not c.paused


def test_wrapped_object_with_weak_tactile_keeps_target_without_tightening():
    c = FeedbackGrasp(PROFILES["output_40"], [220]*20, [0]*20)
    c.seek_contact = False
    empty = {f: Contact(0,0,0,0) for f in FINGER_JOINTS}
    for i in range(8):
        assert not c.update(empty,c.target,i,False)
    assert c.target == [220]*20 and not c.anchors and not c.paused
    for i in range(3):
        c.update(patches(), c.target, 10+i, False)
    assert set(c.anchors)=={"thumb","index"} and not c.corrections

def test_task_parameters_are_bound_before_motion(test_setup):
    plugin, _, bus, *_ = test_setup
    with pytest.raises(ValueError, match="contact_peak"):
        plugin.dispatch("prepare", {"bound_side": "right", "confirm_motion": True,
                                    "contact_peak": 15, "peak_ceiling": 10})
    assert not bus.sent_messages
    settings = settings_from({"output_limit": 47, "speed": 26, "max_corrections": 0})
    assert settings.output_limit == 47 and settings.speed == 26
    c = FeedbackGrasp(settings,[220]*20,[0]*20)
    for i in range(3):
        c.update(patches(),c.target,i,False)
    for i in range(3):
        assert not c.update(patches(total=30,row=7),c.target,4+i,True)
    assert not c.corrections and not c.paused

@pytest.mark.parametrize("object_at", [12, 46])
def test_closes_until_tactile_contact_at_object_dependent_position(object_at):
    profile = settings_from({"gentle_limits": True, "output_limit": 43, "speed": 31,
                             "contact_peak": 4, "peak_ceiling": 21, "approach_travel": 108})
    control = FeedbackGrasp(profile, [220] * 20, [0] * 20)
    empty = patches(peak=0, total=0)
    for second in range(object_at):
        assert control.update(empty, control.target.copy(), second, False)
    at_contact = control.target.copy()
    for second in range(object_at, object_at + 6):
        assert control.update(patches(), control.target.copy(), second, False)
    assert all(control.target[j] == at_contact[j] for j in (0, 1, 15, 16))
    assert control.target[2] < at_contact[2] and control.target[3] < at_contact[3]
    assert control.target[0] > control.lower[0]
    assert control.stable_fingers == {"thumb", "index"}
    assert not control.search_stop


def test_transient_contact_can_resume_but_one_missing_frame_is_ignored():
    control = FeedbackGrasp(PROFILES["output_40"], [220] * 20, [0] * 20)
    empty = patches(peak=0, total=0)
    touched = dict(empty, thumb=Contact(4, 30, 4, 2))
    for second, sample in enumerate([empty, touched, empty, touched, empty]):
        control.update(sample, control.target.copy(), second, False)
    assert control.target[0] == 218 and control.target[1] < 218
    control.update(empty, control.target.copy(), 5, False)
    assert control.target[0] == 216


@pytest.mark.parametrize("limit", ["travel_limit", "no_progress"])
def test_limit_and_stall_report_unverified_and_cancel_inward_error(limit):
    control = FeedbackGrasp(PROFILES["output_40"], [220] * 20, [214] * 20 if limit == "travel_limit" else [0] * 20)
    empty = patches(peak=0, total=0)
    for i in range(40):
        actual = control.target.copy() if limit == "travel_limit" else [220] * 20
        control.update(empty, actual, i / 2, False)
    assert control.search_stop == limit and not control.seek_contact
    assert not control.anchors and not control.stable_fingers
    assert control.target[0] == (214 if limit == "travel_limit" else 220)


def test_gentle_limits_validated_before_connecting(test_setup):
    plugin, _, bus, *_ = test_setup
    for field, value in {"output_limit": 86, "speed": 41, "contact_peak": 6, "peak_ceiling": 151,
                         "approach_travel": 191, "max_corrections": 4}.items():
        with pytest.raises(ValueError, match=field):
            plugin.dispatch("prepare", {"bound_side": "right", "confirm_motion": True,
                                        "gentle_limits": True, field: value})
    assert not bus.sent_messages
    assert settings_from({"output_limit": 56}).output_limit == 56


@pytest.mark.parametrize("make_contact", [True, False])
def test_extended_travel_respects_touch_and_each_joint_limit(make_contact):
    settings = settings_from({"gentle_limits": True, "approach_travel": 160})
    control = FeedbackGrasp(settings, DEFAULT_PREP_POSE, DEFAULT_CLOSE_LIMIT_POSE)
    empty = patches(peak=0, total=0)
    for frame_no in range(200):
        sample = patches() if make_contact and frame_no >= 75 else empty
        control.update(sample, control.target.copy(), frame_no / 10, False)
        assert all(v >= low for v, low in zip(control.target, DEFAULT_CLOSE_LIMIT_POSE))
    assert control.target[15] == 142  # Thumb tip can travel only 113 from 255.
    assert all(control.target[j] == 255 - 192 for j in (3, 4, 18, 19))
    if make_contact:
        assert control.target[1] == 105  # Contact after 150 units, before the new cap.
        assert control.stable_fingers == {"thumb", "index"} and control.search_stop == "fingers_settled"
    else:
        assert control.target[1] == 95 and control.search_stop == "travel_limit"
        assert not control.stable_fingers

@pytest.mark.parametrize(("side", "gentle", "expected_yaw"),
                         [("right", True, 115), ("left", True, 70), ("right", False, 70), ("right", None, 115)])
def test_thumb_alignment_from_prepare_through_close_and_release(test_setup, monkeypatch, side, gentle, expected_yaw):
    plugin, client, _, _, _, _, _ = test_setup
    commands = []
    send_positions = client.set_joint_positions
    def record(target):
        commands.append(list(target))
        return send_positions(target)
    monkeypatch.setattr(client, "set_joint_positions", record)
    monkeypatch.setattr(client, "get_fresh_matrix_touch", lambda **kw: frame({}))
    args = {"bound_side": side, "confirm_motion": True}
    def state():
        return plugin.dispatch("get_state", {"bound_side": side})
    if gentle is None:
        monkeypatch.setenv("L20_DEFAULT_GENTLE_LIMITS", "1")
        plugin = type(plugin)(plugin.hand, plugin.config, plugin.global_config)
        monkeypatch.setattr(plugin, "_get_client_and_ctrl", test_setup[0]._get_client_and_ctrl)
        plugin._acp_callback = test_setup[0]._acp_callback
        schema = plugin.get_tool()["inputSchema"]["properties"]
        assert schema["gentle_limits"]["default"] is True
        assert schema["approach_travel"]["default"] == 150
    prepare_args = args if gentle is None else dict(args, gentle_limits=gentle)
    expected_tip_limit = 195 if side == "right" and gentle is not False else 142
    preview = plugin.dispatch("preview_settings", prepare_args)
    accepted = plugin.dispatch("prepare", prepare_args)
    assert accepted["thumb_tip_min_raw"] == preview["thumb_tip_min_raw"] == expected_tip_limit
    wait_for(state, lambda s: (s["last_result"] or {}).get("result", {}).get("state") == "prepared")
    actual = state()["last_result"]["result"]
    assert actual["thumb_yaw_target"] == expected_yaw
    assert actual["thumb_tip_min_raw"] == expected_tip_limit
    assert actual["gentle_limits"] is (gentle is not False)
    assert actual["settings"]["approach_travel"] == (110 if gentle is False else 150)
    expected = plugin.prep_pose.copy()
    expected[5] = expected_yaw
    assert commands[-1] == expected
    commands.clear()
    if side=="right" and gentle is not False:
        start_right_countdown(plugin,args,monkeypatch)
        operator_message(plugin,"放好了")
    else:
        plugin.dispatch("close_until_contact", args)
    closing = wait_for(state, lambda s: commands and commands[-1][0] <= 170, seconds=9)
    trace = commands.copy()
    assert all(v[5] == expected_yaw and v[15] >= expected_tip_limit for v in trace)
    if side == "right" and gentle is not False:
        assert trace[-1][15] == 195 and trace[-1][0] < trace[-1][15]
    else:
        assert trace[-1][15] < 195
    assert all(v[10] == expected[10] for v in trace)
    assert preview["thumb_alignment_enabled"] is False
    assert closing["tactile_status"]["thumb_alignment_enabled"] is False
    assert closing["tactile_status"]["thumb_tip_min_raw"] == expected_tip_limit
    percent = 75 if side == "right" and gentle is not False else 100
    assert [report["finger_tip_travel_percent"] for report in
            (preview, accepted, actual, closing["tactile_status"])] == [percent] * 4
    # Compare the commands throughout the wrap, including the one-unit outer-digit steps.
    for v in trace:
        for i in range(1, 5):
            assert expected[i+15] - v[i+15] == (expected[i] - v[i]) * percent // 100
    commands.clear()
    plugin.dispatch("release", args)
    wait_for(state, lambda s: (s["last_result"] or {}).get("result", {}).get("hand_opened"))
    assert expected in commands

@pytest.mark.parametrize("finger", FINGER_JOINTS)
def test_three_fresh_frames_on_any_one_finger_stop_only_that_digit(finger):
    profile = settings_from(dict(gentle_limits=True, contact_peak=4, peak_ceiling=57))
    control = FeedbackGrasp(profile, DEFAULT_PREP_POSE, DEFAULT_CLOSE_LIMIT_POSE)
    empty = patches(peak=0, total=0)
    for i in range(5):
        control.update(empty, control.target.copy(), i / 10, False)
    sample = dict(empty)
    sample[finger] = Contact(4, 20, 4, 2)
    for i in range(2):
        control.update(sample, control.target.copy(), 1 + i / 10, False)
        assert not control.anchors
    control.update(sample, control.target.copy(), 1.2, False)
    assert control.stable_fingers == {finger} and set(control.anchors) == {finger}
    held = control.target.copy()
    for i in range(3):
        control.update(sample, control.target.copy(), 2 + i / 10, False)
    for i, name in enumerate(FINGER_JOINTS):
        for j in (i, i + 15):
            if name == finger:
                assert control.target[j] == held[j]
            else:
                assert control.target[j] < held[j]
    assert not control.opposed and not control.corrections and control.seek_contact


def test_alternating_fingers_do_not_confirm_and_new_finger_gets_own_slip_reference():
    profile = settings_from({"max_corrections": 2})
    c = FeedbackGrasp(profile, DEFAULT_PREP_POSE, DEFAULT_CLOSE_LIMIT_POSE)
    empty = patches(peak=0, total=0)
    index = dict(empty, index=Contact(10, 80, 2, 2))
    middle = dict(empty, middle=Contact(4, 10, 8, 2))
    for i, frame in enumerate([index, middle] * 4):
        c.update(frame, c.target.copy(), i / 10, False)
        assert not c.anchors
    for i in range(3):
        c.update(index, c.target.copy(), 1+i/10, False)
    assert c.stable_fingers == {"index"}
    for i in range(3):
        c.update(middle, c.target.copy(), 2+i/10, True)
    assert c.stable_fingers == {"middle"} and c.anchors["middle"] == middle["middle"]
    assert not c.paused and c.corrections == 0


def test_thumb_at_limit_does_not_stop_other_fingers_searching():
    hard_limit = [0] * 20
    hard_limit[0] = hard_limit[15] = 220
    c = FeedbackGrasp(PROFILES["output_40"], [220]*20, hard_limit)
    empty = patches(peak=0,total=0)
    for i in range(8):
        c.update(empty, c.target.copy(), i / 10, False)
    assert c.seek_contact and not c.search_stop and c.target[2] < 215
    for i in range(3):
        c.update(dict(empty, middle=Contact(10, 50, 3, 2)), c.target.copy(), 1+i/10, False)
    assert c.stable_fingers == {"middle"}


def test_card_pressure_fault_reports_which_limit_triggered(test_setup, monkeypatch):
    plugin, client, _, _, _, events, _ = test_setup
    readings = {}
    monkeypatch.setattr(client, "get_fresh_matrix_touch", lambda **kw: frame(readings))
    args = {"bound_side": "left", "confirm_motion": True}
    def state():
        return plugin.dispatch("get_state", {"bound_side": "left"})
    plugin.dispatch("prepare", dict(args, gentle_limits=True, output_limit=70, peak_ceiling=28))
    wait_for(state, lambda s: (s["last_result"] or {}).get("result", {}).get("state") == "prepared")
    readings.update(index=(4, 34), thumb=(5, 1))
    close = plugin.dispatch("close_until_contact", args)
    wrapping = wait_for(state, lambda s: s["tactile_status"]["phase"] == "wrapping")
    assert wrapping["tactile_status"]["peak_limited_fingers"] == ["index"]
    assert wrapping["tactile_status"]["hard_stop_peak_raw"] == 42
    readings["index"] = (4, 42)
    result = wait_for(state, lambda s: (s["last_result"] or {}).get("action_id") == close["action_id"]
                      and s["last_result"]["result"].get("error_code"))["last_result"]["result"]
    assert result["error_code"] == "tactile_peak_limit" and result["search_stop"] is None
    assert result["trigger_fingers"] == ["index"] and result["peak_ceiling_raw"] == 28
    assert result["trigger_peaks"]["thumb"] == 1 and result["trigger_peaks"]["index"] == 42
    assert result["hard_stop_peak_raw"] == 42 and not result["saturated_fingers"]
    wait_for(lambda: events, lambda es: any(e[0] == close["action_id"] and
             e[2].get("error_code") == "tactile_peak_limit" for e in es))


def test_shared_envelope_accepts_independent_maxima():
    settings = settings_from(dict(gentle_limits=True, output_limit=85, peak_ceiling=150,
                                  approach_travel=190, speed=29, max_corrections=0))
    assert settings == GraspSettings(85, 29, 3, 150, 190, 0)


def test_correction_waits_for_corrected_finger_even_if_another_stays_in_contact():
    c = FeedbackGrasp(PROFILES["output_40"], [220] * 20, [0] * 20)
    sample = patches(peak=0, total=0)
    sample.update(index=Contact(8, 60, 2, 2), middle=Contact(8, 60, 3, 3))
    for i in range(3):
        c.update(sample, c.target.copy(), i / 10, False)
    slipping = dict(sample, index=Contact(6, 40, 5, 2))
    c.update(slipping, c.target.copy(), 1.0, True)
    c.update(slipping, c.target.copy(), 1.1, True)
    assert c.corrections == 1 and c.last_evidence == ("index",)
    missing = dict(sample, index=Contact(0, 0, 0, 0))
    for t in (1.7, 2.2, 2.7):
        c.update(missing, c.target.copy(), t, True)
    assert c.paused and c.corrections == 1

def test_canvas_reports_selected_peak_and_allowed_range_before_motion(test_setup):
    plugin, _, bus, *_ = test_setup
    args = {"bound_side": "right", "gentle_limits": True, "output_limit": 70,
            "contact_peak": 5, "peak_ceiling": 60}
    schema = plugin.get_tool()["inputSchema"]
    assert "2–150" in schema["properties"]["peak_ceiling"]["description"]
    assert "2–150" in schema["x-action-params"]["prepare"]["description"]
    preview = plugin.dispatch("preview_settings", args)
    assert preview["settings"]["peak_ceiling"] == 60
    assert preview["parameter_limits"]["peak_ceiling"] == {"minimum": 2, "maximum": 150}
    assert preview["parameter_limits"]["contact_peak"] == {"minimum": 1, "maximum": 5}
    assert preview["parameter_limits"]["output_limit"]["maximum"] == 85
    assert not bus.sent_messages and not plugin._prepared and not plugin._active
    for value in (28, 64, 80, 93, 100, 150):
        preview = plugin.dispatch("preview_settings", dict(args, peak_ceiling=value))
        assert preview["settings"]["peak_ceiling"] == value
    with pytest.raises(ValueError, match="150"):
        plugin.dispatch("preview_settings", dict(args, peak_ceiling=151))
    assert not bus.sent_messages


def test_preview_and_prepare_agree_above_old_limit_and_preview_leaves_selection_intact(test_setup, monkeypatch):
    plugin, client, bus, *_ = test_setup
    monkeypatch.setattr(client, "get_fresh_matrix_touch", lambda **kw: frame({}))
    args = {"bound_side": "left", "gentle_limits": True, "contact_peak": 5, "peak_ceiling": 93}
    preview = plugin.dispatch("preview_settings", args)
    accepted = plugin.dispatch("prepare", dict(args, confirm_motion=True))
    state = lambda: plugin.dispatch("get_state", {"bound_side": "left"})
    final = wait_for(state, lambda s: (s["last_result"] or {}).get("action_id") == accepted["action_id"])
    result = final["last_result"]["result"]
    assert result["state"] == "prepared"
    assert result["parameter_limits"] == accepted["parameter_limits"] == preview["parameter_limits"]
    assert result["hard_stop_peak_raw"] == accepted["hard_stop_peak_raw"] == preview["hard_stop_peak_raw"] == 140
    assert result["settings"] == accepted["settings"] == preview["settings"]
    before = len(bus.sent_messages)
    assert plugin.dispatch("preview_settings", dict(args, peak_ceiling=150))["settings"]["peak_ceiling"] == 150
    assert len(bus.sent_messages) == before
    assert plugin._prepared[("left", "can0")]["settings"].peak_ceiling == 93

def test_upper_envelope_stops_inward_motion_and_saturation_cannot_be_tared_away():
    settings = settings_from({"gentle_limits": True, "peak_ceiling": 150})
    control = FeedbackGrasp(settings, DEFAULT_PREP_POSE, DEFAULT_CLOSE_LIMIT_POSE)
    zero = frame({"middle": (4, 2)})
    baseline = empty_baseline([zero] * 3)
    at_ceiling = contact_patches(frame({"middle": (4, 152)}), baseline)
    control.update(at_ceiling, control.target.copy(), 0., False)
    assert control.relief_count == 1 and not control.anchors
    for force in (151, 255):
        sampled = contact_patches(frame({"middle": (4, force)}), baseline)
        assert sampled["middle"].peak == force - 2
        assert sampled["middle"].saturated is (force == 255)
        control.update(sampled, control.target.copy(), 1., False)
    assert control.overpressure_fingers == ("middle",) and control.paused

def test_contacts_can_arrive_in_sequence_and_lagging_finger_cannot_authorize_transfer():
    settings = settings_from({"gentle_limits": True, "approach_travel": 20})
    c = FeedbackGrasp(settings, [220]*20, [0]*20)
    sample = patches(peak=0,total=0)
    sample["index"] = Contact(8, 30, 3, 2)
    for n in range(90):
        if n == 6:
            sample["ring"] = Contact(9, 25, 2, 2)
        actual = c.target.copy()
        actual[4] = actual[19] = 220  # No simulated movement from the little finger.
        c.update(sample, actual, n / 10, False)
    assert c.search_stop == "no_progress" and c.stable_fingers == {"ring", "index"}
    assert c.remaining_fingers == ["pinky"] and c.travel_limited == {"thumb", "middle"}
    assert c.target[1] == 220 and c.target[3] == 214
    assert c.target[4] == c.target[19] == 220


def test_preview_shows_outer_digit_travel_and_hard_limits_still_win(test_setup):
    plugin, _, bus, *_ = test_setup
    for base, extra in ((150, 180), (190, 228), (255, 255)):
        args = dict(bound_side="right", gentle_limits=base < 255, approach_travel=base)
        assert plugin.dispatch("preview_settings", args)["travel_by_finger"] == {
            "thumb": base, "index": base, "middle": base, "ring": extra, "pinky": extra}
    assert not bus.sent_messages
    start = [250] * 20
    hard = [0] * 20
    hard[3], hard[19] = 118, 122
    c = FeedbackGrasp(settings_from({"approach_travel": 190}), start, hard)
    empty = patches(peak=0,total=0)
    for n in range(240):
        c.update(empty, c.target.copy(), n / 10, False)
    assert c.target[2] == 60 and c.target[3] == 118 and c.target[19] == 122
    assert c.target[18] == c.target[4] == 22 and c.search_stop == "travel_limit"

def test_early_contact_pressure_shifts_during_wrap_are_not_slip_after_handover():
    profile = settings_from({"approach_travel": 10, "max_corrections": 2})
    c = FeedbackGrasp(profile, [220]*20, [0]*20)
    empty = patches(peak=0,total=0)
    for i in range(20):
        index = Contact(12, 80, 2, 2) if i < 7 else Contact(6, 45, 5, 2)
        c.update(dict(empty, index=index), c.target.copy(), i / 10, False)
    assert not c.seek_contact and c.search_stop == "fingers_settled"
    for i in range(3):
        c.update(dict(empty, index=index), c.target.copy(), 2+i/10, True)
    assert not c.corrections and not c.paused
    for i in range(2):
        c.update(dict(empty, index=Contact(4, 25, 7, 2)), c.target.copy(), 3+i/10, True)
    assert c.corrections == 1 and c.last_evidence == ("index",)


def test_pressure_on_one_finger_cancels_residual_inward_error_on_others():
    c = FeedbackGrasp(PROFILES["output_40"], [220]*20, [0]*20)
    empty = patches(peak=0, total=0)
    for i in range(6):
        c.update(dict(empty, index=Contact(5, 15, 2, 2)), c.target.copy(), i / 10, False)
    actual = c.target.copy()
    actual[4] += 2
    c.update(dict(empty, index=Contact(90, 100, 3, 2)), actual, 1, False)
    assert c.paused and c.target[4] == actual[4]

def test_thumb_pad_adjustment_respects_stricter_configured_limit(test_setup):
    plugin, _, bus, *_ = test_setup
    plugin.close_limit_pose[15] = 198
    sample = plugin.dispatch("preview_settings", {"bound_side":"right", "gentle_limits":True, "peak_ceiling":15})
    assert sample["thumb_tip_min_raw"] == 198 and sample["settings"]["peak_ceiling"] == 15
    assert sample["parameter_limits"]["peak_ceiling"] == {"minimum":2, "maximum":150}
    assert not bus.sent_messages

@pytest.mark.parametrize("nominal, hard", [(15, 23), (28, 42), (90, 135), (150, 225)])
def test_subthreshold_overage_keeps_wrapping_until_hard_boundary(nominal, hard):
    c = FeedbackGrasp(settings_from({"gentle_limits": True, "peak_ceiling": nominal}),
                      [220] * 20, [0] * 20)
    empty = patches(peak=0, total=0)
    for i in range(5):
        c.update(empty, c.target.copy(), i / 10, False)
    early_thumb = c.target[0]
    for i, peak in enumerate([nominal, hard - 1, hard - 1, hard - 1]):
        c.update(dict(empty, thumb=Contact(peak, peak, 3, 2), index=Contact(peak, peak, 3, 2)),
                 c.target.copy(), 1+i/10, False)
    assert c.profile.hard_stop_peak_raw == hard
    assert not c.paused and c.stable_fingers == {"thumb", "index"}
    assert c.target[0] >= early_thumb and c.target[2] < early_thumb
    old = c.target.copy()
    c.update(dict(empty, thumb=Contact(hard, hard, 3, 2)), old.copy(), 2., False)
    assert c.paused and c.overpressure_fingers == ("thumb",)
    assert all(a >= b for a,b in zip(c.target,old))


def test_slip_correction_skips_finger_above_soft_limit():
    c = FeedbackGrasp(settings_from({"peak_ceiling": 20, "max_corrections": 2}), [220]*20, [0]*20)
    initial = patches(peak=24,total=110,row=2)
    for i in range(3):
        c.update(initial, c.target.copy(), i/10, False)
    held = c.target.copy()
    mixed = dict(initial, thumb=Contact(22,65,5,2),index=Contact(12,65,5,2))
    for i in range(2):
        c.update(mixed, c.target.copy(), 1+i/10, True)
    assert not c.paused and c.corrections == 1
    assert c.last_evidence == ("index",)
    assert c.target[0] == held[0] and c.target[1] == held[1] - 1


def test_raw_saturation_stops_even_when_corrected_peak_is_below_hard_threshold():
    c = FeedbackGrasp(settings_from({"peak_ceiling": 200}), [220]*20, [0]*20)
    baseline = empty_baseline([frame({"thumb": (2, 2)})]*3)
    hit = contact_patches(frame({"thumb": (2, 255)}),baseline)
    assert hit["thumb"].peak == 253 < c.profile.hard_stop_peak_raw
    c.update(hit,c.target.copy(),0.,False)
    assert c.paused and c.saturated_fingers == ("thumb",)

def test_best_contact_window_survives_zero_and_holds_position():
    c = FeedbackGrasp(settings_from({"approach_travel": 8}), [220]*20, [0]*20)
    empty = patches(peak=0,total=0)
    for i, v in enumerate([0, 21, 0, 6, 8, 10, 7, 0, 0, 0]):
        sample = dict(empty, middle=Contact(v,v,3,2))
        c.update(sample,c.target.copy(),i/10,False)
    assert not c.paused and c.waiting_for_signal and not c.stable_fingers
    best = c.best_contact_windows["middle"]
    assert best["mean_peak_raw"] == (21+0+6)/3
    assert best["frames"] == 3 and best["span_ms"] == 200
    assert c.search_stop == "contact_faded" and c.remaining_fingers
    held = c.target.copy()
    for i in range(12):
        assert not c.update(empty,held.copy(),1+i/10,False)
    assert c.target == held and c.corrections == 0


def test_subthreshold_per_finger_means_and_sample_gaps_do_not_latch():
    c = FeedbackGrasp(PROFILES["output_40"],[220]*20,[0]*20)
    empty = patches(peak=0,total=0)
    for i,(t,f,v) in enumerate([(0,"thumb",4),(.1,"index",6),(.2,"thumb",4),
                                 (.3,"middle",4),(.4,"middle",4),(2,"middle",9),(2.1,"middle",0)]):
        c.update(dict(empty, **{f:Contact(v,v,2,2)}),c.target.copy(),t,False)
    assert not c.best_contact_windows and not c.waiting_for_signal


@pytest.mark.parametrize("drop_when", ["wrapping", "monitoring"])
def test_contact_fade_preserves_hold_and_contact_evidence(test_setup, monkeypatch, drop_when):
    plugin, client, _, _, _, events, _ = test_setup
    args = {"bound_side": "left", "confirm_motion": True}
    readings = {}
    commands = []
    send = client.set_joint_positions
    def record(target):
        commands.append(list(target))
        return send(target)
    monkeypatch.setattr(client, "set_joint_positions", record)
    monkeypatch.setattr(client, "get_fresh_matrix_touch", lambda **kw: frame(readings.copy()))
    def state():
        return plugin.dispatch("get_state", {"bound_side": "left"})
    def phase(expected):
        return wait_for(state, lambda v:v["tactile_status"].get("phase") == expected)["tactile_status"]
    plugin.dispatch("prepare", dict(args, approach_travel=24, max_corrections=2))
    wait_for(state, lambda s:(s["last_result"] or {}).get("result", {}).get("state") == "prepared")
    readings["middle"] = (4, 10)
    close = plugin.dispatch("close_until_contact", args)
    phase(drop_when)
    wait_for(state, lambda s:s["tactile_status"].get("tactile_contact_confirmed"))
    readings.clear()
    review = phase("hold_without_signal")
    assert not review["tactile_contact_confirmed"] and review["grasp_confirmed"] and review["grasp_action_succeeded"]
    assert review["contact_fingers"] == [] and set(review["peaks"].values()) == {0}
    assert review["approach_contact_windows"]["middle"]["mean_peak_raw"] == 10
    assert bool(review["remaining_fingers"]) == (drop_when == "wrapping")
    n = len(commands)
    assert not review["slip_monitoring_available"] and review["corrections"] == 0
    assert len(commands) == n
    readings["index"] = (5, 8)
    monitored = phase("monitoring")
    assert monitored["contact_fingers"] == ["index"] and monitored["slip_monitoring_available"]
    assert monitored["corrections"] == 0 and len(commands) == n
    readings.clear()
    phase("hold_without_signal")
    assert len([e for e in events if e[0] == close["action_id"]]) == 1
    plugin.dispatch("release", args)
    wait_for(state, lambda s:(s["last_result"] or {}).get("result",{}).get("hand_opened"))


def test_historical_contact_cannot_bypass_overpressure_or_start_a_new_grasp():
    profile = PROFILES["output_40"]
    c = FeedbackGrasp(profile,[220]*20,[0]*20)
    empty = patches(peak=0, total=0)
    for n in range(6):
        seen = dict(empty,index=Contact(6,12,3,3)) if n < 4 else empty
        c.update(seen, c.target.copy(), n/20, False)
    assert c.waiting_for_signal
    c.update(dict(empty,thumb=Contact(99,99,2,2)),c.target.copy(),.4,False)
    assert c.paused and c.overpressure_fingers == ("thumb",)
    fresh = FeedbackGrasp(profile,[220]*20,[0]*20)
    assert not fresh.best_contact_windows and not fresh.waiting_for_signal

def test_visual_flag_cannot_override_card_pressure_fault_or_no_contact(test_setup, monkeypatch):
    plugin, client, _, _, _, _, _ = test_setup
    args = {"bound_side": "left", "confirm_motion": True}
    readings = {}
    monkeypatch.setattr(client, "get_fresh_matrix_touch", lambda **kw: frame(readings.copy()))
    def state():
        return plugin.dispatch("get_state", {"bound_side": "left"})
    plugin.dispatch("prepare", dict(args, approach_travel=8, peak_ceiling=15))
    wait_for(state,lambda s:(s["last_result"] or {}).get("result",{}).get("state") == "prepared")
    plugin.dispatch("close_until_contact", args)
    wait_for(state,lambda s:s["tactile_status"].get("phase") == "unverified_hold")
    assert "begin_transfer" not in plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
    readings["thumb"] = (3, 24)
    wait_for(state,lambda s:s["tactile_status"].get("phase") == "needs_support")
    assert "begin_transfer" not in plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
    plugin.dispatch("release", args)
    wait_for(state,lambda s:(s["last_result"] or {}).get("result",{}).get("hand_opened"))

def test_aligned_thumb_tracks_opening_locks_on_touch_and_respects_short_travel():
    empty = patches(peak=0, total=0)
    profile = settings_from({"gentle_limits": True, "approach_travel": 36})
    c = FeedbackGrasp(profile, DEFAULT_PREP_POSE, DEFAULT_CLOSE_LIMIT_POSE, align_thumb=True)
    for i in range(100):
        c.update(empty, c.target.copy(), i / 10, False)
    assert c.search_stop == "travel_limit"
    assert c.initial[15] - c.target[15] == 9
    assert c.initial[0] - c.target[0] == 36
    c = FeedbackGrasp(settings_from({"gentle_limits": True}), DEFAULT_PREP_POSE,
                      DEFAULT_CLOSE_LIMIT_POSE, align_thumb=True)
    for i in range(20):
        c.update(empty, c.target.copy(), i / 10, False)
    before_roll = c.target[10]
    actual = c.target.copy()
    actual[10] -= 2
    contact = dict(empty, thumb=Contact(5, 5, 2, 2), middle=Contact(5, 5, 2, 2))
    for n in range(3):
        c.update(contact, actual if n == 0 else c.target.copy(), 2 + n / 10, False)
    assert c.thumb_middle_contact_seen and c.thumb_rotation_locked
    assert c.target[10] == before_roll - 2
    assert {"thumb", "middle"} <= c.latched
    for n in range(4):
        c.update(empty, c.target.copy(), 3 + n / 10, False)
    assert c.target[10] == before_roll - 2 and c.thumb_middle_contact_seen


def test_flatter_four_finger_wrap_preserves_roots_thumb_and_contact_response():
    profile = settings_from({"approach_travel": 150})
    start = DEFAULT_PREP_POSE.copy()
    limits = DEFAULT_CLOSE_LIMIT_POSE.copy()
    limits[17] = start[17] - 80
    old = FeedbackGrasp(profile, start, limits)
    new = FeedbackGrasp(profile, start, limits, finger_tip_travel_percent=75)
    empty = patches(peak=0, total=0)
    for n in range(200):
        sample = dict(empty, index=Contact(8, 24, 3, 2)) if n >= 20 else empty
        for c in (old, new):
            c.update(sample, c.target.copy(), n/10, False)
        assert new.target[:16] == old.target[:16]
        for j in range(16, 20):
            assert start[j] - new.target[j] == (start[j] - old.target[j]) * 75 // 100
    assert new.target[1] == 215 and new.target[16] == 225
    assert start[17] - new.target[17] == 60
    assert [start[j] - new.target[j] for j in (3, 4, 18, 19)] == [180, 180, 135, 135]
    assert new.search_stop == "fingers_settled" and new.latched == {"index"}
    # A slip correction can still make a one-unit step on a contacted finger,
    # including when its root is already at the limit and its tip has room.
    new.target[1] = new.lower[1]
    before = new.target[16]
    assert new._close(["index"], new.target.copy(), 1)
    assert new.target[16] == before - 1
    assert not new._close(["ring"], new.target.copy(), 1)

@pytest.mark.parametrize("values, expected", [([2, 4, 9], True), ([0, 15, 0], True), ([2, 4, 8], False)])
def test_arithmetic_mean_includes_low_and_zero_frames_and_survives_rest(values, expected):
    c = FeedbackGrasp(settings_from({"contact_peak":5, "peak_ceiling":45}),[220]*20,[0]*20)
    empty = patches(peak=0,total=0)
    for i,v in enumerate(values):
        c.update(dict(empty,middle=Contact(v,v,2,2)),c.target.copy(),i/10,False)
        assert bool(c.best_contact_windows) == (i==2 and expected)
    held = c.target.copy()
    for i in range(8):
        c.update(empty,c.target.copy(),.3+i/10,False)
    assert bool(c.best_contact_windows) == expected and not c.stable_fingers
    if expected:
        assert c.best_contact_windows["middle"]["mean_peak_raw"] == sum(values)/3
        assert c.waiting_for_signal and c.target == held and not c.corrections

def test_mean_only_receipt_stays_successful_without_transfer(test_setup, monkeypatch):
    plugin, client, _, _, _, events, _ = test_setup
    args = {"bound_side":"right","confirm_motion":True}
    monkeypatch.setattr(client, "get_fresh_matrix_touch", lambda **kw: frame({}))
    def state():
        return plugin.dispatch("get_state", {"bound_side":"right"})
    plugin.dispatch("prepare",dict(args, gentle_limits=True, contact_peak=5, peak_ceiling=45))
    wait_for(state, lambda s:(s.get("last_result") or {}).get("result",{}).get("state")=="prepared")
    values = iter([2,4,9])
    monkeypatch.setattr(client, "get_fresh_matrix_touch", lambda **kw: frame({"middle":(4,next(values,0))}))
    action = start_right_countdown(plugin, args, monkeypatch)["action_id"]
    operator_message(plugin, "放好了")
    review = wait_for(state, lambda s:s["tactile_status"]["phase"]=="hold_without_signal")["tactile_status"]
    assert review["grasp_action_succeeded"] and review["grasp_confirmed"]
    assert not review["tactile_contact_confirmed"] and "operator_visual_confirmed" not in review
    assert review["approach_contact_windows"]["middle"]["mean_peak_raw"] == 5
    wait_for(lambda:events, lambda es:any(x[0]==action for x in es))
    receipt = next(x[2] for x in events if x[0]==action)
    assert receipt["grasp_action_succeeded"] and receipt["grasp_confirmed"]
    assert not receipt["tactile_contact_confirmed"] and "本轮抓握动作成功" in receipt["message"]
    plugin.dispatch("release",args)
    wait_for(state,lambda s:(s.get("last_result") or {}).get("result",{}).get("hand_opened"))

@pytest.mark.parametrize("reply", ["等一下", "放好了", None])
def test_placement_timer_is_single_use_and_stoppable(test_setup,monkeypatch,reply):
    plugin, client, _, _, _, _, _ = test_setup
    args = dict(bound_side="right",confirm_motion=True)
    monkeypatch.setattr(client,"get_fresh_matrix_touch",lambda **kw:frame({}))
    def state():return plugin.dispatch("get_state",{"bound_side":"right"})
    def prepare():
        action=plugin.dispatch("prepare",dict(args,gentle_limits=True,approach_travel=15))["action_id"]
        wait_for(state,lambda s:(s.get("last_result") or {}).get("action_id")==action)
        return action
    prepare_id=prepare()
    commands=[]
    send=client.set_joint_positions
    def record(v):
        commands.append((time.monotonic(),list(v)))
        return send(v)
    monkeypatch.setattr(client,"set_joint_positions",record)
    with pytest.raises(ValueError, match="prepare_id"):
        plugin.dispatch("close_until_contact",dict(args,prepare_id="stale",placement_notice=PLACEMENT_NOTICE))
    start=time.monotonic()
    close=start_right_countdown(plugin,args,monkeypatch)
    wait_for(state,lambda s:s["tactile_status"].get("phase")=="waiting_for_placement")
    operator_message(plugin,"放好了",ts=time.time()-30)
    assert not commands
    if reply:operator_message(plugin,reply)
    if reply=="等一下":
        wait_for(state,lambda s:(s.get("last_result") or {}).get("status")=="cancelled")
        assert not commands
        with pytest.raises(ValueError,match="prepare"):
            plugin.dispatch("close_until_contact",dict(args,prepare_id=prepare_id,placement_notice=PLACEMENT_NOTICE))
    else:
        wait_for(lambda:commands,bool,seconds=7)
        elapsed=commands[0][0]-start
        assert elapsed >= 5 if reply is None else elapsed < 4
        with pytest.raises((RuntimeError,ValueError)):
            plugin.dispatch("prepare",dict(args,gentle_limits=True))
        plugin.dispatch("release",args)
        wait_for(state,lambda s:(s.get("last_result") or {}).get("result",{}).get("hand_opened"))

@pytest.mark.parametrize(("trace", "ceiling"), [
    ([0, 0, 20], 12),  # First qualifying window ends on the stop frame.
    ([0, 0, 18], 12),  # Equality also trips the hard stop.
    ([4, 4, 4, 25], 15),  # Already successful before the stop.
])
def test_overpressure_after_success_reports_both_facts(test_setup,monkeypatch, trace, ceiling):
    plugin, client, _, _, _, events, _=test_setup
    args=dict(bound_side="left", confirm_motion=True)
    values=iter([])
    monkeypatch.setattr(client,"get_fresh_matrix_touch",lambda **kw:frame({"thumb":(4,next(values,0))}))
    def state():return plugin.dispatch("get_state",{"bound_side":"left"})
    plugin.dispatch("prepare",dict(args,contact_peak=3,peak_ceiling=ceiling))
    wait_for(state,lambda s:(s.get("last_result") or {}).get("result",{}).get("state")=="prepared")
    values=iter(trace)
    close=plugin.dispatch("close_until_contact",args)
    result=wait_for(state,lambda s:(s.get("last_result") or {}).get("action_id")==close["action_id"])["last_result"]
    assert result["status"]=="error"
    end=result["result"]
    assert end["grasp_action_succeeded"] and end["grasp_confirmed"] and end["torque_released"]
    assert end["error_code"]=="tactile_peak_limit" and end["trigger_peaks"]["thumb"]==trace[-1]
    assert end["tactile_status"]["grasp_confirmed"] and "已成功" in end["message"]
    window = end["approach_contact_windows"]["thumb"]
    assert window["peak_samples_raw"] == trace[-3:]
    assert window["mean_peak_raw"] == round(sum(trace[-3:]) / 3, 2)
    assert not end["holding"] and not end["tactile_status"]["tactile_contact_confirmed"]


def test_canvas_dds_cancel_hits_driver_before_deadline(test_setup,monkeypatch):
    pytest.importorskip("rclpy")
    from common.vendor_runtime import DualDomainROS2
    from rclpy.node import Node
    from std_msgs.msg import String
    plugin,client,*_=test_setup
    ros=DualDomainROS2(165,166)
    ros.start_spin()
    speaker=Node("phanthy_bus_bridge",context=ros.ctx_core)
    publisher=speaker.create_publisher(String,"/remote_control/message",10)
    ros.executor_core.add_node(speaker)
    plugin._ros2=ros
    try:
        plugin.start()
        wait_for(plugin._input_available,bool,seconds=8)
        wait_for(publisher.get_subscription_count,bool,seconds=8)
        monkeypatch.setattr(client,"get_fresh_matrix_touch",lambda **kw:frame({}))
        args=dict(bound_side="right",confirm_motion=True)
        def state():return plugin.dispatch("get_state",{"bound_side":"right"})
        ident=plugin.dispatch("prepare",dict(args,gentle_limits=True))["action_id"]
        wait_for(state,lambda s:(s.get("last_result") or {}).get("action_id")==ident)
        commands=[]
        monkeypatch.setattr(client,"set_joint_positions",lambda v:commands.append(list(v)))
        plugin.dispatch("close_until_contact",dict(args,prepare_id=ident,placement_notice=PLACEMENT_NOTICE))
        wait_for(state,lambda s:s["tactile_status"].get("phase")=="waiting_for_placement")
        publisher.publish(String(data=json.dumps({"text":"等一下","ts":time.time()})))
        wait_for(state,lambda s:(s.get("last_result") or {}).get("status")=="cancelled")
        assert not commands
    finally:
        plugin.stop()
        ros.executor_core.remove_node(speaker)
        speaker.destroy_node()
        ros.shutdown()

@pytest.mark.parametrize("previous_times", [[], [0.1], [0.1, 0.2]])
def test_stop_frame_never_invents_missing_or_stale_samples(previous_times):
    c = FeedbackGrasp(settings_from({"contact_peak":3,"peak_ceiling":12}),[220]*20,[0]*20)
    empty = patches(peak=0,total=0)
    for t in previous_times:
        c.update(empty, c.target.copy(), t, False)
    pose = c.target.copy()
    c.update(dict(empty,thumb=Contact(20,20,3,2)),pose.copy(),2.0,False)
    assert c.paused and not c.best_contact_windows and c.corrections == 0
    assert all(new >= old for new, old in zip(c.target, pose))

def test_same_frame_success_does_not_advance_any_finger_or_start_new_evidence_in_hold():
    cfg = settings_from({"contact_peak": 3, "peak_ceiling": 12})
    empty = patches(peak=0, total=0)
    trigger = dict(empty, thumb=Contact(20, 20, 3, 2))
    for wrapping in (True, False):
        c = FeedbackGrasp(cfg, [220]*20, [0]*20)
        c.seek_contact = wrapping
        for ts in (0.0, 0.1):
            c.update(empty, c.target.copy(), ts, not wrapping)
        pose = c.target.copy()
        c.update(trigger, pose.copy(), 0.2, not wrapping)
        assert all(target >= initial for target, initial in zip(c.target, pose))
        assert c.paused and not c.corrections and bool(c.best_contact_windows) == wrapping
