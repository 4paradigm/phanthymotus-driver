"""Hardware-free workflow, empirical blind mass, failure and ownership tests."""
import copy
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from uuid import uuid4

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from payload_mass import PayloadMassPlugin
from payload_mass_estimator import digest, summarize, validate_profile
from payload_mass_sampling import MassError, SharedSampler, check_frame
from calibrate_payload_mass import fit
from payload_mass_live import evaluate_window


def wait_for(card, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with card._lock:
            result = copy.deepcopy(card._last)
        if predicate(result):
            return result
        time.sleep(.01)
    raise AssertionError(card._last)


def run(card, action, **args):
    result = card.dispatch(action, args)
    if action == "weigh":
        return result
    if result["state"] == "running":
        card._job["thread"].join(3)
        assert not card._job["thread"].is_alive()
        return copy.deepcopy(card._last)
    return result


def test_click_before_hanging_live_ramp_stable_unload_stop(setup):
    card, sampler, loco, _, notes = setup
    assert run(card, "prepare")["state"] == "ready"
    baseline = copy.deepcopy(card._session["baseline"])
    assert run(card, "weigh")["state"] == "measuring"  # still EMPTY
    zero = wait_for(card, lambda r: r.get("stable") and r.get("mass_kg") == 0)
    assert zero["below_resolution"]
    sampler.frame = raw_frame(.2)
    preview = wait_for(card, lambda r: r.get("provisional") and r.get("mass_kg", 0) > .1)
    assert not preview["valid"] and not preview["stable"]
    sampler.frame = raw_frame(.47)
    stable = wait_for(card, lambda r: r.get("stable") and r.get("mass_kg") == pytest.approx(.47, abs=.01))
    assert stable["valid"] and stable["unit"] == "kg"
    assert card._session["baseline"] == baseline  # hanging never re-tares
    assert Path(stable["trial_file"]).exists()
    sampler.frame = raw_frame()
    wait_for(card, lambda r: r.get("stable") and r.get("mass_kg") == 0)
    assert run(card, "stop")["home_confirmed"]
    assert loco._payload_owner is None
    assert not card._last["valid"] and card._last["mass_kg"] is None
    assert [n[1] for n in notes] == ["completed"] * 3
    assert sum(c[0] == "play_recording" for c in loco.commands) == 1


def test_weigh_clicked_with_load_is_rejected_without_retare(setup):
    card, sampler, _, _, _ = setup
    run(card, "prepare")
    baseline = copy.deepcopy(card._session["baseline"])
    sampler.frame = raw_frame(.47)
    assert run(card, "weigh")["error_code"] == "NOT_EMPTY"
    assert card._session["baseline"] == baseline


def test_uncalibrated_live_saves_stable_data_without_inventing_kg(setup):
    card, sampler, _, path, _ = setup
    cfg = json.loads(path.read_text()); cfg["model"] = None; path.write_text(json.dumps(cfg))
    run(card, "prepare"); run(card, "weigh")
    sampler.frame = raw_frame(.47)
    result = wait_for(card, lambda r: r.get("sampling_stable") and r.get("trial_file"))
    assert result["error_code"] == "NOT_CALIBRATED" and result["mass_kg"] is None


@pytest.mark.parametrize("failure,code", [
    (lambda s: setattr(s, "age", 99), "STALE_SOURCE"),
    (lambda s: s.frame["q"].__setitem__(3, 1), "POSE_MISMATCH"),
    (lambda s: s.frame["errors"].__setitem__(3, 1), "MOTOR_FAULT"),
    (lambda s: setattr(s, "epoch", 1), "DATA_ERROR"),
    (lambda s: s.frame.update(workmode=26), "MODE_MISMATCH"),
])
def test_live_fault_clears_previous_numeric_reading(setup, failure, code):
    card, sampler, _, _, _ = setup
    run(card, "prepare"); run(card, "weigh"); sampler.frame = raw_frame(.47)
    wait_for(card, lambda r: r.get("valid"))
    failure(sampler)
    result = wait_for(card, lambda r: r.get("state") == "error")
    assert result["error_code"] == code and result["mass_kg"] is None


def test_hand_loading_velocity_keeps_stream_alive_but_unstable(setup):
    card, sampler, _, _, _ = setup
    run(card, "prepare"); run(card, "weigh")
    sampler.frame = raw_frame(.47); sampler.frame["dq"][3] = .04
    result = wait_for(card, lambda r: r.get("mass_kg", 0) == pytest.approx(.47, abs=.01))
    assert not result["valid"] and not result["stable"]
    sampler.frame["dq"][3] = 0
    assert wait_for(card, lambda r: r.get("stable"))["valid"]


def test_delayed_playback_cancel_retains_owner_until_start_then_stop(setup):
    card, sampler, loco, _, _ = setup
    sent = threading.Event()
    def delayed(action, args):
        sent.set()
        return {"state": "accepted", "command_sent": True, "confirmed_started": False, "workmode": 2}
    loco._do_teaching_action = delayed
    assert card.dispatch("prepare", {})["state"] == "running"
    assert sent.wait(1)
    assert card.dispatch("stop", {})["state"] == "running"
    time.sleep(.1)
    assert loco._payload_owner is card
    assert card._playback["sent"] and not card._playback["observed"]
    sampler.frame["workmode"] = 23
    result = wait_for(card, lambda r: r.get("home_confirmed"))
    assert result["state"] == "finished" and loco._payload_owner is None
    assert any(c[0] == "stop_playback" for c in loco.commands)


def test_never_observed_pending_command_fails_closed(setup):
    card, _, loco, _, _ = setup
    sent = threading.Event()
    def delayed(*args):
        sent.set()
        return {"state": "accepted", "command_sent": True}
    loco._do_teaching_action = delayed
    card.dispatch("prepare", {}); assert sent.wait(1)
    with card._lock:
        result = card.cancel_locked()
    assert result["error_code"] == "STOP_UNCONFIRMED"
    card._job["thread"].join(2)
    assert loco._payload_owner is card
    assert card._last["error_code"] == "STOP_UNCONFIRMED"


def test_return_requires_actual_home_pose(setup):
    card, _, loco, _, _ = setup
    run(card, "prepare")
    def stopped_but_not_home(reason):
        loco.sampler.frame["workmode"] = 2
        return {"confirmed": True}
    loco._do_stop_playback = stopped_but_not_home
    result = run(card, "stop")
    assert result["error_code"] == "POSE_TIMEOUT"
    assert loco._payload_owner is card and not result.get("home_confirmed")


def test_ack_failure_never_sends_recording(setup):
    card, _, loco, _, notes = setup
    result = card.dispatch("prepare", {"_await_response_publication": True})
    card.response_published(result["action_id"], False)
    card._job["thread"].join(2)
    assert not any(c[0] == "play_recording" for c in loco.commands)
    assert not notes


def test_stop_is_accepted_during_active_sampling_and_old_worker_cannot_overwrite(setup):
    card, sampler, loco, _, _ = setup
    run(card, "prepare"); run(card, "weigh")
    old = card._job
    result = run(card, "stop")
    assert result["home_confirmed"] and old["cancel"].is_set()
    old["thread"].join(2)
    assert card._last["home_confirmed"] and loco._payload_owner is None


def test_canvas_has_two_business_actions_and_a_system_stop(setup):
    card, _, _, _, _ = setup
    field = card.get_tool()["inputSchema"]["properties"]["action"]
    titles = {v["const"]: v["title"] for v in field["oneOf"]}
    assert [titles[a] for a in field["enum"] if a not in {"start", "stop", "info", "config"}] == ["曲肘", "测量重量"]
    assert "stop" in field["enum"]


def test_scale_window_rejects_dynamics_and_reports_zero_with_resolution(tmp_path):
    config, _ = calibrated(tmp_path, configuration())
    p = validate_profile("weighing", config["profile"], "left", loading=True)
    empty = [raw_frame(0, n) for n in range(1, 33)]
    baseline = {"profile_fingerprint": p["fingerprint"], "created_s": time.time(), "summary": summarize(empty, p)}
    result = evaluate_window(empty, baseline, config["model"], p, 3, time.time())
    assert result["mass_kg"] == 0 and result["below_resolution"] and result["stable"]
    ramp = [raw_frame(.5*n/32, n) for n in range(1, 33)]
    result = evaluate_window(ramp, baseline, config["model"], p, 3, time.time())
    assert result["mass_kg"] > .4 and not result["stable"] and not result["valid"]


def raw_frame(mass=0, seq=1):
    return {"motor_ids": list(range(21)), "q": [0.] * 21, "dq": [0.] * 21,
            "tau": [1+2*mass, -.5-mass, .2, .7+.6*mass] + [0.] * 17,
            "temperature": [30.] * 21, "errors": [0] * 21,
            "quaternion_xyzw": [0, 0, 0, 1], "gyro": [0, 0, 0], "workmode": 23,
            "source_seq": seq, "source_timestamp": None, "host_monotonic_s": float(seq),
            "host_time_s": time.time(), "error_epoch": 0, "adapter": "SYNTHETIC_ONLY"}


def configuration():
    return {"sample_duration_s": 3, "profile": {
        "arm": "left", "motor_ids": [0, 1, 2, 3],
        "joint_names": ["l_arm_pitch_joint", "l_arm_roll_joint", "l_arm_yaw_joint", "l_elbow_pitch_joint"],
        "hardware_id": "SIMULATION", "sdk_version": "SYNTHETIC", "control_config": "test",
        "hook_id": "test", "mapping_verified": True, "mapping_evidence": "synthetic test",
        "load_range_confirmed": True, "feedback_verified": True, "feedback_evidence": "synthetic",
        "torque_units": "sdk_raw", "accept_host_only_freshness": False,
        "pose_q": [0.] * 21, "quaternion_xyzw": [0, 0, 0, 1], "workmode": 23,
        "playback_hold_verified": True, "hold_evidence": "SYNTHETIC_ONLY",
        "pose_tolerance_rad": .03, "body_tolerance_rad": .03, "max_velocity_rad_s": .03,
        "max_gyro_rad_s": .03, "max_torque_span": .03, "torque_abs_limit": 10,
        "max_temperature_c": 60, "max_temperature_drift_c": 3, "max_mass_kg": 1,
        "noise_floor_raw": .002, "max_calibration_error_kg": .03},
        "motion": {"verified": True, "evidence": "SYNTHETIC_ONLY", "bend_recording_id": 100,
                   "home_q": [.1] * 21, "return_on_stop_verified": True, "timeout_s": 1},
        "model": None}


def trial_file(tmp_path, config, mass, action):
    p = validate_profile("weighing", config["profile"], "left", loading=True)
    frames = [raw_frame(mass, n) for n in range(1, 33)]
    trial = {"id": uuid4().hex, "profile": p, "profile_fingerprint": p["fingerprint"],
             "action": action, "created_s": time.time(), "raw_frames": frames,
             "summary": summarize(frames, p)}
    path = tmp_path / (trial["id"] + ".json")
    path.write_text(json.dumps(trial))
    return str(path)


def calibrated(tmp_path, config):
    manifest = []
    for role, masses in (("train", [0, 0, .3, .3, .8, .8]), ("validation", [.3, .8])):
        for mass in masses:
            manifest.append({"file": trial_file(tmp_path, config, mass, "weigh" if mass else "prepare"),
                             "mass_kg": mass, "role": role, "independent": True, "note": "synthetic"})
    return fit(config, manifest), manifest


class Sampler:
    def __init__(self):
        self.frame = raw_frame(); self.frame["host_monotonic_s"] = time.monotonic()
        self.epoch = 0
        self.age = 0
        self.hook = None
        self.serial = 0
        self.stamp = time.monotonic()

    def diagnostics(self):
        latest = {**copy.deepcopy(self.frame), "host_monotonic_s": self.stamp, "serial": self.serial}
        return {"latest": latest, "host_age_s": self.age, "read_error_count": self.epoch, "received_count": self.serial}

    def read_since(self, cursor, error_epoch):
        if self.epoch != error_epoch:
            raise MassError("DATA_ERROR", "synthetic interruption")
        frames = []
        for _ in range(4):
            self.serial += 1
            self.stamp += .1
            frames.append({**copy.deepcopy(self.frame), "serial": self.serial, "host_monotonic_s": self.stamp})
        return frames

    def collect(self, duration, p, cancel, *, learn_pose_from=None):
        if self.hook:
            self.hook(cancel)
        if cancel.is_set():
            raise MassError("CANCELLED", "cancelled")
        check_frame(self.frame, p)
        return [{**copy.deepcopy(self.frame), "host_monotonic_s": n*.1} for n in range(32)]

    def recent(self, duration, error_epoch):
        if self.epoch != error_epoch:
            raise MassError("DATA_ERROR", "synthetic interruption")
        return [{**copy.deepcopy(self.frame), "host_monotonic_s": n*duration/31} for n in range(32)]


class Loco:
    def __init__(self, sampler):
        self._lock = threading.Lock()
        self._action_lock = threading.RLock()
        self._payload_owner = None
        self._high_ctrl = SimpleNamespace(get_mode=lambda: sampler.frame["workmode"])
        self.sampler = sampler
        self.commands = []
        self.stop_result = {"state": "completed", "confirmed": True}
        self.pose_arrives = True

    def _preempt_servo(self, reason):
        self.commands.append(("pause", reason))

    def is_moving(self):
        return False

    def _do_teaching_action(self, action, args):
        self.commands.append((action, args))
        if self.pose_arrives:
            self.sampler.frame["q"] = [0.] * 21 if args["recording_id"] == 100 else [.1] * 21
        self.sampler.frame["workmode"] = 23
        return {"state": "running", "command_sent": True, "confirmed_started": True, "workmode": 23}

    def _do_stop_playback(self, reason):
        self.commands.append(("stop_playback", reason))
        if self.stop_result.get("confirmed"):
            self.sampler.frame["workmode"] = 2
            self.sampler.frame["q"] = [.1] * 21
        return self.stop_result


@pytest.fixture
def setup(tmp_path):
    cfg, _ = calibrated(tmp_path, configuration())
    path = tmp_path / "config.json"
    path.write_text(json.dumps(cfg))
    sampler = Sampler()
    sampler.frame["q"] = [.1] * 21
    sampler.frame["workmode"] = 2
    loco = Loco(sampler)
    notifications = []
    card = PayloadMassPlugin({"profiles_file": str(path), "data_dir": str(tmp_path / "trials"),
                              "hardware_id": "SIMULATION", "sdk_version": "SYNTHETIC"},
                             "test", sampler=sampler, loco_plugin=loco, notify=lambda *a: notifications.append(a))
    yield card, sampler, loco, path, notifications
    card.stop()


def test_missing_config_never_sends_motion(setup):
    card, _, loco, path, _ = setup
    path.unlink()
    assert run(card, "prepare")["error_code"] == "NOT_CONFIGURED"
    assert not loco.commands


@pytest.mark.parametrize("change,code", [
    (lambda c: c["motion"].update(verified=False), "MOTION_UNVERIFIED"),
    (lambda c: c["motion"].update(bend_recording_id=True), "BAD_PARAMETER"),
    (lambda c: c["profile"].update(hardware_id="different"), "IDENTITY_MISMATCH"),
    (lambda c: c["model"].update(weights=[-1]*4), "UNIDENTIFIABLE"),
    (lambda c: c["model"].update(k=[0]*4), "UNIDENTIFIABLE"),
])
def test_invalid_config_no_motion(setup, change, code):
    card, _, loco, path, _ = setup
    cfg = json.loads(path.read_text()); change(cfg); path.write_text(json.dumps(cfg))
    assert run(card, "prepare")["error_code"] == code
    assert not loco.commands


def test_no_baseline_and_unknown_truth_rejected(setup):
    card, _, loco, _, _ = setup
    assert run(card, "weigh")["error_code"] == "NOT_PREPARED"
    assert run(card, "prepare", known_mass_kg=.47)["error_code"] == "BAD_PARAMETER"
    assert not loco.commands


def test_pose_ack_is_not_arrival(setup):
    card, _, loco, _, _ = setup
    loco.pose_arrives = False
    assert run(card, "prepare")["error_code"] == "POSE_TIMEOUT"
    assert loco._payload_owner is card  # finish/stop still possible


def test_prepare_does_not_interrupt_someone_elses_playback(setup):
    card, sampler, loco, _, _ = setup
    sampler.frame["workmode"] = 23
    assert run(card, "prepare")["error_code"] == "MODE_MISMATCH"
    assert not any(x[0] in ("play_recording", "stop_playback") for x in loco.commands)
    sampler.frame["workmode"] = 2


def test_calibration_rejects_reused_trials_and_zero_response(tmp_path):
    config, manifest = calibrated(tmp_path, configuration())
    with pytest.raises(MassError, match="只能使用一次"):
        fit(config, manifest + [manifest[2]])
    for entry in manifest:
        trial = json.loads(Path(entry["file"]).read_text())
        for frame in trial["raw_frames"]:
            frame["tau"] = raw_frame()["tau"]
        Path(entry["file"]).write_text(json.dumps(trial))
    with pytest.raises(MassError) as err:
        fit(config, manifest)
    assert err.value.code == "UNIDENTIFIABLE"


def test_shared_sampler_maps_ids_and_rejects_stale_duplicate_and_fault():
    sampler = SharedSampler()
    frame = raw_frame()
    frame["motor_ids"] = list(reversed(frame["motor_ids"]))
    for key in ("q", "dq", "tau", "temperature", "errors"):
        frame[key] = list(reversed(frame[key]))
    assert sampler.push(frame)
    assert sampler.diagnostics()["latest"]["tau"][0] == 1
    assert not sampler.push(frame)
    bad = raw_frame(seq=2); bad["tau"][0] = float("nan")
    assert not sampler.push(bad)
    assert sampler.diagnostics()["read_error_count"] == 1


def test_shared_sampler_source_freshness_is_not_getter_receipt():
    profile = validate_profile("weighing", configuration()["profile"], "left", loading=True)
    frame = raw_frame(); frame["source_seq"] = None
    with pytest.raises(MassError) as err:
        check_frame(frame, profile)
    assert err.value.code == "FRESHNESS_UNVERIFIED"


def test_real_loco_reservation_blocks_actions_and_stream_but_allows_stop():
    # Load the real class without importing ROS or any native vendor library.
    import ast
    source = Path(__file__).resolve().parents[1] / "device.py"
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LocoPlugin")
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), cls], type_ignores=[])
    env = {"threading": threading, "time": time, "_get_default_cmd": lambda: "DEFAULT"}
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), env)
    commands = []
    ctrl = SimpleNamespace(publish_cmd=lambda *args: commands.append(args))
    loco = env["LocoPlugin"]({}, "test", None, ctrl)
    def cancel():
        with loco._lock:
            loco._payload_owner = None
        return {"state": "cancelled"}
    loco._payload_owner = SimpleNamespace(cancel_locked=cancel)
    assert loco.dispatch("move", {"_tool_name": "loco"})["error_code"] == "WEIGHING_BUSY"
    loco.publish_velocity(.3, 0, 0)
    assert not commands
    loco._dispatch_direct = lambda *args: {"state": "idle"}
    assert loco.dispatch("stop", {})["state"] == "idle"
    loco.publish_velocity(.3, 0, 0)
    assert len(commands) == 1


def test_actual_sampler_waits_for_continuous_stable_window():
    sampler = SharedSampler()
    profile = validate_profile("weighing", configuration()["profile"], "left", loading=True)
    profile.update(min_samples=5, settle_timeout_s=.2, max_receive_gap_s=.05)
    ended = threading.Event()
    def feed():
        seq = 1
        while not ended.is_set():
            sampler.push(raw_frame(.47, seq)); seq += 1
            ended.wait(.01)
    worker = threading.Thread(target=feed)
    worker.start()
    try:
        frames = sampler.collect(.06, profile, threading.Event())
        assert len(frames) >= 5
        assert frames[-1]["host_monotonic_s"] - frames[0]["host_monotonic_s"] >= .06
    finally:
        ended.set(); worker.join(1)


def test_actual_sampler_dropout_and_cancellation():
    sampler = SharedSampler()
    profile = validate_profile("weighing", configuration()["profile"], "left", loading=True)
    profile.update(min_samples=5, settle_timeout_s=.02, max_receive_gap_s=.02)
    with pytest.raises(MassError):
        sampler.collect(.03, profile, threading.Event())
    cancel = threading.Event(); cancel.set()
    with pytest.raises(MassError) as err:
        sampler.collect(.03, profile, cancel)
    assert err.value.code == "CANCELLED"


def test_reservation_and_real_servo_callback_complete_without_lock_cycle(setup):
    import ast
    import importlib.util
    card, sampler, _, _, _ = setup
    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root / "device.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LocoPlugin")
    module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), cls], type_ignores=[])
    env = {"threading": threading, "time": time, "_get_default_cmd": lambda: "DEFAULT"}
    exec(compile(ast.fix_missing_locations(module), "device.py", "exec"), env)
    commands = []
    ctrl = SimpleNamespace(get_mode=lambda: 2, publish_cmd=lambda *a: commands.append(a))
    loco = env["LocoPlugin"]({}, "test", None, ctrl)
    card._loco, card._lock = loco, loco._action_lock
    spec = importlib.util.spec_from_file_location("weigh_test_real_servo", root / "loco_servo.py")
    servo_module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = servo_module
    spec.loader.exec_module(servo_module)
    servo = servo_module.LocoServoPlugin.__new__(servo_module.LocoServoPlugin)
    servo._lock, servo._running, servo._paused = threading.RLock(), True, False
    servo._activity = {"state": "running"}
    servo._sink = object()
    servo._hold = lambda: None
    entered, continue_callback, reserved = threading.Event(), threading.Event(), threading.Event()
    def submit(sink, command):
        entered.set()
        assert continue_callback.wait(2)
        loco.publish_velocity(.3, 0, 0)
        return None
    servo._submit = submit
    loco.attach_servo(servo)
    def no_motion_job(*args):
        reserved.set()
        return {"state": "test_reserved"}
    card._new_job = no_motion_job
    callback = threading.Thread(target=servo._on_message, args=(SimpleNamespace(data="{}"),), daemon=True)
    result = []
    action = threading.Thread(target=lambda: result.append(card.dispatch("prepare", {})), daemon=True)
    try:
        callback.start(); assert entered.wait(1)
        action.start()
        deadline = time.monotonic() + 1
        while loco._payload_owner is None and time.monotonic() < deadline:
            time.sleep(.005)
        assert loco._payload_owner is card
        continue_callback.set()
        callback.join(1); action.join(1)
        assert not callback.is_alive() and not action.is_alive()
        assert reserved.is_set() and result == [{"state": "test_reserved"}]
        assert not commands  # publication admitted after reservation is suppressed
    finally:
        continue_callback.set()
        sys.modules.pop(spec.name, None)


def test_read_since_does_not_repeat_frames_or_hide_read_errors():
    sampler = SharedSampler()
    assert sampler.read_since(0, 0) == []
    sampler.push(raw_frame(seq=1))
    frames = sampler.read_since(0, 0)
    assert len(frames) == 1 and frames[0]["serial"] == 1
    assert sampler.read_since(1, 0) == []
    sampler.record_error("disconnected")
    with pytest.raises(MassError) as err:
        sampler.read_since(1, 0)
    assert err.value.code == "DATA_ERROR"


def test_live_pipeline_with_real_shared_sampler(tmp_path):
    config = configuration(); config["sample_duration_s"] = 1
    config, _ = calibrated(tmp_path, config)
    path = tmp_path / "live-config.json"; path.write_text(json.dumps(config))
    source = Sampler(); source.frame["q"] = [.1] * 21; source.frame["workmode"] = 2
    sampler, ended = SharedSampler(), threading.Event()
    loco = Loco(source)
    card = PayloadMassPlugin({"profiles_file": str(path), "data_dir": str(tmp_path / "live"),
                              "hardware_id": "SIMULATION", "sdk_version": "SYNTHETIC"},
                             "real_sampler", sampler=sampler, loco_plugin=loco, notify=lambda *a: None)
    def feed():
        seq = 0
        while not ended.is_set():
            seq += 1
            frame = copy.deepcopy(source.frame); frame["source_seq"] = seq
            sampler.push(frame)
            ended.wait(.02)
    thread = threading.Thread(target=feed, daemon=True); thread.start()
    try:
        deadline = time.monotonic() + 1
        while sampler.diagnostics()["received_count"] < 1 and time.monotonic() < deadline:
            time.sleep(.01)
        assert run(card, "prepare")["state"] == "ready"
        assert run(card, "weigh")["state"] == "measuring"
        source.frame = raw_frame(.47)
        result = wait_for(card, lambda r: r.get("valid"), timeout=3)
        assert result["mass_kg"] == pytest.approx(.47, abs=.01)
        assert result["freshness_status"] == "source_sequence"
        source.frame = raw_frame()
        assert run(card, "stop")["home_confirmed"]
    finally:
        card.stop(); ended.set(); thread.join(1)
