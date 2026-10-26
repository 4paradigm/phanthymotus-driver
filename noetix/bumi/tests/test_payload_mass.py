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


def raw_frame(mass=0, seq=1):
    return {"motor_ids": list(range(21)), "q": [0.] * 21, "dq": [0.] * 21,
            "tau": [1+2*mass, -.5-mass, .2, .7+.6*mass] + [0.] * 17,
            "temperature": [30.] * 21, "errors": [0] * 21,
            "quaternion_xyzw": [0, 0, 0, 1], "gyro": [0, 0, 0], "workmode": 2,
            "source_seq": seq, "source_timestamp": None, "host_monotonic_s": float(seq),
            "host_time_s": time.time(), "adapter": "SYNTHETIC_ONLY"}


def configuration():
    return {"sample_duration_s": 3, "profile": {
        "arm": "left", "motor_ids": [0, 1, 2, 3],
        "joint_names": ["l_arm_pitch_joint", "l_arm_roll_joint", "l_arm_yaw_joint", "l_elbow_pitch_joint"],
        "hardware_id": "SIMULATION", "sdk_version": "SYNTHETIC", "control_config": "test",
        "hook_id": "test", "mapping_verified": True, "mapping_evidence": "synthetic test",
        "load_range_confirmed": True, "feedback_verified": True, "feedback_evidence": "synthetic",
        "torque_units": "sdk_raw", "accept_host_only_freshness": False,
        "pose_q": [0.] * 21, "quaternion_xyzw": [0, 0, 0, 1], "workmode": 2,
        "pose_tolerance_rad": .03, "body_tolerance_rad": .03, "max_velocity_rad_s": .03,
        "max_gyro_rad_s": .03, "max_torque_span": .03, "torque_abs_limit": 10,
        "max_temperature_c": 60, "max_temperature_drift_c": 3, "max_mass_kg": 1,
        "noise_floor_raw": .002, "max_calibration_error_kg": .03},
        "motion": {"verified": True, "evidence": "SYNTHETIC_ONLY", "bend_recording_id": 100,
                   "home_recording_id": 101, "home_q": [.1] * 21, "home_workmode": 2, "timeout_s": 1},
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
        self.frame = raw_frame()
        self.epoch = 0
        self.age = 0
        self.hook = None

    def diagnostics(self):
        return {"latest": copy.deepcopy(self.frame), "host_age_s": self.age, "read_error_count": self.epoch}

    def collect(self, duration, p, cancel):
        if self.hook:
            self.hook(cancel)
        if cancel.is_set():
            raise MassError("CANCELLED", "cancelled")
        check_frame(self.frame, p)
        return [{**copy.deepcopy(self.frame), "host_monotonic_s": n*.1} for n in range(32)]


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
        return {"state": "accepted"}

    def _do_stop_playback(self, reason):
        self.commands.append(("stop_playback", reason))
        if self.stop_result.get("confirmed"):
            self.sampler.frame["workmode"] = 2
        return self.stop_result


@pytest.fixture
def setup(tmp_path):
    cfg, _ = calibrated(tmp_path, configuration())
    path = tmp_path / "config.json"
    path.write_text(json.dumps(cfg))
    sampler = Sampler()
    sampler.frame["q"] = [.1] * 21
    loco = Loco(sampler)
    notifications = []
    card = PayloadMassPlugin({"profiles_file": str(path), "data_dir": str(tmp_path / "trials"),
                              "hardware_id": "SIMULATION", "sdk_version": "SYNTHETIC"},
                             "test", sampler=sampler, loco_plugin=loco, notify=lambda *a: notifications.append(a))
    yield card, sampler, loco, path, notifications
    card.stop()


def run(card, action, **args):
    result = card.dispatch(action, args)
    if result["state"] == "running":
        card._job["thread"].join(3)
        assert not card._job["thread"].is_alive()
        return copy.deepcopy(card._last)
    return result


def test_three_step_blind_weight_and_home(setup):
    card, sampler, loco, _, notes = setup
    assert run(card, "prepare")["state"] == "ready"
    assert card._session["baseline"]["summary"]["tau"][0] == 1
    sampler.frame = raw_frame(.47)
    result = run(card, "weigh")
    assert result["valid"] and result["unit"] == "kg"
    assert result["mass_kg"] == pytest.approx(.47, abs=.01)
    assert Path(result["trial_file"]).exists()
    sampler.frame = raw_frame()  # remove object before returning
    assert run(card, "finish")["home_confirmed"] is True
    assert loco._payload_owner is None
    assert card.info()["mass_kg"] is None
    assert [n[1] for n in notes] == ["completed"] * 3


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


def test_unconfigured_calibration_collects_without_fake_kg(setup):
    card, sampler, _, path, _ = setup
    cfg = json.loads(path.read_text()); cfg["model"] = None; path.write_text(json.dumps(cfg))
    assert run(card, "prepare")["state"] == "ready"
    sampler.frame = raw_frame(.47)
    result = run(card, "weigh")
    assert result["state"] == "uncalibrated" and result["mass_kg"] is None
    assert Path(result["trial_file"]).exists()


@pytest.mark.parametrize("failure,code", [
    (lambda s: setattr(s, "age", 99), "STALE_SOURCE"),
    (lambda s: s.frame["q"].__setitem__(3, 1), "POSE_MISMATCH"),
    (lambda s: s.frame["errors"].__setitem__(3, 1), "MOTOR_FAULT"),
    (lambda s: setattr(s, "epoch", 1), "STALE_SOURCE"),
    (lambda s: s.frame.update(workmode=26), "MODE_MISMATCH"),
])
def test_prepared_feedback_failure_clears_valid_mass(setup, failure, code):
    card, sampler, _, _, _ = setup
    run(card, "prepare"); sampler.frame = raw_frame(.47)
    assert run(card, "weigh")["valid"]
    failure(sampler)
    result = card.info()
    assert result["mass_kg"] is None and result["error_code"] == code


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


def test_finish_cannot_move_on_stale_feedback(setup):
    card, sampler, loco, _, _ = setup
    run(card, "prepare")
    sent = len(loco.commands)
    sampler.age = 99
    assert run(card, "finish")["error_code"] == "STALE_SOURCE"
    assert not any(x[0] == "play_recording" for x in loco.commands[sent:])


def test_cancel_during_sampling_never_commits_result(setup):
    card, sampler, loco, _, _ = setup
    entered = threading.Event()
    def block(cancel):
        entered.set(); cancel.wait(2)
    sampler.hook = block
    assert card.dispatch("prepare", {})["state"] == "running"
    assert entered.wait(1)
    assert card.dispatch("stop", {})["state"] == "cancelled"
    card._job["thread"].join(2)
    assert not card._last["valid"] and card._session is None
    assert loco._payload_owner is None


def test_publication_gate_blocks_all_pose_commands(setup):
    card, _, loco, _, notes = setup
    result = card.dispatch("prepare", {"_await_response_publication": True})
    assert not any(x[0] == "play_recording" for x in loco.commands)
    card.response_published(result["action_id"], False)
    card._job["thread"].join(2)
    assert not any(x[0] == "play_recording" for x in loco.commands)
    assert not notes


def test_failed_stop_preserves_exclusion(setup):
    card, sampler, loco, _, _ = setup
    run(card, "prepare"); sampler.frame["workmode"] = 23
    loco.stop_result = {"state": "accepted", "confirmed": False}
    assert card.dispatch("stop", {})["error_code"] == "STOP_UNCONFIRMED"
    assert loco._payload_owner is card
    sampler.frame["workmode"] = 2


def test_changed_load_invalidates_old_display(setup):
    card, sampler, _, _, _ = setup
    run(card, "prepare"); sampler.frame = raw_frame(.47); run(card, "weigh")
    sampler.frame = raw_frame(.7)
    assert card.info()["mass_kg"] is None


def test_repeated_prepare_cannot_tare_loaded_object(setup):
    card, sampler, _, _, _ = setup
    run(card, "prepare"); sampler.frame = raw_frame(.47)
    assert run(card, "prepare")["error_code"] == "ALREADY_PREPARED"


def test_canvas_exposes_exactly_three_business_actions(setup):
    card, _, _, _, _ = setup
    tool = card.get_tool()
    field = tool["inputSchema"]["properties"]["action"]
    system = {"start", "stop", "info", "config"}
    public = [a for a in field["enum"] if a not in system]
    titles = {o["const"]: o["title"] for o in field["oneOf"]}
    assert [titles[a] for a in public] == ["曲肘", "计算重量", "测量完毕"]
    assert tool["type"] == "actuator"
    assert set(public) == set(tool["inputSchema"]["x-completion"]["actions"])


def test_zero_signal_does_not_report_a_weight_and_keeps_baseline_for_retry(setup):
    card, sampler, _, _, _ = setup
    run(card, "prepare")
    result = run(card, "weigh")
    assert result["error_code"] == "BELOW_RESOLUTION" and result["mass_kg"] is None
    assert card._session is not None
    sampler.frame = raw_frame(.47)
    assert run(card, "weigh")["valid"]


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
