"""Native feedback ingestion, freshness, subprocess lifetime and schema checks."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from test_payload_mass import raw_frame
from payload_mass_feedback import DdsFeedback
from payload_mass import PayloadMassPlugin
from payload_mass_sampling import MassError, SharedSampler
from payload_mass_dds_build import schema_blobs


def event(stamp=1):
    return {**raw_frame(), "event": "frame", "adapter": "native_dds_status",
            "source_seq": None, "source_timestamp": stamp,
            "host_received_monotonic_s": time.monotonic()}


def test_native_frame_keeps_source_timestamp_and_coherent_data():
    sampler = SharedSampler()
    feedback = DdsFeedback(sampler, {})
    frame = event(1234567890123456789)
    feedback._accept(frame)
    actual = sampler.diagnostics()["latest"]
    assert actual["source_timestamp"] == frame["source_timestamp"]  # no float conversion
    assert actual["source_seq"] is None
    for name in ("q", "dq", "tau", "errors", "temperature", "quaternion_xyzw", "gyro", "workmode"):
        assert actual[name] == frame[name]
    assert sampler.diagnostics()["freshness_status"] == "source_timestamp"


def test_duplicate_native_timestamp_never_refreshes_host_age():
    sampler = SharedSampler()
    feedback = DdsFeedback(sampler, {})
    feedback._accept(event(100))
    before = sampler.diagnostics()["latest"]["host_monotonic_s"]
    for _ in range(20):
        feedback._accept(event(100))
    d = sampler.diagnostics()
    assert d["received_count"] == 1 and d["source_duplicate_count"] == 20
    assert d["latest"]["host_monotonic_s"] == before
    feedback._accept(event(101))
    assert sampler.diagnostics()["received_count"] == 2


@pytest.mark.parametrize("changes, code", [
    ({"source_timestamp": None}, "FRESHNESS_UNVERIFIED"),
    ({"source_timestamp": 1.5}, "FRESHNESS_UNVERIFIED"),
    ({"source_timestamp": -1}, "FRESHNESS_UNVERIFIED"),
    ({"host_received_monotonic_s": 0}, "STALE_SOURCE"),
    ({"host_received_monotonic_s": time.monotonic() + 100}, "STALE_SOURCE"),
    ({"adapter": "shared_highcontroller_getter"}, "BAD_DATA"),
    ({"event": "error", "error_code": "SOURCE_RESET", "message": "reset"}, "SOURCE_RESET"),
])
def test_invalid_or_pipe_delayed_status_is_not_a_sample(changes, code):
    sampler = SharedSampler()
    with pytest.raises(MassError) as err:
        DdsFeedback(sampler, {})._accept({**event(), **changes})
    assert err.value.code == code
    assert sampler.diagnostics()["received_count"] == 0


def test_schema_metadata_checks_all_shipped_types():
    header = Path(__file__).resolve().parents[1] / "noetix_sdk_bumi/include/Robot_status.hpp"
    blobs = schema_blobs(header.read_text())
    assert len(blobs) == 12
    assert {name for name, _ in blobs} == {"ImuData", "JoyData", "MotorState", "BmsState", "MotorStateArray", "StatusData"}


def test_reader_subprocess_stops_and_is_reaped():
    sampler = SharedSampler()
    script = "import json,time; print(" + repr(json.dumps(event())) + ", flush=True); time.sleep(30)"
    # Real child pipe exercises the thread and shutdown, without DDS or SDK.
    feedback = DdsFeedback(sampler, {}, popen=lambda args, **kw:
                           subprocess.Popen([sys.executable, "-u", "-c", script], **kw))
    try:
        assert feedback.start()
        deadline = time.monotonic() + 2
        while sampler.diagnostics()["received_count"] < 1 and time.monotonic() < deadline:
            time.sleep(.01)
        assert sampler.diagnostics()["received_count"] == 1
    finally:
        feedback.stop()
    assert feedback._process.poll() is not None and not feedback._thread.is_alive()
    assert sampler.diagnostics()["read_error_count"] == 0


def test_missing_native_binary_never_falls_back_to_getter(tmp_path):
    sampler = SharedSampler()
    feedback = DdsFeedback(sampler, {})
    feedback._binary = tmp_path / "missing-reader"
    assert feedback.start() is False
    assert sampler.diagnostics()["read_error_count"] == 1
    assert sampler.diagnostics()["latest"] is None
    feedback.stop()


def test_reader_exit_records_error_and_invalidates_session_epoch():
    sampler = SharedSampler()
    feedback = DdsFeedback(sampler, {}, popen=lambda args, **kw:
                           subprocess.Popen([sys.executable, "-c", "pass"], **kw))
    try:
        assert feedback.start()
        feedback._thread.join(2)
        assert not feedback._thread.is_alive()
        with pytest.raises(MassError) as err:
            sampler.read_since(0, 0)
        assert err.value.code == "DATA_ERROR"
    finally:
        feedback.stop()


@pytest.mark.skipif(not os.environ.get("BUMI_TEST_DDS_READER") or not os.environ.get("BUMI_TEST_DDS_PUBLISHER"),
                    reason="requires compiled native reader and test-only status publisher")
def test_native_dds_to_quick_scale_calibration_unknown_freeze_and_home(monkeypatch, tmp_path):
    """Real native DDS transport and schema; synthetic robot/torque, no hardware."""
    import threading
    from types import SimpleNamespace
    # Loopback only and a unique topic: never discover the actual robot.
    monkeypatch.setenv("CYCLONEDDS_URI", '<CycloneDDS><Domain><General><Interfaces>'
                       '<NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast>'
                       '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
                       '<Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    topic = "PayloadMassSyntheticTest_" + str(os.getpid())
    publisher = subprocess.Popen([os.environ["BUMI_TEST_DDS_PUBLISHER"], "199", topic],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 text=True, bufsize=1)
    def command(text):
        publisher.stdin.write(text + "\n")
        publisher.stdin.flush()
    sampler = SharedSampler()
    # Exercise the production default: discover the unique status type/topic.
    feedback = DdsFeedback(sampler, {"dds_domain": 199})
    feedback._binary = Path(os.environ["BUMI_TEST_DDS_READER"])
    class NativeTestLoco:
        _action_lock = threading.RLock()
        _lock = threading.Lock()
        _payload_owner = None
        def __init__(self):
            self._high_ctrl = SimpleNamespace(get_mode=lambda: sampler.diagnostics()["latest"]["workmode"])
        def _preempt_servo(self, reason):
            pass
        def is_moving(self):
            return False
        def _do_teaching_action(self, action, args):
            command("bend")
            return {"command_sent": True}
        def _do_stop_playback(self, reason):
            command("home")
            return {"confirmed": True}
    results = []
    card = PayloadMassPlugin({"setup_mode": "quick", "recording_id": 12,
                              "data_dir": str(tmp_path / "trials")}, "native_test",
                             sampler=sampler, loco_plugin=NativeTestLoco(), feedback=feedback,
                             publish=lambda r: results.append(r), notify=lambda *a: None)
    def wait(predicate, timeout=9):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.02)
        raise AssertionError({"card": card.info(), "publisher_exit": publisher.poll()})
    try:
        assert card.start()
        wait(lambda: sampler.diagnostics()["received_count"] > 4)
        assert sampler.diagnostics()["freshness_status"] == "source_timestamp"
        assert card.dispatch("prepare", {})["state"] == "running"
        wait(lambda: card.info()["state"] == "ready")
        assert card.dispatch("weigh", {})["state"] == "measuring"
        command("mass .2")
        wait(lambda: card.info().get("sampling_stable") and card.info().get("trial_file"))
        assert card.dispatch("calibrate", {"known_mass_kg": .2})["state"] == "calibrated"
        wait(lambda: card.info().get("valid") and card.info().get("mass_kg") == .2)
        command("mass .35")
        wait(lambda: card.info().get("valid") and card.info().get("mass_kg") == .35)
        command("mass 0")
        wait(lambda: card.info().get("valid") and card.info().get("mass_kg") == 0)
        command("freeze")
        wait(lambda: card.info().get("error_code") == "STALE_SOURCE")
        assert card.info()["mass_kg"] is None and not card.info()["valid"]
        cursor = sampler.diagnostics()["received_count"]
        command("resume")
        wait(lambda: sampler.diagnostics()["received_count"] > cursor + 4)
        assert card.dispatch("stop", {})["state"] == "running"
        wait(lambda: card.info().get("home_confirmed"))
        assert card.info()["state"] == "finished"
        report = {"simulation_only": True, "transport": "native DDS via bundled schema/runtime",
                  "reference_kg": .2, "unknown_kg": .35, "unload_kg": 0,
                  "frozen_source_clears_weight": True, "home_confirmed": True,
                  "received_count": sampler.diagnostics()["received_count"],
                  "events": [{k: r.get(k) for k in ("state", "mass_kg", "valid", "error_code")}
                             for r in results]}
        destination = Path(os.environ.get("BUMI_TEST_DDS_REPORT", str(tmp_path / "native_report.json")))
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        card.stop()
        publisher.terminate()
        try:
            publisher.wait(timeout=2)
        except subprocess.TimeoutExpired:
            publisher.kill()
            publisher.wait(timeout=2)
        publisher.stdin.close()
        publisher.stdout.close()
        publisher.stderr.close()
