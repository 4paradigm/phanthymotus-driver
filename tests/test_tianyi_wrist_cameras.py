"""ROS-/SDK-free contract tests for Tianyi's four D405 wrist cards."""
import importlib.util
from pathlib import Path
import sys
import zlib

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
SOURCE = ROOT / "x-humanoid" / "tianyi2.0" / "wrist_cameras.py"
SPEC = importlib.util.spec_from_file_location("tianyi_wrist_cameras", SOURCE)
wrist = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(wrist)


def session(side="left", serial="261922273335"):
    return wrist.WristCameraSession("test_robot", side, serial)


def test_four_cards_have_independent_names_topics_and_formats():
    cards = [wrist.WristCameraPlugin(session(side), modality).get_tool()
             for side in ("left", "right") for modality in ("rgb", "depth")]
    assert {card["name"] for card in cards} == {
        "camera_wrist_left_rgb", "camera_wrist_left_depth",
        "camera_wrist_right_rgb", "camera_wrist_right_depth",
    }
    assert len({card["topic_out"][0]["topic"] for card in cards}) == 4
    for card in cards:
        assert card["type"] == "sensor"
        modality = card["name"].rsplit("_", 1)[-1]
        assert card["topic_out"][0]["format"] == wrist.FORMATS[modality]


def test_unbound_serial_registers_cards_but_cannot_open_camera():
    left = session(serial="")
    info = left.info("rgb")
    assert info["state"] == "unconfigured"
    assert info["serial"] is None
    assert info["camera_info"][0]["K"] is None
    assert info["camera_info"][0]["source"] == "unknown"
    with pytest.raises(RuntimeError, match="RealSense SDK Serial Number"):
        left.start("rgb")
    assert left._proc is None


def test_default_bindings_match_confirmed_physical_wrists():
    cfg = yaml.safe_load((SOURCE.parent / "config.yaml").read_text())
    bindings = cfg["plugins"]["wrist_cameras"]
    assert bindings["left_serial"] == "261922273335"
    assert bindings["right_serial"] == "262122270344"


def test_bad_or_duplicate_binding_fails_closed_without_usb():
    s = session(serial="not-a-sdk-serial")
    with pytest.raises(RuntimeError, match="12-digit"):
        s.start("depth")
    s = session()
    s.blocked_reason = "left and right must bind different devices"
    with pytest.raises(RuntimeError, match="different devices"):
        s.start("rgb")


def test_stopped_card_does_not_report_old_frame_as_fresh():
    s = session()
    s._status = {"last_frame": {"rgb": 1e20}, "frames": {"rgb": 99}}
    info = s.info("rgb")
    assert info["state"] == "idle"
    assert info["fresh"] is False


def test_depth_output_is_zlib_of_scaled_little_endian_millimetres():
    np = pytest.importorskip("numpy")
    data = np.zeros((480, 640), dtype=np.uint16)
    data[0, 0] = 1000
    data[0, 1] = 500
    mm = np.frombuffer(zlib.decompress(wrist.encode_depth_mm(data, 0.0001)),
                       dtype="<u2").reshape(480, 640)
    assert mm[0, 0] == 100  # 1000 SDK units at 0.1 mm/unit
    assert mm[0, 1] == 50
    assert mm[1, 0] == 0
    assert mm.shape == (480, 640)


def test_depth_never_wraps_out_of_range_values():
    np = pytest.importorskip("numpy")
    data = np.zeros((480, 640), dtype=np.uint16)
    data[0, 0] = 1000
    result = np.frombuffer(zlib.decompress(wrist.encode_depth_mm(data, 1.0)),
                           dtype="<u2")
    assert result[0] == 0


@pytest.mark.parametrize("bad_scale", [0, -1, float("nan"), float("inf")])
def test_bad_depth_scale_is_rejected(bad_scale):
    np = pytest.importorskip("numpy")
    data = np.zeros((480, 640), dtype=np.uint16)
    with pytest.raises(ValueError):
        wrist.encode_depth_mm(data, bad_scale)


def test_depth_shape_is_verified_not_resized_and_guessed():
    np = pytest.importorskip("numpy")
    with pytest.raises(ValueError):
        wrist.encode_depth_mm(np.zeros((424, 240), dtype=np.uint16), 0.001)


def test_sdk_calibration_is_reported_with_provenance():
    s = session()
    s._status = {"intrinsics": {"depth": {
        "K": [600., 0., 320., 0., 600., 240., 0., 0., 1.],
        "D": [0., 0., 0., 0., 0.],
        "sdk_distortion": "distortion.brown_conrady",
    }}}
    info = s.info("depth")
    assert info["camera_info"][0]["source"] == "vendor-spec"
    assert info["camera_info"][0]["K"][0] == 600.
    assert info["camera_info"][0]["vendor"]["sdk_distortion"] == "distortion.brown_conrady"
    assert info["depth_aligned_to_rgb"] is False


def test_worker_is_separate_process_and_not_opened_during_construction():
    s = session()
    assert s._proc is None
    assert s._wanted == set()
    assert s.info("rgb")["state"] == "idle"
    assert s.info("depth")["state"] == "idle"


def test_wrist_cards_are_listed_in_marketplace():
    manifest = yaml.safe_load((SOURCE.parent / "driver.yaml").read_text())
    declared = {card["name"]: card["type"] for card in manifest["cards"]}
    for side in ("left", "right"):
        for channel in ("rgb", "depth"):
            assert declared[f"camera_wrist_{side}_{channel}"] == "sensor"


class CaptureProcess:
    def __init__(self, *, target, args, **kwargs):
        self.args = args
        self.alive = False

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def join(self, timeout):
        self.alive = False

    def close(self):
        pass



@pytest.mark.parametrize("final_channel", ["rgb", "depth"])
def test_rapid_channel_switches_preserve_final_selection(monkeypatch, final_channel):
    # Stub only the camera process; exercise real start/stop and shared memory.
    # The capture process is deliberately not consuming during the burst.
    s = session()
    monkeypatch.setattr(s._ctx, "Process", CaptureProcess)
    assert s.start("rgb")["state"] == "loading"
    worker = s._proc
    selection = worker.args[4]
    try:
        for _ in range(100):
            s.start("depth")
            s.stop("depth")
        if final_channel == "depth":
            s.start("depth")
            s.stop("rgb")
        assert s._proc is worker
        # Bit 0 is RGB; bit 1 is depth. Read the worker's view, not _wanted.
        assert selection.value == (1 if final_channel == "rgb" else 2)
    finally:
        s.stop("rgb")
        s.stop("depth")


def test_capture_worker_protects_python_logs_before_initialization(monkeypatch, capfd):
    import queue
    import threading
    from common import logsafe

    # Restore pytest's streams after the real logsafe.install() replaces them.
    monkeypatch.setattr(sys, "stdout", sys.stdout)
    monkeypatch.setattr(sys, "stderr", sys.stderr)
    monkeypatch.setattr(logsafe, "_installed", False)

    def missing_profile(path):
        print("worker\x00\x1b[31m diagnostic\x1b[0m")
        return False

    monkeypatch.setattr(wrist.os.path, "isfile", missing_profile)
    for name in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FASTRTPS_DEFAULT_PROFILES_FILE",
                 "RCUTILS_COLORIZED_OUTPUT"):
        monkeypatch.setenv(name, wrist.os.environ.get(name, ""))
    status = queue.Queue()
    wrist._capture_worker("test_robot", "left", "261922273335", 85,
                          session()._ctx.Value("B", 1), threading.Event(), status)
    assert "Missing loopback-only DDS profile" in status.get_nowait()["error"]
    assert capfd.readouterr().out == "worker diagnostic\n"


@pytest.mark.parametrize("now,last_frame,expected", [
    (100.1, None, "loading"),
    (100.2, 100.15, "running"),
    (112.0, None, "error"),
])
def test_first_frame_wait_settles_to_running_or_timeout(monkeypatch, now, last_frame, expected):
    from types import SimpleNamespace

    s = session()
    s._proc = SimpleNamespace(is_alive=lambda: True)
    s._wanted = {"rgb"}
    s._requested_at = {"rgb": 100.0}
    if last_frame is not None:
        s._status = {"last_frame": {"rgb": last_frame}}
    monkeypatch.setattr(wrist.time, "monotonic", lambda: now)
    info = s.info("rgb")
    assert info["state"] == expected
    if expected == "loading":
        assert info["error"] is None
        assert info["message"]
    elif expected == "running":
        assert info["fresh"] is True
        assert info["error"] is None
    else:
        assert info["error"] == "D405 not supplying fresh frames"


def test_capture_failure_is_not_hidden_by_loading():
    from types import SimpleNamespace

    s = session()
    s._proc = SimpleNamespace(is_alive=lambda: True)
    s._wanted = {"depth"}
    s._requested_at = {"depth": wrist.time.monotonic()}
    s._status = {"error": "D405 device is busy"}
    info = s.info("depth")
    assert info["state"] == "error"
    assert info["error"] == "D405 device is busy"


def test_loading_transitions_when_worker_reports_first_frame(monkeypatch):
    from types import SimpleNamespace

    s = session()
    s._proc = SimpleNamespace(is_alive=lambda: True)
    s._wanted = {"depth"}
    s._requested_at = {"depth": 100.0}
    monkeypatch.setattr(wrist.time, "monotonic", lambda: 101.0)
    assert s.info("depth")["state"] == "loading"
    s._status = {"last_frame": {"depth": 100.9}, "frames": {"depth": 1}}
    info = s.info("depth")
    assert info["state"] == "running"
    assert info["fresh"] is True
    assert info["frames_published"] == 1


def test_worker_exit_during_loading_reports_error():
    from types import SimpleNamespace

    s = session()
    s._proc = SimpleNamespace(is_alive=lambda: False)
    s._wanted = {"rgb"}
    s._requested_at = {"rgb": wrist.time.monotonic()}
    info = s.info("rgb")
    assert info["state"] == "error"
    assert info["error"] == "D405 capture worker exited"


def test_bundle_starts_all_wrist_previews_without_project_start(monkeypatch):
    import ast
    from common import lifecycle

    tree = ast.parse((SOURCE.parent / "main.py").read_text())
    bundle_class = next(node for node in tree.body
                        if isinstance(node, ast.ClassDef) and node.name == "TianyiDeviceBundle")
    scope = {"DualDomainROS2": object, "os": wrist.os, "_lifecycle": lifecycle}
    exec(compile(ast.Module(body=[bundle_class], type_ignores=[]), "main.py", "exec"), scope)
    monkeypatch.setitem(sys.modules, "wrist_cameras", wrist)
    monkeypatch.setattr(session()._ctx, "Process", CaptureProcess)
    config = yaml.safe_load((SOURCE.parent / "config.yaml").read_text())
    bundle = scope["TianyiDeviceBundle"](
        {"plugins": {"wrist_cameras": config["plugins"]["wrist_cameras"]}},
        "test_robot", None, None)
    try:
        bundle.start_all()  # Driver boot path, before any canvas action:start.
        cards = bundle._plugins
        assert len(cards) == 4
        assert all(card in bundle._started_plugins for card in cards)
        sessions = {card._session for card in cards}
        assert len(sessions) == 2
        assert all(s._proc.is_alive() and s._channels.value == 3 for s in sessions)
        assert bundle.dispatch("camera_wrist_left_rgb", {"action": "stop"})["state"] == "idle"
        left = next(s for s in sessions if s.side == "left")
        assert left._proc.is_alive() and left._channels.value == 2
        bundle.stop_all()
        assert all(s._proc is None for s in sessions)
        assert bundle.dispatch("camera_wrist_left_rgb", {"action": "start"})["state"] == "loading"
        assert left._proc.is_alive() and left._channels.value == 1
    finally:
        bundle.stop_all()


def test_plugins_are_discoverable_by_the_wrist_prefix():
    assert wrist.WristCameraPlugin.PREFIX == "camera_wrist"
    cards = [wrist.WristCameraPlugin(session(side), channel)
             for side in ("left", "right") for channel in ("rgb", "depth")]
    discovered = {card.get_tool()["name"] for card in cards
                  if card.PREFIX == "camera_wrist"}
    assert discovered == {"camera_wrist_left_rgb", "camera_wrist_left_depth",
                          "camera_wrist_right_rgb", "camera_wrist_right_depth"}
