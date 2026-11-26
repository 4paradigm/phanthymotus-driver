"""Contracts for the two SDK station classes and the Helios sensor lifecycle."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from common import logsafe
from common.vendor_runtime import DriverBundle
from rokae.helios import spatial_map_worker as worker
from rokae.helios.spatial_map import HeliosSpatialMap
from test_helios_joints import wait_for


def test_station_ids_types_and_empty_map():
    manager = Mock()
    manager.get_current_map_name.return_value = "lab"
    reader = Mock()
    reader.get_work_points.return_value = []
    reader.get_charge_points.return_value = []
    assert worker.read_stations(manager, reader) == ("lab", [])
    reader.get_work_points.return_value = [SimpleNamespace(id=0, name="取件")]
    reader.get_charge_points.return_value = [SimpleNamespace(id=14, name="一号桩")]
    assert worker.read_stations(manager, reader) == ("lab", [
        {"id": 0, "name": "取件", "type": "work"}, {"id": 14, "name": "一号桩", "type": "charge"},
    ])
    manager.get_current_map_name.side_effect = ["lab", "other"]
    with pytest.raises(ValueError, match="changed maps"):
        worker.read_stations(manager, reader)


def test_location_zero_code_exact_stamp_and_frozen_source(monkeypatch):
    clock = Mock(return_value=20)
    monkeypatch.setattr(worker.time, "monotonic", clock)
    feedback = worker.LocationFeedback()
    source = SimpleNamespace(status=0, confidence=1.0, x=0.05, y=-0.36, theta=-0.012,
                             timestamp_ns=1795671335195000064)
    feedback.receive(source)
    assert feedback.fresh()["status_code"] == 0
    assert feedback.fresh()["timestamp_ns"] == "1795671335195000064"
    clock.return_value = 24
    feedback.receive(source)
    assert feedback.fresh() is None
    source.timestamp_ns += 1
    feedback.receive(source)
    assert feedback.fresh()["confidence"] == 1


def test_canvas_read_start_stop_without_vendor_runtime(monkeypatch, tmp_path):
    monkeypatch.setenv("HELIOS_SDK_PYTHON_DIR", str(tmp_path))
    plugin = HeliosSpatialMap({})  # Existing user configs predate the map SDK settings.
    bundle = DriverBundle([plugin])
    definition, = bundle.get_all_tools()
    assert (definition["name"], definition["type"], definition["multiInstance"]) == ("spatial_map", "sensor", False)
    assert definition["topic_out"] == [{"topic": "/rokae_helios/state/spatial_map", "format": "data/json"},
                                       {"topic": "/rokae_helios/state/spatial_map/preview", "format": "image/jpeg"}]
    assert bundle.dispatch("spatial_map", {"action": "info"})["preview_topic"] == definition["topic_out"][1]["topic"]
    assert bundle.dispatch("spatial_map", {"action": "read"})["data"] == {
        "state": "idle", "stations": None, "map_name": None, "localization": None,
    }
    with pytest.raises(ValueError, match="runtime mounts"):
        bundle.dispatch("spatial_map", {"action": "start"})
    assert bundle.dispatch("spatial_map", {"action": "stop"})["state"] == "idle"


def test_worker_logs_and_full_station_snapshot_use_separate_pipes(tmp_path, monkeypatch, capfd):
    (tmp_path / "py_client_common.py").write_text(
        "import os, sys\n"
        "from common.logsafe import LineAtomicStream\n"
        "class ClientConfig: pass\n"
        "def init_client(config):\n"
        "    assert isinstance(sys.stderr, LineAtomicStream)\n"
        "    os.write(2, b'native\\x00 diagnostic\\n')\n"
        "    print('SDK ready', flush=True)\n"
        "    return True\n"
        "def close_client(): pass\n")
    (tmp_path / "map_manager.py").write_text(
        "def has_map_manager(): return True\ndef get_current_map_name(): return 'lab'\n")
    (tmp_path / "localization.py").write_text("def has_location_dev(): return False\n")
    (tmp_path / "map_reader.py").write_text(
        "from types import SimpleNamespace\n"
        "def has_map_reader(): return True\n"
        "def get_work_points(): return [SimpleNamespace(id=i, name=f'工位{i}') for i in range(256)]\n"
        "def get_charge_points(): return []\n")
    monkeypatch.setattr(sys, "stderr", logsafe.LineAtomicStream(2))
    sensor = HeliosSpatialMap({})
    sensor._python, sensor._sdk = Path(sys.executable), tmp_path
    try:
        sensor._launch()
        wait_for(lambda: sensor.payload()["stations"] is not None)
        stations = sensor.payload()["stations"]
        assert len(stations) == 256 and stations[-1] == {"id": 255, "name": "工位255", "type": "work"}
        assert len(json.dumps(stations, ensure_ascii=False).encode()) > logsafe.PIPE_BUF
        logs = []

        def diagnostics_received():
            logs.extend(capfd.readouterr().err.splitlines())
            return "native diagnostic" in logs and "SDK ready" in logs

        wait_for(diagnostics_received)
        assert not any("工位" in line for line in logs)
    finally:
        sensor.stop()
