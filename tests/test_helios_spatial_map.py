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
    definition, control = bundle.get_all_tools()
    assert control["name"] == "controlled_spatial" and control["topic_out"][0]["format"] == "sensor/mapping"
    assert (definition["name"], definition["type"], definition["multiInstance"]) == ("spatial_map", "sensor", False)
    assert definition["topic_out"] == [{"topic": "/rokae_helios/state/spatial_map", "format": "sensor/mapping"},
                                       {"topic": "/rokae_helios/state/spatial_map/status", "format": "data/json"}]
    assert bundle.dispatch("spatial_map", {"action": "info"})["preview_topic"] == definition["topic_out"][0]["topic"]
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
    (tmp_path / "nav.py").touch()
    (tmp_path / "point_cloud.py").touch()
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


def test_vmr_command_cycle_reports_rejection_completion_and_cancel(tmp_path):
    (tmp_path / "py_client_common.py").write_text(
        "class ClientConfig: pass\ndef init_client(c): return True\ndef close_client(): pass\n")
    (tmp_path / "localization.py").write_text(
        "from types import SimpleNamespace\ndef has_location_dev(): return True\n"
        "def loc_state(cb): cb(SimpleNamespace(status=0,confidence=1,x=0,y=0,theta=0,timestamp_ns=123))\n")
    (tmp_path / "map_reader.py").write_text(
        "def has_map_reader(): return True\ndef get_work_points(): return []\ndef get_charge_points(): return []\n")
    (tmp_path / "point_cloud.py").write_text("def has_point_cloud_dev(): return False\n")
    (tmp_path / "map_manager.py").write_text(
        "class MapInfo: pass\nactive='lab'\nmaps=['lab']\n"
        "def has_map_manager(): return True\ndef get_current_map_name(): return active\n"
        "def get_map_list(): return maps\n"
        "def start_mapping(info): maps.append(info.name); return ''\ndef stop_mapping(): return ''\n"
        "def async_change_map(name):\n import asyncio\n from types import SimpleNamespace as NS\n"
        " global active\n active=name\n f=asyncio.get_running_loop().create_future()\n"
        " f.set_result(NS(err_code=0,data='',err_message='')); return NS(future=f)\n")
    (tmp_path / "nav.py").write_text(
        "import asyncio\nfrom types import SimpleNamespace as NS\n"
        "class NavigateParams: pass\nclass Pose: pass\n"
        "def async_NavigateTo(params):\n loop=asyncio.get_running_loop(); f=loop.create_future()\n"
        " if params.target_position.x==1: loop.call_soon(f.set_result, NS(err_code=0,err_message=''))\n"
        " return NS(id=4,future=f,cancel=lambda: f.set_result(NS(err_code=12,err_message='cancelled')))\n")
    sensor = HeliosSpatialMap({"spatial_db": str(tmp_path / "tags.db")})
    sensor._python, sensor._sdk = Path(sys.executable), tmp_path
    try:
        sensor._launch()
        wait_for(lambda: sensor.payload()["state"] == "online")
        with pytest.raises(ValueError, match="already in use"):
            sensor.command("start_mapping", map_name="lab")
        assert sensor.command("start_mapping", map_name="lab2")["state"] == "mapping"
        assert sensor.command("stop_mapping", map_name="lab2")["state"] == "mapping_stopped"
        assert sensor.command("list_maps")["maps"] == ["lab", "lab2"]
        for x in (1, 2):
            assert sensor.command("navigate", map_name="lab", x=x, y=0, yaw=0)["state"] == "navigating"
            if x == 1:
                wait_for(lambda: sensor.command("nav_status")["state"] == "completed")
            else:
                assert sensor.command("stop_nav")["state"] == "cancel_requested"
                wait_for(lambda: sensor.command("nav_status")["message"] == "cancelled")
        assert sensor.command("load_map", map_name="lab2")["state"] == "map_loaded"
    finally:
        sensor.stop()
