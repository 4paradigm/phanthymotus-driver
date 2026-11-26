"""Contracts for the two SDK station classes and the Helios sensor lifecycle."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from common.vendor_runtime import DriverBundle
from rokae.helios import spatial_map_worker as worker
from rokae.helios.spatial_map import HeliosSpatialMap


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
