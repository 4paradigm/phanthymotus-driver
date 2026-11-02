"""Receive -> shared sink -> real C++ worker, linked to a fake chassis only."""
import importlib.util
import json
from pathlib import Path
import time
import types

from test_tianyi_loco_servo import setup
from test_tianyi_sdk_worker import native_worker

ROOT=Path(__file__).resolve().parents[1]


def test_direct_callback_and_staged_stops_end_to_end(setup,native_worker,monkeypatch):
    from common.control import ControlSink
    spec=importlib.util.spec_from_file_location("direct_sdk_backend",
        ROOT/"x-humanoid/tianyi2.0/slamtec_velocity.py")
    backend=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)
    monkeypatch.setattr(setup.module,"time",time)
    monkeypatch.setattr(setup.module,"ControlSink",ControlSink)
    monkeypatch.setattr(setup.module,"SlamtecVelocityBackend",backend.SlamtecVelocityBackend)
    monkeypatch.setattr(setup.module,"StaleVelocityCommand",backend.StaleVelocityCommand)
    monkeypatch.setattr(setup.module,"CommandLeaseExpired",backend.CommandLeaseExpired)
    plugin=setup.create(sdk_binary=str(native_worker),sdk_host="ok")
    assert plugin.dispatch("start",{})["state"] == "running"
    time.sleep(0.005)
    def send(seq,values=None,**extra):
        stamp=int(time.time()*1000)
        message={"schema":"motus.control/1","mode":"twist","dof":6,
            "source":"navi","priority":0,"seq":seq,"ttl_ms":200,
            "stamp_ms":stamp,"obs_stamp_ms":stamp,
            "values":[0.2,0,0,0,0,-0.3] if values is None else values,**extra}
        plugin._on_message(types.SimpleNamespace(data=json.dumps(message)))
    send(1)
    info=plugin._info()
    assert info["counts"]["accepted"] == 1
    assert info["dispatch"]["mode"] == "direct_callback"
    assert info["backend"]["action_id"] == 1
    time.sleep(0.11)
    send(2)
    assert plugin._info()["backend"]["native"]["stats"]["action_starts"] == 1
    # No ROS tick: native code independently zeros velocity and retains action.
    time.sleep(0.35)
    plugin._tick()
    held=plugin._info()
    assert held["output"] == [0]*6
    assert held["backend"]["action_id"] == 1
    assert held["backend"]["native"]["stats"]["soft_stops"] == 1
    assert held["backend"]["native"]["stats"]["heartbeat_refreshes"] >= 1
    send(3)
    resumed=plugin._info()["backend"]
    assert resumed["action_id"] == 1
    assert resumed["native"]["stats"]["action_starts"] == 1
    assert resumed["native"]["stats"]["confirmed_stops"] == 0
    # Explicit zero is immediate, even directly after a nonzero callback.
    send(4,[0]*6)
    stopped=plugin._info()["backend"]
    assert stopped["action_id"] == 0
    assert stopped["native"]["stats"]["confirmed_stops"] == 1
