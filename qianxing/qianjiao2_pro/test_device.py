import time
import pytest
from device import QianjiaoDevice

def live():
    d = QianjiaoDevice({"mock": True, "heartbeat_rate": 20})
    d.start(); time.sleep(.08)
    return d

def test_mapping_and_safety():
    d = live()
    assert d.status()["connected"]
    assert d.dispatch("rov_control", {"action":"arm"})["state"] == "armed"
    result = d.dispatch("rov_control", {"action":"move", "forward":1, "yaw":-1})
    assert result["channels"]["forward"] == 1900
    assert result["channels"]["yaw"] == 1100
    d.stop()

def test_range_rejected():
    d = live(); d.dispatch("rov_control", {"action":"arm"})
    with pytest.raises(ValueError): d.dispatch("rov_control", {"action":"move", "pitch":1.1})
    d.stop()
