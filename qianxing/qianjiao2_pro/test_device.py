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

def test_status_packet_parser():
    import json, struct
    payload = json.dumps({"depth": 1.25, "temperature": 21}).encode()
    packet = bytes([3, 0, 0, 0]) + struct.pack("<I", len(payload)) + bytes([1, 0, 0, 0]) + payload
    assert QianjiaoDevice._parse_status_packet(packet)["depth"] == 1.25
    assert QianjiaoDevice._parse_status_packet(packet[:10]) is None
