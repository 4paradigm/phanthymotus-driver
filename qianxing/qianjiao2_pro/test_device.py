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
    assert d.dispatch("control", {"action":"unlock"})["state"] == "armed"
    result = d.dispatch("control", {"action":"move", "forward":1, "yaw":-1, "duration":0.1})
    assert result["state"] == "queued"
    assert result["action_id"]
    d.stop()

def test_range_rejected():
    d = live(); d.dispatch("control", {"action":"unlock"})
    with pytest.raises(ValueError): d.move({"pitch":1.1, "duration":0.1})
    d.stop()

def test_status_packet_parser():
    import json, struct
    payload = json.dumps({"depth": 1.25, "temperature": 21}).encode()
    packet = bytes([3, 0, 0, 0]) + struct.pack("<I", len(payload)) + bytes([1, 0, 0, 0]) + payload
    assert QianjiaoDevice._parse_status_packet(packet)["depth"] == 1.25
    assert QianjiaoDevice._parse_status_packet(packet[:10]) is None
