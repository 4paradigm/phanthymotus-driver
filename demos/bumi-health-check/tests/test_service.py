import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer
import pytest
from health_service.service import HealthService, configuration
from health_service.server import handler
from test_evaluator import samples, check, NOW

def populated():
    service = HealthService("test_bumi")
    for source, sample in samples().items():
        service.receive(source, json.dumps(sample["data"]))
    return service

def test_startup_and_stop_restart_require_new_messages():
    service = HealthService("test_bumi")
    assert service.dispatch("check")["status"] == "数据不足"
    service = populated()
    assert service.dispatch("check")["status"] == "正常"
    service.dispatch("stop")
    service.receive("battery", '{"soc":90}')
    assert not service.samples
    assert service.dispatch("check")["status"] == "数据不足"
    service.dispatch("start")
    assert service.dispatch("check")["status"] == "数据不足"

@pytest.mark.parametrize("source,age", [("battery",3.01),("imu",0.151),("joints",0.301),("motion_state",1.501)])
def test_each_source_stale(source, age):
    data = samples()
    data[source]["received_monotonic"] = NOW-age
    assert check(data)["checks"][source]["status"] == "数据不足"

@pytest.mark.parametrize("source,payload", [
    ("battery","invalid json"), ("battery",'{"soc":NaN}'),
    ("imu",'{"quaternion":[0,0,0,0]}'),
    ("imu",'{"quaternion":[1]}'),
    ("joints",'{"joints":"invalid"}'),
    ("joints",json.dumps({"joints":[{"idx":0}]*21})),
    ("motion_state",'{"fresh":false,"reason":"SDK read failed"}'),
    ("motion_state",'{"fresh":true,"workmode":{"code":2}}'),
])
def test_invalid_message_replaces_last_good_sample(source,payload):
    service = populated()
    service.receive(source,payload)
    report = service.dispatch("check")
    assert report["checks"][source]["status"] == "数据不足"

def test_receiver_failure_never_reports_normal():
    service = populated()
    service.transport_error = "spin failed"
    assert service.dispatch("check")["status"] == "数据不足"
    assert service.dispatch("check")["transport_error"] == "spin failed"

def test_intervals_are_configurable():
    data = samples()
    data["imu"]["received_monotonic"] = NOW-0.2
    assert check(data)["status"] == "数据不足"
    assert check(data,{"intervals":{"imu":0.1}})["status"] == "正常"

@pytest.mark.parametrize("namespace,config", [
    ("",{}),("a/b",{}),("robot",{"battery_min_soc":120}),
    ("robot",{"temperature_limits":{"mainboard":50}}),
    ("robot",{"intervals":{"imu":0}}),
    ("robot",{"intervals":{"typo":1}}),
])
def test_invalid_config_fails(namespace,config):
    with pytest.raises(ValueError):
        configuration(namespace,config)

def test_http_mcp_and_notifications():
    service = HealthService("test_bumi")
    http = ThreadingHTTPServer(("127.0.0.1",0), handler(service))
    thread = threading.Thread(target=http.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{http.server_port}/mcp"
    def call(method,params=None,notification=False):
        payload={"jsonrpc":"2.0","method":method,"params":params or {}}
        if not notification:
            payload["id"]=1
        request=urllib.request.Request(url,json.dumps(payload).encode(),
                                       {"Content-Type":"application/json"})
        with urllib.request.urlopen(request) as response:
            return response.status, response.read()
    try:
        assert json.loads(call("initialize")[1])["result"]["serverInfo"]["name"] == "bumi-health-check"
        assert call("notifications/initialized",notification=True) == (202,b"")
        tools=json.loads(call("tools/list")[1])["result"]["tools"]
        assert [t["name"] for t in tools] == ["health_check"]
        result=json.loads(call("tools/call",{"name":"health_check","arguments":{"action":"check"}})[1])
        assert json.loads(result["result"]["content"][0]["text"])["status"] == "数据不足"
        bad=json.loads(call("tools/call",{"name":"loco","arguments":{"action":"start"}})[1])
        assert "error" in bad
    finally:
        http.shutdown()
        http.server_close()
        thread.join()
