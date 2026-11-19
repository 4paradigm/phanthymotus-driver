"""Managed environments are shared by reference, never by browser credentials."""
import json

from simulator.mujoco.plugin import SimulationPlugin
from simulator.mujoco.client import MotionClient


def test_managed_connection_is_private_durable_and_core_replay_cannot_undo_it(tmp_path, monkeypatch):
    calls = []
    def call(client, action, **args):
        calls.append((client.endpoint, action, args))
        if action == "connection":
            return {"endpoint": "http://installed:15732", "api_key": "installed-private"}
        if action == "status":
            return {"jobs": [{"id": "ready-a", "state": "succeeded", "action": "install"}]}
        return {"targets": [{"id": "gpu-server", "label": "GPU Server"}]}
    monkeypatch.setattr(MotionClient, "call", call)
    monkeypatch.setattr(MotionClient, "health", lambda _: {"state": "ready"})
    cfg = {"state_dir": str(tmp_path), "endpoint": "http://previous:15732", "api_key": "previous-private",
           "environment_endpoint": "http://manager:15734", "environment_token": "manager-private"}
    plugin = SimulationPlugin(cfg)
    result = plugin.dispatch("config", {"environment_job": "ready-a"})
    assert result["config_values"]["environment_job"] == "ready-a"
    assert result["config_values"]["endpoint"] == "http://installed:15732"
    assert plugin._binding_path.stat().st_mode & 0o777 == 0o600
    for private in ("installed-private", "previous-private", "manager-private"):
        assert private not in json.dumps(result)
    for _ in range(3):
        plugin.dispatch("config", {"environment_job": "ready-a", "endpoint": cfg["endpoint"], "api_key": cfg["api_key"]})
    assert sum(action == "connection" for _, action, _ in calls) == 1
    restored = SimulationPlugin(cfg)
    assert restored.dispatch("info", {})["config_values"]["environment_job"] == "ready-a"
    assert restored._client.endpoint == "http://installed:15732"
    restored.dispatch("config", {"endpoint": cfg["endpoint"], "api_key": cfg["api_key"]})
    assert restored._client.endpoint == "http://installed:15732"
    # An explicit manual switch clears the private reference, even with the same endpoint.
    restored.dispatch("config", {"environment_job": "", "endpoint": "http://installed:15732"})
    assert restored.dispatch("info", {})["config_values"]["environment_job"] == ""
    assert not restored._binding_path.exists()
    assert plugin.dispatch("env_targets", {})["targets"][0]["label"] == "GPU Server"
    assert plugin.dispatch("inspect", {"mode": "env_status"})["jobs"][0]["id"] == "ready-a"
    assert calls[-1] == ("http://manager:15734", "status", {})


def test_inspection_allowlist_cannot_install_or_fetch_private_connection(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(MotionClient, "call", lambda client, action, **args: calls.append((action, args)) or {})
    plugin = SimulationPlugin({"state_dir": str(tmp_path)})
    for mode in ("connection", "env_install", "simulation_claim", "simulation_run"):
        assert plugin.dispatch("inspect", {"mode": mode, "job_id": "x"})["state"] == "error"
    assert calls == []
    plugin.dispatch("inspect", {"mode": "env_status", "job_id": "job-a", "cursor": 17,
                                "command": "ignored", "api_key": "ignored", "endpoint": "http://ignored"})
    assert calls == [("status", {"job_id": "job-a", "cursor": 17})]
