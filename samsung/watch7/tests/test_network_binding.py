"""MCP should be loopback-only by default; bridge test exposure is opt-in."""
import pytest

from main import resolve_mcp_bind_host


def test_mcp_defaults_to_loopback(monkeypatch):
    monkeypatch.delenv("WATCH7_MCP_BIND_HOST", raising=False)
    monkeypatch.delenv("WATCH7_PRIVATE_DOCKER_TEST", raising=False)
    assert resolve_mcp_bind_host() == "127.0.0.1"


def test_mcp_bridge_binding_is_explicitly_opted_in(monkeypatch):
    monkeypatch.setenv("WATCH7_MCP_BIND_HOST", "0.0.0.0")
    monkeypatch.delenv("WATCH7_PRIVATE_DOCKER_TEST", raising=False)
    with pytest.raises(RuntimeError, match="PRIVATE_DOCKER_TEST"):
        resolve_mcp_bind_host()
    monkeypatch.setenv("WATCH7_PRIVATE_DOCKER_TEST", "1")
    assert resolve_mcp_bind_host() == "0.0.0.0"


def test_mcp_cannot_bind_arbitrary_lan_ip(monkeypatch):
    monkeypatch.setenv("WATCH7_PRIVATE_DOCKER_TEST", "1")
    monkeypatch.setenv("WATCH7_MCP_BIND_HOST", "192.168.1.88")
    with pytest.raises(RuntimeError):
        resolve_mcp_bind_host()
