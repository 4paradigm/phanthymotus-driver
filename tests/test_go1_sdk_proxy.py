"""Offline SDK-worker initialization; no native SDK or robot commands."""
import builtins
import importlib.util
from pathlib import Path
from queue import Queue
from types import SimpleNamespace
from unittest.mock import Mock


GO1 = Path(__file__).resolve().parents[1] / "unitree" / "go1"


def test_worker_installs_atomic_logging_before_sdk_import(monkeypatch, capsys):
    spec = importlib.util.spec_from_file_location("go1_logsafe_proxy", GO1 / "sdk_proxy.py")
    proxy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(proxy)
    install = Mock()
    client = Mock(available=False)
    client.snapshot.return_value = {"fresh": False}
    client.diagnostics.return_value = {"source": "offline-test"}
    factory = Mock(return_value=client)
    import_module = builtins.__import__

    def offline_import(name, *args, **kwargs):
        if name == "common":
            return SimpleNamespace(logsafe=SimpleNamespace(install=install))
        if name == "go1_sdk_client":
            # Importing the real SDK could print immediately, before construction.
            install.assert_called_once_with(check_fd=False)
            print("offline SDK import after logsafe")
            return SimpleNamespace(Go1HighSdkClient=factory)
        return import_module(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", offline_import)
    commands, results = Queue(), Queue()
    commands.put({"cmd": "snapshot"})
    commands.put({"cmd": "diagnostics"})
    commands.put(None)
    proxy._sdk_worker(commands, results, "offline", "127.0.0.1", 0, 0)
    factory.assert_called_once_with(network_iface="offline", target_ip="127.0.0.1",
                                    target_port=0, local_port=0)
    assert results.get_nowait() == {"available": False}
    assert results.get_nowait() == {"result": {"fresh": False}}
    assert results.get_nowait() == {"result": {"source": "offline-test"}}
    assert results.empty()
    client.start.assert_called_once_with()
    client.stop.assert_called_once_with()
    client.move.assert_not_called()
    client.stop_move.assert_not_called()
    client.set_posture.assert_not_called()
    client.set_gait.assert_not_called()
    assert "offline SDK import after logsafe" in capsys.readouterr().out
