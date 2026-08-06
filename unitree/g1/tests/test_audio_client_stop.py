import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


G1_DIR = Path(__file__).resolve().parents[1]
CLIENT_PATH = (
    G1_DIR / "unitree_sdk2py" / "g1" / "audio" / "g1_audio_client.py"
)


class _Client:
    def __init__(self, service_name, enable_lease):
        self.service_name = service_name
        self.enable_lease = enable_lease
        self.calls = []
        self.call_result = (0, "")

    def _Call(self, api_id, parameter):
        self.calls.append((api_id, parameter))
        return self.call_result


def _package(name):
    module = types.ModuleType(name)
    module.__path__ = []
    return module


def _load_audio_client():
    client_module = types.ModuleType("unitree_sdk2py.rpc.client")
    client_module.Client = _Client
    api_module = types.ModuleType(
        "unitree_sdk2py.g1.audio.g1_audio_api",
    )
    constants = {
        "AUDIO_SERVICE_NAME": "audio",
        "AUDIO_API_VERSION": "1.0",
        "ROBOT_API_ID_AUDIO_TTS": 1001,
        "ROBOT_API_ID_AUDIO_ASR": 1002,
        "ROBOT_API_ID_AUDIO_START_PLAY": 1003,
        "ROBOT_API_ID_AUDIO_STOP_PLAY": 1004,
        "ROBOT_API_ID_AUDIO_GET_VOLUME": 1005,
        "ROBOT_API_ID_AUDIO_SET_VOLUME": 1006,
        "ROBOT_API_ID_AUDIO_SET_RGB_LED": 1007,
    }
    for name, value in constants.items():
        setattr(api_module, name, value)

    stubs = {
        "unitree_sdk2py": _package("unitree_sdk2py"),
        "unitree_sdk2py.rpc": _package("unitree_sdk2py.rpc"),
        "unitree_sdk2py.rpc.client": client_module,
        "unitree_sdk2py.g1": _package("unitree_sdk2py.g1"),
        "unitree_sdk2py.g1.audio": _package("unitree_sdk2py.g1.audio"),
        "unitree_sdk2py.g1.audio.g1_audio_api": api_module,
    }
    name = "unitree_sdk2py.g1.audio.g1_audio_client"
    spec = importlib.util.spec_from_file_location(name, CLIENT_PATH)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


class AudioClientStopTests(unittest.TestCase):
    def test_play_stop_propagates_real_rpc_code(self):
        module = _load_audio_client()
        client = module.AudioClient()
        client.call_result = (37, "failure")

        result = client.PlayStop("g1_speaker")

        self.assertEqual(result, 37)
        self.assertEqual(client.calls[0][0], 1004)
        self.assertEqual(
            json.loads(client.calls[0][1]),
            {"app_name": "g1_speaker"},
        )


if __name__ == "__main__":
    unittest.main()
