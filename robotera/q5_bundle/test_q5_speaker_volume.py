"""Offline checks for Q5 speaker startup volume without ROS/vendor imports."""

from __future__ import annotations

import ast
import audioop
import struct
import subprocess
import threading
import types
from pathlib import Path
from unittest.mock import patch


SOURCE = Path(__file__).with_name("legacy_device.py")


def _speaker_methods(*names, **bindings):
    tree = ast.parse(SOURCE.read_text())
    speaker = next(node for node in tree.body
                   if isinstance(node, ast.ClassDef) and node.name == "SpeakerPlugin")
    methods = [node for node in speaker.body
               if isinstance(node, ast.FunctionDef) and node.name in names]
    assert len(methods) == len(names)
    isolated = ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
        name="SpeakerPlugin", bases=[], keywords=[], body=methods,
        decorator_list=[])], type_ignores=[]))
    namespace = {
        "__file__": str(SOURCE), "audioop": audioop, "Path": Path,
        "_Q5_XOS_CHAT_LOCK": threading.RLock(), **bindings,
    }
    exec(compile(isolated, str(SOURCE), "exec"), namespace)
    return namespace["SpeakerPlugin"]


def test_speaker_sets_system_volume_before_startup_check():
    speaker_type = _speaker_methods("_start_for_topic")
    speaker = speaker_type()
    calls = []
    speaker._volume = 2
    speaker._running = False
    speaker._topic = ""
    speaker.stop = lambda: calls.append("stop")
    speaker._prepare_xos_route = lambda: calls.append("route")
    speaker._set_system_volume = lambda value: calls.append(("volume", value))
    speaker._play_startup_sound = lambda: calls.append("self_check")
    speaker._start_playback = lambda topic: calls.append(("playback", topic))

    speaker._start_for_topic("/tts/audio")
    assert calls == ["stop", "route", ("volume", 2), "self_check",
                     ("playback", "/tts/audio")]

    speaker._running = True
    speaker._topic = "/tts/audio"
    speaker._start_for_topic("/tts/audio")
    assert len(calls) == 5


def test_speaker_scales_startup_pcm_like_live_pcm():
    written = bytearray()

    class Sink:
        def write(self, data):
            written.extend(data)

        def flush(self):
            pass

        def close(self):
            pass

    class Process:
        stdin = Sink()
        stderr = None

        def wait(self, timeout):
            assert timeout == 3

    speaker_type = _speaker_methods(
        "_play_startup_sound", "_scale_pcm",
        subprocess=types.SimpleNamespace(
            Popen=lambda *args, **kwargs: Process(), PIPE=subprocess.PIPE,
            DEVNULL=subprocess.DEVNULL),
        time=types.SimpleNamespace(sleep=lambda seconds: None),
        _q5_alsa_speaker_command=lambda *args: "alsa",
        _q5_speaker_playback_shell=lambda command: command,
        _q5_ssh_args=lambda command: [command],
        _raise_if_remote_process_exited=lambda proc, label: None,
    )
    speaker = speaker_type()
    speaker._volume = 2
    speaker._input_gain = 6.0
    speaker._device = "hw:2,0"
    speaker._output_rate = 44100
    speaker._output_channels = 2
    original = struct.pack("<h", 10000) * 20

    with patch.object(Path, "read_bytes", return_value=original):
        speaker._play_startup_sound()

    assert bytes(written) == audioop.mul(original, 2, 0.12)
    assert written != original
