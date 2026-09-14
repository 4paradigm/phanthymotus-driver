"""Focused, ROS-free tests for persistent vision capture."""

from __future__ import annotations

import io
from pathlib import Path
import tempfile
import threading
import time
import unittest

import vision_capture


class _Worker:
    _running = True

    def __init__(self):
        self.sequence = 0
        self.after_sequences = []

    def wait_for_frame(self, kind, after_sequence=None, timeout_s=5.0):
        self.assert_kind(kind)
        self.after_sequences.append(after_sequence)
        if after_sequence is not None:
            assert after_sequence == self.sequence
        self.sequence += 1
        return {"data": f"frame-{self.sequence}".encode(), "timestamp_ms": 0}, self.sequence

    @staticmethod
    def assert_kind(kind):
        assert kind == "rgb"


class _Client:
    def __init__(self, worker):
        self.camera_worker = worker


class _Process:
    def __init__(self, command, written):
        self.command = command
        self.stdin = self
        self.stderr = io.BytesIO()
        self._written = written
        self._closed = False
        self._returncode = None

    @property
    def closed(self):
        return self._closed

    def write(self, data):
        self._written.append(data)

    def close(self):
        self._closed = True

    def poll(self):
        return self._returncode

    def wait(self, timeout=None):
        del timeout
        Path(self.command[-1]).touch()
        self._returncode = 0
        return 0

    def kill(self):
        self._returncode = -9


class VisionCaptureTests(unittest.TestCase):
    def test_recording_waits_for_new_worker_sequences_and_encodes_distinct_frames(self):
        worker = _Worker()
        written = []
        original_popen = vision_capture.subprocess.Popen
        vision_capture.subprocess.Popen = lambda command, **_: _Process(command, written)
        try:
            with tempfile.TemporaryDirectory() as directory:
                plugin = vision_capture.Plugin({"output_dir": directory, "fps": 15}, "", None, _Client(worker))
                result = plugin._record_video(1, threading.Event())
        finally:
            vision_capture.subprocess.Popen = original_popen
        self.assertTrue(result["ok"])
        self.assertGreater(result["frames"], 2)
        self.assertEqual(len(written), result["frames"])
        self.assertEqual(len(set(written)), len(written))
        self.assertTrue(all(sequence is not None for sequence in worker.after_sequences[1:]))

    def test_stop_cancels_async_recording_and_notifies_acp_once(self):
        worker = _Worker()
        plugin = vision_capture.Plugin({}, "", None, _Client(worker))
        started = threading.Event()
        notifications = []
        original_notify = vision_capture._acp_notify

        def fake_record(requested, cancel_event, active=None):
            del requested, active
            started.set()
            self.assertTrue(cancel_event.wait(1))
            return plugin._cancelled_result()

        plugin._record_video = fake_record
        vision_capture._acp_notify = lambda *args: notifications.append(args)
        try:
            queued = plugin.dispatch("record_video", {"duration_s": 5})
            self.assertTrue(started.wait(1))
            stopped = plugin.stop()
            self.assertEqual(stopped["state"], "idle")
            self.assertEqual(stopped["action_id"], queued["action_id"])
            for _ in range(100):
                if notifications:
                    break
                time.sleep(0.01)
        finally:
            vision_capture._acp_notify = original_notify

        self.assertEqual(len(notifications), 1)
        self.assertEqual(notifications[0][0], queued["action_id"])
        self.assertEqual(notifications[0][1], "cancelled")
        self.assertEqual(plugin.dispatch("info", {})["active_recording"], None)


if __name__ == "__main__":
    unittest.main()
