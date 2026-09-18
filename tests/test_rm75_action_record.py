import threading
import time
from pathlib import Path

from realman.rm75_6f_v.action_record import ActionRecord


class FakeClient:
    motion_enabled = True

    def __init__(self):
        self.motion_gate = threading.Lock()
        self.run_state = 1
        self.never_finishes = False

    def command(self, method, *args):
        if method == "rm_set_program_id_run":
            self.run_state = 1
        return 0

    def call(self, method, *args):
        if method == "rm_save_trajectory":
            Path(args[0]).write_text("trajectory\n", encoding="utf-8")
            return 1
        if method == "rm_get_program_run_state":
            if self.never_finishes:
                return {"run_state": 1}
            if self.run_state == 1:
                self.run_state = 0
                return {"run_state": 1}
            return {"run_state": 0}
        raise AssertionError(method)

    def upload_recording(self, *args, **kwargs):
        self.uploaded = (args, kwargs)


class FakePlugin:
    def __init__(self):
        self.client = FakeClient()
        self.completions = []

    def _acp_callback(self, *args):
        self.completions.append(args)


def test_record_save_and_replay(tmp_path):
    plugin = FakePlugin()
    card = ActionRecord(plugin, {"directory": str(tmp_path), "replay_timeout_seconds": 2}, [(-1, 1)] * 7)
    assert card.dispatch("record_start", {"name": "wave", "confirm_motion": True})["state"] == "recording"
    saved = card.dispatch("record_stop", {})
    assert saved["state"] == "saved"
    assert card.dispatch("list", {})["actions"]["wave"]["slot"] == 1
    card.dispatch("replay", {"name": "wave", "confirm_motion": True})
    time.sleep(2.1)
    assert plugin.completions and plugin.completions[0][1] == "completed"


def test_replay_timeout_reports_error(tmp_path):
    plugin = FakePlugin()
    plugin.client.never_finishes = True
    card = ActionRecord(plugin, {"directory": str(tmp_path), "replay_timeout_seconds": 1}, [(-1, 1)] * 7)
    card.timeout = 1
    (tmp_path / "wave.project.txt").write_text("project\n", encoding="utf-8")
    (tmp_path / "index.json").write_text('{"wave":{"name":"wave","slot":1}}', encoding="utf-8")
    card.dispatch("replay", {"name": "wave", "confirm_motion": True})
    time.sleep(2.2)
    assert plugin.completions and plugin.completions[0][1] == "error"
