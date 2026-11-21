"""Check that stereo startup touches only the required vendor stream."""
import sys
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import stereo_camera as stereo


@pytest.mark.parametrize(
    "running,desired,live_flag,expected",
    [
        (True, True, 0, ["status", "restart"]),
        (True, True, 1, ["status"]),
        (False, False, 0, ["status", "start"]),
    ],
)
def test_stereo_enables_the_active_template_and_only_restarts_if_needed(
    tmp_path, running, desired, live_flag, expected,
):
    baseline = "[camera]\ncam_share_yuv = 0\n"
    mode = "[camera]\ncam_is_mono = 0\ncam_img_width = 2560\ncam_img_height = 960\n"
    (tmp_path / "Config1.ini").write_text(baseline)
    (tmp_path / "Config3.ini").write_text(mode)
    live = tmp_path / "config_send.ini"
    live.write_text(f"cam_share_yuv = {live_flag}\n")
    status = {"running": running, "desired": desired, "args": ["Config3.ini"]}

    with patch.object(stereo, "_VIDEO_ROOT", tmp_path), \
         patch.object(stereo, "_video_request", return_value=status) as request:
        stereo.ensure_vendor_stereo()

    assert [call.args[0] for call in request.call_args_list] == expected
    assert (tmp_path / "Config1.ini").read_text() == baseline
    template = (tmp_path / "Config3.ini").read_text()
    assert stereo._ini_value(template, "cam_share_yuv") == "1"
    assert live.read_text() == f"cam_share_yuv = {live_flag}\n"
