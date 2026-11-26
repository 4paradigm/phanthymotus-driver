"""A3 snapshot card: latest-frame snapshot from the mirror cache, JPEG output."""

import importlib
import os
import time
from types import SimpleNamespace

import pytest


@pytest.fixture
def snapshot():
    return importlib.import_module("agibot.A3.vision_capture")


JPEG = b"\xff\xd8fakejpegpayload\xff\xd9"


def make_card(snapshot, tmp_path, frame=JPEG, frame_age=0.5):
    nodes = SimpleNamespace(latest_frames={})
    if frame is not None:
        nodes.latest_frames[snapshot.FRAME_KEY] = (frame, time.monotonic() - frame_age)
    card = snapshot.VisionCapturePlugin(nodes, {"output_dir": str(tmp_path / "cap")})
    return card, nodes


def test_capture_writes_jpeg_and_reports_paths(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path)
    result = card.dispatch("capture_photo", {})
    assert result["ok"] is True
    assert result["state"] == "captured"
    assert result["filename"].startswith("chest_front_d457_rgb_")
    assert result["filename"].endswith(".jpg")
    assert result["mime"] == "image/jpeg"
    assert result["size"] == len(JPEG)
    assert 0 <= result["frame_age_s"] <= 1.0
    path = tmp_path / "cap" / "photos" / result["filename"]
    assert path.read_bytes() == JPEG
    # 无点号开头的临时文件残留
    assert not any(p.name.startswith(".") for p in (tmp_path / "cap" / "photos").iterdir())


def test_stale_cache_is_unavailable_instead_of_saving_old_frame(snapshot, tmp_path):
    card, nodes = make_card(snapshot, tmp_path,
                            frame_age=snapshot.DEFAULT_MAX_FRAME_AGE_S + 5)
    result = card.dispatch("capture_photo", {})
    assert result["ok"] is False
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert "过期" in result["message"]
    # 旧帧不能落盘：调用方会把存档当“刚拍的”
    assert not (tmp_path / "cap" / "photos").exists()
    info = card.dispatch("info", {})
    assert info["frame_available"] is True   # 缓存里确实有帧
    assert info["frame_fresh"] is False      # 但它已经不是“现在”
    assert info["max_frame_age_s"] == snapshot.DEFAULT_MAX_FRAME_AGE_S
    # 流恢复（新的单调时间戳）后拍照恢复
    nodes.latest_frames[snapshot.FRAME_KEY] = (JPEG, time.monotonic())
    assert card.dispatch("capture_photo", {})["ok"] is True


def test_frame_with_unknown_timestamp_is_unavailable(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path)
    card.nodes.latest_frames[snapshot.FRAME_KEY] = (JPEG, None)
    result = card.dispatch("capture_photo", {})
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert "过期" in result["message"]


def test_max_frame_age_is_configurable(snapshot, tmp_path):
    nodes = SimpleNamespace(latest_frames={
        snapshot.FRAME_KEY: (JPEG, time.monotonic() - 0.5)})
    strict = snapshot.VisionCapturePlugin(
        nodes, {"output_dir": str(tmp_path / "strict"), "max_frame_age_s": 0.1})
    assert strict.dispatch("capture_photo", {})["code"] == "CAMERA_UNAVAILABLE"
    default = snapshot.VisionCapturePlugin(
        nodes, {"output_dir": str(tmp_path / "default")})
    assert default.dispatch("capture_photo", {})["ok"] is True


def test_capture_without_frame_is_unavailable_not_hanging(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path, frame=None)
    result = card.dispatch("capture_photo", {})
    assert result["ok"] is False
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert result["state"] == "error"


def test_capture_rejects_truncated_jpeg(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path, frame=b"\xff\xd8half-only")
    result = card.dispatch("capture_photo", {})
    assert result["code"] == "CAMERA_UNAVAILABLE"


def test_capture_custom_name_rules(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path)
    ok = card.dispatch("capture_photo", {"image_name": "demo_1"})
    assert ok["ok"] is True
    assert ok["filename"] == "demo_1.jpg"
    # 同名再拍：拒绝且不覆盖
    dup = card.dispatch("capture_photo", {"image_name": "demo_1"})
    assert dup["code"] == "FILE_EXISTS"
    assert (tmp_path / "cap" / "photos" / "demo_1.jpg").read_bytes() == JPEG
    # 非法名字（路径分隔符/扩展名/非 ASCII）；空串=留空自动命名，不算非法
    for bad in ("../evil", "a/b", "with.jpg", "有中文", ".hidden"):
        result = card.dispatch("capture_photo", {"image_name": bad})
        assert result["code"] == "INVALID_ARGUMENT", bad
    auto = card.dispatch("capture_photo", {"image_name": ""})
    assert auto["ok"] is True and auto["filename"] != ""


def test_list_orders_newest_first_and_skips_tempfiles(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path)
    first = card.dispatch("capture_photo", {"image_name": "older"})
    time.sleep(0.02)
    second = card.dispatch("capture_photo", {})
    # 一个未发布的临时文件不应出现在 list 里
    (tmp_path / "cap" / "photos" / ".draft.tmp").write_bytes(b"x")
    listed = card.dispatch("list", {})
    names = [f["filename"] for f in listed["files"]]
    assert names == [second["filename"], "older.jpg"]
    assert listed["files"][0]["channel_reply_path"].endswith(second["filename"])


def test_delete_lifecycle(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path)
    saved = card.dispatch("capture_photo", {"image_name": "gone"})
    assert card.dispatch("delete", {"name": saved["filename"]})["ok"] is True
    assert card.dispatch("delete", {"name": saved["filename"]})["code"] == "NOT_FOUND"
    assert card.dispatch("delete", {"name": "no-such.jpg"})["code"] == "NOT_FOUND"
    assert card.dispatch("delete", {"name": "not-a-jpeg.png"})["code"] == "INVALID_ARGUMENT"
    assert card.dispatch("delete", {"name": None})["code"] == "INVALID_ARGUMENT"


def test_info_reports_frame_and_last_capture(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path)
    info = card.dispatch("info", {})
    assert info["frame_available"] is True
    assert info["frame_fresh"] is True
    assert info["stream"] == "chest_front_d457_rgb"
    assert info["last_capture"] is None
    card.dispatch("capture_photo", {})
    info = card.dispatch("info", {})
    assert info["last_capture"]["status"] == "captured"


def test_unknown_action_returns_none(snapshot, tmp_path):
    card, _ = make_card(snapshot, tmp_path)
    assert card.dispatch("record_video", {}) is None
    assert card.dispatch("start", {}) is not None  # 生命周期动作保留


def test_tool_contract(snapshot):
    card, _ = make_card(snapshot, __import__("pathlib").Path("/tmp/unused-cap"))
    tool = card.get_tool()
    assert tool["name"] == "vision_capture"
    assert tool["type"] == "actuator"  # 与 go1/g1 的 vision_capture 分类一致
    actions = tool["inputSchema"]["properties"]["action"]["enum"]
    assert actions == ["capture_photo", "list", "delete", "info"]
    assert "record_video" not in actions  # 一期不含录像
    assert "position" not in tool["inputSchema"]["properties"]  # 一期单机位
    # 拍照同步返回路径，不申报 x-completion（那是录像等长动作的 ACP 等待表）
    assert "x-completion" not in tool["inputSchema"]


def test_channel_reply_path_follows_env(snapshot, tmp_path, monkeypatch):
    channel_root = tmp_path / "channel"
    monkeypatch.setenv("PHANTHY_CHANNEL_OUTPUT_DIR", str(channel_root))
    card, _ = make_card(snapshot, tmp_path)
    result = card.dispatch("capture_photo", {"image_name": "chan"})
    assert result["channel_reply_path"] == str(channel_root / "photos" / "chan.jpg")
    monkeypatch.delenv("PHANTHY_CHANNEL_OUTPUT_DIR")
    result = card.dispatch("capture_photo", {"image_name": "chan2"})
    # 未配置通道目录时回退 /work 默认值——路径仍是相对存档目录的结构
    assert result["channel_reply_path"].endswith(os.path.join("photos", "chan2.jpg"))
