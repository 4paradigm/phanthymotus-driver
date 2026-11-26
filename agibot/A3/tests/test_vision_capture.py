"""A3 snapshot card: reads LatestFrameCache, writes JPEG, one result envelope."""

import importlib
import os
import time
from types import SimpleNamespace

import pytest

JPEG = b"\xff\xd8fakejpegpayload\xff\xd9"


class Clock:
    """Injectable monotonic clock so frame age is deterministic in tests."""

    def __init__(self, now=1000.0):
        self.now = now

    def __call__(self):
        return self.now


@pytest.fixture
def snapshot():
    return importlib.import_module("agibot.A3.vision_capture")


@pytest.fixture
def latest_frames():
    return importlib.import_module("agibot.A3.latest_frames")


def make_card(snapshot, latest_frames, tmp_path, frame=JPEG, frame_age=0.5, **config):
    clock = Clock()
    cache = latest_frames.LatestFrameCache(clock=clock)
    nodes = SimpleNamespace(latest_frames=cache)
    card = snapshot.VisionCapturePlugin(
        nodes, {"output_dir": str(tmp_path / "cap"), **config})
    if frame is not None:
        assert cache.put(snapshot.FRAME_KEY, frame)
        clock.now += frame_age
    return card, cache, clock


def test_capture_writes_jpeg_and_reports_paths(snapshot, latest_frames, tmp_path):
    card, _, _ = make_card(snapshot, latest_frames, tmp_path)
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


def test_card_registers_its_stream_and_needs_the_cache(snapshot, latest_frames, tmp_path):
    card, cache, _ = make_card(snapshot, latest_frames, tmp_path)
    assert cache.is_registered(snapshot.FRAME_KEY)
    # 换一个没人登记的 key 不会进缓存：缓存只复制被消费的流。
    assert cache.put("camera_head_left_fisheye", JPEG) is False
    # 没有帧缓存属于接线错误，构建时就该炸，而不是运行期空转。
    with pytest.raises(AttributeError):
        snapshot.VisionCapturePlugin(SimpleNamespace(), {"output_dir": str(tmp_path)})


def test_stale_cache_is_unavailable_instead_of_saving_old_frame(snapshot, latest_frames, tmp_path):
    card, cache, clock = make_card(
        snapshot, latest_frames, tmp_path,
        frame_age=snapshot.DEFAULT_MAX_FRAME_AGE_S + 5)
    result = card.dispatch("capture_photo", {})
    assert result["ok"] is False
    assert result["state"] == "error"
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert "过期" in result["message"]
    # 旧帧不能落盘：调用方会把存档当“刚拍的”
    assert not (tmp_path / "cap" / "photos").exists()
    info = card.dispatch("info", {})
    assert info["ok"] is True
    assert info["frame_available"] is True   # 缓存里确实有帧
    assert info["frame_fresh"] is False      # 但它已经不是“现在”
    assert info["max_frame_age_s"] == snapshot.DEFAULT_MAX_FRAME_AGE_S
    # 流恢复（新的单调时间戳）后拍照恢复
    assert cache.put(snapshot.FRAME_KEY, JPEG)
    clock.now += 0.01
    assert card.dispatch("capture_photo", {})["ok"] is True


def test_frame_from_the_future_is_unavailable(snapshot, latest_frames, tmp_path):
    card, _, clock = make_card(snapshot, latest_frames, tmp_path, frame_age=0.0)
    clock.now -= 5.0  # 时钟回退：时间戳比“现在”还晚
    result = card.dispatch("capture_photo", {})
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert "时间戳异常" in result["message"]


def test_max_frame_age_is_configurable(snapshot, latest_frames, tmp_path):
    strict, _, _ = make_card(snapshot, latest_frames, tmp_path,
                             frame_age=0.5, max_frame_age_s=0.1)
    assert strict.dispatch("capture_photo", {})["code"] == "CAMERA_UNAVAILABLE"
    default, _, _ = make_card(snapshot, latest_frames, tmp_path / "default",
                              frame_age=0.5)
    assert default.dispatch("capture_photo", {})["ok"] is True


def test_bad_jpeg_never_enters_the_cache(snapshot, latest_frames, tmp_path):
    card, cache, _ = make_card(snapshot, latest_frames, tmp_path, frame=None)
    # 准入在缓存：半张 JPEG 存不进去，卡片看到的是“暂无可用帧”。
    assert cache.put(snapshot.FRAME_KEY, b"\xff\xd8half-only") is False
    assert cache.age(snapshot.FRAME_KEY) is None
    result = card.dispatch("capture_photo", {})
    assert result["code"] == "CAMERA_UNAVAILABLE"


def test_capture_without_frame_is_unavailable_not_hanging(snapshot, latest_frames, tmp_path):
    card, _, _ = make_card(snapshot, latest_frames, tmp_path, frame=None)
    result = card.dispatch("capture_photo", {})
    assert result["ok"] is False
    assert result["code"] == "CAMERA_UNAVAILABLE"
    assert result["state"] == "error"


def test_capture_custom_name_rules(snapshot, latest_frames, tmp_path):
    card, _, _ = make_card(snapshot, latest_frames, tmp_path)
    ok = card.dispatch("capture_photo", {"image_name": "demo_1"})
    assert ok["ok"] is True
    assert ok["filename"] == "demo_1.jpg"
    # 同名再拍：拒绝且不覆盖
    dup = card.dispatch("capture_photo", {"image_name": "demo_1"})
    assert dup["code"] == "FILE_EXISTS"
    assert dup["state"] == "error"
    assert (tmp_path / "cap" / "photos" / "demo_1.jpg").read_bytes() == JPEG
    # 非法名字（路径分隔符/扩展名/非 ASCII）；空串=留空自动命名，不算非法
    for bad in ("../evil", "a/b", "with.jpg", "有中文", ".hidden"):
        result = card.dispatch("capture_photo", {"image_name": bad})
        assert result["code"] == "INVALID_ARGUMENT", bad
        assert result["state"] == "error", bad
    auto = card.dispatch("capture_photo", {"image_name": ""})
    assert auto["ok"] is True and auto["filename"] != ""


def test_list_orders_newest_first_and_skips_tempfiles(snapshot, latest_frames, tmp_path):
    card, _, _ = make_card(snapshot, latest_frames, tmp_path)
    first = card.dispatch("capture_photo", {"image_name": "older"})
    time.sleep(0.02)
    second = card.dispatch("capture_photo", {})
    # 一个未发布的临时文件不应出现在 list 里
    (tmp_path / "cap" / "photos" / ".draft.tmp").write_bytes(b"x")
    listed = card.dispatch("list", {})
    assert listed["ok"] is True
    names = [f["filename"] for f in listed["files"]]
    assert names == [second["filename"], "older.jpg"]
    assert listed["files"][0]["channel_reply_path"].endswith(second["filename"])


def test_delete_lifecycle(snapshot, latest_frames, tmp_path):
    card, _, _ = make_card(snapshot, latest_frames, tmp_path)
    saved = card.dispatch("capture_photo", {"image_name": "gone"})
    assert card.dispatch("delete", {"name": saved["filename"]})["ok"] is True
    assert card.dispatch("delete", {"name": saved["filename"]})["code"] == "NOT_FOUND"
    assert card.dispatch("delete", {"name": "no-such.jpg"})["code"] == "NOT_FOUND"
    assert card.dispatch("delete", {"name": "not-a-jpeg.png"})["code"] == "INVALID_ARGUMENT"
    assert card.dispatch("delete", {"name": None})["code"] == "INVALID_ARGUMENT"


def test_info_reports_frame_and_last_capture(snapshot, latest_frames, tmp_path):
    card, _, _ = make_card(snapshot, latest_frames, tmp_path)
    info = card.dispatch("info", {})
    assert info["ok"] is True
    assert info["frame_available"] is True
    assert info["frame_fresh"] is True
    assert info["stream"] == "chest_front_d457_rgb"
    assert info["last_capture"] is None
    card.dispatch("capture_photo", {})
    info = card.dispatch("info", {})
    assert info["last_capture"]["status"] == "captured"


def test_unknown_action_returns_none(snapshot, latest_frames, tmp_path):
    card, _, _ = make_card(snapshot, latest_frames, tmp_path)
    # MCP 层把 None 翻成 -32601 Unknown tool；卡内不要再造一个错误形状。
    assert card.dispatch("record_video", {}) is None
    assert card.dispatch("start", {}) is not None  # 生命周期动作保留


def test_tool_contract(snapshot, latest_frames):
    card, _, _ = make_card(snapshot, latest_frames, __import__("pathlib").Path("/tmp/unused-cap"))
    tool = card.get_tool()
    assert tool["name"] == "vision_capture"
    assert tool["type"] == "actuator"  # 与 go1/g1 的 vision_capture 分类一致
    actions = tool["inputSchema"]["properties"]["action"]["enum"]
    assert actions == ["capture_photo", "list", "delete", "info"]
    assert "record_video" not in actions  # 一期不含录像
    assert "position" not in tool["inputSchema"]["properties"]  # 一期单机位
    # 拍照同步返回路径，不申报 x-completion（那是录像等长动作的 ACP 等待表）
    assert "x-completion" not in tool["inputSchema"]


def test_channel_reply_path_follows_env(snapshot, latest_frames, tmp_path, monkeypatch):
    channel_root = tmp_path / "channel"
    monkeypatch.setenv("PHANTHY_CHANNEL_OUTPUT_DIR", str(channel_root))
    card, _, _ = make_card(snapshot, latest_frames, tmp_path)
    result = card.dispatch("capture_photo", {"image_name": "chan"})
    assert result["channel_reply_path"] == str(channel_root / "photos" / "chan.jpg")
    monkeypatch.delenv("PHANTHY_CHANNEL_OUTPUT_DIR")
    result = card.dispatch("capture_photo", {"image_name": "chan2"})
    # 未配置通道目录时回退 /work 默认值——路径仍是相对存档目录的结构
    assert result["channel_reply_path"].endswith(os.path.join("photos", "chan2.jpg"))
