"""「停止智能控制」必须真的停下机器人。

这张卡没有生命周期状态 —— 它的真实动作是 `start_mapping` / `navigate_to_*` 那些，
所以 `start` / `stop` / `info` 一直是三个写死的返回值。对 `start` 和 `info` 那是
无害的（画布只要它别显示成坏的）；**对 `stop` 不是**。

agent-core 的「停止智能控制」就是给画布上每张卡片下发 `action: stop`。而这里收下
之后返回 `{"state": "idle"}` 然后什么都不做 —— 于是导航在飞的时候按停止，机器人
继续走到目标点，界面显示已经停了。

天轶实测 2026-09-22 发现：停止之后 `controlled_spatial` 仍报 running（那只是常量），
而底盘那次恰好没有在飞的动作，所以没出事。

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_tianyi_controlled_spatial_stop.py -q
"""

from __future__ import annotations

import importlib.util
import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "x-humanoid" / "tianyi2.0"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(BUNDLE))

spec = importlib.util.spec_from_file_location(
    "tianyi_controlled_spatial", BUNDLE / "controlled_spatial.py")
mod = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(mod)
except Exception as error:      # pragma: no cover - 环境问题，不是逻辑
    pytest.skip(f"天轶 bundle 导入失败：{error}", allow_module_level=True)


class _FakeSlamtec:
    def __init__(self, raises=False):
        self.cancels = 0
        self._raises = raises

    def cancel_current_action(self):
        self.cancels += 1
        if self._raises:
            raise RuntimeError("底盘没有应答")
        return {"ok": True}


def _card(nav_active: bool, raises: bool = False):
    """一张只装了这条用例需要的那几个字段的卡片。

    不跑 `__init__`：那会去连底盘。这里测的是 `dispatch` 的一个分支，而它只碰
    `_slamtec` 与三个导航状态字段。
    """
    card = mod.ControlledSpatialPlugin.__new__(mod.ControlledSpatialPlugin)
    card._slamtec = _FakeSlamtec(raises)
    card._nav_active = nav_active
    card._nav_arrived = threading.Event()
    card._nav_action_id = 4321 if nav_active else None
    card._protected_actions = ()
    return card


def test_stop_cancels_an_in_flight_navigation():
    """**这条是整个文件的理由。** 此前 stop 只返回一个常量，机器人继续走。"""
    card = _card(nav_active=True)
    card._nav_arrived.set()

    result = card.dispatch("stop", {})

    assert card._slamtec.cancels == 1, "必须真的调用底盘的取消"
    assert result["state"] == "idle"
    assert result.get("cancelled_nav") is True, "取消了就要说出来"
    # 状态要一起清干净：留着 _nav_active=True 会让下一次导航读到上一次的残留。
    assert card._nav_active is False
    assert card._nav_action_id is None
    assert not card._nav_arrived.is_set()


def test_stop_does_not_touch_the_chassis_when_nothing_is_navigating():
    """停止是每次「停止智能控制」都会发的，不该对一张闲着的卡片乱发底盘请求。"""
    card = _card(nav_active=False)

    result = card.dispatch("stop", {})

    assert card._slamtec.cancels == 0
    assert result == {"state": "idle"}


def test_a_failed_cancel_is_reported_instead_of_swallowed():
    """吞掉异常就退回成原来那个"永远成功"，而机器人还在走。"""
    card = _card(nav_active=True, raises=True)

    result = card.dispatch("stop", {})

    assert result["state"] == "error"
    assert "仍在移动" in result["message"]
