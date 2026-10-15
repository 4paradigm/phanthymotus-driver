"""Real Core/MCP/browser workflow, synthetic G1 plant; no robot connection."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import importlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "unitree/g1/tests"))
from test_g1_teaching import g1_teaching, call, wait_for
from teaching_browser_support import workbench_site, browser_page, confirm_in_browser, capture, pytestmark


@pytest.fixture
def site(g1_teaching, monkeypatch, tmp_path):
    service, card, *_ = g1_teaching
    with workbench_site(service, [card, service], monkeypatch, tmp_path,
                        "test-g1", "宇树 G1（模拟 SDK）") as value:
        yield value


def test_g1_record_trial_save_and_agent_call(site, browser_page, g1_teaching, tmp_path):
    from playwright.sync_api import expect
    page, ui, errors = browser_page
    _, service, config = site
    _, card, arm, sample, channel, receipts = g1_teaching
    assert not channel.positions
    ui.locator('#check-btn').click()
    expect(ui.locator('#next-btn')).to_be_enabled()
    expect(ui.locator('#checks .check-result.ok')).to_have_count(4)
    capture(page, 'g1-01-check.png')
    ui.locator('#next-btn').click()
    expect(ui.locator('#begin-btn')).not_to_be_visible()
    expect(ui.locator('#record-btn')).to_be_enabled()
    ui.locator('#record-btn').click()
    expect(ui.locator('#frame-count')).not_to_have_text('0')
    capture(page, 'g1-02-record.png')
    page.wait_for_timeout(250)
    ui.locator('#finish-btn').click()
    expect(ui.locator('#review')).to_be_visible()
    expect(ui.locator('#start-hint')).to_contain_text('先低速移动')
    expect(ui.locator('#duration')).to_contain_text('10 关节')
    expect(ui.locator('#trial-btn')).to_be_enabled()
    expect(ui.locator('#plot')).not_to_be_visible()
    ui.locator('#motion-details summary').click()
    expect(ui.locator('#frame-values tr:has(td)')).to_have_count(10)
    expect(ui.locator('#frame-values')).not_to_contain_text('夹爪')
    expect(ui.locator('#start-values')).not_to_contain_text('0.05')
    ui.locator('#motion-details summary').click()
    capture(page, 'g1-03-review.png')
    ui.locator('#trial-btn').click()
    expect(ui.locator('#confirm-description')).to_contain_text('先低速移动')
    confirm_in_browser(ui)
    expect(ui.locator('#trial-status')).to_contain_text('试运行已完成', timeout=10000)
    assert len(receipts) == 1 and receipts[0][1] == 'completed'
    ui.locator('#effect').check()
    ui.locator('#to-save-btn').click()
    ui.locator('#skill-name').fill('G1 双臂演示')
    ui.locator('#skill-conditions').fill('原厂固定假手，物体位置固定，移向起点的路径畅通。')
    capture(page, 'g1-04-save.png')
    ui.locator('#save-btn').click()
    expect(ui.locator('#confirmation-result')).to_be_visible()
    capture(page, 'g1-05-saved.png')
    installed = config.ConfigDB().get('skills')['installed']
    assert len(installed) == 1 and installed[0]['active']
    assert installed[0]['requiredTools'] == ['g1_teaching']
    import mcp_client
    from event.llm import Event
    async def invoke_saved():
        import prompt
        skills = importlib.import_module('event.skills')
        skill = installed[0]
        assert skill['slug'] in prompt._env_static({})
        await skills.Tools().activate_skill(skill['slug'])
        assert skill['instruction'] in prompt._env_static({})
        target = config.main['services']['mcp'][0]
        await mcp_client._connect_one(target['id'], target['name'], target['url'], '')
        name = 'mcp__test-g1__g1_teaching__run'
        schemas = {s['name']: s for s in Event()._get_bound_tool_schemas()}
        assert set(schemas[name]['parameters']['required']) == {'asset_id', 'execute', 'preview_id', 'workspace_clear', 'take_control'}
        preview = json.loads(await mcp_client.call_tool('mcp__test-g1__g1_teaching__preview',
            {'asset_id': service.draft['id'], 'instance_id': 'teach'}))
        assert not preview['executed']
        result = json.loads(await mcp_client.call_tool(name,
            {'asset_id': service.draft['id'], 'execute': True, 'preview_id': preview['preview_id'],
             'workspace_clear': True, 'take_control': True, 'instance_id': 'teach'}))
        await asyncio.to_thread(wait_for, lambda: service.last_run is not None)
        assert service.last_run['action_id'] == result['action_id']
        assert service.last_run['status'] == 'completed'
        assert receipts[-1][0] == result['action_id'] and len(receipts) == 2
        skills._runtime_activated.discard(skill['slug'])
    try:
        with ThreadPoolExecutor(max_workers=1) as executor:
            executor.submit(lambda: asyncio.run(invoke_saved())).result(timeout=20)
    finally:
        mcp_client.registry.pop('test-g1', None)
    page.get_by_role('button', name='关闭工作台').click()
    page.locator('#btn-skills').click()
    expect(page.locator('#skill-installed-list')).to_contain_text('G1 双臂演示')
    page.locator('.skill-card-name').filter(has_text='G1 双臂演示').click()
    expect(page.locator('#skill-installed-list')).to_contain_text('G1_23')
    expect(page.locator('#skill-installed-list')).to_contain_text('固定手')
    page.locator('#skill-package-refresh').click()
    expect(page.locator('#skill-package-export-status')).to_contain_text('调用说明与型号信息已更新')
    import os
    if directory := os.environ.get('TEACHING_SCREENSHOT_DIR'):
        page.locator('#skill-overlay .modal').screenshot(path=str(Path(directory) / 'g1-07-skill-detail.png'))
    with page.expect_download() as pending_download:
        page.locator('#skill-package-export').click()
    action_file = tmp_path / 'g1.motus-skill'
    pending_download.value.save_as(action_file)
    assert action_file.stat().st_size > 0
    page.locator('#skill-package-open').click()
    page.locator('#skill-package-file').set_input_files(action_file)
    expect(page.locator('#skill-package-summary')).to_contain_text('G1_23')
    expect(page.locator('#skill-package-install')).to_be_enabled()
    if directory := os.environ.get('TEACHING_SCREENSHOT_DIR'):
        page.locator('#skill-package-overlay .modal').screenshot(path=str(Path(directory) / 'g1-06-import.png'))
    before = len(channel.positions)
    card.cfg['trajectory_device_id'] = 'second-compatible-g1'
    page.locator('#skill-package-install').click()
    expect(ui.locator('#trial-status')).to_contain_text('等待本机试运行')
    assert len(config.ConfigDB().get('skills')['installed']) == 1
    assert len(channel.positions) == before
    ui.locator('#inspect-btn').click()
    expect(ui.locator('#trial-btn')).to_be_enabled()
    ui.locator('#trial-btn').click()
    confirm_in_browser(ui)
    expect(ui.locator('#trial-status')).to_contain_text('试运行已完成', timeout=10000)
    ui.locator('#effect').check()
    ui.locator('#to-save-btn').click()
    expect(ui.locator('#skill-name')).to_have_value('G1 双臂演示')
    expect(ui.locator('#compatibility-label')).to_contain_text('G1_23')
    ui.locator('#skill-name').fill('本机导入的双臂演示')
    ui.locator('#save-btn').click()
    expect(ui.locator('#confirmation-result')).to_be_visible()
    assert len(config.ConfigDB().get('skills')['installed']) == 2
    assert not errors
