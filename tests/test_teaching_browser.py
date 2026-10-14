"""Cross-repository browser acceptance using real APIs and a simulated PiPER.

Opt in with TEACHING_CORE_ROOT=<phanthymotus checkout>. Requires Playwright,
Chromium, uvicorn and the Core API dependencies. Never opens robot hardware.
"""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import socket
import sys
import threading
import time
from http.server import ThreadingHTTPServer

import pytest

from test_piper_dual import pair, ready, frame, wait_for
from test_piper_teaching import teaching, call, restore_simulated_start, METADATA
from common.vendor_runtime import DriverBundle, make_handler

CORE = os.environ.get('TEACHING_CORE_ROOT')
pytestmark = pytest.mark.skipif(not CORE, reason='set TEACHING_CORE_ROOT for cross-repository browser acceptance')


@pytest.fixture
def site(teaching, pair, monkeypatch, tmp_path):
    pytest.importorskip('playwright.sync_api')
    import uvicorn
    import fastapi
    from fastapi.staticfiles import StaticFiles
    sys.path.insert(0, str(Path(CORE) / 'agent-core/src'))
    monkeypatch.setenv('DB_PATH', str(tmp_path / 'core.db'))
    import config
    from api import skills, mcp_manage, canvas
    monkeypatch.setattr(canvas, '_editor_session', None)

    # Core's real SQLite path is isolated from any running user installation.
    monkeypatch.setattr(config, 'DB_PATH', str(tmp_path / 'core.db'))
    config._seed_defaults()
    config.main['skills'] = {'installed': []}
    config.main['canvas_layout'] = {'cards': [{'id': 'core', 'mcpId': 'agentcore', 'toolName': 'decision_core', 'x': 70, 'y': 70},
        {'id': 'teach', 'mcpId': 'test-piper', 'toolName': 'piper_teaching', 'x': 390, 'y': 70}],
        'execConnections': [{'fromCardId': 'core', 'toCardId': 'teach', 'toMcpId': 'test-piper', 'toToolName': 'piper_teaching'}]}
    bundle = DriverBundle([pair, teaching])
    driver = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(lambda: bundle, 'test-piper', 'test-piper'))
    driver_thread = threading.Thread(target=driver.serve_forever, daemon=True); driver_thread.start()
    config.main['services'] = {'mcp': [{'id': 'test-piper', 'name': '松灵 PiPER（模拟 SDK）',
        'transport': 'http', 'url': f'http://127.0.0.1:{driver.server_port}/mcp', 'tools': bundle.get_all_tools()}]}
    ready(pair)
    stop_input = threading.Event()
    def feed_vr():
        sequence = 1
        while not stop_input.wait(.02):
            if pair.motion.mode == 'teleop':
                held = pair.motion.reason not in ('starting', 'waiting_for_input', 'release_both_grips_to_map')
                pair.motion.receive(frame(sequence, held))
                sequence += 1
    input_thread = threading.Thread(target=feed_vr, daemon=True); input_thread.start()
    app = fastapi.FastAPI()
    app.include_router(skills.router, prefix='/api')
    app.include_router(mcp_manage.router, prefix='/api')
    app.include_router(canvas.router, prefix='/api')
    app.mount('/', StaticFiles(directory=Path(CORE) / 'agent-core/web', html=True))
    sock = socket.socket(); sock.bind(('127.0.0.1', 0)); sock.listen(128)
    port = sock.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(app, log_level='error', lifespan='off'))
    thread = threading.Thread(target=server.run, kwargs={'sockets': [sock]}, daemon=True); thread.start()
    wait_for(lambda: server.started)
    yield f'http://127.0.0.1:{port}', teaching, config
    stop_input.set(); input_thread.join(2)
    server.should_exit = True; thread.join(3)
    driver.shutdown(); driver.server_close(); driver_thread.join(2)


@pytest.fixture
def browser_page(site):
    from playwright.sync_api import sync_playwright
    with sync_playwright() as playwright:
        options = {'executable_path': os.environ['PLAYWRIGHT_CHROMIUM_EXECUTABLE']} if os.environ.get('PLAYWRIGHT_CHROMIUM_EXECUTABLE') else {}
        browser = playwright.chromium.launch(**options)
        page = browser.new_page(viewport={'width': 1440, 'height': 1200}, device_scale_factor=1)
        errors = []; page.on('pageerror', lambda error: errors.append(str(error)))
        # Real canvas card, host, skill UI and API. Only unrelated startup and
        # unused 3D renderers are excluded from this hardware-free test.
        page.route('**/js/app.js*', lambda route: route.fulfill(content_type='text/javascript', body="""
            import './auth.js';
            import {initCanvas} from './canvas.js';
            import {initSkills} from './skills.js';
            localStorage.setItem('phanthy_access_token', 'test-only-token');
            document.getElementById('app').style.display=''; initSkills();
            document.getElementById('btn-skills').classList.remove('hidden');
            const response=await fetch('/api/mcp');
            await initCanvas((await response.json()).data);
        """))
        for file, name in [('skeleton', 'Skeleton'), ('pointcloud', 'PointCloud'), ('mapping', 'Mapping')]:
            page.route('**/js/renderers/'+file+'.js', lambda route, *unused, name=name: route.fulfill(
                content_type='text/javascript', body=f'export const {name}Renderer={{canRender:()=>false}};'))
        page.route('**/api/config/project-running', lambda route: route.fulfill(json={'running': True}))
        page.route('**/api/config/auto-start', lambda route: route.fulfill(json={'auto_start': False}))
        page.route('https://**/*', lambda route: route.abort())
        page.goto(site[0])
        assert not errors, errors
        page.locator('[data-card-id="teach"] .canvas-exec-btn').click()
        workbench = page.frame_locator('[data-workbench] iframe')
        workbench.locator('#connection').filter(has_text='驱动已连接').wait_for()
        yield page, workbench, errors
        browser.close()


def confirm_in_browser(ui):
    ui.locator('#confirm').wait_for(state='visible')
    ui.locator('#confirm-control').check()
    ui.locator('#confirm-go').click()
    ui.locator('#confirm').wait_for(state='hidden')


def enter_recording(ui):
    from playwright.sync_api import expect
    ui.locator('#check-btn').click()
    expect(ui.locator('#next-btn')).to_be_enabled()
    ui.locator('#next-btn').click()
    expect(ui.locator('#record')).to_be_visible()


def capture(page, name):
    directory = os.environ.get('TEACHING_SCREENSHOT_DIR')
    if directory:
        Path(directory).mkdir(parents=True, exist_ok=True)
        page.locator('#canvas-toast').wait_for(state='hidden')
        page.frames[1].evaluate('scrollTo(0, 0)')
        page.locator('[data-workbench] .modal').screenshot(path=str(Path(directory) / name))


def test_record_to_installed_skill_with_real_core_proxy_and_sqlite(site, browser_page, pair):
    from playwright.sync_api import expect
    page, ui, errors = browser_page
    _, service, config = site
    assert page.locator('#skill-teach-btn').count() == 0
    expect(page.locator('[data-card-id="teach"] select[data-key="action"] option')).to_have_text(['录制技能', '运行技能'])
    expect(ui.locator('#next-btn')).to_be_disabled()
    assert all(arm.driver.arm.calls == [] for arm in pair.arms.values())
    ui.locator('#check-btn').click()
    expect(ui.locator('#next-btn')).to_be_enabled()
    expect(ui.locator('#checks .check-result.ok')).to_have_count(6)
    capture(page, '01-check.png')
    ui.locator('#next-btn').click()
    ui.locator('#begin-btn').click(); confirm_in_browser(ui)
    expect(ui.locator('#record-btn')).to_be_enabled()
    ui.locator('#record-btn').click()
    expect(ui.locator('#frame-count')).not_to_have_text('0')
    capture(page, '02-record.png')
    page.wait_for_timeout(450)
    ui.locator('#finish-btn').click()
    expect(ui.locator('#review')).to_be_visible()
    expect(ui.locator('#start-hint')).to_contain_text('起点未匹配')
    expect(ui.locator('#trial-btn')).to_be_disabled()
    expect(ui.locator('#plot')).not_to_be_visible()
    capture(page, '03-start-mismatch.png')
    expect(ui.locator('#save-btn')).to_be_disabled()
    wait_for(lambda: not call(service, 'info')['busy'])
    draft = call(service, 'info')['draft']
    restore_simulated_start(service, pair, draft)
    ui.locator('#inspect-btn').click(); expect(ui.locator('#start-hint')).to_contain_text('起点匹配')
    ui.locator('#trial-btn').click(); confirm_in_browser(ui)
    expect(ui.locator('#trial-status')).to_contain_text('试运行已完成', timeout=10000)
    expect(ui.locator('#clip-check')).to_contain_text('录制完整')
    expect(ui.locator('#start-hint')).to_contain_text('再次试运行前需复查')
    expect(ui.locator('#save-btn')).to_be_disabled()
    ui.locator('#effect').check()
    capture(page, '04-trial.png')
    ui.locator('#to-save-btn').click()
    for key in ('name', 'conditions'): ui.locator('#skill-'+key).fill(METADATA[key])
    expect(ui.locator('#phrase-preview')).to_contain_text(METADATA['name'])
    capture(page, '05-save.png')
    artifact = os.environ.get('TEACHING_SCREENSHOT')
    if artifact:
        page.frames[1].evaluate('scrollTo(0, 0)')
        page.screenshot(path=artifact, full_page=True)
    # The asset may be saved while Core temporarily fails. Retry must not create
    # another asset, require another trial, or duplicate the installed skill.
    page.route('**/api/skills/import', lambda route: route.fulfill(status=503,
        content_type='application/json', body=json.dumps({'detail': '测试：本机保存暂时不可用'})), times=1)
    ui.locator('#save-btn').click()
    expect(ui.locator('#error')).to_contain_text('本机保存暂时不可用')
    expect(ui.locator('#save-btn')).to_be_enabled()
    ui.locator('#save-btn').click()
    expect(ui.locator('#confirmation-result')).to_be_visible()
    expect(ui.locator('#confirmation-result')).to_contain_text('已保存')
    capture(page, '06-saved.png')
    installed = config.ConfigDB().get('skills')['installed']
    assert len(installed) == 1 and installed[0]['name'] == METADATA['name']
    assert installed[0]['active'] and service.draft['id'] in installed[0]['instruction']
    assert installed[0]['oneLiner'] == installed[0]['description'] == METADATA['name']
    # Exercise the same split-tool schema and injected instance ID that the
    # decision agent uses, without asking a model to operate simulated hardware.
    import mcp_client
    from event.llm import Event
    async def agent_run():
        import importlib
        import prompt
        event_skills = importlib.import_module('event.skills')
        skill = installed[0]
        assert skill['slug'] in prompt._env_static({})
        assert skill['instruction'] not in prompt._env_static({})
        await event_skills.Tools().activate_skill(skill['slug'])
        assert skill['instruction'] in prompt._env_static({})
        target = config.main['services']['mcp'][0]
        await mcp_client._connect_one(target['id'], target['name'], target['url'], '')
        schemas = Event()._get_bound_tool_schemas()
        name = 'mcp__test-piper__piper_teaching__run'
        assert name in {s['name'] for s in schemas}
        assert 'mcp__test-piper__piper_teaching__ui' not in {s['name'] for s in schemas}
        restore_simulated_start(service, pair, draft)
        result = await mcp_client.call_tool(name, {'asset_id': draft['id'], 'execute': False, 'instance_id': 'teach'})
        preview = json.loads(result)
        assert preview['executed'] is False
        result = json.loads(await mcp_client.call_tool(name, {'asset_id': draft['id'], 'execute': True,
            'preview_id': preview['preview_id'], 'workspace_clear': True, 'take_control': True, 'instance_id': 'teach'}))
        assert result['state'] == 'running'
        await asyncio.to_thread(wait_for, lambda: not call(service, 'info')['busy'])
        assert service.last_run['status'] == 'completed'
        assert service.last_run['action_id'] == result['action_id']
        event_skills._runtime_activated.discard(skill['slug'])
    try:
        with ThreadPoolExecutor(max_workers=1) as executor:
            executor.submit(lambda: asyncio.run(agent_run())).result(timeout=15)
    finally:
        mcp_client.registry.pop('test-piper', None)
    page.get_by_role('button', name='关闭工作台').click()
    page.locator('#btn-skills').click()
    expect(page.locator('#skill-installed-list')).to_contain_text(METADATA['name'])
    assert not errors


def test_failed_preflight_blocks_entry_and_is_rechecked_before_motion(site, browser_page, pair):
    from playwright.sync_api import expect
    page, ui, errors = browser_page
    pair.arms['left'].motion_enabled = False
    ui.locator('#check-btn').click()
    expect(ui.locator('[data-check="permission"]')).to_contain_text('未通过')
    expect(ui.locator('#next-btn')).to_be_disabled()
    expect(ui.locator('#check-summary')).to_have_text('还有 1 项待处理')
    capture(page, '00-check-blocked.png')
    assert pair.motion.mode == 'idle'
    assert all(arm.driver.arm.calls == [] for arm in pair.arms.values())
    pair.arms['left'].motion_enabled = True
    enter_recording(ui)
    # An old green check is never permission to begin a later motion.
    pair.arms['left'].motion_enabled = False
    ui.locator('#begin-btn').click()
    expect(ui.locator('#prepare')).to_be_visible()
    expect(ui.locator('[data-check="permission"]')).to_contain_text('未通过')
    expect(ui.locator('#confirm')).not_to_be_visible()
    assert pair.motion.mode == 'idle'
    assert not errors


def test_missing_conversation_connection_is_visible_before_recording(site, browser_page):
    from playwright.sync_api import expect
    page, ui, _ = browser_page
    _, _, config = site
    page.get_by_role('button', name='关闭工作台').click()
    config.main['canvas_layout'] = {'cards': [], 'execConnections': []}
    page.locator('[data-card-id="teach"] .canvas-exec-btn').click()
    expect(ui.locator('#chat-hint')).to_contain_text('piper_teaching')
    expect(ui.locator('#chat-hint')).to_contain_text('对话调用待连接')


def test_opaque_frame_cannot_read_core_token_or_use_arbitrary_network(site, browser_page):
    page, ui, errors = browser_page
    frame_obj = page.frames[1]
    assert frame_obj.evaluate("() => {try{return parent.localStorage.getItem('phanthy_access_token')}catch(e){return e.name}}") == 'SecurityError'
    assert frame_obj.evaluate("async () => {try{await fetch('/api/skills');return 'allowed'}catch(e){return 'blocked'}}") == 'blocked'
    assert page.locator('iframe').get_attribute('sandbox') == 'allow-scripts'
    page.set_viewport_size({'width': 390, 'height': 844})
    assert frame_obj.evaluate('document.documentElement.scrollWidth <= innerWidth')
    assert not errors


def test_unadvertised_bridge_actions_are_rejected(site, browser_page):
    page, _, errors = browser_page
    # Load a deliberately adversarial page from the otherwise trusted test driver.
    _, service, _ = site
    page.get_by_role('button', name='关闭工作台').click()
    original = service.dispatch
    def dispatch(action, args):
        if action == 'ui':
            return {'protocol': 'motus.workbench/1', 'html': '''<script>
              window.replies=[];
              addEventListener('message', e => {if(!e.ports[0])return;const p=e.ports[0];p.onmessage=e=>replies.push(e.data);
                p.postMessage({id:'bad-method',method:'fetch',args:{url:'/api/skills'}});
                p.postMessage({id:'bad-action',method:'call',args:{action:'delete_everything'}});
                p.postMessage({id:'bad-save',method:'call',args:{action:'save'}});
              });</script>'''}
        return original(action, args)
    service.dispatch = dispatch
    try:
        page.locator('[data-card-id="teach"] .canvas-exec-btn').click()
        page.locator('iframe').wait_for()
        frame_obj = page.frames[1]
        frame_obj.wait_for_function('window.replies?.length === 3')
        assert all(r.get('error') for r in frame_obj.evaluate('replies'))
        assert not errors
    finally:
        service.dispatch = original
