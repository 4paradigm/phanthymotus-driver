"""Hardware-free browser harness for the shared workbench and real Core API."""
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
import os
from pathlib import Path
import socket
import sys
import threading
import time

import pytest
from common.vendor_runtime import DriverBundle, make_handler

CORE = os.environ.get('TEACHING_CORE_ROOT')
pytestmark = pytest.mark.skipif(not CORE, reason='set TEACHING_CORE_ROOT for cross-repository browser acceptance')


def wait_for(predicate):
    deadline = time.monotonic() + 8
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('browser server did not become ready')
        time.sleep(.01)



@contextmanager
def workbench_site(teaching, plugins, monkeypatch, tmp_path, mcp_id, name):
    pytest.importorskip('playwright.sync_api')
    import uvicorn
    import fastapi
    from fastapi.staticfiles import StaticFiles
    sys.path.insert(0, str(Path(CORE) / 'agent-core/src'))
    monkeypatch.setenv('DB_PATH', str(tmp_path / 'core.db'))
    import config
    from api import skills, mcp_manage, canvas, file
    import auth
    monkeypatch.setattr(auth, '_token', 'test-only-token')
    monkeypatch.setattr(canvas, '_editor_session', None)

    # Core's real SQLite path is isolated from any running user installation.
    monkeypatch.setattr(config, 'DB_PATH', str(tmp_path / 'core.db'))
    config._seed_defaults()
    config.main['skills'] = {'installed': []}
    config.main['canvas_layout'] = {'cards': [{'id': 'core', 'mcpId': 'agentcore', 'toolName': 'decision_core', 'x': 70, 'y': 70},
        {'id': 'teach', 'mcpId': mcp_id, 'toolName': teaching.name, 'x': 390, 'y': 70}],
        'execConnections': [{'fromCardId': 'core', 'toCardId': 'teach', 'toMcpId': mcp_id, 'toToolName': teaching.name}]}
    bundle = DriverBundle(plugins)
    from common import file_intake
    monkeypatch.setenv('ACCESS_TOKEN', 'test-only-token')
    # file_intake reads FILE_INTAKE_DIR at import; set the resolved default.
    monkeypatch.setattr(file_intake, 'DEFAULT_BASE_DIR', str(tmp_path / 'driver-uploads'))
    class Handler(make_handler(lambda: bundle, mcp_id, mcp_id)):
        def do_POST(self):
            if file_intake.serve_upload(self):
                return
            super().do_POST()
    driver = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    driver_thread = threading.Thread(target=driver.serve_forever, daemon=True); driver_thread.start()
    config.main['services'] = {'mcp': [{'id': mcp_id, 'name': name,
        'transport': 'http', 'url': f'http://127.0.0.1:{driver.server_port}/mcp', 'tools': bundle.get_all_tools()}]}
    app = fastapi.FastAPI()
    app.include_router(skills.router, prefix='/api')
    app.include_router(mcp_manage.router, prefix='/api')
    app.include_router(canvas.router, prefix='/api')
    app.include_router(file.router, prefix='/api')
    app.mount('/', StaticFiles(directory=Path(CORE) / 'agent-core/web', html=True))
    sock = socket.socket(); sock.bind(('127.0.0.1', 0)); sock.listen(128)
    port = sock.getsockname()[1]
    monkeypatch.setenv('AGENT_CORE_URL', f'http://127.0.0.1:{port}')
    server = uvicorn.Server(uvicorn.Config(app, log_level='error', lifespan='off'))
    thread = threading.Thread(target=server.run, kwargs={'sockets': [sock]}, daemon=True); thread.start()
    wait_for(lambda: server.started)
    try:
        yield f'http://127.0.0.1:{port}', teaching, config
    finally:
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


def capture(page, name):
    directory = os.environ.get('TEACHING_SCREENSHOT_DIR')
    if directory:
        Path(directory).mkdir(parents=True, exist_ok=True)
        page.locator('#canvas-toast').wait_for(state='hidden')
        page.frames[1].evaluate('scrollTo(0, 0)')
        page.locator('[data-workbench] .modal').screenshot(path=str(Path(directory) / name))
