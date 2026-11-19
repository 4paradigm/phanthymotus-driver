"""Simulation card owns only its bound remote session, never local physics."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from simulator.mujoco.plugin import SimulationPlugin, build_plugins


class Service:
    configured = True
    endpoint = "http://127.0.0.1:15732"
    has_key = True

    def __init__(self):
        self.calls = []
        self.next_id = 0
        self.sessions = {}

    def call(self, action, **args):
        self.calls.append((action, args))
        if action == "simulation_create":
            self.next_id += 1
            session = {"id": str(self.next_id), "profile_id": args["profile_id"], "state": "paused"}
            self.sessions[session["id"]] = session
            return {"session": session, "snapshot": {}}
        if action == "simulation_list":
            return {"sessions": [session for session in self.sessions.values() if session["state"] != "closed"]}
        session = self.sessions[args["session_id"]]
        if action == "simulation_close":
            session["state"] = "closed"
        return {"session": session}


def card():
    plugin = SimulationPlugin({"endpoint": "http://127.0.0.1:15732", "api_key": "secret"})
    service = Service()
    plugin._client = service
    return plugin, service


def test_start_never_creates_or_runs_a_simulation():
    plugin, service = card()
    plugin.start()
    assert plugin.dispatch("start", {})["busy"] is False
    assert not service.calls


def test_invalid_boot_config_remains_configurable():
    plugin = SimulationPlugin({"endpoint": "http://user:secret@host", "api_key": "secret"})
    assert plugin.dispatch("info", {})["state"] == "unconfigured"
    assert plugin.get_tools()[0]["name"] == "simulation"
    assert plugin.dispatch("config", {"endpoint": "http://server:15732"})["state"] == "ready"


def test_instance_stop_only_pauses_its_own_session():
    plugin, service = card()
    a = plugin.dispatch("simulation_create", {"instance_id": "a", "profile_id": "g1"})
    b = plugin.dispatch("simulation_create", {"instance_id": "b", "profile_id": "g1"})
    assert a["session"]["id"] != b["session"]["id"]
    plugin.dispatch("stop", {"instance_id": "a"})
    assert service.calls[-1] == ("simulation_pause", {"session_id": a["session"]["id"]})
    assert plugin.dispatch("info", {"instance_id": "b"})["session_id"] == b["session"]["id"]


def test_cannot_operate_another_bound_session_or_orphan_current_session():
    plugin, service = card()
    plugin.dispatch("simulation_create", {"profile_id": "g1"})
    count = len(service.calls)
    assert plugin.dispatch("simulation_create", {"profile_id": "g1"})["state"] == "error"
    assert plugin.dispatch("simulation_run", {"session_id": "other"})["state"] == "error"
    assert len(service.calls) == count


def test_connection_switch_requires_closing_session_and_preserves_secret():
    plugin, _ = card()
    plugin.dispatch("simulation_create", {"profile_id": "g1"})
    result = plugin.dispatch("config", {"endpoint": "http://other:15732"})
    assert result["state"] == "error" and result["error"]
    assert "secret" not in json.dumps(plugin.dispatch("info", {})).replace("config_secrets", "")
    plugin.dispatch("simulation_close", {})
    assert plugin.dispatch("info", {})["session_id"] is None
    assert plugin.dispatch("config", {"endpoint": "http://other:15732"})["state"] == "ready"


def test_reattach_recovers_a_server_session_after_plugin_restart():
    original, service = card()
    session = original.dispatch("simulation_create", {"profile_id": "g1"})["session"]
    restored, _ = card()
    restored._client = service
    assert restored.dispatch("simulation_list", {})["sessions"] == [session]
    result = restored.dispatch("simulation_attach", {"session_id": session["id"], "instance_id": "a"})
    assert result["session"]["id"] == session["id"]
    assert restored.dispatch("info", {"instance_id": "a"})["session_id"] == session["id"]
    refused = restored.dispatch("simulation_attach", {"session_id": session["id"], "instance_id": "b"})
    assert refused["state"] == "error"
    assert restored.dispatch("info", {"instance_id": "b"})["session_id"] is None


def test_credentials_can_rotate_without_losing_the_bound_session():
    plugin, _ = card()
    created = plugin.dispatch("simulation_create", {"profile_id": "g1"})["session"]
    result = plugin.dispatch("config", {"api_key": "rotated-secret"})
    assert result["state"] == "ready"
    assert result["session_id"] == created["id"]
    assert plugin._client._api_key == "rotated-secret"
    assert "rotated-secret" not in json.dumps(result)


def test_reapplying_saved_connection_preserves_client_and_session():
    plugin, service = card()
    session_id = plugin.dispatch("simulation_create", {"profile_id": "g1"})["session"]["id"]
    result = plugin.dispatch("config", {"endpoint": service.endpoint, "api_key": "secret"})
    assert plugin._client is service
    assert result["session_id"] == session_id
    plugin.dispatch("config", {"endpoint": service.endpoint, "api_key": "new-secret"})
    assert plugin._client is not service
    assert plugin._client._api_key == "new-secret"
    assert plugin.dispatch("info", {})["session_id"] == session_id


def test_authoritative_list_clears_missing_session_after_server_restart():
    plugin, service = card()
    plugin.dispatch("simulation_create", {"profile_id": "g1"})
    service.sessions.clear()
    assert plugin.dispatch("simulation_list", {}) == {"sessions": []}
    assert plugin.dispatch("info", {})["session_id"] is None
    assert plugin.dispatch("simulation_create", {"profile_id": "g1"})["session"]["id"] == "2"


def test_connection_failure_does_not_discard_owned_session():
    from simulator.mujoco.client import ServiceError
    plugin, service = card()
    plugin.dispatch("simulation_create", {"profile_id": "g1"})
    def unavailable(*args, **kwargs):
        raise ServiceError("网络暂时不可用")
    service.call = unavailable
    assert plugin.dispatch("simulation_list", {})["state"] == "error"
    assert plugin.dispatch("info", {})["session_id"] == "1"


def test_simulation_payload_is_allowlisted():
    plugin, service = card()
    plugin.dispatch("simulation_create", {"profile_id": "g1", "model_id": "policy1", "control_mode": "policy", "model_path": "/etc/passwd"})
    assert service.calls[-1] == ("simulation_create", {"profile_id": "g1", "model_id": "policy1", "control_mode": "policy"})
    plugin.dispatch("simulation_perturb", {"body_id": 1, "force": [10, 0, 0], "duration_s": .1})
    assert service.calls[-1][1] == {"session_id": "1", "body_id": 1, "force": [10, 0, 0], "duration_s": .1}


def test_render_uses_owned_session_and_only_declared_camera_parameters():
    plugin, service = card()
    plugin.dispatch("simulation_create", {"profile_id": "g1"})
    camera = {"width": 960, "height": 540, "azimuth": 135, "elevation": -20,
              "distance": 3, "lookat": [0, 0, .8], "offset": 4,
              "jpeg_quality": 70, "detail": "compact"}
    plugin.dispatch("simulation_render", {**camera, "model_path": "/etc/passwd", "quality": 100})
    assert service.calls[-1] == ("simulation_render", {"session_id": "1", **camera})
    count = len(service.calls)
    assert plugin.dispatch("simulation_render", {"session_id": "someone-else"})["state"] == "error"
    assert len(service.calls) == count
    schema = plugin.get_tools()[0]["inputSchema"]
    assert "simulation_render" in schema["x-workbench"]["actions"]
    assert schema["properties"]["width"]["type"] == "integer"
    assert schema["properties"]["azimuth"]["type"] == "number"
    assert schema["properties"]["lookat"] == {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}


def test_standard_inspect_only_maps_to_reading_the_owned_frame_or_state():
    plugin, service = card()
    plugin.dispatch("simulation_create", {"profile_id": "g1"})
    plugin.dispatch("inspect", {"width": 960, "height": 540})
    assert service.calls[-1] == ("simulation_render", {"session_id": "1", "width": 960, "height": 540})
    plugin.dispatch("inspect", {"mode": "state"})
    assert service.calls[-1] == ("simulation_inspect", {"session_id": "1"})
    plugin.dispatch("inspect", {"mode": "state", "detail": "compact", "width": 640, "jpeg_quality": 70})
    assert service.calls[-1] == ("simulation_inspect", {"session_id": "1", "detail": "compact"})
    count = len(service.calls)
    for mode in ("run", "simulation_run", "close", "train", "reset"):
        assert plugin.dispatch("inspect", {"mode": mode})["state"] == "error"
    assert plugin.dispatch("inspect", {"session_id": "other"})["state"] == "error"
    assert len(service.calls) == count
    schema = plugin.get_tools()[0]["inputSchema"]
    assert "inspect" in schema["x-workbench"]["actions"]
    assert {"frame", "state", "env_status"}.issubset(schema["properties"]["mode"]["enum"])


def test_workbench_is_self_contained_with_actual_session_controls():
    plugin, _ = card()
    tool = plugin.get_tools()[0]
    html = plugin.dispatch("ui", {})["html"]
    assert len(html) < 250000 and "<script src=" not in html and "fetch(" not in html
    assert "simulation_inspect" in tool["inputSchema"]["x-workbench"]["actions"]
    assert "canvas" in html and "simulation_replay" in html
    assert '<option value="render">MuJoCo 画面</option>' in html
    assert '<option value="skeleton">骨架</option>' in html
    assert "call('inspect',{mode:'frame'" in html
    assert tool["configSchema"]["properties"]["api_key"]["x-sensitive"]


def test_bundle_can_be_disabled_without_loading_any_physics():
    assert build_plugins({"plugins": {"simulation": {"enabled": False}}}, "mujoco") == []
    assert len(build_plugins({}, "mujoco")) == 1


def test_shared_environment_connection_and_config_override(monkeypatch):
    monkeypatch.setenv("MOTION_SERVICE_URL", "http://server:15732")
    monkeypatch.setenv("MOTION_API_TOKEN", "environment-secret")
    plugin = SimulationPlugin({"endpoint": "", "api_key": ""})
    assert plugin.dispatch("info", {})["config_values"]["endpoint"] == "http://server:15732"
    plugin.dispatch("config", {"endpoint": "http://other:15732"})
    assert plugin._client._api_key == "environment-secret"
    explicit = SimulationPlugin({"endpoint": "http://explicit:15732", "api_key": "explicit-secret"})
    assert explicit._client.endpoint == "http://explicit:15732"
    assert explicit._client._api_key == "explicit-secret"


def test_cross_repository_clients_do_not_drift():
    # Both bundles deploy independently. Their tiny HTTP boundary is kept in
    # sync when the sibling checkout is available, without a runtime coupling.
    driver = Path(__file__).resolve().parents[1]
    sibling = driver.parent / "phanthymotus/actucore/plugins/motion/client.py"
    if sibling.is_file():
        assert (driver / "simulator/mujoco/client.py").read_text() == sibling.read_text()


def test_browser_controls_real_mujoco_through_existing_workbench_bridge(tmp_path):
    """Opaque iframe -> plugin -> native physics; the HTTP hop is covered separately."""
    import importlib
    import os
    import sys
    chrome = Path(os.environ.get("MOTION_TEST_CHROME", "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"))
    default_main = Path(__file__).resolve().parents[2] / "phanthymotus"
    main_root = Path(os.environ.get("MOTION_TEST_MAIN_ROOT", str(default_main)))
    source = main_root / "actucore/motion_service/simulation.py"
    if not chrome.exists() or not source.exists():
        pytest.skip("Chrome and companion motion service are needed; set MOTION_TEST_CHROME/MOTION_TEST_MAIN_ROOT")
    pytest.importorskip("mujoco")
    playwright = pytest.importorskip("playwright.sync_api")
    sys.path.insert(0, str(source.parents[1]))
    try:
        backend = importlib.import_module("motion_service.simulation")
    finally:
        sys.path.remove(str(source.parents[1]))
    model = tmp_path / "robot.xml"
    model.write_text('''<mujoco><option timestep="0.002" gravity="0 0 -9.81"/>
      <worldbody><geom type="plane" size="2 2 .1"/>
        <body name="robot" pos="0 0 1"><freejoint/><geom type="sphere" size=".1" mass="1"/>
          <body name="arm" pos="0 0 .15"><joint name="hinge" axis="0 1 0"/>
            <geom type="capsule" fromto="0 0 0 .2 0 0" size=".03" mass=".1"/>
          </body></body></worldbody></mujoco>''')
    profile = {"id": "fixture", "label": "Physics fixture", "model_path": str(model), "joint_names": ["hinge"],
               "control_hz": 50, "default_joint_pos": [0], "kp": [10], "kd": [1], "quaternion_order": "wxyz"}
    manager = backend.SimulationManager({"fixture": profile}, tmp_path)
    surviving = manager.call("simulation_create", {"profile_id": "fixture"})["session"]
    plugin = SimulationPlugin({"endpoint": "http://127.0.0.1:15732", "api_key": "test-only",
                               "state_dir": str(tmp_path / "binding")})
    service = Service()
    plugin._client = service
    render_failure = False

    def remote(action, **args):
        service.calls.append((action, dict(args)))
        if action == "simulation_render" and render_failure:
            from simulator.mujoco.client import ServiceError
            raise ServiceError("MuJoCo 渲染失败：EGL context unavailable")
        if action == "catalog":
            return {"profiles": [profile], "models": [], "datasets": []}
        return manager.call(action, args)

    service.call = remote

    def host(message):
        assert message["method"] == "call"
        return plugin.dispatch("simulation", message["args"])

    html = plugin.dispatch("ui", {})["html"]
    try:
        with playwright.sync_playwright() as p:
            browser = p.chromium.launch(executable_path=str(chrome), headless=True)
            page = browser.new_page(viewport={"width": 1200, "height": 900})
            errors = []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.expose_function("hostCall", host)
            # Use a real host origin before attaching the opaque srcdoc frame;
            # recent Chrome crashes when it is attached to an unset about:blank
            # SiteInstance. The route stays entirely inside this test process.
            page.route("http://workbench.test/", lambda route: route.fulfill(
                content_type="text/html", body='<iframe sandbox="allow-scripts" style="width:1100px;height:820px;border:0"></iframe>'))
            page.goto("http://workbench.test/")
            page.evaluate("""html => {
              const f=document.querySelector('iframe');f.onload=()=>{
                const c=new MessageChannel();c.port1.onmessage=async e=>{
                  try {c.port1.postMessage({id:e.data.id,result:await window.hostCall(e.data)});}
                  catch(error){c.port1.postMessage({id:e.data.id,error:error.message});}
                };f.contentWindow.postMessage({protocol:'motus.workbench/1'},'*',[c.port2]);
              };f.srcdoc=`<meta http-equiv="Content-Security-Policy" content="default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src data: blob:; connect-src 'none'; form-action 'none'; base-uri 'none'">${html}`;
            }""", html)
            frame = page.frame_locator("iframe")
            playwright.expect(frame.locator("#appStatus")).to_have_text("已连接")
            playwright.expect(frame.locator(".session-row")).to_have_count(1)
            frame.locator(".session-row button").click()
            playwright.expect(frame.locator("#sessionState")).to_have_text("已暂停")
            playwright.expect(frame.locator("#controlState")).to_have_text("无人控制")
            playwright.expect(frame.locator("#run")).to_be_disabled()
            playwright.expect(frame.locator("#renderedView")).to_be_visible(timeout=15000)
            frame.locator("#claim").click()
            playwright.expect(frame.locator("#controlState")).to_have_text("控制中")
            frame.locator("#closeSession").click()
            playwright.expect(frame.locator("#sessionsView")).to_be_visible()
            assert manager.sessions[surviving["id"]].state == "closed"

            # This fixture has no motor: explicitly select free physics.
            frame.locator("#controlMode").select_option("free_physics")
            frame.locator("#create").click()
            playwright.expect(frame.locator("#sessionState")).to_have_text("已暂停")
            playwright.expect(frame.locator("#controlState")).to_have_text("控制中")
            playwright.expect(frame.locator("#renderedView")).to_be_visible(timeout=15000)
            image_size = frame.locator("#renderedView").evaluate("image => [image.naturalWidth,image.naturalHeight]")
            assert 0 < image_size[0] <= 960 and 0 < image_size[1] <= 640
            assert any(action == "simulation_render" and args["detail"] == "compact" and args["jpeg_quality"] == 88 for action, args in service.calls)
            before_camera = frame.locator("#renderedView").get_attribute("src")
            frame.locator("#view").press("ArrowRight")
            playwright.expect(frame.locator("#renderedView")).not_to_have_attribute("src", before_camera)
            camera_calls = [args for action, args in service.calls if action == "simulation_render"]
            assert camera_calls[-1]["azimuth"] == 140

            frame.locator('[data-debug="recording"]').click()
            frame.locator("#record").click()
            frame.locator('[data-debug="diagnostics"]').click()
            frame.locator("#step").click()
            playwright.expect(frame.locator("#simTime")).to_have_text("0.02 s")
            session_id = plugin.dispatch("info", {})["session_id"]
            session = manager.sessions[session_id]
            assert session.reply()["snapshot"]["root_pose"]["pos"][2] < 1
            frame.locator('[data-debug="recording"]').click()
            frame.locator("#recordStop").click()
            before_replay = session.data.qpos.copy()
            frame.locator("#replay").click()
            playwright.expect(frame.locator("#replayBar")).to_be_visible()
            playwright.expect(frame.locator("#status")).to_have_text("回放")
            assert any(action == "simulation_render" and args.get("offset") == 0 for action, args in service.calls)
            assert (session.data.qpos == before_replay).all()
            frame.locator("#live").click()

            # Terminal playback permits explicit retry and disables stepping.
            with session.lock:
                session.state = "completed"
            frame.locator("#refreshView").click()
            playwright.expect(frame.locator("#sessionState")).to_have_text("动作完成")
            playwright.expect(frame.locator("#run")).to_be_enabled()
            playwright.expect(frame.locator("#run")).to_have_text("再次运行")
            frame.locator('[data-debug="diagnostics"]').click()
            playwright.expect(frame.locator("#step")).to_be_disabled()
            frame.locator("#reset").click()
            playwright.expect(frame.locator("#simTime")).to_have_text("0.00 s")
            playwright.expect(frame.locator("#sessionState")).to_have_text("已暂停")

            render_failure = True
            frame.locator("#refreshView").click()
            playwright.expect(frame.locator("#empty")).to_have_text("MuJoCo 渲染失败：EGL context unavailable")
            playwright.expect(frame.locator("#renderedView")).to_be_hidden()
            playwright.expect(frame.locator("#viewMode")).to_have_value("render")
            render_failure = False
            frame.locator("#refreshView").click()
            playwright.expect(frame.locator("#renderedView")).to_be_visible(timeout=15000)
            frame.locator("#release").click()
            playwright.expect(frame.locator("#controlState")).to_have_text("无人控制")
            playwright.expect(frame.locator("#run")).to_be_disabled()
            frame.locator("#claim").click()
            frame.locator("#closeSession").click()
            playwright.expect(frame.locator("#sessionsView")).to_be_visible()
            assert not errors
            assert session.state == "closed"
            browser.close()
    finally:
        manager.close()
