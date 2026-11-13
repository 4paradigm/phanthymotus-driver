"""Standard Bumi artifact and real bundle contracts; never start a robot/server."""
from pathlib import Path
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import types

import pytest
import yaml

from common.traffic_light import TrafficLightPlugin
from common.traffic_light._ros_lifecycle import dispose_node
from test_traffic_light import detection, jpeg, lamp, load_bumi, ros, rpc_for, stamp

ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"
PRODUCTION_FILES = {"__init__.py", "schemas.py", "color_classifier.py", "processor.py",
                    "plugin.py", "_ros_lifecycle.py"}


@pytest.fixture(scope="module")
def standard_context(tmp_path_factory):
    """Use the directory+extras layout of build.sh, never the repository root."""
    context = tmp_path_factory.mktemp("bumi-build-context")
    shutil.copytree(BUMI, context, dirs_exist_ok=True)
    metadata = yaml.safe_load((BUMI / "driver.yaml").read_text())
    for extra in metadata.get("build_context_extras", []):
        source = (BUMI / extra).resolve()
        shutil.copytree(source, context / source.name)
    return context


@pytest.fixture(scope="module")
def image_files(standard_context, tmp_path_factory):
    """Materialise final-stage source COPYs; no Docker, RUN or SDK execution."""
    image = tmp_path_factory.mktemp("bumi-image-files")
    copies = []
    text = re.sub(r"\\\n\s*", " ", (standard_context / "Dockerfile").read_text())
    for line in text.splitlines():
        tokens = shlex.split(line, comments=True)
        if not tokens:
            continue
        if tokens[0].upper() == "FROM":
            copies.clear()
        if tokens[0].upper() == "COPY" and not any(t.startswith("--from=") for t in tokens):
            assert not any(t.startswith("--") for t in tokens), "update this source-COPY model for new flags"
            copies.append(tokens[1:])
    for operands in copies:
        destination = image / operands[-1].lstrip("/")
        for name in operands[:-1]:
            source = standard_context / name
            assert source.exists(), f"COPY source missing from standard context: {name}"
            if source.is_dir():
                shutil.copytree(source, destination, dirs_exist_ok=True)
            else:
                target = destination / source.name if operands[-1].endswith("/") else destination
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
    return image


def test_bumi_standard_context_contains_traffic_light(standard_context, image_files):
    assert not (standard_context / "perception").exists()
    assert not (standard_context / "phanthymotus").exists()
    for name in PRODUCTION_FILES:
        source = standard_context / "common" / "traffic_light" / name
        shipped = image_files / "work" / "common" / "traffic_light" / name
        assert source.is_file() and shipped.is_file()
        assert source.read_bytes() == shipped.read_bytes()
    assert (image_files / "work" / "main.py").read_bytes() == (BUMI / "main.py").read_bytes()
    assert not (image_files / "work" / "perception").exists()


def test_bumi_traffic_light_imports_without_sibling_checkout(image_files):
    # -I ignores PYTHONPATH and the working directory. Only the staged /work is
    # added, so imports cannot be rescued by the development checkout.
    script = r'''
import importlib.abc, importlib.util, json, pathlib, sys, types
work = pathlib.Path(sys.argv[1]).resolve()
sys.path.insert(0, str(work))
class RejectCheckout(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('perception', 'utils', 'noetix', 'highcontrol_py', 'mediacontrol_py'):
            raise AssertionError('forbidden checkout/SDK dependency: '+fullname)
sys.meta_path.insert(0, RejectCheckout())
for name in ('rclpy','rclpy.node','rclpy.qos','rclpy.executors','std_msgs','std_msgs.msg','sensor_msgs','sensor_msgs.msg'):
    sys.modules[name] = types.ModuleType(name)
class Node:
    def __init__(self, name): self.subscriptions=[]; self.destroyed=False
    def get_clock(self): return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=1000000000000))
    def create_publisher(self, cls, topic, qos): return types.SimpleNamespace(publish=lambda msg:None)
    def create_subscription(self, cls, topic, callback, qos): self.subscriptions.append(topic)
    def create_timer(self, period, callback): pass
    def destroy_node(self): self.destroyed=True
class Executor:
    def __init__(self): self.nodes=[]
    def add_node(self, node): self.nodes.append(node)
    def get_nodes(self): return self.nodes
    def remove_node(self, node): self.nodes.remove(node)
sys.modules['rclpy.node'].Node=Node
qos=sys.modules['rclpy.qos']; qos.QoSProfile=lambda **kw:types.SimpleNamespace(**kw)
for name in ('ReliabilityPolicy','HistoryPolicy','DurabilityPolicy'):
    setattr(qos,name,types.SimpleNamespace(BEST_EFFORT=1,KEEP_LAST=1,VOLATILE=1))
sys.modules['std_msgs.msg'].String=type('String',(),{})
sys.modules['sensor_msgs.msg'].CompressedImage=type('CompressedImage',(),{})
sys.modules['rclpy'].executors=sys.modules['rclpy.executors']
import common
logsafe=types.ModuleType('common.logsafe'); logsafe.install=lambda **kw:None
common.logsafe=logsafe; sys.modules['common.logsafe']=logsafe
from common.traffic_light import TrafficLightPlugin, classify_color
import common.traffic_light.plugin as implementation
assert pathlib.Path(implementation.__file__).resolve().is_relative_to(work)
import numpy as np
assert classify_color(np.zeros((20,20,3),np.uint8)).color=='unknown'
spec=importlib.util.spec_from_file_location('isolated_bumi',work/'main.py')
main=importlib.util.module_from_spec(spec); spec.loader.exec_module(main)
default=main._load_config(); cfg={'plugins':{'traffic_light':default['plugins']['traffic_light']}}
executor=Executor(); bundle=main.BumiDeviceBundle(cfg,'isolated',executor,None,None)
bundle.start_all(); assert not executor.nodes
tools=bundle.get_all_tools(); assert [t['name'] for t in tools]==['traffic_light']
args={'instance_id':'isolated','input_topic':'/r/camera/color/objects'}
assert bundle.dispatch('traffic_light',{'action':'start',**args})['state']=='running'
assert len(executor.nodes)==1 and len(executor.nodes[0].subscriptions)==2
assert bundle.dispatch('traffic_light',{'action':'info','instance_id':'isolated'})['state']=='running'
assert bundle.dispatch('traffic_light',{'action':'latest','instance_id':'isolated'})['state']=='waiting'
node=executor.nodes[0]
assert bundle.dispatch('traffic_light',{'action':'stop','instance_id':'isolated'})['state']=='idle'
assert node.destroyed and not executor.nodes
bundle.stop_all(); bundle.stop_all()
assert not any(n=='perception' or n.startswith('perception.') or n=='utils' or n.startswith('utils.') for n in sys.modules)
print('isolated Bumi traffic_light lifecycle passed')
'''
    environment = dict(os.environ)
    environment.pop("CONFIG_PATH", None)
    environment.pop("PHANTHY_PERCEPTION_ROOT", None)
    result = subprocess.run([sys.executable, "-I", "-B", "-c", script,
                             str(image_files / "work")], cwd=image_files,
                            env=environment, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "isolated Bumi traffic_light lifecycle passed" in result.stdout


@pytest.fixture
def bumi(ros, monkeypatch):
    main = load_bumi(monkeypatch)
    bundle = main.BumiDeviceBundle({"plugins": {"traffic_light": {"enabled": True}}},
                                  "r", ros.executor, None, None)
    yield types.SimpleNamespace(main=main, bundle=bundle, plugin=bundle._plugins[0],
                                rpc=rpc_for(main, bundle))
    bundle.stop_all()


def call(bundle, action, **args):
    return bundle.dispatch("traffic_light", {"action": action, **args})


def feed(runtime, ros, payload):
    runtime.image(types.SimpleNamespace(data=jpeg(lamp()), header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(**payload["source_stamp"]))))
    runtime.detection(ros.String(json.dumps(payload)))


def default_bundle(ros, monkeypatch):
    """Keep the default YAML intact; unrelated hardware plugins are inert doubles."""
    main = load_bumi(monkeypatch)

    def passive(names):
        class Plugin:
            def __init__(self, *args, **kwargs): pass
            def get_tools(self): return [{"name": name, "type": kind} for name, kind in names]
            def start(self): pytest.fail("default registration must not start hardware")
            def stop(self): pass
        return Plugin

    device = types.ModuleType("device")
    device.CameraPlugin = passive([("camera", "sensor"), ("depth", "sensor")])
    device.VisionCapturePlugin = passive([("vision_capture", "actuator")])
    visual = types.ModuleType("visual_odometry")
    visual.VisualOdometryPlugin = passive([("rgbd_visual_odometry", "sensor")])
    monkeypatch.setitem(sys.modules, "device", device)
    monkeypatch.setitem(sys.modules, "visual_odometry", visual)
    config = yaml.safe_load((BUMI / "config.yaml").read_text())
    bundle = main.BumiDeviceBundle(config, "r", ros.executor, None, None)
    return main, bundle


def test_default_bumi_bundle_lists_traffic_light_once(ros, monkeypatch):
    main, bundle = default_bundle(ros, monkeypatch)
    listed = rpc_for(main, bundle)("tools/list")["result"]["tools"]
    assert sum(tool["name"] == "traffic_light" for tool in listed) == 1
    assert len([p for p in bundle._plugins if isinstance(p, TrafficLightPlugin)]) == 1
    assert not ros.created


@pytest.mark.parametrize("controllers_available", [False, True])
def test_traffic_light_registration_does_not_require_controllers(ros, monkeypatch, controllers_available):
    class NoSDKCalls:
        def __getattr__(self, name): pytest.fail("SDK access: " + name)

    main = load_bumi(monkeypatch)
    controller = NoSDKCalls() if controllers_available else None
    bundle = main.BumiDeviceBundle({"plugins": {"traffic_light": {"enabled": True}}},
                                  "r", ros.executor, controller, controller)
    assert [t["name"] for t in bundle.get_all_tools()] == ["traffic_light"]
    bundle.start_all()
    bundle.stop_all()
    assert not ros.created


def test_bumi_mcp_traffic_light_lifecycle(bumi, ros):
    def invoke(action):
        result = bumi.rpc("tools/call", {"name": "traffic_light", "arguments": {
            "action": action, "instance_id": "c", "input_topic": "/r/camera/color/objects"}})
        assert "error" not in result
        return json.loads(result["result"]["content"][0]["text"])

    started = invoke("start")
    assert started["state"] == "running" and started["rgb_topic"] == "/r/camera/color"
    assert started["topic_out"][0]["topic"] == "/r/camera/color/objects/traffic_light"
    node = ros.executor.nodes[0]
    assert [(cls.__name__, topic) for cls, topic, _ in node.subscription_types] == [
        ("CompressedImage", "/r/camera/color"), ("String", "/r/camera/color/objects")]
    assert node.publisher_topics[0][0] is ros.String
    assert all(qos.depth == 2 and qos.reliability == 1 and qos.durability == 1
               for _, _, qos in node.subscription_types + node.publisher_topics)
    assert invoke("info")["state"] == "running"
    assert invoke("latest")["state"] == "waiting"
    assert invoke("stop") == {"state": "idle", "cleanup_pending": []}
    assert invoke("stop")["state"] == "idle" and not ros.executor.nodes


def test_bundle_boot_does_not_open_traffic_light_sources(bumi, ros):
    bumi.bundle.start_all()
    assert not ros.created and not ros.executor.nodes and not bumi.plugin._instances
    assert call(bumi.bundle, "info")["state"] == "idle"
    assert call(bumi.bundle, "start")["state"] == "error"  # no invented source
    assert not ros.created
    assert call(bumi.bundle, "start", instance_id="c", input_topic="/r/color/objects")["state"] == "running"
    assert len(ros.created) == 1
    bumi.bundle.stop_all()
    bumi.bundle.stop_all()
    assert not bumi.plugin._instances and not ros.executor.nodes


def test_bumi_traffic_light_preserves_exact_stamp_join(bumi, ros):
    call(bumi.bundle, "start", instance_id="c", input_topic="/r/color/objects")
    runtime = bumi.plugin._instances["c"]
    old_stamp = stamp(ros.clock.ns())
    runtime.image(types.SimpleNamespace(data=jpeg(lamp()), header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(**old_stamp))))
    ros.clock.advance(.01)
    payload = detection(ros.clock)
    runtime.detection(ros.String(json.dumps(payload)))
    result = call(bumi.bundle, "latest", instance_id="c")
    assert result["state"] == "waiting" and result["reason"] == "matching_image_unavailable"
    assert result["signals"][0]["color"] == "unknown" and not runtime.engine.tracks
    feed(runtime, ros, payload)
    runtime.detection(ros.String(json.dumps(payload)))
    assert call(bumi.bundle, "latest", instance_id="c")["signals"][0]["observations"] == 1
    for index in (2, 3):
        ros.clock.advance(.2)
        payload = detection(ros.clock, seq=index)
        feed(runtime, ros, payload)
    result = call(bumi.bundle, "latest", instance_id="c")
    assert result["state"] == "ok" and result["frame_stamp"] == payload["source_stamp"]
    assert result["signals"][0]["color"] == "red" and result["signals"][0]["stability"] == "stable"
    ros.clock.advance(1.1)
    stale = call(bumi.bundle, "latest", instance_id="c")
    assert stale["state"] == "stale" and stale["signals"][0]["color"] == "unknown"


def test_bumi_traffic_light_instance_cleanup_isolated(bumi, ros):
    for key in ("a", "b"):
        call(bumi.bundle, "start", instance_id=key, input_topic=f"/{key}/color/objects")
    old = bumi.plugin._instances["a"]
    payload = detection(ros.clock)
    feed(old, ros, payload)
    assert old.engine.frames and old.engine.tracks
    old.node.destroy_fails = True
    failed = call(bumi.bundle, "stop", instance_id="a")
    assert failed == {"state": "error", "cleanup_pending": ["a"]}
    assert bumi.plugin._instances["a"] is old and not old.running
    assert call(bumi.bundle, "info", instance_id="b")["state"] == "running"
    assert not old.engine.frames and not old.engine.pending and not old.engine.tracks
    old.node.destroy_fails = False
    assert call(bumi.bundle, "stop", instance_id="a")["state"] == "idle"
    ros.clock.advance(.2)
    assert call(bumi.bundle, "observe", instance_id="a")["state"] == "running"
    new = bumi.plugin._instances["a"]
    assert new is not old and not new.engine.frames and not new.engine.tracks
    count = len(old.node.messages)
    feed(old, ros, payload)
    assert len(old.node.messages) == count and not new.engine.tracks
    new.detection(ros.String(json.dumps(payload)))
    assert new.engine.latest()["reason"] == "before_observation_start"
    bumi.bundle.stop_all()
    assert not ros.executor.nodes and not bumi.plugin._instances
    for runtime in (old, new):
        assert not runtime.engine.frames and not runtime.engine.pending and not runtime.engine.tracks


def test_bumi_traffic_light_metadata_matches_runtime(ros, monkeypatch):
    main, bundle = default_bundle(ros, monkeypatch)
    metadata = yaml.safe_load((BUMI / "driver.yaml").read_text())
    config = yaml.safe_load((BUMI / "config.yaml").read_text())
    assert config["plugins"]["traffic_light"]["enabled"] is True
    assert [t for t in metadata["cards"] if t["name"] == "traffic_light"] == [
        {"name": "traffic_light", "type": "processor"}]
    listed = rpc_for(main, bundle)("tools/list")["result"]["tools"]
    tool = next(t for t in listed if t["name"] == "traffic_light")
    assert tool["type"] == "processor" and tool["multiInstance"] is True
    assert tool["topic_in"][0]["format"] == "data/json"
    assert tool["topic_out"][0]["format"] == "data/json"


def test_bundle_stop_cleans_other_instances_and_retains_failed_handle(bumi, ros):
    for key in ("a", "b"):
        call(bumi.bundle, "start", instance_id=key, input_topic=f"/{key}/color/objects")
    failed = bumi.plugin._instances["a"]
    failed.node.destroy_fails = True
    with pytest.raises(RuntimeError, match="TrafficLightPlugin.*unsuccessful stop result"):
        bumi.bundle.stop_all()
    assert list(bumi.plugin._instances) == ["a"]
    assert not failed.running and not ros.executor.nodes
    assert bumi.plugin.stop() == {"state": "error", "cleanup_pending": ["a"]}
    failed.node.destroy_fails = False
    bumi.bundle.stop_all()
    assert not bumi.plugin._instances


@pytest.mark.parametrize("failure", ["remove_false", "remove_raises", "still_registered", "destroy_false", "destroy_raises"])
def test_local_node_disposal_failure_is_retryable_and_idempotent(failure):
    node = types.SimpleNamespace(destroy_node=lambda: None)
    nodes, calls = [node], []
    def remove(value):
        calls.append("remove")
        if failure == "remove_raises": raise RuntimeError("remove failed")
        if failure == "remove_false": return False
        if failure != "still_registered": nodes.remove(value)
    def destroy():
        calls.append("destroy")
        if failure == "destroy_raises": raise RuntimeError("destroy failed")
        return False if failure == "destroy_false" else None
    node.destroy_node = destroy
    executor = types.SimpleNamespace(get_nodes=lambda:nodes, remove_node=remove)
    result = dispose_node(executor, node)
    assert not result.complete and result.error
    if failure not in ("destroy_false", "destroy_raises"): assert calls == ["remove"]
    executor.remove_node = lambda value: nodes.remove(value)
    node.destroy_node = lambda: calls.append("destroy_retry")
    assert dispose_node(executor, node, progress=result).complete
    count = len(calls)
    assert dispose_node(executor, node, progress=result).complete and len(calls) == count
