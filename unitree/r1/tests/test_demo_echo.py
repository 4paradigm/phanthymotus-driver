import json
import sys
from pathlib import Path

R1_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R1_DIR))

from demo_echo import DemoEchoPlugin

def make_plugin():
    return DemoEchoPlugin(plugin_config={"enabled": True}, namespace="test", executor=None)


def test_tool_schema():
    plugin = make_plugin()
    tool = plugin.get_tool()
    assert tool["name"] == "demo_echo"
    assert tool["type"] == "processor"
    assert tool["inputSchema"]["properties"]["action"]["enum"] == ["run"]

def test_lifecycle():
    plugin = make_plugin()
    assert plugin.dispatch("start", {}) == {"state": "Plugin started."}
    assert plugin.dispatch("stop", {}) == {"state": "Plugin stopped."}

def test_run():
    result = make_plugin().dispatch("run", {"text": "Hello, World!"})
    assert result == {
        "state": "completed", 
        "input": "Hello, World!",
        "output": "received:Hello, World!"}

    json.dumps(result,ensure_ascii=False)  # Ensure the result is JSON serializable


def test_empty_text_is_rejected():
    result = make_plugin().dispatch("run", {"text": ""})
    assert result == {
        "state": "error",
        "error": "Input text cannot be empty."
    }
    json.dumps(result,ensure_ascii=False)  # Ensure the result is JSON serializable 
