"""Public MCP schemas must expose the runtime actions used by canvas clients."""

from test_tianyi_loco_servo import setup, start
from common.control import parse_descriptor


def test_every_public_action_has_its_own_parameter_contract(setup):
    plugin = setup.create()
    tool = plugin.get_tool()
    schema = tool["inputSchema"]
    actions = schema["properties"]["action"]["enum"]
    assert set(actions) == {"start", "stop", "pause", "resume", "info", "config"}
    contracts = schema["x-action-params"]
    assert set(contracts) == set(actions)
    assert contracts["start"]["params"][0] == "input_topic"
    for action in ("stop", "pause", "resume", "info"):
        assert contracts[action]["params"] == []
    settings = set(tool["configSchema"]["properties"])
    assert set(contracts["start"]["params"]) == settings | {"input_topic"}
    assert set(contracts["config"]["params"]) == settings
    assert settings <= set(schema["properties"])
    assert schema["required"] == ["action"]
    assert schema["properties"]["input_topic"]["default"] == "/actucore/navi/cmd"


def test_public_config_fields_match_form_constraints_without_aliasing(setup):
    tool = setup.create().get_tool()
    public = tool["inputSchema"]["properties"]
    form = tool["configSchema"]["properties"]
    for name, field in form.items():
        expected = dict(field)
        expected.pop("default", None)
        assert public[name] == expected
        assert public[name] is not field
        assert "default" not in public[name]
        assert "default" in field
    public["dry_run"]["description"] = "changed by an action-schema consumer"
    assert public["dry_run"] != form["dry_run"]


def test_settings_advertised_by_config_action_reach_runtime_configuration(setup):
    plugin = start(setup)
    payload = {"dry_run": True, "rotate_only": True, "require_standing": False,
               "speed_preset": "fast",
               "min_vx": .02, "linear_acceleration": .4, "angular_acceleration": 1.5,
               "wz_limit": .6}
    schema = plugin.get_tool()["inputSchema"]
    assert set(payload) == set(schema["x-action-params"]["config"]["params"])
    result = plugin.dispatch("config", payload)
    assert result["ok"]
    for key, value in payload.items():
        assert result[key] == value


def test_speed_preset_form_is_explicit_and_action_schema_has_no_stale_default(setup):
    tool = setup.create().get_tool()
    form = tool["configSchema"]["properties"]["speed_preset"]
    action = tool["inputSchema"]["properties"]["speed_preset"]
    assert form["enum"] == ["slow", "normal", "fast"]
    assert form["default"] == "normal"
    assert "default" not in action


def test_driver_response_is_descriptor_metadata_not_an_operator_setting(setup):
    response = {"wz": {"gain": .76, "delay_s": .12, "settling_s": .35,
                       "source": "measured"}}
    descriptor = setup.module.build_descriptor(response=response)
    assert parse_descriptor(descriptor).response == response
    plugin = setup.create(response=response)
    assert plugin.dispatch("info", {})["control_interface"]["response"] == response
    assert "response" not in plugin.get_tool()["configSchema"]["properties"]
