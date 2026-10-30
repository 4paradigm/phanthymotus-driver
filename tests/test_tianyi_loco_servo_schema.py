"""Public MCP schemas must expose the runtime actions used by canvas clients."""

from test_tianyi_loco_servo import setup, start


def test_every_public_action_has_its_own_parameter_contract(setup):
    plugin = setup.create()
    tool = plugin.get_tool()
    schema = tool["inputSchema"]
    actions = schema["properties"]["action"]["enum"]
    assert set(actions) == {"start", "stop", "pause", "resume", "info", "config"}
    contracts = schema["x-action-params"]
    assert set(contracts) == set(actions)
    assert contracts["start"]["params"] == ["input_topic"]
    for action in ("stop", "pause", "resume", "info"):
        assert contracts[action]["params"] == []
    settings = set(tool["configSchema"]["properties"])
    assert set(contracts["config"]["params"]) == settings
    assert settings <= set(schema["properties"])
    assert schema["required"] == ["action"]
    assert schema["properties"]["input_topic"]["default"] == "/actucore/navi/cmd"


def test_public_config_fields_match_form_constraints_without_aliasing(setup):
    tool = setup.create().get_tool()
    public = tool["inputSchema"]["properties"]
    form = tool["configSchema"]["properties"]
    for name, field in form.items():
        assert public[name] == field
        assert public[name] is not field
    public["dry_run"]["description"] = "changed by an action-schema consumer"
    assert public["dry_run"] != form["dry_run"]


def test_settings_advertised_by_config_action_reach_runtime_configuration(setup):
    plugin = start(setup)
    payload = {"dry_run": True, "rotate_only": True, "require_standing": False,
               "min_vx": .02, "linear_acceleration": .4, "angular_acceleration": 1.5,
               "wz_limit": .6, "yaw_latency_gain": 1.0}
    schema = plugin.get_tool()["inputSchema"]
    assert set(payload) == set(schema["x-action-params"]["config"]["params"])
    result = plugin.dispatch("config", payload)
    assert result["ok"]
    for key, value in payload.items():
        assert result[key] == value
