"""Lifecycle answers for plugins that do not implement one.

Most plugins in this bundle are stateless actuators — a gesture, a dock command
— and their `dispatch` knows only the actions they actually perform. The
framework nonetheless sends `start`, `stop` and `info` to every card on the
canvas, so those plugins answer `{"error": "unknown action: start"}`.

That was harmless while agent-core read a bare `error` as success. Once it began
treating one as a failed start — correctly, because the old reading hid real
breakage — `home` failed its card, and since start-project is strict, one such
card rolled the entire project back. The robot came up with nothing running; it
was reported as the camera having lost its data, because `camera_head` never got
started either.

The bundle owns the lifecycle for these plugins: it is what constructs them and
what tracks whether it has done so. When the plugin declines the question, the
bundle's own view is the answer. Kept here, free of ROS and of the bundle, so it
can be tested directly.
"""

# The actions the framework sends to every card whatever the tool does.
LIFECYCLE_ACTIONS = ("start", "stop", "info")


def is_unknown_action(result) -> bool:
    """Did the plugin decline the action rather than fail at it?

    The two have to be told apart. A refusal carries no `state` — the plugin is
    saying it does not know the verb. A genuine failure carries `state: error`
    and is the whole reason agent-core now looks at these replies, so it must
    pass through untouched.
    """
    if not isinstance(result, dict) or "state" in result:
        return False
    return str(result.get("error", "")).strip().lower().startswith("unknown action")


def lifecycle_reply(action: str, started: bool) -> dict:
    """What the bundle reports on the plugin's behalf.

    `started` is whether the bundle has constructed this plugin, which for a
    stateless actuator is the only sense in which it is running.
    """
    if action == "stop":
        # Nothing was armed, so nothing needs disarming.
        return {"state": "idle"}
    return {"state": "running" if started else "idle"}
