"""Lifecycle answers for plugins that implement no lifecycle of their own.

Most plugins in these bundles are stateless actuators — a gesture, a dock
command, a speaker — whose `dispatch` knows only the actions they perform. The
framework nonetheless sends `start`, `stop` and `info` to every card on the
canvas, so those plugins answer `{"error": "unknown action: start"}`, or return
nothing at all.

That was harmless while agent-core read a bare `error` as success. It no longer
does, and correctly so: the old reading hid real breakage, including a Unitree
speaker that came up bound to nothing and still reported 已就绪. But it meant a
card with nothing to start failed, and because start-project is strict, one such
card rolls the **entire** project back — the robot comes up with nothing running.
On Tianyi that surfaced as the head camera having no data, because `camera_head`
never got started either.

The bundle owns the lifecycle for these plugins: it constructs them and starts
them. When a plugin declines the question, the bundle's own view is the answer.

Kept free of ROS, of any SDK and of the bundles themselves, so the rules can be
tested directly — see tests/test_common_lifecycle.py.
"""

# The actions the framework sends to every card whatever the tool does.
LIFECYCLE_ACTIONS = ("start", "stop", "info")


def is_declined(result) -> bool:
    """Did the plugin decline the action rather than fail at it?

    The two must be told apart, and the distinction is the whole point:

    - **Declined** — no reply at all, or an `unknown action` error with no
      `state`. The plugin is saying it does not know the verb.
    - **Failed** — anything carrying `state`, and any other error text. This is
      what agent-core now exists to surface and must pass through untouched. A
      speaker answering `Missing input_topic` really is bound to nothing and
      still fails.
    """
    if result is None:
        return True
    if not isinstance(result, dict) or "state" in result:
        return False
    return str(result.get("error", "")).strip().lower().startswith("unknown action")


def reply(action: str, started: bool = True) -> dict:
    """What the bundle reports on the plugin's behalf.

    `started` is whether the bundle has this plugin running. Bundles that
    construct and start every plugin at boot can leave it at the default; the
    one that starts them lazily passes what it knows.
    """
    if action == "stop":
        # Nothing was armed, so nothing needs disarming.
        return {"state": "idle"}
    return {"state": "running" if started else "idle"}
