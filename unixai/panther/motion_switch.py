"""The operator's switch for motion: one small file, outside the image, read at start-up.

`config.yaml` ships with `safety.commissioning_motion_enabled: false` and an image cannot ask an
operator to edit it: a change made inside the container is lost with the container. This module
is the supported way to turn motion on at a robot, and it is built so that the default stays a
refusal:

* The file lives on the host (`deploy/service.yml` mounts `/opt/phanthy-motus/data/panther`
  read-only into the container), so it survives a new image or container.
* **Only one key is read from it**, `commissioning_motion_enabled`. The caps (speeds, step sizes,
  the lift's maximum) stay in `config.yaml`: a file that could also raise them would let one
  careless edit remove every limit at once.
* **Anything other than the YAML boolean `true` is "off"**: a missing file or directory, an
  unreadable or malformed file, `"true"` as a string, `1`, `yes`, a list. A mistake can therefore
  only ever leave motion refused, never switch it on.
* It is read once, when the driver starts, not on every command. Switching motion on or off
  means restarting the container, which also ends anything in flight.

The result says where the decision came from, so the card and the log can tell an operator
*why* motion is off, not just that it is.
"""
from __future__ import annotations

import os

DEFAULT_PATH = "/opt/phanthy-motus/data/panther/commissioning.yaml"
ENV_VAR = "PANTHER_COMMISSIONING_FILE"
KEY = "commissioning_motion_enabled"


def resolve_motion_switch(config: dict, environ=None, open_file=open) -> tuple:
    """`(enabled, source)`. `enabled` is True only if the operator's file says `true`.

    `source` is a short sentence: which file decided, or why the default (off) applies. The image's
    own `config.yaml` value is honoured as the fallback, so a robot with no operator file behaves
    exactly as before (off), and a developer's local config can still turn it on in a test.
    """
    environ = os.environ if environ is None else environ
    baseline = ((config or {}).get("safety") or {}).get(KEY) is True
    path = environ.get(ENV_VAR) or DEFAULT_PATH
    try:
        with open_file(path, encoding="utf-8") as handle:
            raw = handle.read()
    except FileNotFoundError:
        return baseline, f"no operator file at {path}; using config.yaml ({'on' if baseline else 'off'})"
    except OSError as exc:
        return False, f"cannot read {path} ({type(exc).__name__}); motion stays off"
    try:
        import yaml

        document = yaml.safe_load(raw)
    except Exception as exc:  # noqa: BLE001
        return False, f"{path} is not valid YAML ({type(exc).__name__}); motion stays off"
    if not isinstance(document, dict) or KEY not in document:
        return False, f"{path} has no `{KEY}` key; motion stays off"
    value = document[KEY]
    if value is True:
        return True, f"{path} sets {KEY}: true"
    if value is False:
        return False, f"{path} sets {KEY}: false"
    return False, f"{path}: {KEY} is {value!r}, not the boolean true; motion stays off"


def apply_motion_switch(config: dict, environ=None, open_file=open) -> dict:
    """`config` with `safety.commissioning_motion_enabled` decided by `resolve_motion_switch`.

    Returns the same dict, changed in place, with the reason in `safety.motion_switch_source`.
    """
    enabled, source = resolve_motion_switch(config, environ, open_file)
    safety = config.setdefault("safety", {})
    safety[KEY] = enabled
    safety["motion_switch_source"] = source
    return config
