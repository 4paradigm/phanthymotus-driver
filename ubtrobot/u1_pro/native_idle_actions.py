"""Access the U1's ROSA-only idle preference through its existing vendor runtime.

The public /robo SDK does not expose this switch. The driver already has host
PID/IPC namespaces and privileged mode; nsenter uses the host Docker CLI to run
the vendor's ROSA client in the behavior container. No credentials, Docker
socket mount, direct configuration writes, or service restarts are needed.
"""

from __future__ import annotations

import json
import re
import subprocess
import threading


DEFAULT_NATIVE_CONTAINER = "walker-behavior.behavior-1"
IDLE_ACTION_KEY = "idle_action_enabled"
CONFIG_SERVICE = "/sys/config/user_manager/set_config"

# Pass the payload as a positional argument, never interpolate it into shell code.
# The inner timeout also terminates ROSA if the outer Docker client disappears.
_SET_CONFIG = (
    "source /opt/walker/setup.bash && "
    "exec timeout --signal=TERM --kill-after=2s 18s "
    "rosa service call --no-daemon " + CONFIG_SERVICE +
    ' config_msgs/srv/SetConfig "$1"'
)
_READ_CONFIG = """
import json
with open('/data/user_data/user_config_center.json', encoding='utf-8') as stream:
    entries = json.load(stream)['data']
values = [item['value'] for item in entries if item['name'] == 'idle_action_enabled']
if len(values) != 1:
    raise RuntimeError('idle_action_enabled is missing or duplicated in vendor user config')
value = values[0]
if type(value.get('type')) is not int or value['type'] != 1 or type(value.get('bool_value')) is not bool:
    raise RuntimeError('idle_action_enabled is not a vendor boolean config value')
print(json.dumps({'enabled': value['bool_value']}))
"""


class IdleActionControl:
    def __init__(self, container: str = DEFAULT_NATIVE_CONTAINER):
        if not isinstance(container, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", container):
            raise ValueError("idle_actions.native_container must be a Docker container name")
        self.container = container
        self._lock = threading.RLock()

    def _exec(self, arguments: list[str], timeout: float) -> str:
        command = ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--wd=/", "--",
                   "/usr/bin/docker", "exec", self.container, *arguments]
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("U1 Pro idle_action operation timed out; query status before retrying") from exc
        except OSError as exc:
            raise RuntimeError("U1 Pro idle_action requires nsenter and access to the host vendor runtime") from exc
        if result.returncode:
            detail = (result.stderr or result.stdout).strip()[-240:]
            raise RuntimeError(f"U1 Pro idle_action native command failed ({result.returncode}): {detail}")
        return result.stdout

    def status(self) -> dict:
        """Read the persisted vendor preference, not an inferred pose/motion state."""
        with self._lock:
            output = self._exec(["python3", "-c", _READ_CONFIG], timeout=5)
            try:
                value = json.loads(output)
            except (ValueError, TypeError) as exc:
                raise RuntimeError("U1 Pro idle_action returned invalid configuration data") from exc
            if not isinstance(value, dict) or type(value.get("enabled")) is not bool:
                raise RuntimeError("U1 Pro idle_action readback did not contain a boolean")
            return {"ok": True, "code": "OK", "data": {"enabled": value["enabled"]},
                    "source": "vendor_user_config"}

    def set_enabled(self, enabled: bool) -> dict:
        if type(enabled) is not bool:
            raise ValueError("idle_action enabled must be a boolean")
        with self._lock:
            payload = {"config": [{"name": IDLE_ACTION_KEY,
                                   "value": {"type": 1, "bool_value": enabled}}]}
            output = self._exec(["bash", "-c", _SET_CONFIG, "idle-action-config",
                                 json.dumps(payload)], timeout=25)
            # ROSA can exit normally even when the service rejects the request.
            # Require the service response, not just an exit code or request echo.
            response = re.search(
                r"^response: config_msgs\.srv\.SetConfig\.Response\(data=(True|False), message=(.*)\)\s*$",
                output, re.MULTILINE,
            )
            if not response:
                raise RuntimeError("U1 Pro idle_action received no recognized config service response; query status")
            if response.group(1) != "True":
                raise RuntimeError(f"U1 Pro idle_action config service rejected the request: {response.group(2)[:240]}")
            state = self.status()
            actual = state["data"]["enabled"]
            if actual is not enabled:
                raise RuntimeError(f"U1 Pro idle_action state mismatch: requested {enabled}, got {actual}")
            return {"ok": True, "requested": enabled, "enabled": actual, "state": state}
