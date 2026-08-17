"""Restricted SSH client for the M20 Pro NOS ``drmap`` utility."""

from __future__ import annotations

import re
import subprocess


_MAP_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")


class NOSMappingClient:
    """Run a fixed allow-list of mapping commands on the navigation host."""

    def __init__(self, config, runner=None):
        self.config = dict(config or {})
        self.host = str(self.config.get("host", "10.21.31.106"))
        self.user = str(self.config.get("user", "user"))
        self.port = int(self.config.get("port", 22))
        self.identity_file = str(self.config.get("identity_file", "/run/secrets/m20_nos_ssh_key"))
        self.known_hosts_file = str(self.config.get("known_hosts_file", "/run/secrets/m20_nos_known_hosts"))
        self.timeout = float(self.config.get("timeout", 15))
        self._runner = runner or subprocess.run

    def _ssh(self, remote_command, *, accepted_codes=(0,), timeout=None):
        command = [
            "ssh", "-T",
            "-p", str(self.port),
            "-i", self.identity_file,
            "-o", "BatchMode=yes",
            "-o", "IdentitiesOnly=yes",
            "-o", "HostKeyAlgorithms=ssh-ed25519",
            "-o", "StrictHostKeyChecking=yes",
            "-o", f"UserKnownHostsFile={self.known_hosts_file}",
            "-o", "ConnectTimeout=5",
            f"{self.user}@{self.host}",
            remote_command,
        ]
        try:
            completed = self._runner(
                command,
                capture_output=True,
                text=True,
                timeout=self.timeout if timeout is None else timeout,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f"NOS SSH unavailable: {exc}") from exc
        if completed.returncode not in accepted_codes:
            detail = (completed.stderr or completed.stdout or "remote command failed").strip()
            raise RuntimeError(f"NOS command failed ({completed.returncode}): {detail[:500]}")
        return (completed.stdout or "").strip(), completed.returncode

    @staticmethod
    def validate_map_name(name):
        value = str(name or "")
        if not _MAP_NAME.fullmatch(value):
            raise ValueError("map_name 只能包含字母、数字、下划线和连字符，长度为 1 到 64")
        return value

    def start_mapping(self, map_name, *, activate=True):
        name = self.validate_map_name(map_name)
        remote = f"sudo -n /usr/local/sbin/phanthy-m20-mapping start {name} {'true' if activate else 'false'}"
        output, _ = self._ssh(remote, timeout=max(self.timeout, 30))
        return {"state": "mapping", "map_name": name, "activate_on_stop": bool(activate), "output": output[-1000:]}

    def stop_mapping(self):
        output, _ = self._ssh("sudo -n /usr/local/sbin/phanthy-m20-mapping stop", timeout=max(self.timeout, 60))
        return {"state": "saved", "output": output[-1000:]}

    def status(self):
        output, code = self._ssh(
            "systemctl is-active mapping.service",
            accepted_codes=(0, 3, 4),
        )
        state = "mapping" if code == 0 and output == "active" else "idle"
        active_map, _ = self._ssh("readlink -f /var/opt/robot/data/maps/active", accepted_codes=(0, 1))
        return {"state": state, "service": output or "unknown", "active_map": active_map or None}

    def list_maps(self):
        output, _ = self._ssh(
            "find /var/opt/robot/data/maps -mindepth 1 -maxdepth 1 -type d -printf '%f\\n'"
        )
        return {"state": "ready", "maps": sorted(line for line in output.splitlines() if line)}
