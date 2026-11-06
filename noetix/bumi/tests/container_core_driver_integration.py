"""Real Core HTTPS API -> MCP -> guard-only Driver local validation client.

TLS and hostname verification stay enabled. A test token may be supplied only
through an environment variable and is never logged or written to state files.
Run inside a dedicated local container, with a private Core resource volume.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import ssl
import sys
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode
from urllib.request import Request, build_opener, HTTPSHandler, ProxyHandler


TOOL = "image_quality_guard"
CAPABILITY = {"schema": "motus.lifecycle/1", "id_parameter": "run_id",
              "required_for": ["start", "stop"]}


def payload(value):
    data = value.get("data", value) if isinstance(value, dict) else value
    if isinstance(data, dict) and "content" in data:
        data = data["content"]
    if isinstance(data, list):
        for item in data:
            if isinstance(item, dict) and item.get("type") == "text":
                result = json.loads(item.get("text", "{}"))
                if isinstance(result, dict):
                    return result
        return {}
    return data if isinstance(data, dict) else {}


class Verification:
    def __init__(self, args):
        self.args = args
        self.results = []
        self.active = None
        self.mcp_id = None
        self.owner = None
        self.token = os.environ.get(args.token_env, "")
        if any(character in self.token for character in "\r\n"):
            raise RuntimeError("test token must contain a single line")
        if not args.core_url.startswith("https://"):
            raise RuntimeError("Core validation requires HTTPS")
        context = ssl.create_default_context(cafile=args.ca)
        assert context.check_hostname and context.verify_mode == ssl.CERT_REQUIRED
        # Internal service traffic must not be sent through a personal proxy.
        self.opener = build_opener(ProxyHandler({}), HTTPSHandler(context=context))

    def check(self, name, condition):
        if not condition:
            raise AssertionError(name)
        self.results.append({"check": name, "result": "PASS"})

    def request(self, method, path, body=None, *, allow_error=False):
        data = None if body is None else json.dumps(body, allow_nan=False).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["Authorization"] = "Bearer " + self.token
        request = Request(self.args.core_url.rstrip("/") + path, data=data,
                          headers=headers, method=method)
        try:
            with self.opener.open(request, timeout=30) as response:
                value = json.load(response)
        except HTTPError as error:
            if not allow_error:
                raise RuntimeError("Core HTTP request failed, status=" + str(error.code)) from None
            try:
                value = json.load(error)
            except ValueError:
                value = {"code": error.code}
        except URLError:
            raise RuntimeError("Core HTTPS connection or verified TLS handshake failed") from None
        if not isinstance(value, dict):
            raise RuntimeError("Core returned an unexpected response shape")
        if not allow_error and value.get("code", 200) != 200:
            raise RuntimeError("Core rejected request: " + str(payload(value).get("code", value.get("code"))))
        return value

    def call(self, action, ticket=None, *, allow_error=False, extra=None):
        args = {"action": action, "instance_id": self.args.instance_id}
        if action == "start":
            args["input_topic"] = self.args.input_topic
        if extra:
            args.update(extra)
        body = {"tool": TOOL, "arguments": args}
        if ticket is not None:
            body["lifecycle"] = {"run_id": ticket["run_id"]}
        return self.request("POST", "/api/mcp/" + quote(self.mcp_id, safe="") + "/call",
                            body, allow_error=allow_error)

    def setup(self):
        health = self.request("GET", "/api/config/status")
        self.check("actual Core HTTPS application responds", health.get("code") == 200)
        state = payload(self.request("GET", "/api/lifecycle/state"))
        self.owner = state.get("owner_epoch")
        self.check("single lifecycle owner is initialized", state.get("enabled") is True and bool(self.owner))
        registered = payload(self.request("POST", "/api/mcp", {
            "name": "image-quality-local-verification", "transport": "http",
            "url": self.args.driver_url, "category": "driver"}))
        self.mcp_id = registered.get("id")
        self.check("real Driver is registered through Core", bool(self.mcp_id))
        ping = payload(self.request("POST", "/api/mcp/" + quote(self.mcp_id, safe="") + "/ping"))
        self.check("Core reaches real Driver MCP", ping.get("online") is True)
        listed = self.request("GET", "/api/mcp/" + quote(self.mcp_id, safe="") + "/tools").get("data")
        tools = listed.get("tools", []) if isinstance(listed, dict) else listed
        if not isinstance(tools, list):
            raise RuntimeError("Core tools endpoint did not return tool definitions")
        definitions = [tool for tool in tools if isinstance(tool, dict) and tool.get("name") == TOOL]
        self.check("Guard tool is unique and discovered", len(definitions) == 1)
        definition = definitions[0]
        self.check("lifecycle capability survives MCP discovery", definition.get("x-lifecycle") == CAPABILITY)
        self.check("system run_id is absent from user configuration",
                   "run_id" not in definition.get("configSchema", {}).get("properties", {}))

    def status(self):
        query = urlencode({"mcp_id": self.mcp_id, "tool": TOOL, "instance_id": self.args.instance_id})
        return payload(self.request("GET", "/api/lifecycle/status?" + query))

    def bind(self):
        actual = self.status()
        bound = payload(self.request("POST", "/api/lifecycle/bind", {
            "mcp_id": self.mcp_id, "tool": TOOL, "instance_id": self.args.instance_id,
            "expected_owner": self.owner, "driver_boot_id": actual.get("driver_boot_id"),
            "expected_epoch": actual.get("admission_epoch")}))
        self.epoch = bound.get("admission_epoch")
        self.check("Core admission is bound and verified", bound.get("bound") is True and bool(self.epoch))

    def reserve(self):
        ticket = payload(self.request("POST", "/api/lifecycle/runs", {
            "scope": "card", "mcp_id": self.mcp_id, "tool": TOOL,
            "instance_id": self.args.instance_id, "expected_owner": self.owner,
            "expected_epoch": self.epoch}))
        self.check("Core issues run ticket", bool(ticket.get("run_id")))
        self.active = ticket
        return ticket

    def info(self):
        return payload(self.call("info"))

    def assert_running(self, ticket, name):
        info = self.info()
        self.check(name, info.get("state") == "running"
                   and info.get("run_id") == ticket["run_id"] and info.get("resources_pending") is True)

    def stop(self, ticket):
        result = payload(self.call("stop", ticket))
        self.check("matching original ticket stops its own run", result.get("state") == "idle")
        if self.active and self.active["run_id"] == ticket["run_id"]:
            self.active = None

    def initial(self):
        self.setup()
        self.bind()
        first = self.reserve()
        self.check("first real Runtime starts", payload(self.call("start", first)).get("state") == "running")
        self.check("same-ticket duplicate start is idempotent",
                   payload(self.call("start", first)).get("state") == "running")
        self.assert_running(first, "first run owns the live Driver resources")
        missing = payload(self.call("stop", allow_error=True))
        self.check("missing run_id stop is rejected", missing.get("code") == "MISSING_RUN_ID")
        self.assert_running(first, "missing stop leaves first run intact")
        forged = payload(self.call("stop", allow_error=True, extra={"run_id": first["run_id"]}))
        self.check("business arguments cannot forge system ticket", forged.get("code") == "SYSTEM_PARAMETER_FORBIDDEN")
        self.stop(first)
        second = self.reserve()
        self.check("same Card and Topic receive a fresh run_id", second["run_id"] != first["run_id"])
        self.check("second Runtime starts", payload(self.call("start", second)).get("state") == "running")
        old = payload(self.call("stop", first, allow_error=True))
        self.check("old stop is terminal or rejected", old.get("code") in {"ALREADY_STOPPED", "STALE_RUN", "RUN_ID_MISMATCH"})
        self.assert_running(second, "old stop cannot close the new same-binding run")
        self.stop(second)
        duplicate = payload(self.call("stop", second))
        self.check("duplicate stop remains idempotent", duplicate.get("code") == "ALREADY_STOPPED")
        self.check("Driver has no remaining owned resources", self.info().get("resources_pending") is False)

    def leave_active(self):
        self.setup()
        self.bind()
        ticket = self.reserve()
        self.check("restart fixture run starts", payload(self.call("start", ticket)).get("state") == "running")
        self.assert_running(ticket, "restart fixture is actually running")
        actual = self.status()
        state = {"owner_epoch": self.owner, "mcp_id": self.mcp_id, "run_id": ticket["run_id"],
                 "instance_id": self.args.instance_id, "driver_boot_id": actual["driver_boot_id"],
                 "driver_url": self.args.driver_url, "input_topic": self.args.input_topic}
        path = Path(self.args.state_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(state, stream, allow_nan=False)
        os.chmod(path, 0o600)
        self.active = None  # Deliberately left running for the Core-kill phase.
        self.check("original fixture ticket is saved only in container state file", path.is_file())

    def recover(self):
        saved = json.loads(Path(self.args.state_file).read_text(encoding="utf-8"))
        self.check("recovery uses the original fixture binding",
                   saved.get("instance_id") == self.args.instance_id
                   and saved.get("driver_url") == self.args.driver_url
                   and saved.get("input_topic") == self.args.input_topic)
        self.setup()
        self.check("Core restart changed owner epoch", saved["owner_epoch"] != self.owner)
        self.check("persistent MCP registration keeps its identity", saved["mcp_id"] == self.mcp_id)
        ticket = {"run_id": saved["run_id"]}
        self.assert_running(ticket, "Driver original run survives Core abnormal exit")
        old = payload(self.call("stop", ticket, allow_error=True))
        self.check("new Core rejects unregistered old ticket", old.get("code") == "STALE_RUN")
        self.assert_running(ticket, "unrecovered stop cannot affect Driver owner")
        actual = self.status()
        self.check("recovery observes the exact original Driver boot and run",
                   actual.get("driver_boot_id") == saved["driver_boot_id"]
                   and actual.get("active_run_id") == ticket["run_id"])
        adopted = payload(self.request("POST", "/api/lifecycle/recover", {
            "confirm": True, "mcp_id": self.mcp_id, "tool": TOOL,
            "instance_id": self.args.instance_id, "expected_owner": self.owner,
            "driver_boot_id": actual["driver_boot_id"], "run_id": ticket["run_id"]}))
        self.check("explicit recovery grants stop-only ownership", adopted.get("recovered_for_stop_only") is True)
        self.active = ticket
        restart = payload(self.call("start", ticket, allow_error=True))
        self.check("recovered ticket cannot restart", restart.get("code") == "RECOVERY_REQUIRED")
        self.stop(ticket)
        self.check("recovered stop releases real Driver resources", self.info().get("resources_pending") is False)

    def cleanup_after_error(self):
        if self.active is None or self.mcp_id is None:
            return
        try:
            result = payload(self.call("stop", self.active, allow_error=True))
            self.results.append({"check": "failure cleanup", "result":
                                 "PASS" if result.get("state") == "idle" else "FAILED",
                                 "code": result.get("code")})
        except Exception:
            self.results.append({"check": "failure cleanup", "result": "FAILED"})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", required=True, choices=("initial", "leave-active", "recover"))
    parser.add_argument("--core-url", default="https://phanthy-motus:15678")
    parser.add_argument("--driver-url", default="http://iqg-driver:15704/mcp")
    parser.add_argument("--ca", default="/work/resource/certs/cert.pem")
    parser.add_argument("--token-env", default="IQG_TEST_ACCESS_TOKEN")
    parser.add_argument("--instance-id", default="iqg-local-card")
    parser.add_argument("--input-topic", default="/iqg_local/camera/color")
    parser.add_argument("--state-file", default="/work/resource/iqg-integration-state.json")
    args = parser.parse_args()
    verifier = None
    try:
        verifier = Verification(args)
        getattr(verifier, args.phase.replace("-", "_"))()
        print(json.dumps({"phase": args.phase, "result": "PASS", "tls_verified": True,
                          "checks_passed": len(verifier.results), "checks": verifier.results}), flush=True)
        return 0
    except Exception as error:
        if verifier is not None:
            verifier.cleanup_after_error()
        # Never dump requests, Authorization headers or environment values.
        print(json.dumps({"phase": args.phase, "result": "FAILED", "error_type": type(error).__name__,
                          "error": str(error)[:300], "checks": verifier.results if verifier else []}), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
