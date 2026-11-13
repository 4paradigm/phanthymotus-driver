"""Run one station photo workflow through the existing authenticated API.

This tests the camera/API/file chain, separately from an LLM following the Skill.
Uses only Python's standard library. No credentials are saved in reports.
"""
import argparse
import datetime as dt
import getpass
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import ssl
import sys
import urllib.error
import urllib.parse
import urllib.request
import uuid


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class API:
    def __init__(self, base, token, insecure=False):
        self.base = base.rstrip("/")
        url = urllib.parse.urlsplit(base)
        if url.scheme not in ("http", "https") or not url.hostname or url.username or url.password or url.query or url.fragment:
            raise ValueError("Invalid base URL")
        if insecure and url.hostname not in ("localhost", "127.0.0.1", "::1"):
            raise ValueError("--insecure is restricted to the local SSH tunnel")
        context = ssl._create_unverified_context() if insecure else ssl.create_default_context()
        self.opener = urllib.request.build_opener(NoRedirect(), urllib.request.HTTPSHandler(context=context))
        self.token = token

    def request(self, path, body=None):
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(self.base + path, data=data,
                                        headers={"Authorization": "Bearer " + self.token,
                                                 "Content-Type": "application/json"})
        with self.opener.open(request, timeout=25) as response:
            return response.read(20 * 1024 * 1024 + 1)

    def json(self, path, body=None):
        return json.loads(self.request(path, body))

    def call(self, mcp_id, action):
        path = "/api/mcp/" + urllib.parse.quote(mcp_id, safe="") + "/call"
        return unwrap(self.json(path, {"tool": "vision_capture", "arguments": {"action": action}}))


def unwrap(result):
    if not isinstance(result, dict) or result.get("code") != 200 or result.get("isError"):
        raise ValueError("Gateway returned an error")
    data = result.get("data")
    if isinstance(data, list):
        texts = [item.get("text") for item in data if isinstance(item, dict) and item.get("type") == "text"]
        if len(texts) != 1:
            raise ValueError("Expected one business result in MCP text content")
        data = json.loads(texts[0])
    elif isinstance(data, str):
        data = json.loads(data)
    if not isinstance(data, dict) or data.get("isError") or data.get("ok") is not True:
        raise ValueError("Camera operation failed: " + str(data.get("error", "ok=true was not returned") if isinstance(data, dict) else "invalid result"))
    return data


def fresh_age(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 2:
        raise ValueError("No confirmed camera frame within 2 seconds")


def discover_mcp(registry, requested=None):
    if not isinstance(registry, dict) or registry.get("code") != 200 or not isinstance(registry.get("data"), list):
        raise ValueError("Cannot read MCP registry")
    matches = []
    for item in registry["data"]:
        if requested and item.get("id") != requested:
            continue
        for tool in item.get("tools", []):
            if tool.get("name") == "vision_capture":
                actions = tool.get("inputSchema", {}).get("properties", {}).get("action", {}).get("enum", [])
                if {"info", "capture_photo"}.issubset(actions):
                    matches.append(item["id"])
    if len(matches) != 1:
        raise ValueError("Expected one vision_capture tool; bind it or choose --mcp-id")
    return matches[0]


def run(api, station, output, mcp_id=None):
    if not station.strip() or len(station.strip()) > 100:
        raise ValueError("Station name must contain 1 to 100 characters")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).strftime("%Y%m%d-%H%M%S")
    label = re.sub(r"[^\w-]+", "_", station.strip(), flags=re.UNICODE)[:30]
    stem = stamp + "-" + label + "-" + uuid.uuid4().hex[:8]
    report_path = output / (stem + ".json")
    report = {"station": station.strip(), "status": "failed",
              "recorded_at": dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(),
              "test_scope": "camera_api_file_chain; LLM skill execution is a separate acceptance step"}
    capture_submitted = False
    try:
        mcp_id = discover_mcp(api.json("/api/mcp"), mcp_id)
        report["mcp_id"] = mcp_id
        info = api.call(mcp_id, "info")
        fresh_age(info.get("latest_frame_age_s"))
        if info.get("active_recording") is not None:
            raise ValueError("Camera is recording; finish that task before photo inspection")
        capture_submitted = True
        photo = api.call(mcp_id, "capture_photo")
        remote_path = photo.get("file_path")
        if photo.get("media_type") != "photo" or not isinstance(remote_path, str) or not remote_path.strip():
            raise ValueError("Camera did not return a photo path")
        path = PurePosixPath(remote_path)
        if not path.is_absolute() or ".." in path.parts or path.suffix.lower() not in (".jpg", ".jpeg"):
            raise ValueError("Camera did not return an absolute JPEG path")
        report.update(remote_file_path=remote_path, captured_at_raw=photo.get("captured_at"),
                      camera_time_zone="not declared by camera", frame_age_s=photo.get("frame_age_s"))
        fresh_age(photo.get("frame_age_s"))
        content = api.request("/api/file/download?" + urllib.parse.urlencode({"path": remote_path}))
        if not 4 <= len(content) <= 20 * 1024 * 1024 or not content.startswith(b"\xff\xd8\xff") or not content.endswith(b"\xff\xd9"):
            raise ValueError("Downloaded file is not a complete JPEG")
        local = output / (stem + ".jpg")
        local.write_bytes(content)
        report.update(status="success", local_file_path=str(local.resolve()), bytes=len(content),
                      sha256=hashlib.sha256(content).hexdigest(), file_verified=True)
    except (OSError, ValueError, KeyError, TypeError, urllib.error.URLError) as error:
        report.update(error=str(error), capture_may_have_completed=capture_submitted)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report, report_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--station", required=True)
    parser.add_argument("--base-url", default="https://127.0.0.1:15678")
    parser.add_argument("--mcp-id")
    parser.add_argument("--out", type=Path, default=Path(__file__).with_name("records"))
    parser.add_argument("--insecure", action="store_true", help="Self-signed HTTPS over the local SSH tunnel")
    args = parser.parse_args()
    token = os.environ.get("PHANTHY_ACCESS_TOKEN") or getpass.getpass("Dashboard Access Token: ")
    try:
        report, path = run(API(args.base_url, token, args.insecure), args.station, args.out, args.mcp_id)
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("Report: " + str(path.resolve()))
    return 0 if report["status"] == "success" else 1


if __name__ == "__main__":
    sys.exit(main())
