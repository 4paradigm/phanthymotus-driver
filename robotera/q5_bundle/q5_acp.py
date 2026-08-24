"""Small ACP completion helper shared by asynchronous Q5 control cards."""

from __future__ import annotations

import json
import os
import ssl
import time
import urllib.request


def notify(action_id: str | None, status: str, result: dict, tool: str) -> None:
    if not action_id:
        return
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    payload = json.dumps({"action_id": action_id, "status": status,
                          "result": result, "tool": tool, "ts": time.time()}).encode()
    request = urllib.request.Request(f"{url}/api/acp/complete", data=payload,
                                     headers={"Content-Type": "application/json"}, method="POST")
    context = ssl._create_unverified_context() if url.startswith("https://") else None
    try:
        urllib.request.urlopen(request, timeout=5, context=context).read()
    except Exception as exc:
        print(f"[Q5 ACP] callback failed for {action_id}: {exc}", flush=True)
