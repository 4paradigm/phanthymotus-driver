"""ACP delivery for pick_place; independent of other actuator cards."""

import json
import os
import ssl
import time
import urllib.request


class Completion:
    ATTEMPTS = 3
    RETRY_DELAY = 0.5
    REQUEST_TIMEOUT = 5

    def __init__(self, tool_name):
        ca_cert = os.environ.get("AGENT_CORE_CA_CERT")
        if not ca_cert:
            raise RuntimeError("AGENT_CORE_CA_CERT is required for action completion")
        # Validate the trust configuration before accepting any motion.
        self.context = ssl.create_default_context(cafile=ca_cert)
        self.url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/") + "/api/acp/complete"
        self.tool_name = tool_name

    def send(self, action_id, status, result):
        payload = json.dumps({"action_id": action_id, "status": status, "result": result,
                              "tool": self.tool_name, "ts": time.time()}).encode()
        error = None
        for attempt in range(self.ATTEMPTS):
            try:
                request = urllib.request.Request(self.url, data=payload, method="POST",
                                                 headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(request, timeout=self.REQUEST_TIMEOUT,
                                            context=self.context) as response:
                    acknowledgement = json.load(response)
                if (not isinstance(acknowledgement, dict)
                        or acknowledgement.get("ok") is not True
                        or acknowledgement.get("action_id") != action_id):
                    raise RuntimeError("Agent Core did not acknowledge this action_id")
                return "accepted", None
            except Exception as exc:
                error = str(exc)
                if attempt + 1 < self.ATTEMPTS:
                    # Also covers completion arriving before Core registers pending.
                    # Only the identical notification is retried, never the motion.
                    time.sleep(self.RETRY_DELAY)
        return "failed", error
