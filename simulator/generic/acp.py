"""Action Completion Protocol callback.

Same wire shape as `x-humanoid/tianyi2.0/device.py`'s `_acp_notify`, with the
transport injectable so the tests can assert what would have been posted without
standing up an Agent Core.

Two rules this module exists to make hard to get wrong:

* **A preempted action reports `cancelled`, never `completed`.** agent-core has
  already released the barrier by the time a cancel lands, so posting nothing at
  all looks harmless — but `/api/acp/complete` also enqueues an `action_complete`
  steering event, which is how the LLM learns the leg ended and how far it got.
  A late `cancelled` is harmless; a late `completed` tells the model it reached a
  waypoint it abandoned.
* **Exactly one post per `action_id`.** Enforced upstream by the world's
  compare-and-set. A duplicate does not fail on the agent-core side —
  `mark_action_complete` simply overwrites `_pending_results` — so it is
  invisible at runtime and only makes the transcript wrong.
"""

from __future__ import annotations

import json
import os
import ssl
import sys
import threading
import time

_transport = None
_observers: list = []
_lock = threading.RLock()


def set_transport(fn) -> None:
    """Replace the HTTP post. ``fn(url, payload_dict)``; ``None`` restores it."""
    global _transport
    with _lock:
        _transport = fn


def add_observer(fn) -> None:
    """``fn(body)`` for every post, successful or not.

    `sim_report` uses this so the run record holds **what was actually sent to
    agent-core**, rather than the world's internal job payload. Those are not the
    same shape and conflating them is a live trap: a job dict has `result` as a
    Slamtec status code (`-1` for cancelled), while an ACP body has `result` as
    the whole payload. `-1 or {}` is truthy, so the confusion surfaces as an
    AttributeError deep inside the oracle rather than as a wrong verdict — but
    only because the code happened to be strict.
    """
    with _lock:
        _observers.append(fn)


def clear_observers() -> None:
    with _lock:
        _observers.clear()


def notify(action_id: str, status: str, result: dict, tool: str = "") -> dict:
    """POST one terminal transition. Never raises — a failed callback must not
    take down the tick thread that produced it."""
    payload = {"action_id": action_id, "status": status, "result": result,
               "tool": tool, "ts": time.time()}
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678") + "/api/acp/complete"
    with _lock:
        transport = _transport
        observers = list(_observers)
    for observe in observers:
        try:
            observe(payload)
        except Exception as exc:
            print(f"[acp] observer failed: {exc}", file=sys.stderr, flush=True)
    try:
        (transport or _post)(url, payload)
    except Exception as exc:
        print(f"[acp] callback failed for {action_id}: {exc}", file=sys.stderr, flush=True)
    return payload


def _post(url: str, payload: dict) -> None:
    import urllib.request

    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    request = urllib.request.Request(url, data=json.dumps(payload).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
    urllib.request.urlopen(request, timeout=5, context=context)
