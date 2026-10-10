"""Best-effort timestamps for G1 locomotion response diagnostics."""
import json
import os
import threading
import time


def trace_event(stage, **data):
    try:
        record = {"stage": stage, "mono_s": time.monotonic(),
                  "pid": os.getpid(), "thread": threading.current_thread().name, **data}
        print(f"[LocoTrace] {json.dumps(record)}", flush=True)
    except Exception:
        pass
