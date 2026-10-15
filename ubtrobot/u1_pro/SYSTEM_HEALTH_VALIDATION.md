# system_health validation

## Reproduce local checks

Run from the driver repository root with Python 3.12+, PyYAML, NumPy, Pillow,
and pytest available. The card itself adds no runtime dependencies.

```bash
PYTHONPATH=ubtrobot/u1_pro python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 -m pytest tests/test_dockerfile_copies.py -k u1_pro -q
python3 scripts/check_service_yml.py ubtrobot/u1_pro
PYTHONPYCACHEPREFIX=/tmp/u1-health-pycache python3 -m compileall -q ubtrobot/u1_pro
bash -n build.sh
git diff --check
```

Local results: **104 unit tests passed**, including 32 health tests;
**2 Dockerfile COPY checks passed**. Service validation reports **0 failures
and 1 existing CycloneDDS known gap**. Compilation, shell syntax, and whitespace
checks pass. The service check does not establish DDS connectivity.

Tests use synthetic Linux files and ROS/vendor substitutes, covering:

- Metrics and thresholds, unreadable data, thermal/hwmon labels, cgroup v1/v2
  membership and ancestor pressure, hidden limits, and container attribution.
- Existing camera frame timestamps, restart freshness, stale/error states,
  and the shared playback queue. An isolated camera or playback error makes
  `complete` false even when every other observation is normal.
- Immediate first sampling and a real 1-second cadence without queries,
  cache-only queries and copied results, stale cache, and recovery after errors.
- Idempotent start, stop/join and restart, blocked-read cancellation and timeout,
  preventing overlapping collectors, and worker-start failure cleanup.
- Card registration/configuration and MCP initialize/list/call single wrapping.
  Health sampling and queries do not call robot methods or create device resources.

## Hardware verification boundary

Local tests do **not** prove U1 Pro hardware health or deployed behavior.
The PR review bot builds the ARM64 image; match its result to the current PR head.
No live driver entry point, deployment, service restart, or robot operation is
part of these local checks, because the existing driver startup performs SDK
initialization and changes firmware behavior switches.

Before accepting hardware behavior, verify the image on the target U1 Pro,
Agent Core registration and DDS connectivity, the actual container/host `/proc`,
cgroup and filesystem views, thermal sensor identity/permissions and applicable
thresholds. Compare observed values with board-local readings. Use already
running camera streams and existing playback tasks to compare freshness and
completion, and confirm that health sampling itself starts no device or behavior.
If a device must be started to produce test data, obtain robot-operation
authorization separately. Hidden limits and unidentified sensors remain unknown.
