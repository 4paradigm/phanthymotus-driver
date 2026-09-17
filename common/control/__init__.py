"""motus.control/1 — the command path from an execution model to an actuator.

A driver already absorbs one vendor SDK's differences, but until now that work
was only reachable by agent-core, whose input is uniform by construction (text
in, MCP `tools/call` out). An execution model — a VLA policy, a navigation
stack — produces tens of commands per second, which is neither practical nor
appropriate to send through `tools/call`. It needs a data-plane path, and the
data plane had no agreement on it: `control/joint` and `control/velocity`
existed only as format strings in agent-core's topic-inference table, with no
fields, no units, no joint order, and no driver subscribing to either.

This package is that agreement. Two halves:

- `descriptor` — what a driver declares it accepts. The **authoritative**
  definition of the action interface; a URDF is at best a supplement, since it
  carries no units, no control rate, and no statement of whether the driver
  takes absolute or incremental positions.
- `sink` — `ControlSink`, the check chain every command passes before it
  reaches a motor: contract reconciliation, freshness, arbitration between
  sources, clamping, hard limits, force-torque abort, watchdog, and escalation.

Deliberately free of ROS, of any vendor SDK, and of the bundles themselves —
the same reason `common/lifecycle.py` is. Subscribing to a topic and decoding
JSON is the driver's job; what arrives here is a plain dict, so the whole of
the safety logic is testable with a fake clock and a fake `apply` on a laptop.
See tests/test_control_sink.py.

Design notes and the reasoning behind each check:
`phanthymotus/docs/vla-integration.md` § "通用控制接口".
"""

from .descriptor import SCHEMA, Descriptor, Group, parse_descriptor
from .sink import ControlSink, Outcome, Verdict

__all__ = [
    "SCHEMA",
    "Descriptor",
    "Group",
    "parse_descriptor",
    "ControlSink",
    "Outcome",
    "Verdict",
]
