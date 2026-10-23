# On-site posture requests with automatic state observation

This finite request mode is separate from vendor ownership and from a verified
autonomous motion contract. It is off unless a protected deployment file is
explicitly mounted. The existing onsite supervision session applies; the
operator has a working handheld stop and excludes concurrent commands.

The motion card needs one action click: `stand` or `lie`. There is no manual
pose confirmation or subsequent confirmation action, and no request ID needs
to be copied. If omitted, the request ID is generated internally. Identical
request IDs remain durable retry identities and never send a posture twice.

`stand` publishes state 1 once; `lie` publishes state 4 once. The task then
automatically observes fresh, distinct post-send target-state feedback. ACP
completion covers **sending the request and observing the reported target
state** (`completion_level: reported_motion_state`). It does not assert that
the trajectory has physically finished or the robot has stood/laid steadily.
The result explicitly records `physical_pose_completion_verified:false` and
`physical_state_unknown:true`. Publishing alone, cached data, a repeated
sequence or elapsed time cannot complete the observation task. No automatic
state 17, state 2, NAV_CMD, charging command, mode change or SDK is provided.

The existing 15-second setting bounds automatic reported-state observation;
it is not an operator response window or a robot-side stopping guarantee.
Cancellation, timeout, HES, stale feedback or authorization loss close our
worker and report error with `operator_intervention_required:true` and
`stop_confirmed:false`. Closing the worker cannot retract a published posture
request. A failed worker remains closed until a reviewed restart. Successful
state observation leaves the idle worker available for another unique request.
No independent physical-completion evidence is invented.

`start` only observes MOTION_INFO/HES on the existing separate reader.
`prepare_test` is an optional command-channel initialization operation that
publishes nothing and creates no task; deployments prepare it in advance.
Execute also prepares the worker when permitted, subject to the original
request TTL. No extra user confirmation is added.

`S10_OPERATOR_AUTHORIZATION_FILE` selects an absolute root-owned regular file,
not a symlink or group/world-writable file, mounted read-only. Exact schema:

```json
{
  "schema": "s10_pro.operator_authorization/1",
  "execution_authorized": true,
  "session_id": "unique-reviewed-operator-session",
  "robot_session": "SHA256 of s10-nos-boot: followed by the current host boot ID",
  "allowed_actions": ["stand", "lie"],
  "issued_at_ms": 0,
  "expires_at_ms": 1,
  "operator_present": true,
  "handheld_estop_verified": true,
  "no_competing_commands": true,
  "provenance": "onsite_operator_confirmation"
}
```

The timestamps above are placeholders, not an active authorization. A real
session lasts at most one hour. Changing/removing the file, changing its
permissions, rebooting or expiry revokes the process permanently. Queries do
not renew approval. Host boot identity does not claim motion firmware identity.
Normal `S10_EXECUTION_AUTHORIZATION_FILE` and this mode are mutually exclusive;
the canvas cannot grant authorization. Persistent tasks/outbox are preserved,
and restart never replays a physical request or rewrites its terminal result.
