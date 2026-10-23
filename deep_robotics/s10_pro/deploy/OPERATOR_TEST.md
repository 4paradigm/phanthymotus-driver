# On-site supervised posture test

This is a separate, finite test mode, not vendor control ownership or a verified
autonomous motion contract. It is off unless a protected deployment file is
explicitly mounted. The operator must remain beside the robot with a working
handheld emergency stop and prevent concurrent commands. No velocity, state 17,
state 2, charging command, mode change, SDK or automatic release is provided.

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

The timestamps above are placeholders and are not an active authorization.
The real lifetime must be at most one hour. Changing/removing the file, changing
its permissions, rebooting or expiry revokes the process permanently. Queries
never renew approval. Host boot identity does not claim motion firmware identity.
The normal `S10_EXECUTION_AUTHORIZATION_FILE` and this mode are mutually exclusive.
The canvas cannot provide either authorization.

On the existing motion card:

1. `start` observes MOTION_INFO and HES on the existing independent reader.
2. `prepare_test` explicitly prepares a private DDS command worker. It publishes
   nothing and creates no task; this avoids worker startup consuming request TTL.
3. `stand` sends state 1 once; `lie` sends state 4 once. The accepted response
   contains `request_id` and the durable ACP task identity. A retry of the same
   request never sends again. Requests require fresh feedback and fresh HES=0.
4. After visually confirming the requested physical pose, call `confirm_pose`
   with that `request_id` and `observed_action` (`stand` or `lie`) within 15 seconds.
   Both the explicit visual report and fresh distinct post-send target feedback
   are required before the task reports completed. This is operator-confirmed
   completion, not a vendor acknowledgement or an independent automated proof.

`stopmotion`, timeout, stale feedback, HES or approval loss close our command
worker and report error with `physical_state_unknown`,
`operator_intervention_required`, and `stop_confirmed:false`. Worker closure
cannot retract a published posture command. The operator uses the physical
handheld stop when needed. The process does not reopen a failed worker; a later
reviewed restart is required. Success may leave the idle worker available for
another unique finite posture request in the same authorization session.

No code sends state 17 after state 1. The guide requires standing completion
before that transition, but does not establish that state 1 alone proves the
trajectory has finished. A later RL/velocity feature requires separate work.
Persistent tasks and the ACP outbox are reused; restart never replays a task.
The on-site operator must report their observation; telemetry or an assistant
must never invent that visual confirmation.
