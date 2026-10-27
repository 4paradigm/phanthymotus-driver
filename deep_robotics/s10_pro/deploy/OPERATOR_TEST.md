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
request. A failed worker remains closed until the driver is restarted; a
restart never replays that failed request. Successful
state observation leaves the idle worker available for another unique request.
No independent physical-completion evidence is invented.

`start` only observes MOTION_INFO/HES on the existing separate reader.
`prepare_test` is an optional command-channel initialization operation that
publishes nothing and creates no task; deployments prepare it in advance.
Execute also prepares the worker when permitted, subject to the original
request TTL. No extra user confirmation is added.

## Persistent application permission

The explicitly requested persistent authorization uses
`s10_pro.operator_authorization/2`, `lifetime:"until_revoked"`, and
`expires_at_ms:null`. It has no one-hour or other automatic expiry. An unchanged
valid grant remains applicable after a driver restart or host reboot on the
same NOS machine; restarting does not request another confirmation.
V2 `issued_at_ms` is an audit record, not a wall-clock validity gate. An RTC
reset or clock rollback after power loss does not require clock correction or
another authorization. Private permits use the current process monotonic clock
and host boot; each action deadline remains finite and at most 15 seconds.

This is application permission for the listed finite posture requests. It is
not a manufacturer control-ownership lease, proof of exclusive robot control,
or a verified physical-stop guarantee. The existing scope remains `stand` and
`lie`; indefinite permission does not enable velocity, state 17, mode switching
or charging. Each request still has its own finite deadline and must pass the
current independent feedback/HES checks. No new confirmation action is added.

`S10_OPERATOR_AUTHORIZATION_FILE` selects an absolute root-owned regular file,
not a symlink or group/world-writable file, mounted read-only. The exact V2
manifest shape is:

```json
{
  "schema": "s10_pro.operator_authorization/2",
  "execution_authorized": true,
  "session_id": "unique-operator-authorization",
  "robot_identity": "<SHA256 of s10-nos-machine: followed by this NOS machine ID>",
  "allowed_actions": ["stand", "lie"],
  "issued_at_ms": 0,
  "expires_at_ms": null,
  "operator_present": true,
  "handheld_estop_verified": true,
  "no_competing_commands": true,
  "provenance": "onsite_operator_confirmation",
  "lifetime": "until_revoked"
}
```

The identity and issue time above are placeholders, not an active grant. The
identity is the SHA256 of `s10-nos-machine:` concatenated with the 32 lowercase
hexadecimal characters from the NOS host's `/etc/machine-id`; its trailing
newline is removed. Keep the raw machine ID on the host. Do not publish it or
substitute the container's own machine ID.

Merge these entries into the existing driver's volumes/environment; preserve
its state, ACP trust, configuration and DDS mounts:

```yaml
volumes:
  - /etc/machine-id:/run/s10/robot-machine-id:ro
  - /userdata/phanthy-motus-s10/operator-authorization.json:/run/s10/operator-authorization.json:ro
environment:
  - S10_OPERATOR_AUTHORIZATION_FILE=/run/s10/operator-authorization.json
  - S10_OPERATOR_MACHINE_ID_FILE=/run/s10/robot-machine-id
```

Both source files must already exist and be root-owned regular files without
group/world write permissions. The machine-identity path defaults to
`/run/s10/robot-machine-id`; the environment variable makes this mounting
explicit. V2 checks the unchanged grant contents, file permissions and stable
machine identity on subsequent queries. Its private command permits still use
the current host boot and a finite per-action deadline; they are not renewed
vendor leases.

Persistent permission never resumes or replays a previous physical action.
Keep `motion-tasks.sqlite3` and `acp-outbox.sqlite3` across restarts. An
unfinished task is recovered as an error with stopping unconfirmed, and an
existing request ID returns its original result. Booting the machine or
restarting the driver does not automatically issue `stand` or `lie`.

## Revocation

To revoke a running deployment, edit the already mounted host grant **in
place**, setting `execution_authorized` to `false` while retaining its protected
ownership and permissions. The contents then differ from the bound grant, so
the running process permanently rejects it. Restoring the old bytes does not
reactivate that process. An explicit application-level `revoke()` also latches
revocation for that process.

For removal, stop the driver, remove the authorization environment/mount and
host grant, then recreate the service. A Docker bind mount of an individual
file can retain the old inode after a host-side rename, replacement or unlink;
do not rely on deleting/replacing that host pathname alone to revoke the
already mounted copy. Revocation, file-trust failure or machine-identity change
rejects this application's new commands. An active task that detects the loss
closes its command worker and reports error; neither outcome is a physical
robot stop. The on-site handheld remains the physical intervention mechanism.

Normal `S10_EXECUTION_AUTHORIZATION_FILE` and this mode are mutually exclusive;
the canvas cannot grant or extend authorization. Machine identity does not
claim motion firmware identity or vendor control ownership.

## Legacy bounded authorization

V1 remains supported for existing deployments with an explicit maximum
one-hour expiry and a binding to the current host boot:

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

The timestamps above are placeholders, not an active authorization. V1
requires integer issue/expiry milliseconds and a duration no greater than
3,600,000 ms. Expiry or a changed host boot invalidates it; queries do not
renew approval. Adding `lifetime` or setting a V1 expiry to `null` does not
convert it into V2. File revocation and durable-task rules above apply to both
versions. V1 remains bounded for compatibility; a V2 `until_revoked` grant has
no automatic time limit.
