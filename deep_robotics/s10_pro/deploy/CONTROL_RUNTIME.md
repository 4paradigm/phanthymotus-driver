# S10 deployment authorization and per-action machine contract

This is an offline implementation seam. No verified authority source or posture
interruption implementation for the current robot is included. Defaults remain
read-only. Connecting a handheld, releasing HES or receiving MOTION_INFO cannot
complete those integrations.

`main.py` loads `S10_EXECUTION_AUTHORIZATION_FILE` once at process construction.
Without it, all sensor cards remain available and command execution stays off.
The provided `control-authorization.disabled.json` is a safe disabled example.
Do not change a canvas setting to grant deployment authorization.

An active authorization file must be an owner-root regular file, not a symlink,
not group/world writable, and mounted read-only in the driver. It has exactly:

| Field | Required meaning |
| --- | --- |
| schema | `s10_pro.execution_authorization/1` |
| execution_authorized | literal `true` only after deployment review |
| adapter | locally installed trusted `python.module:factory` |
| robot_identity / firmware_identity | verified identities of this exact machine |
| contract_sha256 | canonical fingerprint of the complete original adapter contract |
| allowed_actions | explicit subset of `stand`, `lie`, `velocity` |
| issued_at_ms / expires_at_ms | integer Unix milliseconds, at most four hours apart |

No active example or fabricated machine grant is shipped. Use
`control_runtime.contract_fingerprint()` only on a reviewed actual contract.
Authorization must still be current after adapter startup and authority reads.
Changes, removal, invalid permissions or expiry revoke it for the process;
restoring the old file does not resume execution. A monotonic deadline prevents
wall-clock rollback from extending the initial approval. A restart constructs a
new session; it never resumes/replays a physical task.

The trusted factory receives a copy of the deployment config and returns:

- `contract`: `s10_pro.control_contract/2`, exact identities, common evidence and
  independently verified `actions` entries.
- `authority_provider`: a **passive cache read** of an observation already
  received from a verified ownership/mode source. Do not acquire ownership,
  block on I/O, call a controller, synthesize a lease, or stamp the query time.
- optional `posture_stop_factory`: a separate locally installed factory reference
  returning an object with `send` and `feedback`. Production runs this object in
  `BoundedPostureProcess`, started only during explicit action preparation.
- optional `close`: passive bounded cleanup of the authority adapter.

Factory imports are trusted executable code, not a sandbox. The framework
cannot guarantee arbitrary Python import/cleanup side effects or a hard timeout
for a cache callback. Review those modules before selecting them. No factory is
loaded from tool arguments or a canvas configuration.

An authority observation includes exact identities, allowed actions, owned/fresh
booleans, **original** `received_monotonic`, original `expires_monotonic`, and
`age_s`. It expires within 0.5 seconds of receipt. Querying only increases age;
it never renews receipt or expiry. `regular_mode_confirmed` and
`navigation_mode_confirmed` come from separately verified live sources. The
backend checks mode, identity, action scope and deployment approval on every
send. DDS command RPC timeout uses the strictest verified allowed-action send
budget, up to 0.2 seconds.

Posture action entries require common identity/firmware/authority/feedback
proofs plus `mode: regular`, `interruption_verified: true`, specific stop and
loss-behavior evidence, `stop_semantics: verified_posture_adapter`,
`max_uncontrolled_s` in 0.1–1.0, and `send_timeout_s` in 0.01–0.2 (less than half
the uncontrolled bound). Velocity independently requires `mode: navigation`,
physical `units`, a verified robot deadman in 0.1–1.0, and
`zero_velocity_until_feedback`. V2 rejects sending over lost authority;
`stop_on_authority_loss` must be false. These ranges are application policy,
not manufacturer measurements.

A verified stand policy does not require a verified velocity policy. Conversely,
stand/lie remain unavailable if either the send or feedback interruption
adapter is absent. The backend never guesses a state-2 cancel or uses NAV_CMD
zero speed to claim posture interruption. Interruption feedback must contain
matching request identity and two fresh distinct observations **after** sending;
a receipt or a zero measured speed is insufficient. The private posture process
has bounded send/feedback calls, is killed on timeout/cancellation, and is never
automatically restarted. Timeout remains stop-unconfirmed, not success.

`info`, `status` and card `start` never start command workers. Explicit execute
prepares the worker only after authorization, action contract, mode, authority
and feedback checks. Initialization creates no command publisher; another
check precedes task acceptance and every send. Prepare failure creates no task
or ACP completion; request TTL also applies after preparation. The current
preparation phase holds the plugin lock, bounded by approximately five seconds;
concurrent status/config/stop may wait before any task has been accepted.

Preserve `state/motion-tasks.sqlite3`, `state/acp-outbox.sqlite3`, TLS validation,
the separate motion reader and all existing speed/duration/feedback limits.
Real ownership, firmware compatibility, posture interruption and velocity
loss-of-command behavior still require machine/vendor evidence and subsequent
physical acceptance.
