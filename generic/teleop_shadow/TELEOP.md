# Teleop Shadow Wire Contract

This document defines the first deployable teleoperation slice. It is a protocol and diagnostics target, not a robot controller. Every response and capability declaration must continue to report `mode=shadow` and `actuation_enabled=false`.

Both MCP tool descriptors advertise `x-teleop.protocol=motus.teleop.shadow.v1`
and `x-teleop.dispatch_contract=motus.teleop.dispatch.recording.v1`; Core uses
those exact values to select the session and final-dispatch validation contracts.

## Authority and session identity

Agent Core owns session authority. After a passive lifecycle `start`, Core calls:

```json
{
  "action": "prepare_shadow",
  "session_id": "11111111-1111-1111-1111-111111111111",
  "epoch": 1,
  "fence": "replace_with_24_or_more_urlsafe_chars"
}
```

The Driver contributes a process-unique `boot_id`. The resulting identity tuple is:

```text
boot_id + session_id + epoch + fence
```

The epoch must strictly increase during one Driver boot. A restart changes `boot_id`, invalidating all earlier Frames and tickets. The fence is confidential authority material: MCP `status`, `teleop_state`, `/health`, `/offer` answers, and retained public Frame diagnostics never return it.

## Lifecycle and watchdog behavior

| Event | Result |
|---|---|
| `start` | Readiness only; state remains `idle` or the current safe state |
| `prepare_shadow` with newer epoch | `prepared_shadow`; resets latest Pose and transport state |
| First valid deadman/tracking Frame with fresh lease | `active_shadow` |
| Pose older than configured timeout | `hold / pose_timeout` |
| MCP heartbeat lease expires | irreversibly revokes authority, clears identity/Pose/RTC, and reports `hold / lease_timeout`; a new epoch is required |
| deadman false or any required tracking false | `hold` |
| RTC channel/peer disconnect | `hold` |
| Valid Frame with a strictly higher `clutch_sequence` | resumes a recoverable HOLD only when the required transport is fully ready |
| `pause` | `paused`; Frames cannot resume it |
| `soft_stop` | acknowledged `hold / soft_stop`; Frames cannot resume it |
| `release` or lifecycle `stop` | clears Pose and closes RTC; no output is possible |

`release` atomically revokes the session/fence/lease, clears retained Pose, and advances the internal transport generation. Delayed heartbeat, pause, soft-stop, Frame, ticket, and RTC callback traffic from that authority cannot leave `released`; only `prepare_shadow` with a newer epoch/fence can create another session. A paused session likewise cannot be converted back into a recoverable HOLD by a delayed soft-stop.

`soft_stop` retains the Core lease and authority only so Core can observe and
release the held session. It is motion-inhibited and cannot be undone by a Pose
Frame or higher `clutch_sequence`; recovery is release followed by a newer
`prepare_shadow`.

Lease expiry uses the same independent `authority_valid=false` latch; authorization never depends on the mutable display `reason`. Every authority boundary checks its monotonic deadline while holding the runtime lock, so even a heartbeat arriving after the deadline but before the watchdog tick is rejected. The first expiry clears session/fence/lease/Pose and RTC state and advances the internal generation exactly once. Delayed pause, soft-stop, heartbeat, Frame, ticket, and RTC callbacks cannot overwrite the latch. Only a higher-epoch `prepare_shadow` sets `authority_valid=true` again.

All watchdog ages use the server monotonic clock. `client_monotonic_ns` is diagnostic sequencing data and is never trusted as the server watchdog clock.

The Core lease and Pose freshness are independent:

- only an authenticated MCP `heartbeat` renews the Core lease;
- an RTC `heartbeat` message is explicitly rejected;
- `peer_ping`, channel activity, and accepted Pose Frames do not renew the lease;
- Pose arrival updates only Pose freshness.

## Strict Frame v1

Frames are JSON objects with no unknown fields and a maximum canonical JSON size of 64 KiB. Numbers must be finite. `sequence` and `clutch_sequence` are integers in `[0, 9223372036854775807]` (`2^63 - 1`) so recording-v1 values remain representable by Core. Positions are metres in `[-100, 100]`; orientations are normalized `[x,y,z,w]` quaternions; axes/buttons and optional twists are bounded by the validator.

```json
{
  "schema_version": 1,
  "boot_id": "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa",
  "session_id": "11111111-1111-1111-1111-111111111111",
  "epoch": 1,
  "fence": "replace_with_24_or_more_urlsafe_chars",
  "sequence": 42,
  "client_monotonic_ns": 1234567890,
  "mode": "shadow",
  "deadman": true,
  "clutch_sequence": 3,
  "tracking": {
    "head": true,
    "left_controller": true,
    "right_controller": true
  },
  "head": {
    "position": [0.0, 1.7, 0.0],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "left_controller": {
    "position": [-0.3, 1.2, -0.2],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "right_controller": {
    "position": [0.3, 1.2, -0.2],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "controllers": {
    "left": {"axes": [0.0, 0.0], "buttons": [0.0, 1.0]},
    "right": {"axes": [0.0, 0.0], "buttons": [0.0, 1.0]}
  },
  "base_twist": {
    "linear": [0.0, 0.0, 0.0],
    "angular": [0.0, 0.0, 0.0]
  }
}
```

`base_twist` is the only optional field. When a tracking flag is false, its matching Pose must be `null`; when true, that Pose is required. `sequence` must strictly increase per prepared session. The Driver retains only the latest normalized Frame, not an unbounded queue.

`submit_shadow_frame` accepts this same contract through MCP for low-rate diagnostics and replay tests. Production high-rate Pose uses the RTC Pose channel.

## WebRTC transport

The browser wire subcontracts are explicitly versioned as
`motus.teleop.rtc-frame.v1` and `motus.teleop.rtc-control.v1`; the SDP exchange
is `motus.teleop.webrtc-offer-answer.v1`. They are advertised in the capability
digest so a behavior change cannot silently reuse a pinned session contract.

The client creates both channels before generating its offer:

| Label | Contract | Payload |
|---|---|---|
| `teleop-control` | reliable and ordered (`ordered=true`, no retransmit/lifetime limit) | UTF-8 JSON control messages, max 8 KiB |
| `teleop-pose` | unordered, no retransmission (`ordered=false`, `maxRetransmits=0`) | browser-safe RTC Frame v1 UTF-8 JSON, max 64 KiB |

Unknown labels, duplicate labels, or different reliability parameters are closed. The service uses `aiortc` for SDP, ICE, DTLS, SCTP, and data channels.

The RTC wire Frame contains the same motion fields as Frame v1 but must omit
`boot_id`, `session_id`, `epoch`, and `fence`. After verifying the offer ticket,
the Driver atomically captures the active authority and runtime generation and
binds that immutable snapshot to the peer. It rejects client-supplied private
identity fields, injects the peer binding server-side, and then runs the same
strict Frame/runtime/final-dispatch checks as the authenticated MCP diagnostic
path. The complete authority-bearing Frame v1 remains required for
`submit_shadow_frame` over MCP.

An RTC Frame is accepted only when its peer still belongs to the current internal session generation and both required channels are simultaneously open. A Pose-only connection cannot activate a session. If either channel closes, remaining Pose traffic is rejected and cannot reclutch out of HOLD; a complete transport reconnect must restore both channels first. This gate is atomic with Frame acceptance. The explicitly low-rate MCP `submit_shadow_frame` diagnostic path is independent of RTC readiness.

Authenticated Core, never browser JavaScript, sends an offer to the Driver's
loopback `POST /offer` endpoint with the Driver Bearer header:

```json
{"type":"offer","sdp":"v=0...","ticket":"<one-time ticket>"}
```

The response is a standard `answer` plus sanitized binding metadata. It never includes the fence. A ticket is signed, not encrypted, and contains the fence in its claims; production browser flows therefore send only SDP to authenticated Core, which proxies this private request and returns only `{type,sdp}`. The Driver rejects `/offer` without the configured Core Driver Bearer credential.

## One-time offer ticket

Core and the Driver share `MOTUS_TELEOP_TICKET_SECRET` (at least 32 bytes). A ticket is:

```text
base64url(canonical-json-claims) + "." + base64url(HMAC-SHA256)
```

Required claims are `v`, `aud`, `boot_id`, `session_id`, `epoch`, `fence`, `capability_digest`, `sdp_sha256`, `iat`, `exp`, and `jti`. The audience is `teleop-shadow-rtc`; the default maximum lifetime is 30 seconds. Verification requires:

- a valid HMAC;
- an unexpired, bounded lifetime;
- exact active boot/session/epoch/fence/capability binding;
- SHA-256 binding to the exact offer SDP;
- a previously unseen `jti` in the bounded replay cache.

The ticket is consumed before SDP negotiation, so a failed negotiation still requires a new ticket. An absent secret is allowed only for an explicitly registration-disabled, loopback-bound local diagnostic process; that process reports health not ready, omits signaling from its MCP descriptor, and returns fail-closed `503` from `/offer`. Registration, non-loopback binding, or any configured empty/too-short/invalid secret fails process startup.

## Control messages

`peer_ping` and `status` return only state and explicitly report `lease_renewed=false`. A message with `type=heartbeat` returns `rtc_cannot_renew_lease`. `pause`, `soft_stop`, and `release` return `rtc_control_requires_core`; callers must use the authenticated Core REST API, which keeps Core's ownership state and audit trail synchronized with Driver MCP state. Private authority fields are invalid on the RTC control channel.

## Recording final-dispatch boundary

There is deliberately no robot adapter in this package. Instead, a
`RecordingAdapter` makes the complete final-dispatch decision visible as bounded
`would_apply` and `would_stop` records.

Motion and stop do not share a replaceable queue. Motion has a latest-only
mailbox of depth one; stop requests use a non-droppable priority queue and carry
their own acknowledgement. Before every `would_apply`, the serial owner thread
rechecks authority digest, session generation, dispatch generation, server-side
deadline, deadman, and tracking. The fence is consumed by admission and is never
given to the adapter or returned in dispatch state.

Preparing over an active session invalidates the previous generation and waits
for a stop acknowledgement before the new authority is installed. Adapter
failure latches a fault and schedules the independent safe-stop path. Graceful
close revokes authority, waits for stop acknowledgement, closes the adapter, and
rejects later Frames. A new process has a new `boot_id` and performs
`startup_safe` before reporting ready.

Future robot-specific PRs consume the same fenced intent/stop contract behind a
hardware safety policy. They must not weaken this Driver's shadow-only contract
or reinterpret recording state as proof of robot actuation. Physical adapters
also need a downstream command TTL/watchdog so `kill -9` converges to zero even
when the Driver process cannot run its graceful stop path.
