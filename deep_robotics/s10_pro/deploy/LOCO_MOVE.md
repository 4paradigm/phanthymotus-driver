# Unified S10 locomotion card

## Reference and scope

Reviewed upstream main at `07f0e55c95a06a54d588c947c2d25d19c058d61a`:

- `unitree/go2/device.py`: one locomotion tool with `stand_up`, `stand_down`,
  `move` and `stop_move`, plus per-action `x-action-params`.
- `deep_robotics/lynx_m20/device.py`: ROS stand advances state 1 then state 17.

S10 keeps its existing limits, policy scopes, DDS transport, fault checks,
durable tasks, command mutex and ACP delivery. It does not copy the other
robots' speed limits, continuous motion, damping or gait commands.

## Public contract

One `loco_move` actuator exposes these actions:

| Action | Behavior |
| --- | --- |
| stand_up | State 0: request 1, observe its fresh post-send feedback, wait until stationary, then request 17 and observe its fresh post-send feedback. State 1: only perform the second stage. Stationary state 17: succeed without sending a posture or velocity command. |
| stand_down | Request lie and wait for final state 0, not intermediate state 4. |
| move | Existing forward/backward/in-place-turn finite motion with direction, speed and duration. It still requires stationary move mode; it never stands automatically. |
| stop_move | Cancel the active application request. An active finite move sends zero commands and checks stopped feedback through its existing worker. With no active request, it reports no application motion; it does not claim to stop external or vendor motion. |
| info, status, config, start, stop | Existing controller configuration and observation lifecycle. |

`stand_up` needs both existing `stand` and `move_mode` grants. Each stage obtains
its own original-action permit. No new authority is synthesized. The entire
sequence shares one durable request/action ID, mutex and finite deadline
(current production configuration: 15 seconds). Only the final outcome creates
an ACP completion for `loco_move`; intermediate stages never complete the parent
request. A failed, cancelled or interrupted sequence is not resumed or replayed.
A valid terminal request retry returns its stored result. Reported state is not
independent physical-pose verification. State 17 does not prove navigation mode
on the vendor remote, and entering it sends no navigation velocity.

## Topics and saved-canvas migration

The driver advertises `loco_move` and the independent `relative_move` instead of
advertising separate `motion` and `timed_move` tools. `model` remains a resource.
The standard bundle has eight public tools in total.

| Old reference | New reference |
| --- | --- |
| motion / stand | loco_move / stand_up (now includes entering move mode) |
| motion / move_mode | loco_move / stand_up |
| motion / lie | loco_move / stand_down |
| timed_move / implicit tool action | loco_move / action=move, retaining direction/speed/duration |
| motion or timed_move status port | Removed; use loco_move info/status |
| motion or timed_move event port | Removed; action completion uses ACP |

The relative tool name and its status/event topics remain unchanged. Both tools
still share the same physical controller and mutex. Stopping either card can
cancel the shared task, as before.

Before deploying, back up the saved layout and tool settings, refresh the MCP
catalogue, replace the two legacy cards with one loco_move card, and remove links from
the retired status/event ports. The merged card declares no output ports;
info/status queries and ACP completion remain available. Lifecycle/config is shared; conflicting settings
on the two old cards must be resolved deliberately. Existing solutions are not
silently rewritten by this source change, and old public tool calls are rejected.
Retired internal action names remain in old task history for deduplication.
A physical task already executing is not migrated in place; update only idle.

## Validation boundary

Core's canvas submits nonempty inputs even when `x-action-params` hides them.
The merged tool therefore accepts and discards the known `direction`, `speed`
and `duration` fields for non-`move` actions. Both the schema and dispatch follow
this rule; unrelated unknown fields still fail validation. The default action
remains the read-only `status`. Switching actions does not execute movement.

The implementation is tested with synthetic fresh feedback, transport records,
fault/revocation/cancellation cases and real tool dispatch. It has not been
accepted on hardware. The local editor displays the real new schemas but never
creates a robot publisher or grants movement authority. It cannot validate
physical transitions. Deploying the Driver does not require a Core source change.

## Local speed range update

At the owner's request, finite forward/backward movement accepts 0.06-1.00 m/s,
and finite turns accept 0.08-1.00 rad/s. The user-facing speed input maximum is
1.0, with units determined by direction. The selected default remains 0.08;
the internal omitted-turn default remains 0.15 rad/s. Time bounds and repeated
zero-command stopping are unchanged.

The loco_move configuration defaults and shipped example allow max_vx=1.0 and
max_yaw_rate=1.0. Explicit existing site/card settings remain binding: a saved
lower cap is not overwritten or silently bypassed. Parameter validation, backend
limits and DDS permit checks use the same expanded finite-motion bounds.
Relative movement is separate and retains 0.12 m/s translation and 0.20 rad/s
turning caps, including in the DDS adapter.

The other robots' driver ranges informed the interface; this limit is the user's
local development choice, not a manufacturer capability or hardware acceptance
claim. Physical speed acceptance must be recorded separately from deployment.

## Execution mode and diagnostics

Only the protected operator Prepare-policy mode is supported by this public
catalogue. Legacy execution-contract configuration is rejected explicitly.
An unconfigured driver advertises the same tools with control locked. Automatic
Prepare now requires separately approved version-2 startup/renewal permissions;
legacy explicit policies are not auto-activated. Per-stage action permits and
all live telemetry checks remain unchanged. See ../README_AUTOMATIC_PREPARE.md.
A failed stand_up reports failed_stage, failure_phase, reason and blocked_by;
raw exception messages and authorization credentials are not included.
