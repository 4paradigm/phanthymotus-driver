# PiPER direction micro-adjustment — arm-1 test candidate, 2026-10-14

Users choose a direction and execute once. Each request moves the gripper's end-effector reference point by 1 mm, at 1% SDK speed, while preserving orientation. SDK speed percent does not represent a fixed mm/s. No distance, speed, joint, mode, or permission entry is needed for direction actions.

| Action | Card label | Robot base direction |
| --- | --- | --- |
| `move_forward` | Move forward · 1 mm | +X |
| `move_backward` | Move backward · 1 mm | −X |
| `move_left` | Move left · 1 mm | +Y |
| `move_right` | Move right · 1 mm | −Y |
| `move_up` | Raise · 1 mm | +Z |
| `move_down` | Lower · 1 mm | −Z |

Directions are fixed to the robot base, independent of the camera and observer. Default `select_action` sends no control command. The old `move` request and its distance/speed fields are rejected with an interface-update message, so an old caller cannot silently execute a different operation. Existing Stop, gripper actions, explicit teaching exit, lifecycle, Info, and on-demand camera remain available. Only set-gripper-opening displays a numeric field. Card-owned text and LLM action descriptions remain English; Core-owned buttons retain their current labels.

## Motion and readiness

The pinned SDK and MOVE_L remain in use. The controller target is calculated from one original measured end pose. The internal helper can subdivide 1–10 mm into targets at most 1 mm apart, referenced to that original pose; completion of each step is required before submitting the next. Public actions expose only 1 mm. A larger-step UI remains deferred until real-arm acceptance.

Each step requires fresh joint, end-pose component, and controller feedback. Success requires post-command feedback, reached status, position tolerance 0.5 mm, orientation tolerance 1 degree, and stability for 0.5 seconds. The whole operation has a 30-second motion deadline. Completion reports measured displacement as well as requested distance. Subdivision is not an IK solver and does not guarantee that a direction is reachable.

Healthy standby (including fully enabled standby) or all-disabled feedback uses existing automatic preparation at 1% speed. Already-ready CAN control does not repeat preparation. Teaching, partial enable, device faults, and stale feedback retain their existing rejection rules. Stop and each submission share the existing send lock/cancellation mechanism; no next step is submitted after cancellation.

## Rejection and recovery

The observed issue motivating this change was a successful teaching exit followed by MOVE_L target rejection. Current joint angles within their configured limits do not establish Cartesian target reachability.

Target-limit, no-solution, and singularity reports with no device error bits are classified as `direction_unavailable`, separately from motor/device faults. The message identifies the requested direction when a new target was submitted and says movement may be partial. Pre-existing target rejection is reported as still active. A reached flag never overrides a controller error.

The driver stops subsequent goals when any rejection occurs. It never ignores the error, resets the arm, releases torque, changes orientation, homes, or chooses another path automatically. A best-effort measured hold is attempted only if fresh feedback is healthy and still permits control. If a target error remains active, subsequent arm and gripper tasks remain blocked; Info remains available. Recovery requires the robot to report a healthy state externally before a new request can proceed. Automatic error clearing or gripper operation during a latched target error is **not claimed as supported** and needs a separately verified firmware procedure.

## Diagnostics

Each validated task records its request, preflight state, original start/target, per-step targets, confirmed steps, command count, and actual feedback. Target errors retain the first rejected feedback separately from later observation. Details are available in `arm_control(action=info).task.diagnostics`, and through the existing English parameter topic's Task field.

A per-device journal is written to `<data_dir>/arm-control-events.jsonl`, rotated at 2 MiB with one backup (`.jsonl.1`). It contains validated action arguments and device feedback, never environment variables or tokens. Request and target records must be saved before sending controls. A storage failure at that point fails closed. A terminal-record failure leaves the actual result intact and adds a diagnostic warning. The journal survives driver restarts; in-memory task state starts idle on restart.

Core's read-only topic discovery sometimes supplies `input_topic=""` on Info. The driver accepts this empty framework value only for Info. Unexpected fields on movement remain rejected. Core code and renderer are unchanged. The control card still declares skeleton/parameter topics, with the existing background model and 10 Hz feedback service.

## Validation and deployment boundary

Simulation covers all direction mappings, fixed defaults, old-interface rejection, numeric gripper validation, original-reference subdivision, second-step rejection, partial displacement, first-error recording, cancellation between steps, persistent target logging before transmission, log-write failure, standby preparation, external recovery gating, and existing lifecycle/monitor regressions.

`tests/piper_direction_browser.mjs` serves the unchanged Core card and monitor against the production driver contract with recording fake hardware. It checks default no-op, six parameter-free direction selections, action-specific gripper fields, actual request payload, simulated production completion, task-parameter update, PiPER model loading, and skeleton/parameter subscriptions. It never connects to the robot or server.

This candidate is deployed to arm 1 for user testing and is not physically accepted. Local regression: **203 passed, 195 subtests passed**. The deployed candidate image also passed **45** related unittest tests offline with no hardware/device mapping or network. The unchanged Core card/monitor browser check passed. For subsequent arm-1 deployments: re-read state and intelligent-control status, preserve the live CAN1/camera-2 binding and camera worker, back up the current image/config/canvas, migrate only arm 1, and verify Core/arm 2 are unchanged. Do not use deployment checks to send motion/reset/mode/enable commands. Actual direction/reference point, held orientation, limited-direction handling and Stop require user-arranged physical acceptance. The existing target error will not be cleared by deploying this code.

Common agreements remain: no Core modifications, no arm-2 operation, preserve old implementations/backups and the original untracked `smoke-test.py`, no unrelated SDK migration, and user testing before PR push. Intended integration is PR #382; this candidate's Bot Version has not been generated.

## 2026-10-14 deployment record

- Image: `agilex-piper:local-arm1-direction-b38149e6`.
- Previous image: `agilex-piper:local-arm1-teaching-hold-ef3d4022`.
- Backup: `/opt/phanthy-motus/backups/piper-arm1-direction-20261014-124447/`.
- Intelligent control was freshly confirmed off and arm 1 idle before restart. CAN1/camera-2 binding and configuration hash, camera worker hash, and Core/arm-2 image and start-time identities were checked unchanged.
- Post-deployment read-only verification confirmed the six direction actions, removed legacy fields, fresh progressing feedback, PiPER model, and camera publication at 10 fps. No physical action was invoked.
- The controller still reports TARGET_POS_EXCEEDS_LIMIT. Deployment did not clear that error; motion/gripper tasks remain blocked until healthy feedback returns.
- Evidence: workspace `acceptance/20261014_piper_direction/`, including browser result/screenshot, simulated rejection/journal, full Python test log, deployment helper and live read-only verification.
- PR #382 has not been pushed/updated for this candidate, pending user testing. Bot Version: 尚未生成.

## 2026-10-14 manual Start/Stop compatibility fix

The real Core sends `start` before every manual Execute, including Stop. Previously, the inherited Start rejected any task-owning gate as "An operation is still stopping", even when it was merely running. This obscured the busy state and could prevent the manual Stop action from reaching the driver.

Arm control's Start now returns a harmless `busy` or `stopping` acknowledgment while it owns a task. It does not clear cancellation, reopen stopped readiness flags, release the gate, send a control, or queue an action. The following movement request is still rejected as busy; the following Stop can reach its cancellation path. After the worker releases its gate, a later Start can ready the card normally. Other tools and the historical runtime are unchanged. Journals now explicitly include the action in the request object even when the framework has removed it from argument fields.

Tests reproduce actual Core sequences: Start → action → Start → second action; Start → action → Start → Stop; and Start while a stopped-card worker drains. Full regression: **206 passed, 195 subtests passed**; offline candidate **48 unittest tests passed**; unchanged Core card/monitor browser verification passed.

Deployed arm-1 image `agilex-piper:local-arm1-lifecycle-ea016db5`; backup `/opt/phanthy-motus/backups/piper-arm1-lifecycle-20261014-125747/`. Freshly checked intelligent control off and no active task before restart. Core/arm 2/config/camera worker identities remain unchanged. Post-restart feedback and camera verified read-only; no real control commands executed.

The original forward request did **not** succeed: recorded start `[53.907, 2.765, 220.173]` mm, target `[54.907, 2.765, 220.173]` mm, measured displacement 0 mm, timeout at 30 seconds. The driver then attempted its existing healthy-current-pose hold; later feedback again reported target-limit rejection. These observations do not establish why MOVE_L did not move or prove the hold caused the error. This compatibility fix does not change actuation or claim physical movement acceptance. The active target-limit report continues to block physical tasks.

Evidence: workspace `acceptance/20261014_piper_lifecycle/`, including the original task snapshot, test log, browser result, deployment helper, and post-deployment read-only verification. PR #382 still waits for user testing; this candidate's Bot Version remains 尚未生成.

## 2026-10-14 repeated MOVE_L submission investigation (local candidate)

The forward task submitted the same target 589 times over 30 seconds, with 0 mm observed displacement. Later requests fail in preflight against a retained `TARGET_POS_EXCEEDS_LIMIT` report; they do not demonstrate that each newly selected direction is unreachable. The timeout snapshot was still NORMAL and a measured Cartesian hold was attempted before the subsequent target-limit report. The original trigger of that report is not yet established.

Read-only controller queries on CAN1 returned joint limits (degrees): [-150,150], [0,180], [-170,0], [-100,100], [-70,70], [-180,180]. All six maximum joint speeds are 0.2 rad/s. End maxima are 0.5 m/s, 0.13 rad/s, 0.8 m/s², and 0.2 rad/s². No parameter was changed. CAN status remained `0104020200000000`. Joint feedback [0.555,5.128,-8.791,3.305,25.304,-11.388] degrees is within both deployed and reported controller limits. Pinned official FK with its 2-degree offset reproduces reported end position [53.907,2.765,220.173] mm to rounding precision; this does not validate firmware IK or physical clearance.

[The manufacturer's MOVE_L discussion](https://github.com/agilexrobotics/pyAgxArm/discussions/65) explicitly recommends one target submission and warns that periodic ModeCtrl/EndPoseCtrl submissions can cause replanning. This is guidance for a related firmware/model, not proof of the cause on this S-V1.9-0 device. The local candidate now submits each MOVE_L step once and continues cancellation/fault/freshness/completion polling at 20 Hz. The existing 30-second timeout, best-effort measured hold, and rejection protections remain. No reset, limit expansion, SDK migration, or control-mode workaround was added.

New tests cover a controller whose planning delay restarts on every target and an unmoving controller that must not receive a stream of repeated goals. Hardware verification is still required to determine whether this resolves the zero-motion symptom and whether a separate target-rejection issue remains. The candidate is local only; deploying it cannot clear the current controller report. PR #382 has not been updated; Bot Version: 尚未生成.

### Single-target candidate deployment

Deployed to arm 1 after the user disabled intelligent control and read-only preflight confirmed it off, connected/fresh feedback and no running task. Image: `agilex-piper:local-arm1-single-target-396956ee`; previous image: `agilex-piper:local-arm1-lifecycle-ea016db5`. Backup: `/opt/phanthy-motus/backups/piper-arm1-single-target-20261014-131820/`. Only `intent_motion.py` was overlaid on the deployed image. Original and deployed source hashes were checked.

Local regression: 208 passed, 195 subtests passed. The candidate image passed 50 unittest tests without hardware mapping or network. Post-deployment read-only checks confirmed progressing fresh joint/end feedback, model, and camera output. Core/arm-2 image and start time, live configuration, and camera-worker hashes remained unchanged. No physical control/reset command was invoked during deployment.

The controller still reports `TARGET_POS_EXCEEDS_LIMIT(0x4)`, so movement acceptance remains blocked until the physical controller reports normal externally. Driver task state `idle` after restart does not imply that this controller error is cleared. Evidence: workspace `acceptance/20261014_piper_single_target/`. User testing is pending; PR #382 not updated, Bot Version 尚未生成.
