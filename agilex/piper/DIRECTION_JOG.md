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
