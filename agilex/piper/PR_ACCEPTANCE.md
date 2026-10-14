# PiPER review candidate acceptance — 2026-10-14

## Current implementation

One Arm control canvas node discovers the background skeleton, English parameters and optionally enabled camera. Public direction actions request a 1 mm base-frame translation at 1% SDK speed with fixed orientation. MOVE_L targets are submitted once, with feedback-confirmed completion. Gripper operations and Stop share cancellable preparation/execution. An explicit guarded teaching exit holds measured joints without reset/homing/torque release. Existing SDK, dependencies and Core renderer remain unchanged.

## Software verification

- Latest PiPER and shared request-logging regression: **212 passed, 195 subtests passed** (`python -m pytest -q tests/test_piper*.py tests/test_vendor_runtime_logging.py`).
- Candidate image: **63 unittest tests passed** offline, with no network or robot device mappings, covering intent control, teaching exit and camera lifecycle.
- Unmodified Core browser harness: one control canvas node, default no-op, six direction selectors, gripper-specific fields, simulated production request/result, PiPER model and skeleton/parameter/image subscriptions; a simulated JPEG renders with expected dimensions. No page script errors.
- This scope retains tests for stale component frames, faults/limits, cancellation and concurrent Start/Stop, target-reference invariance, cached-arrival rejection, diagnostics, namespaces, camera ownership/stalls/recovery and monitor connection-stop races. These are simulations, not physical acceptance claims.

## Physical verification and binding

User-operated tests; the assistant performed read-only state/metadata/CAN checks and deployment only. Before restarts, intelligent control was confirmed off and no task running. Original source/config/Compose/canvas backups were kept. Core and arm-2 image/start-time identities were preserved. No agent-triggered motion/mode/enable/reset was used as a deployment health check.

The first CAN1 candidate repeatedly submitted a MOVE_L target with 0 mm displacement and timed out, later retaining a target-limit report. Changing to one submission produced two completed forward trials. A subsequent backward trial moved [-0.806,-0.012,-0.488] mm and timed out: its target error was **0.525 mm**, exceeding the unchanged 0.5 mm criterion. It is not counted as passed, and the original target-limit trigger is not conclusively established.

After cable reconnection, CAN1 carried two fixed feedback sets under the same IDs at approximately 400 Hz. CAN0 had a single feedback source at approximately 200 Hz. At the user's request, only arm 1 was rebound to the independent CAN0 slave. The camera retained its serial/interface binding; its physical association with this slave was not inferred from its software device label.

On CAN0, firmware query/exit result was S-V1.9-0. Explicit teaching exit completed, then the user executed the following six trials. Every movement task journal recorded one target submission and completion around 1.06–1.07 s. CAN0 feedback was recorded passively through the trials; this receiving socket did not observe outgoing control frames, so independent wire-level submission counts are not claimed.

| Trial | Action | XYZ displacement (mm) | Target error (mm) | Result |
| --- | --- | --- | --- | --- |
| 1 | Forward | +0.949, +0.028, +0.294 | 0.300 | completed |
| 2 | Right | -0.002, -1.020, +0.031 | 0.037 | completed |
| 3 | Forward | +0.814, -0.010, +0.200 | 0.273 | completed |
| 4 | Forward | +0.834, -0.057, +0.218 | 0.280 | completed |
| 5 | Right | +0.026, -1.024, +0.014 | 0.038 | completed |
| 6 | Right | +0.047, -0.997, -0.112 | 0.121 | completed |

Final observed state: fresh feedback, CAN control, six motors enabled, NORMAL and no active task. All displacement/accuracy numbers are controller-derived feedback, not external metrology. The user accepted submitting for review after these tests; this does not establish complete motion acceptance.

The canvas contains only Arm control. Background camera output through Core was checked at approximately 10 fps after deletion of the camera node (and resuming passive acquisition). A five-second stream sample had 50 new frames, no duplicate or offline frames; frames were not saved. The actual browser visit was blocked by tooling, so UI verification on the server relied on user refresh and backend checks; the unmodified browser-rendering check used simulated data.

## Remaining acceptance and known boundaries

- Backward/left/up/down on the current isolated slave, full-path orientation/sideways deviation, real Stop during preparation/motion/gripper with measured latency, automatic standby/all-disabled preparation and gripper actions are not fully accepted.
- The earlier CAN1 backward accuracy failure remains a recorded limitation; do not enlarge tolerances or claim all directions work solely from forward/right trials.
- LLM/user usability, visible gripper default hint, physical skeleton-following confirmation and camera-to-arm association need final verification. SDK/CAN connection alone does not certify movement.
- CAN hardware identity is not persistently bound; duplicate-ID shared-bus feedback must be resolved by topology rather than selecting whichever sample last arrived.
- SDK COS migration remains a separate maintenance item. SDK source and hash stay pinned. Bot clean-build success/platform must be checked separately from the deployed amd64 source overlays.

## Scope and evidence

Robot logic, camera modules, tests and build COPY/component metadata are driver-local. No Agent Core files or new Python/apt dependencies were added in this update. The PR's older common/vendor_runtime.py change only escapes and caps untrusted HTTP request log text; its shared impact is explicitly justified, does not alter MCP/DDS/lifecycle or introduce dependencies, and has regression coverage.

Local evidence directories are acceptance/20261014_piper_single_target, acceptance/20261014_piper_background_camera and acceptance/20261014_piper_can0_binding in the project workspace. Raw CAN logs, robot-specific config, canvas backups and credentials are not committed. Previous stage documents are historical records; this file is the current acceptance summary. Bot Version for this new head is pending until its build table appears.
