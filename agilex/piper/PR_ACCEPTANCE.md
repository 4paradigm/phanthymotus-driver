# PiPER review candidate acceptance — 2026-10-14

## Current implementation

One Arm control canvas node discovers the background skeleton, English parameters and optionally enabled camera. Public direction actions request a 1 mm base-frame translation at 1% SDK speed with fixed orientation. MOVE_L targets are submitted once, with feedback-confirmed completion. Gripper operations and Stop share cancellable preparation/execution. An explicit guarded teaching exit holds measured joints without reset/homing/torque release. Existing SDK, dependencies and Core renderer remain unchanged.

## Software verification

- Latest PiPER, shared logging, lifecycle and Docker COPY regression after the deployment-gate and production-diagnostic fixes: **297 passed, 241 subtests passed** (`python -m pytest -q tests/test_piper*.py tests/test_vendor_runtime_logging.py tests/test_common_lifecycle.py tests/test_dockerfile_copies.py`).
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

## Review response: deployment motion gate

Production tasks now require the operator's PIPER_MOTION_ENABLED gate before CAN connection/preflight and immediately before teaching exit, preparation and movement/gripper controls. Start never changes it; the public request-control UI remains absent. With the default unset/0 gate, all six directions, all three gripper actions and teaching exit emit no SDK commands. Stop and passive interfaces remain available.

Regressions cover ready CAN, standby/all-disabled and teaching states, default/unset deployment, asynchronous worker failure and gate closure after feedback or preparation. Positive motion fixtures and the simulated Core action backend explicitly use PIPER_MOTION_ENABLED=1 instead of relying on a bypass. This fix has not been redeployed or physically reaccepted: the server was replaced by a separate temporary old-image test deployment. Prior physical trial numbers above precede this repair.

SDK COS migration remains outstanding for the concrete artifact-upload dependency recorded in SDK_ARTIFACT.md; existing commit/hash/URL override are retained. Shared HTTP logging already has a justified scope and requires no additional change.

## Review response: production diagnostic interface

The passive diagnostic now validates the four production tools, reads Arm control Info snapshots before/after, validates the six-axis model and queries camera Info. It no longer calls removed piper_status actions, assumes CAN0, or treats an already-enabled deployment gate as proof of diagnostic motion. Mismatched images fail before tool calls. Disconnected/stale feedback and mode/enable changes fail without automatic recovery. Optional photos explicitly confirm capture; no arm Start/Stop, mode, motor or motion commands are requested.

Six added regression tests exercise the actual production plugin bundle through a fake MCP transport and fake SDK. They cover enabled/disabled deployment gates, obsolete interfaces, disconnection/staleness, feedback changes, optional capture and CLI reports/removal of the obsolete firmware flag. SDK command recordings stay empty. The retained direct_control adapter is documented as a dependency of the historical split_status_card interface and legacy regression fixtures, not installed by production. No deployment or physical action was performed for this correction. COS migration remains paused at the user's request.

## ARM64 update and current Jetson acceptance

The latest software regression is **304 passed, 243 subtests passed**. Native Jetson ARM64 execution passed **225 unittest tests**, plus actual isolated DDS transport with eight skeleton/eight parameter frames, SDK import and JPEG encode/decode. The new tests cover permission revocation during teaching-exit firmware/stability waits and immediately before initial, followup and recovery SDK controls. The runtime permission callback is checked before each control send; no dependency, SDK version or Core change was introduced.

Native ARM64 build helpers support Docker without buildx; cross-platform builds without buildx fail explicitly. The optional offline source-overlay Dockerfile is development-only and does not replace the clean production Dockerfile or a Bot release.

The operator later authorized Jetson Docker deployment and stopping other Jetson containers without deleting them. PiPER runs with independent Core/data, its own DDS domain/namespace and one Arm control card. The target directly connected USB-CAN now provides about 200 Hz status/joint feedback and is configured with a serial-based host binding. Its camera uses the color-interface/serial binding. The Intel deployment's earlier nonpersistent CAN binding and motion trials above are historical; they do not establish this Jetson's physical acceptance.

Live verification inside the new Core received **40 real skeleton, 40 fresh English parameter and 41 valid JPEG frames in four seconds**. Model names, finite measured radians, end pose and gripper feedback passed. Observed robot state was teaching mode, all six motors/gripper already enabled, no reported faults, idle task. Deployment/verification did not send motion, mode, enable/disable or reset commands. Read-only CAN firmware/limit queries are separate from control commands.

The operator-authorized manual deployment gate is enabled. Jetson motion, automatic preparation, software teaching exit, gripper/Stop latency, browser-visible following and physical camera association remain unaccepted. No firmware is asserted as verified for software teaching exit in the deployment config. Reboot/replug binding persistence also remains to be tested. Details and limitations are in JETSON_ARM64.md; raw evidence and host credentials/config remain outside Git. COS migration is still explicitly paused; new Bot Version is pending.

## Older Core retained-field compatibility

The Jetson's unmodified Core submits populated hidden fields when the action changes. After Set gripper opening, even a read-only Info request carrying width_mm=50 reproduced "Unexpected parameters for this action: width_mm". The driver now discards only its declared width_mm field for other Arm control actions; it never uses that stale width to generate a movement/gripper target. Unknown keys still fail, and Set gripper opening retains type/finite/range/default validation. Regressions cover a fixed 1 mm direction target and clean audit request, passive lifecycle/Stop, unknown-key rejection and unchanged teaching-mode motion guards. Core source is unchanged. This correction is not yet deployed; deployment waits until the operator turns off intelligent control.

Local PiPER/shared regression after this correction: **307 passed, 247 subtests passed**. The teaching-exit regression also supplies a retained width and verifies unchanged measured-joint holding, no gripper command and no width in the task audit.
