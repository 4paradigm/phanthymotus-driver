# PR title

feat(bumi): add authorized sound-facing and bounded follow control

# PR body

## What / Why

Add a default-off `bumi_sound_facing` actuator Card that faces stable sound directions and reobserves after each owned turn. It reuses the bundle's existing direction source, IMU rotation worker and chassis arbitration. Boot, Canvas start and configuration changes never enable following.

## Implementation

- Independent operator Ed25519 challenge/signature bound to device, process, intent, session, chassis and deadline; normal Core/MCP transports require no Core patch.
- Explicit enable/renew/disable/cancel actions, a maximum 30-second lease and a non-extendable operator deadline. Stable independent capture windows, circular angles, hysteresis, matching local receipts, braking and IMU settling gate successive turns.
- Owner-scoped cancellation, wakeup of revoked source queries, retirement of delayed results and checks after preflight/immediately before nonzero publication.
- Pinned authorization dependencies, isolated ARM64 build/smoke tooling and real-Core API/ACP/Canvas integration tooling with an explicit simulated-bus option.

## Safety boundaries

The independent operator private key stays off Core, Driver and Agent storage/tools. Only a trusted public key is pinned at Driver startup. Missing/unavailable authority fails closed. A normal access token, public session ID, dangerous flag or Canvas label is not a motion grant. Stop requires no approval; expired/revoked/retired sessions cannot resume. Unconfirmed stopping retains chassis ownership.

The Agent must not administer the Driver/key/container/host or the signer. This deployment boundary was not verified or changed here. Software receipts and zero-command publication do not prove physical stopping. No new walking, planning or motor-control loop is introduced.

## Test evidence

- **181 passed**, plus one opt-in real ROS test skipped, in directed Sound Facing tests.
- **697 passed, 4 failed, 1 skipped** in the extended related regression. The same four global upload/Pico/S10-Pro failures reproduce on untouched latest main (**144 passed, 4 failed** in those global files).
- **21 real Core/API/Agent/ACP and native Canvas read-only scenarios passed**, with normal token authentication and real signature verification. ROS transport, LLM decisions, acoustic inputs and motor controller were explicitly synthetic. Core production lifespan stayed off.
- **160 cancellation/wakeup pressure cases passed** with the original timing assertions. Measured software signal timing is not physical braking evidence.

Full current ARM64 build/smoke is **not completed**: this WSL environment has no usable Docker integration/daemon. Matching ARM64 wheels and a standalone source context were checked; these are not an image build. The helper is ready for an already available isolated ARM64-capable engine and never publishes or changes shared builders.

See `SOUND_FACING_TESTING.md` for commands, source hashes and retained failures. Earlier reports of an ARM64 image are historical and are not used to certify this commit.

## Known limitations / Remaining hardware validation

Real ROS/DDS, target SDK startup, microphone mounting/calibration and capture-time semantics, physical yaw direction, feedback loss, stop/revoke/timeout and sustained tracking remain unverified on Bumi. A historical isolated slow-exit observation is retained in review notes; this change fixes the reproduced missing wakeup and adds direct/pressure validation without claiming an exact historic root cause.

This is ready for **Draft review**, with default enablement off. Keep ordinary approval/merge gated on the current ARM64 result, deployment trust boundary and staged hardware acceptance. No real robot or shared production service was operated.
