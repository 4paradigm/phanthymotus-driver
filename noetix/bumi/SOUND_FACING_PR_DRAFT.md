# PR title

feat(bumi): add signed continuous sound-facing sessions

# PR description

## Summary

Add a default-off `bumi_sound_facing` Card that follows changing sound directions
using the existing Bumi direction source and IMU rotation worker. Stable, fresh,
independent acoustic windows can trigger successive owned turns; there is no
arbitrary cumulative-angle limit. Each turn still obeys existing motion limits,
preflight, timeout, braking, body ownership and IMU settling requirements.

**Submission gate:** this candidate is not yet Draft PR Ready under the requested
security criteria. The Driver changes are locally validated, but
Agent administrative privileges are currently unrestricted (confirmed by the
operator); deployment isolation remains required. Do not treat this
text or the synthetic tests as approval to enable physical motion.

## Implementation

- Integrate Card registration, manifest, configuration, lifecycle and Docker COPY
  with the same Bundle Loco instance; do not add a second localization algorithm,
  speed-control loop or SDK channel.
- Expose start, stop, info, prepare_authorization, authorize_follow, enable_follow,
  renew_follow, disable_follow, cancel and config. Startup/config/restart do not
  enable following. Internal turns use owned local receipts; public Loco ACP
  behavior remains available.
- Check capture/publication timestamps, independent non-overlapping windows,
  circular angles, hysteresis and fresh post-settle evidence. Forward/backward
  clock discontinuities revoke the session; delayed clock sampling is separately
  reported without relaxing the 250 ms clock-jump protection.
- Move source health outside cancellation-critical locks. Bound query waiting and
  worker count, discard stale generations, and signal cancellation through an
  independent owner-specific Loco path before lifecycle waiting. A late query
  cannot revive a stopped session.

## Operator authorization and safety

Pin an Ed25519 operator public key once at trusted Driver startup. A separate
operator workstation signs a challenge scoped to this device namespace, process,
intent, session and chassis resource. The encrypted private key stays off Core,
Driver and Agent-accessible storage. The interactive signing tool requires a
TTY, passphrase and explicit approval.

A signature permits one session for at most 300 seconds from challenge creation.
Agent may enable and explicitly renew within that operator deadline, including
multiple consecutive turns without repeated approval. Public IDs, ordinary MCP
parameters, access tokens and dangerous flags grant no motion authority. Stop,
cancel, expiry, revocation, faults and configuration changes retire the intent;
a retired session cannot be renewed. Missing/unavailable authority fails closed.
Safety stop does not require approval.

The actual Loco worker checks the memory-only permission guard after preflight
and immediately before nonzero publication, including after rate-limit waits.
Zero/braking commands remain allowed. Unconfirmed braking or IMU quiet keeps
ownership reserved rather than reporting a safe release. Software cancellation
and IMU receipts do not establish physical stopping time.

## Testing

Tests ran in isolated snapshots with ephemeral signing keys, synthetic acoustic
messages and Fake Controllers; no real robot SDK or motors were used.

- Original three Sound Facing files: **111 passed**.
- Authorization **36 passed**; blocked-source cancellation/concurrency **14
  passed**; real PTY operator CLI **1 passed**.
- Full Bumi regression: **537 passed, 1 skipped**.
- Final extended confirmation: **678 passed, 4 failed, 1 skipped**. An earlier
  post-fix run was **677 passed, 5 failed, 1 skipped** (see below). These counts
  include the Sound Facing/Bumi subsets and must not be added together.
- Four of those failures reproduce on latest official Driver main (**144 passed,
  4 failed** for the two global files): two Pico upload-glue checks, one global
  bundle-count expectation, and one S10 Pro missing sibling COPY. No unrelated fixes are included.
- Opt-in real ROS2/MCP/Fake Controller integration: initial clock-related failure,
  subsequent **1 passed**. It covers three turns, cancellation and no orphan ACPs.
- Current official Core/API/Canvas/Agent/ACP harness: initial clock-related
  failure, subsequent **20 scenarios passed**. Actual API/auth/MCP/Canvas/Agent
  dispatch and ROS bridge were used; LLM output/controller/operator were synthetic.
- Two generic Core ACP baseline edge reproductions still **fail**: early callback
  loss and unconsumed duplicate-result overwrite. Internal Card turns do not
  depend on these paths; Core was not modified.
- Ten cancellation evidence cases acknowledged the software signal with maximum
  measured delay **0.0001704210 s** (assertion bound 0.2 s), not physical stop time.

Raw host-clock sampling confirmed four realtime jumps while monotonic/raw/
boottime remained continuous. Later integration runs were scheduled after a host
correction, without changing product clocks or thresholds. Initial failed runs
are retained. The standalone ROS test is skipped in the ordinary suite and was
executed separately; no skipped or failed run is described as passing.

The fifth failure was a background-caller exit exceeding 0.3 s in one blocked-
source cancellation test. Its software cancellation assertion passed. Thirty
fresh-process repetitions of the seven cases (210 cases) passed with unchanged
thresholds. The single slow-exit failure remains recorded; a separate timed-wait probe
did not reproduce it and its precise cause is not established. Final full-suite
confirmation retained only the four baseline failures. Triage of the one-off
failure remains required under the requested readiness criteria. A separate deterministic concurrent health-result race was
reproduced, fixed and covered; final ROS and Core runs pass after that fix.

Final ARM64 build and an emulated, non-root, read-only, network-none container
smoke test passed. The smoke used real signatures and production Loco with a
Fake Controller; real SDK import was forbidden. The final image is linux/arm64,
1,126,040,442 bytes (whole image; incremental size unmeasured), digest
`sha256:95e19c100247f7d9cc316d130ec0c49680d4bcbf6dc52d3473d59e5e3a27e16b`.
Its runtime hashes match final Core evidence and source. Python 3.10/3.11 compile
and patch checks passed. Four existing device.py Pyflakes warnings also reproduce
on latest main; no new warnings remain. Python 3.12 was not executed. Detailed
commands and retained red runs are in `SOUND_FACING_REVIEW.md`.

## Deployment boundary and known limitations

Runtime adds `cryptography>=46` to requirements and the actual Docker pip list. Only the public key belongs in the Driver;
the signer/private key is excluded from the runtime image. A permanently blocked
source callback can leave one daemon query pending until it returns, but cannot
hold the state lock or restart motion.

Current Core has no independent operator motion-grant interface. This PR adds
no hidden Core dependency or fake production approval. Core's supplied Bash and
Docker capabilities can cross the verifier's trust boundary if Agent can rewrite
Driver/container/key/host resources. Deployment must remove those administrative
paths from Agent and demonstrate isolation before authorization is considered
end-to-end verified. Merely setting a read-only public-key mount is insufficient
if Agent can replace the container through Docker.

Full Core production lifespan, target Jetson runtime, official CI, browser
Dashboard/Canvas clicks and live model behavior remain unverified. Hardware
acceptance is also outstanding: true acoustic coordinates/calibration, timestamp
quality, multiple sources/self-noise, real turn accuracy, slip, braking and
long-duration tracking. `physical_motion_verified` remains false. Use
`SOUND_FACING_HARDWARE_ACCEPTANCE.md` for staged acceptance; do not run the default
factory bootstrap against shared services as part of offline testing.

## Reviewer notes

Target: **4paradigm/phanthymotus-driver:main**. Latest checked main:
`79e3740ec02f7156e66f563b106b671a851f8d43`; Core harness:
`2b00f1db2f81b80c27a5249a54301acfeb19d068`.

Open #514 is waist-joint animation rather than acoustic chassis tracking;
#376/#408 are reused. Review the permission lifecycle, delayed preflight/publish guards, stop epoch and
stale-result handling, reservation release and motion arbitration. Coordinate
concurrent Bumi registration/motion changes before merge. Hardware and production
trust-boundary gates must remain explicit; local simulation is not robot
acceptance. No remote push, PR operation, image publication or robot operation
was performed during this repair.
