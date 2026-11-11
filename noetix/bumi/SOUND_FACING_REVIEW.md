# Sound Facing repair and verification record

Reviewed 2026-11-11. Branch: `feat/bumi-sound-facing`. Original Driver HEAD:
`814b3ca953d33eba0aca9a61dcefa773f5da483b`. Official Driver main checked at
`79e3740ec02f7156e66f563b106b671a851f8d43`; Core main checked at
`2b00f1db2f81b80c27a5249a54301acfeb19d068`. All 19 original feature files were
backed up before editing; the resulting candidate has 26 files. No remote write,
robot connection, motor command, shared-service restart or other project edit
was performed.

## Readiness and trust dependency

Formal PR: **NO-GO**. Draft PR: **NO-GO under the requested completion criteria**.
Physical acceptance: **NO-GO**. The code rejects unsigned Agent calls and fixes
blocked-source cancellation. Production authorization also requires an Agent
that cannot replace Driver code, its public key, its container or host resources.
The operator confirmed that those administrative privileges are currently
unrestricted; deployment isolation is therefore an open submission gate.

Current official Core provides ACP bookkeeping, authentication and Canvas
bindings, but no independent bounded operator motion grant for this Card. Its
peer signing identity is stored in the Core database; an Agent-readable identity
is not an independent operator authority. Asset execution policy does not supply
this continuous-motion permission. Core also registers Bash
(`agent-core/src/event/llm.py`, execution in `event/desktop.py`); its supplied
compose exposes a Docker socket. Administrative tools could replace any
in-process verifier. A normal access token, Canvas source label, `dangerous`,
or public session ID cannot fix that boundary.

No Core patch is hidden in this Driver change. A concrete deployment restriction
and adversarial verification remain required before declaring authorization
fully closed. With no valid pinned key/signature, the safe default is denial.

## Independent operator signatures

`sound_facing_authority.py` verifies Ed25519 signatures using a public key pinned
once at Driver startup. Only the public key belongs on the robot. The private
key stays on a separate trusted operator machine. The operator tool generates
an encrypted private key and requires a TTY, passphrase and explicit approval.
Never expose its key, passphrase, signer or administrative tools to Agent.

The signed request binds schema, device namespace, process nonce, pending
intent, session, chassis resource and a duration of 1–300 seconds. Its monotonic
deadline starts at challenge creation; delayed delivery or replay cannot extend
it. `prepare_authorization` only creates a challenge; `authorize_follow` only
checks the proof. Neither moves. Agent can then `enable_follow` once and renew
within that deadline. Normal turns require no additional approval. Stop, fault,
configuration change and restart retire the intent; renewal cannot revive it.

The production Loco reservation carries a trusted local memory-only predicate.
It checks cancellation, session identity and both deadlines before accepting
motion, after delayed preflight and immediately before nonzero publication
(after rate-limit waits). Zero commands remain permitted. Deadlines are installed
before initial health checking; post-claim exceptions clean up the reservation.
Tests use real signatures with ephemeral keys. There is no production test-unlock
flag or MCP key override.

## Stop-lock repair and ownership

External source health is outside engine and Card lifecycle locks. A bounded
probe requests a 50 ms wait, with at most one outstanding external worker.
Each waiting caller keeps its own query result, so a concurrent new query cannot
erase a completed result. Generation/freshness checks still reject retired data.
Invalidating its generation discards late results. A permanently blocked callback
may leave one daemon worker pending until it returns; it cannot hold engine
state or authorize motion.

Stop/cancel/disable first signal the engine and owner-specific Loco interrupt,
independently of source health and before lifecycle waiting. Interruption stays
sticky before action ID publication. Existing ownership, worker braking,
terminal receipt, IMU quiet and body arbitration remain enforced. Late enable
or health results cannot clear a newer stop or restart a retired session. Safety
stop never requires authorization.

The original reproduction was also rerun using actual SoundDirectionPlugin.info
and its process lock: disable completed before releasing that source lock,
with a 0.000245495 s software signal delay and no restart or deadlock.

Ten directed evidence cases acknowledged Fake Controller cancellation. Maximum
measured software signal delay was 0.0001704210 seconds (assertion bound 0.2 s).
This is not physical braking time. Tests also cover actual Card stop/disable/
cancel, late enable, stale-result replacement, renew/revoke and bounded workers.

## Source clocks

Clock pairing samples monotonic-before / wall / monotonic-after. A delayed pair
is rejected separately as `capture_clock_sampling_delayed`; the existing 250 ms
jump protection was not relaxed. Forward/backward jumps revoke the session;
subsequent valid samples cannot automatically resume it.

The first real ROS run and first Core run hit `capture_clock_discontinuity`.
A separate 40-second probe captured four host realtime jumps (approximately
0.258, 0.263, 2.270 and 2.276 seconds), while monotonic/raw/boottime intervals
remained approximately 0.010–0.011 seconds. This directly identifies host wall
clock correction. One later run per harness was scheduled immediately after
observing a host correction. Product clocks and thresholds were unchanged;
those runs passed. Initial failures and raw samples are retained.

## Executed tests

Execution used isolated `/tmp` snapshots and environments. ROS/Core explicitly
excluded the real robot SDK. Counts are current, not historical.

| Scope | Result |
|---|---|
| Original three Sound Facing files | 111 passed (Bumi subset) |
| Authorization / cancellation / operator CLI | 36 / 14 / 1 passed |
| Bumi regression | 537 passed, 1 skipped |
| Driver extended regression, final confirmation | 678 passed, 4 failed, 1 skipped |
| Earlier post-fix extended run | 677 passed, 5 failed, 1 skipped (see below) |
| Latest official Driver baseline, same two global files | 144 passed, 4 failed |
| Real ROS2/MCP + Fake Controller | initial clock failure; subsequent 1 passed |
| Real Core/API/Canvas/Agent/ACP | initial clock failure; subsequent 20 scenarios passed |
| Generic Core ACP edge reproductions | 2 failed, baseline defects |

The four extended failures are the global bundle-count expectation
`test_every_http_bundle_is_wired`, the two Pico upload-glue checks
`test_the_glue_precedes_the_body_read_and_any_path_check` and
`test_the_glue_is_self_contained`, and S10 Pro
`test_every_imported_sibling_is_copied`. They also fail on latest official main;
no unrelated device fix is included. The ordinary Bumi skip is opt-in ROS,
executed separately above. Subset counts must not be added to full-suite counts.

Core exercised real FastAPI/auth, MCP conversion, Canvas API, Agent dispatcher
and ROS bridge; LLM decisions, acoustic input, motor controller and operator
approval were synthetic. Full production lifespan was disabled because it
touches host services. Browser clicks, production startup and a live model/robot
were not tested. Public Loco ACP pending success/failure/timeout/late cancel were
exercised. Internal turns use local owned receipts, with no orphan ACP callbacks.
Generic Core early-callback loss and unconsumed duplicate-result overwrite
remain independently reproduced baseline defects outside this Driver PR.

Earlier red runs remain recorded: initial expectation/terminal receipt failures;
a PTY CLI failure opening non-seekable `/dev/tty` in `r+` mode (fixed by separate
input/output streams); and the host-clock failures. Independent review reproduced
the previous delayed-preflight expiry gap using actual Loco/FakeHighCtrl. Current
code rejects the same expired request without a nonzero command. These failed
runs are not described as passing.

A later final Core run failed closed with `sound_direction_unavailable`. A
concurrent-query result race was then reproduced deterministically: a second
query erased the first waiter's completed healthy result. Per-query result
association fixed that race; the regression failed before and passed after the
patch. Final real ROS and all 20 Core scenarios passed with matching runtime
source hashes.

The first extended run after this fix had an additional failure: one cancellation
test's background caller did not exit within 0.3 s. Its independent cancellation
signal met the unchanged 0.2 s bound and the session was revoked. The failure is
retained as `extended-verified.*`; 30 fresh-process runs of all seven blocked-source
cases (210 cases) subsequently passed without changing thresholds. Thread-stack
diagnostics were added for recurrence. The exact cause of that single slow exit
is not established; do not report it as proven environmental or silently erase it.
A 40-second timed-wait probe did not reproduce the slow exit (801 Event waits,
maximum 0.050587 s; 799 sleeps, maximum 0.050777 s). The subsequent complete
confirmation passed every Bumi case and retained only the four known baseline
failures: 678 passed, 4 failed, 1 skipped. This does not establish the precise
cause of the earlier one-off failure. Its triage remains a P1 validation issue;
under the requested completion criteria, it must be explained before readiness.

Reproduce in an isolated checkout with Python 3.10+, pytest, PyYAML, NumPy,
cryptography and the existing test dependencies:

```bash
python -m pytest noetix/bumi/tests/test_sound_facing.py noetix/bumi/tests/test_sound_facing_card.py noetix/bumi/tests/test_sound_facing_motion.py -q
python -m pytest noetix/bumi/tests/test_sound_facing_authority.py noetix/bumi/tests/test_sound_facing_cancellation.py noetix/bumi/tests/test_sound_facing_operator_cli.py -q
NO_PROXY=127.0.0.1,localhost no_proxy=127.0.0.1,localhost python -m pytest noetix/bumi/tests tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py tests/test_bumi_imu_distance.py tests/test_bumi_leg_odometry.py tests/test_bumi_camera_specs.py tests/test_bumi_odom.py tests/bumi_visual_odometry tests/test_file_upload_endpoint.py tests/test_dockerfile_copies.py -q
```

## Build, packaging and remaining acceptance

Runtime requirements include `cryptography>=46`; Docker copies all three runtime
Sound Facing modules and checks native Ed25519 import. The signing tool and
private key are not copied. Both requirements.txt and the actual Docker pip
installation list include the dependency. An ARM64 wheel was resolved separately;
that is dependency evidence only.

ARM64 full build: **passed**, using the final Dockerfile and native SDK
compilation in a dedicated bounded Buildx builder (credential-free configuration,
separate context, no push). Final local image:
`bumi-sound-facing-review:b3hjmhax-final`, digest
`sha256:95e19c100247f7d9cc316d130ec0c49680d4bcbf6dc52d3473d59e5e3a27e16b`.
Docker inspect reports linux/arm64 and 1,126,040,442 bytes; this is the whole local
image, not an increment measured against an official baseline image. The existing
Dockerfile LD_LIBRARY_PATH warning remains; it was not introduced by this change.

A no-network, read-only, non-root container with no robot device mapping passed
real Ed25519 verification, denial before approval, one production Loco/Fake
Controller turn (50 synthetic commands), stop/release and rejection of retired
renewal. Real SDK import was forbidden and the production entrypoint never ran.
Container Python is 3.10.12, cryptography 50.0.2. The four runtime source hashes
match both the worktree and final Core receipt. The task's builder was removed;
the owned local image remains. This is emulation, not Jetson/Bumi acceptance.

All 16 changed Python modules compiled under Python 3.10 and 3.11. Pyflakes has
four unused-import diagnostics in device.py, all reproduced on latest main; no
new diagnostics remain. Python 3.12 execution was unavailable. Patch whitespace
and patch application against latest main passed. Latest official Bumi paths were unchanged from the
original base (37 intervening official commits). Original branch history is
retained without reset/rebase. Refreshed open PR search found no equivalent
sound-following Card; #514 is waist_sway (joint waist animation), not acoustic
chassis tracking. Its device/registration edits require merge-time coordination,
as do #504/#506/#508/#510. Existing #376/#408 remain the reused foundations.

Agent privilege isolation, target Jetson startup, official CI, browser acceptance,
real acoustic coordinates/calibration, self-noise, slip, physical stopping and
long-duration tracking remain unvalidated. See
`SOUND_FACING_HARDWARE_ACCEPTANCE.md`; `physical_motion_verified` stays false.

Evidence is local temporary execution output in
`/tmp/sound-facing-fix-20261111-b3hjmhax/logs/`: `bumi-verified.*`,
`extended-verified.*`, `extended-confirmation.*`, `latest-baseline.*`, `cancel-evidence/`, `ros-1.*`, `ros-2.*`,
`core-1.*`, `core-2.*`, `core-final.*`, `core-verified.*`, `ros-verified.*`,
`clock-probe.json`, `core-edge.*`, `arm64-final-build.log`, `arm64-smoke.log`,
`original-stop-lock-fixed.json`, `probe-race-red.log`, `probe-race-green.log`,
`cancel-stress.json`, `timed-waits.json`, `static-final.json`,
`pyflakes-final-result.json`.
These logs are not committed robot data, credentials or portable production
validation artifacts.

## Actual task runners

The task executed these commands from the temporary task root; each runner
records its underlying interpreter, pytest/harness arguments, working directory,
exit status and duration in the linked result JSON files. These paths are local
execution evidence and require the retained temporary environments.

```bash
python /tmp/sound-facing-fix-20261111-b3hjmhax/final_checks.py
python /tmp/sound-facing-fix-20261111-b3hjmhax/run_tests.py bumi-verified extended-verified
python /tmp/sound-facing-fix-20261111-b3hjmhax/run_tests.py extended-confirmation
python /tmp/sound-facing-fix-20261111-b3hjmhax/run_tests.py latest-baseline
python /tmp/sound-facing-fix-20261111-b3hjmhax/run_live_aligned.py ros-verified core-verified
python /tmp/sound-facing-fix-20261111-b3hjmhax/stress_cancellation.py
/tmp/sound-facing-fix-20261111-b3hjmhax/env310/bin/python -B /tmp/sound-facing-fix-20261111-b3hjmhax/probe_timed_waits.py
python /tmp/sound-facing-fix-20261111-b3hjmhax/static_checks.py
python /tmp/sound-facing-fix-20261111-b3hjmhax/run_lint.py
```

ARM64 build used the actual final Dockerfile, `docker buildx build --platform
linux/arm64 --load` and a task-exclusive builder with 3 GiB memory/2 CPUs. It did
not invoke the repository build/publish script. The container smoke used
`--platform linux/arm64 --network none --read-only --cap-drop ALL
--security-opt no-new-privileges --user 65534:65534`, a 512 MiB/1 CPU limit and a
read-only temporary test-script bind. SDK import was explicitly forbidden.
