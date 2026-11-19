# Sound Facing testing — 2026-11-13

All results below were executed in this task. Historical 112/621 counts and an earlier ARM64 image claim are not reused as current evidence. Driver baseline: `18211f9902549ce931867a3622cbb96d4c79183b`; actual Core source: `541d53461818f220a56f902e5c7093be61e155d8`.

| Scope | Current result |
|---|---|
| Sound Facing/authorization/cancellation/CLI/direction/build-helper tests | 181 passed, 1 opt-in real ROS test skipped |
| Bumi and related Driver regression | 697 passed, 4 failed, 1 skipped |
| Same global tests on an untouched latest Driver snapshot | 144 passed, the same 4 failed |
| Real Core HTTP/auth/MCP/Agent/ACP plus native Canvas read-only display | 21 scenarios passed |
| Revocation/cancellation pressure, fresh processes | 20 runs × 8 cases = 160 passed; maximum measured software signal delay 0.000057842 s |
| ARM64 dependency resolution | CPython 3.10 cryptography/CFFI and RealSense wheels resolved; not an image build |
| Full ARM64 image/build/container smoke | BLOCKED: current WSL Docker integration is unavailable; no usable daemon/buildx |
| Real ROS/DDS, microphone, vendor SDK, physical motion | Not executed |

Do not add subset counts to the regression total. Python was 3.10.12; authorization used real Ed25519 signatures with ephemeral test keys, not a production authorization override. The 160-case pressure test retained the original 0.2 s signal / 0.3 s background-return assertions. These are measured software timings, not real-time or physical-braking guarantees.

## Commands

Use a task-owned Python environment with pytest, NumPy, PyYAML and `requirements.sound-facing.lock` installed. HTTP tests need permission to bind loopback sockets.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q noetix/bumi/tests/test_sound_facing*.py --tb=short
NO_PROXY=127.0.0.1,localhost no_proxy=127.0.0.1,localhost PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q noetix/bumi/tests tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py tests/test_bumi_imu_distance.py tests/test_bumi_leg_odometry.py tests/test_bumi_camera_specs.py tests/test_bumi_odom.py tests/bumi_visual_odometry tests/test_file_upload_endpoint.py tests/test_dockerfile_copies.py --tb=short
python noetix/bumi/tools/verify_sound_facing_core.py --core /path/to/disposable/agent-core --transport inprocess --output /path/to/new-result
```

The integration uses the unmodified Core FastAPI application, normal token authentication, MCP discovery/schema conversion, actual Agent dispatcher, Canvas APIs and real ACP pending storage/callbacks. LLM decisions, the microphone source and controller are synthetic. `--transport inprocess` is a test-only ROS API transport without DDS or hardware; its capture clock is explicitly synthetic and monotonic-derived. The production clock guards remain unchanged. The default `--transport ros2` requires a real isolated ROS context and loopback DDS profile; it was not run here. Full Core lifespan is deliberately off because it starts host services. Local Core self-registration metadata is invoked explicitly.

Optional native Canvas inspection uses `--browser-python /path/to/python-with-playwright --chromium /path/to/chrome --three-dir /path/to/cached-three-0.164`. It loads the actual Canvas, checks the Card's manual actions and opens read-only details. It never starts the project or executes browser motion. Start/stop/cancel are tested through the real Canvas HTTP API. The cached Three.js version is 164, matching upstream's import map; no Core source is modified and no public service is needed for the test. `page_errors=[]` in the passing receipt.

## Failure attribution

The four remaining regression failures are unchanged baseline failures:

- `test_every_http_bundle_is_wired`;
- Pico `test_the_glue_precedes_the_body_read_and_any_path_check`;
- Pico `test_the_glue_is_self_contained`;
- S10 Pro `test_every_imported_sibling_is_copied` (`pointcloud.py`, `reader.py`).

They reproduce on the untouched latest Driver snapshot with the same interpreter and loopback permission. No unrelated device code or expectations were modified. An initial sandbox run could not create HTTP sockets; it is retained and was rerun with loopback permission, without skipping tests.

The first Core run passed its 20 scenarios but failed while recording a git revision from an archive rather than a checkout. Snapshot revision recording was fixed, and the complete run passed. The first browser inspection blocked upstream's Three.js imports; the second had an incorrect test assumption that start/stop appear in the manual action dropdown. Native Canvas handles lifecycle through project controls. The harness was corrected to use real Core registration metadata, a valid executor link, cached matching assets and the correct manual actions; the 21-case run passed. These harness failures are retained.

An older review describes one slow background-caller exit. Its exact historic cause cannot be recovered from this task's available artifacts. This task does not relabel it as proven environmental: it fixes the independently reproduced missing revocation wakeup, tests the wakeup directly, and records 160 unchanged-bound pressure cases. Python thread scheduling and SDK hard hangs are not hard-real-time guarantees; retain physical stop validation.

## ARM64 reproduction

```bash
python noetix/bumi/tools/build_sound_facing_arm64.py --output /path/to/new-build
# To inspect context only, without Docker:
python noetix/bumi/tools/build_sound_facing_arm64.py --prepare-only --output /path/to/new-context
```

The helper prepares a standalone Bumi/common context, records source hashes, creates only its uniquely named Docker-container builder, limits that builder to 3 GiB / 2 CPUs, builds `linux/arm64 --load`, inspects architecture/size and runs a non-root, read-only, network-none smoke with no device mapping and a forbidden-SDK import guard. It removes only its own builder; it never pushes, switches shared builders, starts Docker or installs binfmt. The smoke verifies imports, signatures, denial/revocation and a read-only Card start/stop. It never runs the production CMD. A working ARM64-capable Docker/Buildx engine, registry/package access and target-compatible emulation/native execution are external prerequisites. See the [Docker-container driver options](https://docs.docker.com/build/builders/drivers/docker-container/).

Only the new authorization dependency group is locked; pre-existing Bumi image packages remain unchanged. Image size and incremental image cost are unmeasured here. Full ARM64 and target-robot startup remain required; wheel resolution and prepared source are not a substitute.

Evidence: `/home/utopus/phanthy-work/artifacts/dual-card-pr-ready-20261113/bumi/`. Key files: `verification/regression-delivery.log`, `verification/upstream-global-final.log`, `verification/directed-final.log`, `verification/cancellation-stress.json`, `core-browser-verified/receipt.json`, `core-browser-verified/browser/`, `arm64-build/BUILD.json`.
