# Compatibility validation — 2026-10-21

## PR #408 standalone

Baseline: remote `7d5b2673`. Final relevant regression: **581 passed, 1 skipped
in 66.04s** (no hardware). Command:

```sh
python3 -m pytest -q tests/bumi_fusion tests/test_bumi* noetix/bumi/tests \
  tests/test_common_lifecycle.py tests/test_control_rotation.py \
  tests/test_camera_info.py tests/test_dockerfile_copies.py \
  tests/test_deploy_restart_policy.py
```

The focused compatibility/bundle/camera run was **61 passed**; it is included in
the broad run, not an additional count. Newly introduced failure tests were run
before their fixes: missing binding integration; frozen relative origin; expired
lease renewal; IMU bridge construct/start/stop isolation; exact-lease/topic ACK;
missing/changed live source ports; default standalone IMU; rejected legacy topic
cannot fall back; four-direction fusion and lateral fault paths; immediate cancel;
stale watchdog context; raw camera branch crash isolation. Existing motion/media
regressions cover rotation, stop, mutual exclusion, ACP, audio and photo/video.
An independent reviewer reproduced the immediate-cancel quality defect and stale
watchdog action race; both now have deterministic regression tests.

Simulations use a first-order synthetic gait response, with model truth, scaled
odometry estimate and target recorded separately. The nominal four-direction
model stays within its modeled band; a 15% scale error still returns `done` with
more than 10cm true endpoint error. These tests do not execute RTAB-Map or certify
one metre ±5cm. All results retain `accuracy_validated: false`.

Python compile checks and Git whitespace checks passed. The original #425
platform patch plus the supplemental patch passed apply-check, actual application
and Python compilation in a temporary snapshot of platform
`6df601f612818716bb0dc7b180a9467e2c43172c`. No platform checkout was changed.
Saved-graph tests execute the real graph resolver and driver binding consumer,
including renewal/ACK handling, using transport replacements; they are not a
complete running backend/browser test.

## Full repository collection

`python3 -m pytest -q` stopped during collection: **45 errors, 3 skipped**.
Vendor SDK imports, duplicate test module names and Python 3.9 type expressions
remain environment/baseline limitations. Failed paths:

```text
ERROR agibot/AimDK_X2/tests/test_device.py
ERROR brainco/revo2/tests/test_device.py
ERROR chasing/qianjiao_p200_pro/test_device.py
ERROR limx/tron2/tests/test_camera.py
ERROR limx/tron2/tests/test_completion.py
ERROR limx/tron2/tests/test_gripper_display.py
ERROR limx/tron2/tests/test_motion_readiness.py
ERROR limx/tron2/tests/test_sensors.py
ERROR noetix/bumi/noetix_sdk_bumi/examples_py/test_high.py
ERROR noetix/bumi/noetix_sdk_bumi/examples_py/test_low.py
ERROR noetix/bumi/noetix_sdk_bumi/examples_py/test_media.py
ERROR noetix/bumi/noetix_sdk_bumi/examples_py/test_policy.py
ERROR pndbotics/adam/test_arm_controls.py
ERROR pndbotics/adam/test_battery.py
ERROR pndbotics/adam/test_battery_status.py
ERROR pndbotics/adam/test_loco_contract.py
ERROR pndbotics/adam/test_state_flattening.py
ERROR pndbotics/adam/test_upper_body_cards.py
ERROR pndbotics/adam/test_vision_capture.py
ERROR robotera/q5_bundle/test_basic_sensors.py
ERROR tests/test_franka_discovery.py
ERROR tests/test_franka_driver.py
ERROR tests/test_piper_camera_live.py
ERROR tests/test_tianyi_bridge.py
ERROR ubtrobot/u1_pro/test_device.py
ERROR unitree/as2w/unitree_sdk2py/test/crc/test_crc.py
ERROR unitree/as2w/unitree_sdk2py/test/rpc/test_client_example.py
ERROR unitree/as2w/unitree_sdk2py/test/rpc/test_server_example.py
ERROR unitree/g1/test_slam_rpc.py
ERROR unitree/g1/tests/test_fsm_posture.py - TypeError: unsupported operand t...
ERROR unitree/g1/tests/test_speaker_pacing.py - TypeError: unsupported operan...
ERROR unitree/g1/unitree_sdk2py/test/crc/test_crc.py
ERROR unitree/g1/unitree_sdk2py/test/rpc/test_api.py
ERROR unitree/g1/unitree_sdk2py/test/rpc/test_client_example.py
ERROR unitree/g1/unitree_sdk2py/test/rpc/test_server_example.py
ERROR unitree/go1/tests/test_vision_capture.py
ERROR unitree/go2/unitree_sdk2py/test/crc/test_crc.py
ERROR unitree/go2/unitree_sdk2py/test/rpc/test_api.py
ERROR unitree/go2/unitree_sdk2py/test/rpc/test_client_example.py
ERROR unitree/go2/unitree_sdk2py/test/rpc/test_server_example.py
ERROR unitree/r1/tests/test_switch_mode_acp.py - TypeError: unsupported opera...
ERROR unitree/r1/unitree_sdk2py/test/crc/test_crc.py
ERROR unitree/r1/unitree_sdk2py/test/rpc/test_api.py
ERROR unitree/r1/unitree_sdk2py/test/rpc/test_client_example.py
ERROR unitree/r1/unitree_sdk2py/test/rpc/test_server_example.py
```

## Not performed by this session

- ARM64 image builds, image size/digest measurement, publishing or performance timing.
- ROS launch, RTAB-Map/EKF/message/TF integration, or the provided ROS probe.
- Full platform backend/browser/peer integration or deployment of either patch.
- Robot deployment, live camera/audio/motion regression or fault-stop latency measurement.
- Installation extrinsics, IMU units/quaternion/gravity, host-read delay qualification,
  real recordings/replays, or physical distance/heading precision acceptance.

Docker/Podman/ROS tools and hardware are unavailable locally. Remote PR build-bot
comments are observations about older commits, not builds performed here or proof
of the new combination. Calibration and initialization gates have not been loosened.
