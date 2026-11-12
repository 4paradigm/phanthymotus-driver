# Shared Depth Cards Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: use test-driven implementation in the three isolated model worktrees; coordinator integrates contract checks and final fresh review.

**Goal:** RGB/depth/IMU cards provide calibrated computation inputs and share exactly the original depth output for display, VO and Point Cloud.
**Architecture:** Reuse each model's existing camera acquisition and normalized RGB-D code. Move public outputs/declarations into original cards; coordinate per-card start/stop around one owner. Keep independent PRs.
**Tech Stack:** Python, existing ROS2 Humble/RealSense/Orbbec, numpy/cv2/zlib; no dependencies.
**Spec:** docs/superpowers/specs/2026-11-12-shared-depth-cards-design.md

## Global Constraints
- No hardware/network access to robots or deployments.
- Depth: 640x480 little-endian uint16 measured mm, zero invalid, image/depth-zlib, original depth topic only.
- Preserve capture timestamps/provenance, source geometry and existing protection thresholds.
- Body IMU and camera IMU are distinct; missing optional TF/IMU leaves valid depth available.
- Separate PR519, PR499, PR471; Perception PR371 only if integration requires a generic change.

## Review Focus
- One card stop/start must not kill another card or revive stale frames.
- Driver lazy-start must not mark every sibling active or emit stopped-card outputs.
- Actual shared depth payload/header/K must agree; no old display fallback on that topic.
- Camera disabled or IMU unsupported must give useful availability without body-IMU substitution.
- Point Cloud/VO provenance must keep the same source/clock/frame and valid identity mapping.

### Task 1: Tianyi existing card integration
Files: x-humanoid/tianyi2.0/{main.py,device.py,rgbd_camera.py,driver.yaml,config.yaml,README.md}; tests/test_tianyi*.
Interfaces: private RgbdCameraPlugin feeds original camera/head/depth and matching RGB; original cards expose live camera_info and motion metadata.
- [ ] Write/run failing tests for one original depth port, three-card registration and lifecycle, normalized pixels/capture headers.
- [ ] Replace duplicate display depth producer with shared calibrated producer; integrate RGB/IMU card outputs and remove camera_rgbd registration.
- [ ] Run all Tianyi, shared mount/control tests; update migration docs; commit only task files.

### Task 2: G1 existing card integration (isolated G1 worktree)
Files: unitree/g1/{device.py,main.py,rgbd_camera.py,driver.yaml,README.md}; unitree/g1/tests and relevant root camera tests.
Interfaces: original camera_depth is sole depth source, camera_rgb declares paired RGB, body imu card declares distinct camera motion; shared RealSense lifecycle.
- [ ] Add/run failing card registration, non-1mm scaling, capture time and shared-owner lifecycle tests.
- [ ] Publish normalized computation depth directly on old depth topic; remove old raw depth encoder/publisher, distribute camera_info/IMU and remove separate card.
- [ ] Run G1 and camera contract regressions, update migration docs, leave diff for coordinator review.

### Task 3: Bumi existing card integration (isolated Bumi worktree)
Files: noetix/bumi/{device.py,main.py,rgbd_camera.py,driver.yaml,README.md}; tests/test_bumi* and tests/bumi_visual_odometry.
Interfaces: original depth card is sole calibrated depth; original camera exposes matched RGB; original body imu card appends separately named camera IMU; same RealSense owner/clock.
- [ ] Add/run failing unique publisher, capture-time/scale, per-card restart and missing optional input tests.
- [ ] Replace display depth path with normalized computation producer at original topic; preserve color display, provenance and independent optional availability.
- [ ] Run Bumi/capture/IMU regressions, update migration docs, leave diff for coordinator review.

### Task 4: Consumer integration, review and push
Files: generic Perception tests/implementation only if demonstrated necessary; per-model integration tests use real independent consumer contracts.
- [ ] Replay each model's real produced depth and declarations through generic VO validation and Point Cloud metric/SLAM provenance checks; same topic/header/geometry required.
- [ ] Run meaningful driver scopes plus full Perception suite; report SDK/environment collection blockers separately.
- [ ] Fresh read-only whole-change review; fix demonstrated issues with failing/passing tests.
- [ ] Commit/push model branches separately, verify remote SHAs and clean trees; no merges/deployment/hardware.
