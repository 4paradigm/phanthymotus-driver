# Tianyi single RGB and valid common field implementation plan

> Execute inline with red/green regressions and a final fresh read-only review. User requested original RGB output reuse and supplied a screenshot of black padding; prior authorization covers offline implementation and PR519 push. No hardware/SSH/deployment. Scope: Tianyi only.

Goal: sole original /<ns>/camera/head JPEG is calibrated RGB for display and generic VO, paired with the sole original depth output. Remove separate /camera/rgbd/color output/publisher. Remove remap-created black padding by selecting a common valid RGB/depth ray window and updating both remap tables and K, not by stretching RGB alone. Preserve original raw photo/video capture and vendor-service watchdog.

Evidence: current make_geometry uses depth's wider field. A 640x480 color-focal520/depth-focal400 probe reproduces a 492x368 valid RGB image inside black padding (58.94% valid pixels). Screenshot's actual device intrinsics are not available; matching physical root cause remains unverified until user-authorized sampling.

Task 1 — Geometry (rgbd_camera.py, tests/test_tianyi_rgbd_camera.py, tests/test_tianyi_body_mount.py):
- Add failing white-image remap and measured-depth/ray/K tests for unequal lenses, shifted centres and nonzero distortion.
- Derive common valid centered 4:3 field from both projection maps; adjust virtual pinhole K and recompute maps. Fail closed if no usable common field. Preserve 640x480, axial measured mm, nearest-neighbour depth and original capture headers.
- Validate actual codec output has no remap padding, natural invalid depth is preserved, camera_info agrees and metric pointcloud ray projection remains correct.

Task 2 — One original RGB (rgbd_camera.py/main.py/device.py, shared card/watchdog tests):
- Add failing test for exactly one original RGB output, declared geometry on that topic, no duplicate original publisher/encoder, monitor-only watchdog start/stop.
- Publish calibrated source RGB directly on /camera/head. Original CameraPlugin provides service/watchdog monitoring without publishing/encoding a second RGB stream. Snapshot raw capture is untouched.
- Update ports, startup readiness/lifecycle tests and migration docs. Registered RGB now follows existing shared pair hz (default8); view is common field, not guaranteed identical to old raw image.

Task 3 — Verify/review/push:
- Run complete Tianyi/shared-mount/control scope, consumer actual-payload replay and relevant generic regressions. Do not claim physical registration/native VO or on-device renderer acceptance from synthetic tests.
- Fresh review, fix verified findings, commit and push PR519 separately. G1/Bumi unchanged, no deployment or hardware test.
