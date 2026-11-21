# Tianyi Estimated Body TF Implementation Plan

User approved use of estimated extrinsics with truthful configuration/diagnostics, no extra card prompts; global camera mode and 50 ms software synchronization remain required.

1. Add an opt-in, read-only companion node `estimated_body_tf.py` and configuration. Pin the audited XARM chain, use p52 and p51-p52, estimate base_link=model base, and use existing public head-to-optical values. Preserve provenance and accuracy_validated=false. Test matrices, malformed feedback, missing/bracketing data, clock bounds and expiry.
2. Map motion-board realtime to host realtime using persistent authenticated SSH round trips (credentials only from deployment environment). Reject RTT >8 ms, mapping age >2 s, wall-clock jumps, stale captures and total mapping uncertainty + joint bracket >50 ms. Reconnect without blocking ROS callbacks. Publish only capture-stamped base_link -> Orbbec root TF, composing the inverse native camera-internal TF, with a separate diagnostic status topic; no command publishers.
3. Package companion/config in Tianyi image and document opt-in launch. Regression-test offline. Run an isolated companion on the robot against current camera/driver, retain estimation provenance in diagnostics, verify global mode and real body_mount availability/consumer matching. Do not move the robot or restart the hardware camera. The authorized driver update requires backup and an idle-state check before restart. Only persist the companion after validation.
4. Review, commit and push PR519. Report exact verification and remaining spatial/long-duration limitations.

Success: complete estimated chain available at actual capture timestamps, invalid time inputs rejected, no competing parent for optical frames, unchanged image/odometry contracts, no measured-calibration claims.
