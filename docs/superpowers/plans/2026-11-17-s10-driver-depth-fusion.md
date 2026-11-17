# S10 standalone camera_front_depth completion

User requirements: driver-only changes; one card `camera_front_depth`, no
additional visual_depth/YOLO cards, topics or calibration entry; one output
`/{namespace}/camera_front/depth`; no overlay marking inferred regions.
No deployment, robot configuration, vendor changes, motion or dependency
installation on the robot.

Implementation choice: run official YOLO26n-depth ONNX in the driver's bounded
worker with CPU ONNX Runtime, so the existing ROS image and host runtime need
no GPU configuration or perception-service dependency. Pin and checksum the
model and bake it into future driver builds. Read the driver's already
rectified front JPEG through a bounded latest-frame cache with generation and
receipt-time checks. Use existing camera calibration and extrinsics unchanged.

Projection -> bounded 2-D fill -> monocular prediction -> robust log-affine
metric calibration using only original LiDAR pixels -> bounded local residual
correction -> preserve measured/local values -> one depth-zlib output. Hold
out one fifth of reference pixels from both calibration stages and reject
inconsistent fits before restoring original values. No default/guessed scale,
no synthetic calibration references, no API or code changes in perception.

Checks: RED/GREEN fusion math, invalid references, correspondence rejection,
no interpolation leakage; inference letterbox/crop and model artifact checks;
front JPEG freshness/generation and standalone-card integration; actual
captured RGB/cloud offline replay, same-frame coverage, withheld residuals,
Jetson CPU latency using existing runtime in a temporary read-only process;
S10 regression and repository test command. Test clock/receipt matching does
not establish RTSP exposure synchronization or final extrinsic validation.

Current ledger: 2-D fill independently reviewed; final path-support union
check corrected with regression. Existing focused suite306passed8skipped;
legacy monitor failures and repository80collection errors recorded separately.
Fusion unit tests7passed. Driver card rename tests2passed. Actual TensorRT
YOLO/DepthART comparisons done read-only; CPU ONNX artifact/runtime verification
and card integration remain. No platform repo files modified.

Final verification: 323 S10 tests passed,8skipped with the declared ONNX/Pillow
runtime in a work-computer venv. Real ONNX executed locally and in an isolated
Jetson CPU replay; all3frames100%nonzero with every raw/local pixel preserved.
Heldoutmedian1.10–1.12%,P9016.14–16.92%; inference+fusion1.15–1.30s/frame.
The .5m reference exclusion was removed: only the configured valid range
selects original refs; near returns are not automatically classified as noise.
Generated values enforce both depth bounds. Final review found no remaining
actionable issues after support-union and min-range regressions were fixed.
Repository pytest80collection errors match the pre-change80line-for-line.
Final image build was unavailable (no local Docker CLI); no robot deployment,
dependency installation, platform changes or navigation startup occurred.
Evidence/report: diagnostics/s10-monocular-fusion-20261117/REPORT.md at the
parent workspace. Legacy canvas tool identifiers were not rewritten.
