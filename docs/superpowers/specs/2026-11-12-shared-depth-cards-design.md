# Shared depth through existing sensor cards

Approved in chat: implement on Tianyi/G1/Bumi, offline tests only, push separately to PR519/499/471. No SSH, hardware, deployment or robot movement.

Existing RGB, depth and IMU cards expose calibrated information. Retire the separate camera_rgbd card. Preserve original display RGB and body IMU endpoints. RGB may append its registered computational image output; IMU appends explicitly named camera motion sensor/imu (body data/json is never substituted). Depth has exactly its original single image/depth-zlib output: Tianyi /<ns>/camera/head/depth, G1/Bumi /<ns>/camera/depth. No duplicate /camera/rgbd/depth publisher/encoding.

The one depth stream is 640x480 little-endian uint16 measured axial millimetres, zero invalid, zlib compressed in CompressedImage. Reuse the previously validated computation geometry, scale normalization and capture-time mapping; matching RGB shares actual pinhole K, frame and clock. Declare camera_info on the actual public topics, discover body_mount sidecars automatically, and preserve clock provenance (Bumi unverified stays unverified). No guessed transforms, receipt-time fallbacks or threshold relaxation. Missing optional IMU/body TF must not disable valid depth. Invalid depth geometry/units/capture timing fails closed (shared display also pauses); do not silently publish different geometry at the same topic.

Reuse one camera owner per robot. Cards cooperate over that owner: stopping one input must not shut down another live card, repeated start is idempotent, restart cannot reuse stale frames/epochs. Shared acquisition may remain alive while a card is stopped, but the stopped card must stop its outputs. Whole bundle shutdown releases resources. Do not introduce generic cross-model frameworks or new dependencies.

Point Cloud consumes the same depth topic and real camera_info; metric_cloud produces measured PointCloud2 plus provenance. Generic VO consumes matching RGB/depth and optional camera IMU. SLAM consumes its state/odom and the metric pointcloud with matching source/clock/frame metadata. Camera-reference mapping does not need fabricated body TF; navigation/body accuracy remains unaccepted.

Compatibility: original depth topic/name/type retained; old camera_rgbd canvas wires must move to the original three cards. Depth view can change to computation geometry (especially Bumi). Retain existing depth publication configurations where possible; bound queues so slow consumers do not create backlog. No promise of hardware performance from offline tests.
