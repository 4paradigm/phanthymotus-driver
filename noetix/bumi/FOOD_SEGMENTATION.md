# Food Segmentation Card

## Status and scope

`food_segmentation` is an optional Bumi `processor` Card that consumes JPEG
frames over ROS 2 DDS, performs instance segmentation, and publishes structured
metadata plus a JPEG overlay. MCP controls discovery and lifecycle; complete
images are not transported through MCP.

The implementation has been verified on Windows CPU with a local YOLO11n-seg
checkpoint and with ROS-free/Fake Runtime tests. Real ROS 2 Humble DDS, Bumi
Jetson/ARM64, Canvas and robot-camera validation have **not** yet been run.

The Card is disabled by default. The production Bumi image currently copies
the adapter source but deliberately does not install Ultralytics, PyTorch or a
model checkpoint.

## Supported food classes

The runtime reads class names from model metadata and verifies the standard
COCO IDs rather than trusting an unverified class ordering.

| COCO ID | Class |
| ---: | --- |
| 46 | banana |
| 47 | apple |
| 48 | sandwich |
| 49 | orange |
| 50 | broccoli |
| 51 | carrot |
| 52 | hot dog |
| 53 | pizza |
| 54 | donut |
| 55 | cake |

All other model classes are filtered. Multiple instances of the same class
remain separate.

## Experimental checkpoint

- Model: `yolo11n-seg.pt`
- Task verified from model metadata: `segment`
- Official source: <https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-seg.pt>
- Size observed in the local experiment: `6,182,636` bytes
- SHA256:

  ```text
  55ED65C56C91713D23E8402371C6C49A6FD84F257F7DCE452E8D70E41DCBE152
  ```

This hash identifies the checkpoint used by the local experiment. Deployment
must verify the mounted file independently. The checkpoint is not stored in
this repository and must not be baked into an image without license and
maintainer approval.

## DDS input and output

Default topics use the resolved Bumi namespace:

| Direction | Topic | ROS 2 type | Card format |
| --- | --- | --- | --- |
| Input | `/<namespace>/camera/color` | `sensor_msgs/msg/CompressedImage` | `image/jpeg` |
| Output | `/<namespace>/food_segmentation/metadata` | `std_msgs/msg/String` | `data/json` |
| Output | `/<namespace>/food_segmentation/overlay` | `sensor_msgs/msg/CompressedImage` | `image/jpeg` |

The input must be a non-empty JPEG that decodes to an `H x W x 3` image. The
Canvas-resolved `input_topic` passed to the MCP `start` action overrides the
configured default for that run. A running Card must be stopped before changing
its input topic.

The QoS matches the Bumi camera stream: best effort, volatile, keep-last with
depth one. The ROS callback only replaces a latest-frame slot; one worker owns
model inference, so old pending frames may be dropped under load.

### Metadata

Metadata uses schema `food-segmentation/1` and includes:

- source ROS timestamp and image size;
- processed instance count;
- class ID, class name, confidence and half-open `bbox_xyxy`;
- full-image mask size and `binary-rle-row-major-v1` counts;
- mask pixel area and image-area ratio;
- isolated per-instance validation errors and processing diagnostics.

`binary-rle-row-major-v1` is the project-specific row-major binary RLE defined
by `food_segmentation_contract.py`. It is not COCO RLE. The Mask is produced
from the segmentation tensor; a bounding-box rectangle is never substituted.

### Overlay

The overlay preserves the input header and timestamp, colors each decoded
instance Mask, and adds its class, confidence and bounding box before JPEG
encoding. Empty detections still produce legal empty metadata and an unchanged
image overlay.

## MCP lifecycle

The Tool name and prefix are both `food_segmentation`. Supported actions:

- `start`: load the explicitly configured local checkpoint, create ROS
  publishers/subscription, and start one inference worker. Repeated calls with
  the same input topic are idempotent.
- `stop`: reject new work, discard the pending frame, wait for active inference,
  then remove and destroy the ROS Node. If the wait times out, ROS resources
  remain alive until that worker exits so it cannot publish through destroyed
  publishers.
- `info`: report state, effective topics, input freshness, runtime state,
  counters, thresholds and the most recent error/processing duration.

Example Canvas/MCP start arguments:

```json
{
  "action": "start",
  "input_topic": "/bumi_01/camera/color"
}
```

## Configuration

The checked-in default remains fail-closed:

```yaml
plugins:
  food_segmentation:
    enabled: false
    model_path: ""
    input_topic: ""
    metadata_topic: ""
    overlay_topic: ""
    confidence_threshold: 0.25
    mask_threshold: 0.5
    imgsz: 640
    device: ""
    max_rate_hz: 2
    input_timeout_s: 2
    stop_timeout_s: 10
    jpeg_quality: 85
    max_compressed_bytes: 8388608
    max_image_pixels: 2500000
    max_instances: 50
    max_total_rle_runs: 500000
    max_metadata_bytes: 4194304
```

An empty `input_topic` selects `/<namespace>/camera/color`. Empty output topics
select their namespace defaults. Topic values may use `{namespace}`.

For a future approved deployment, mount the checkpoint under the existing
persistent `/opt/phanthy-motus/data` volume and configure its exact container
path, for example:

```yaml
    enabled: true
    model_path: /opt/phanthy-motus/data/models/food_segmentation/yolo11n-seg.pt
    device: "0"  # Example only; use only after the Jetson runtime is verified.
```

The directory above is a proposed convention, not an already deployed path.
The Card never interprets `yolo11n-seg.pt` as a downloadable model name: the
path must already exist. If Ultralytics is missing, `start` returns an error
similar to:

```text
Ultralytics is not installed; install it in an isolated environment
```

The Tool remains discoverable when configured enabled because Ultralytics is
imported only inside runtime `load()`.

### Resource limits

The Card fails closed on resource-heavy frames instead of publishing a partial
or geometrically altered Mask:

- `max_compressed_bytes` rejects the DDS JPEG payload before decoding;
- `max_image_pixels` preflights standard JPEG SOF dimensions before decode and
  rechecks the decoded image before model inference;
- `max_instances` bounds backend instances before contract/RLE construction;
- `max_total_rle_runs` is enforced by the streaming row-major encoder, which
  stops scanning the Mask before appending an over-budget run;
- `max_metadata_bytes` is enforced while JSON chunks are encoded, before a
  complete string, overlay, or DDS sample is constructed.

The Ultralytics adapter also passes the instance cap as `max_det`, rejects a
backend that returns more detections before converting Mask tensors, and keeps
accepted Masks as NumPy arrays instead of expanding them into Python lists.

An over-limit frame increments `limit_errors`, records `last_error`, and emits
neither metadata nor overlay. It is not silently truncated, and a bbox is never
substituted for the Mask. The checked-in limits support common camera frames up
to roughly 2.5 megapixels; adjust them only after measuring the target DDS and
Jetson memory budget.

## Local tests

From the repository root on Windows PowerShell:

```powershell
$env:PYTHONUTF8='1'
python -m pytest tests/bumi_food_segmentation -q
python -m pytest tests/test_bumi_food_segmentation.py -q
python -m pytest tests/test_bumi_bundle.py noetix/bumi/tests -q
python -m pytest tests/test_dockerfile_copies.py -q -k 'noetix and bumi'
python -m compileall -q noetix/bumi/food_segmentation.py noetix/bumi/food_segmentation_backend.py noetix/bumi/food_segmentation_contract.py noetix/bumi/food_segmentation_ultralytics.py tests/bumi_food_segmentation tests/test_bumi_food_segmentation.py
git diff --check
```

The real-model smoke test is opt-in and does not download anything:

```powershell
$env:FOOD_SEGMENTATION_MODEL_PATH='E:\path\to\yolo11n-seg.pt'
$env:FOOD_SEGMENTATION_TEST_IMAGE='E:\path\to\known-food-image.jpg'
$env:FOOD_SEGMENTATION_EXPECTED_CLASS='banana'
python -m pytest tests/bumi_food_segmentation/test_real_model_smoke.py -q
```

The caller must supply a licensed image whose expected class is detectable.

## Real ROS 2 Humble integration test

This test uses real rclpy nodes, a real `MultiThreadedExecutor`, real DDS
publishers/subscribers and ROS message classes, with a Fake Runtime so it needs
no model or CUDA:

```bash
cd /path/to/phanthymotus-driver
source /opt/ros/humble/setup.bash
# Source the repository/container workspace as applicable:
test ! -f /ros_ws/install/setup.bash || source /ros_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Optional: keep nodes alive for 30 seconds and print the randomized topics.
export FOOD_SEGMENTATION_ROS2_OBSERVE_SECONDS=30
python3 -m pytest tests/bumi_food_segmentation/test_ros2_integration.py -q -s
```

Expected result is one passing test. A skip means ROS 2 or OpenCV was not
available and must not be reported as a DDS pass.

The test uses a randomized `/food_segmentation_test_<suffix>/...` namespace.
While it is running, another sourced shell can inspect it with:

```bash
ros2 topic list | grep food_segmentation_test
ros2 topic echo --once /food_segmentation_test_<suffix>/metadata std_msgs/msg/String
ros2 topic echo --once /food_segmentation_test_<suffix>/overlay sensor_msgs/msg/CompressedImage
ros2 topic info -v /food_segmentation_test_<suffix>/metadata
ros2 topic info -v /food_segmentation_test_<suffix>/overlay
ros2 node list | grep food_segmentation_test
```

The test's `finally` block stops the Card, removes/destroys its client Node,
shuts down the Executor and calls `rclpy.shutdown()`. After completion,
`ros2 node list | grep food_segmentation_test` should return no matches; no
manual service restart or ROS daemon reset is required.

## Bumi/Jetson environment validation

Before selecting an ARM64 runtime, copy or check out the repository on the
robot host and run the read-only probe locally:

```bash
bash noetix/bumi/deploy/jetson_env_probe.sh > jetson_env_probe.txt
```

The script installs nothing, changes no power mode, modifies no Docker setting,
restarts no service, writes no configuration and uploads nothing. It omits the
hostname and network configuration. Its output can still include the Jetson
SKU, package versions, internal library paths and model filenames; review and
redact those fields before sharing outside the team.

Required validation evidence includes JetPack/L4T, Python, CUDA, TensorRT,
memory, Docker/NVIDIA runtime, OpenCV source, installed inference packages and
the approved checkpoint mount. The repository alone does not establish those
facts.

## Runtime choices

The current `UltralyticsSegmentationRuntime` supports the PyTorch/Ultralytics
route only. A bare ONNX Runtime or TensorRT engine is not a drop-in replacement.
Those routes need separate implementations of the existing
`SegmentationRuntime` protocol plus output, letterbox, Mask and bbox alignment
tests.

No production route is selected until the Jetson probe and license review are
complete.

## License and deployment boundary

The contract, pipeline, adapter boundary, ROS Card and tests in this repository
are project-authored code. Ultralytics, PyTorch, OpenCV, NumPy and the official
checkpoint are third-party components.

Ultralytics documents AGPL-3.0 and Enterprise licensing options. Embedded,
robotics, private or commercial distribution must be reviewed against the
actual use case: <https://www.ultralytics.com/license>. This document does not
assert that AGPL or Enterprise requirements have been satisfied.

Do not add Ultralytics, PyTorch or the checkpoint to the formal Bumi image
until maintainers approve the runtime, image-size impact, model distribution,
security/update policy and license path.

## Outstanding hardware validation

The following remain **NOT RUN**:

- real Bumi ROS 2 camera-to-processor DDS flow;
- real Canvas input binding and two-output consumption;
- Jetson ARM64 CPU/GPU inference and memory/temperature measurements;
- CUDA/TensorRT/ONNX compatibility;
- FastDDS fragmentation behavior for large metadata messages;
- formal Docker image build and checkpoint mount/hash verification;
- long-running start/stop, camera interruption and recovery on hardware.
