# TRON2 head visual follow

An independent MCP actuator card that turns the head to follow a person selected
near the middle of the head-camera image, then returns to pitch/yaw 0° after the
person leaves. It reuses the installed TRON2 `move_head` driver and existing YOLO
results; it does not control the arms/grippers or alter the original driver.

## Card operations

| Operation | Behavior |
| --- | --- |
| `follow` | Select a central person across two distinct frames and follow their upper torso. |
| `pause` | Stop sending further targets; an already accepted turn can finish. |

Lifecycle `start` and diagnostic `info` are supported internally but hidden by
the existing Canvas renderer. There is no visible `status` action. Startup,
container restart and project lifecycle start never enable following.

The service is `TRON2 Head Visual Follow`; its single tool is
`head_visual_follow`, with no user tuning fields other than `action`.

## Tracking and motion behavior

- Acquire near the image center; associate subsequent person boxes by overlap,
  displacement and size. Hold when matching is ambiguous. This is spatial
  tracking, not face identity recognition, and does not guarantee identity
  through crossings or long occlusions.
- Short occlusions hold the current direction. At least three fresh negative
  observations over 1.5 seconds trigger a return to neutral. Camera/inference
  failure or stale images never count as evidence that the person has left.
- Each request sends the entire currently computed target. There is **no 4°
  per-request cap**. Duration scales with angular distance at approximately
  6°/s; target limits remain pitch −55°…30° and yaw ±45°.
- Commands are serialized through `move_head`. Matching action IDs, acknowledged
  completion, fresh feedback and driver release are required before another
  target. Only that action's explicit stationary proof can replace a duplicate
  settling window; initial admission still requires stable observations.
- Only frames received after the completed turn can produce the next target.
  This remains a sequence of MoveH turns with arrival checks, **not continuous
  servo control or in-flight replanning**. Fast crossings can outpace the input
  detector or leave the association gate.
- External head control, arm motion/displacement, unknown request results or
  driver faults disable following. No automatic retries, mode changes or error
  resets are sent. `pause` is not an emergency stop and cannot cancel a turn
  already accepted by the existing driver.

## Dependencies and integration

Use the existing, field-verified TRON2 0.11.x motion driver on loopback `15791`.
Older/different versions fail the compatibility gate and require a separate
compatibility review; this PR does not upgrade the main driver or teaching.
The native VR head follower must be disabled for the existing manual head
interface; this service never changes that setting.

A separately installed detector must already be running on loopback `15792`
with the **head camera** `/limx_tron2/camera` as input and `person` in its model
vocabulary. No new model is bundled or downloaded. Wrist-camera input is refused.
The service reads `/health` from these two local dependencies and only posts
`move_head` requests to `/mcp` on `15791`. It neither starts nor reconfigures the
detector, camera streams, Core or robot driver. Keep this card paused while
using other motion/teaching cards.

The detector health contract used by `parse_frame()` is:

```json
{
  "active": true,
  "available": true,
  "input_topic": "/limx_tron2/camera",
  "last_error": null,
  "latest_result": {
    "available": true,
    "input_topic": "/limx_tron2/camera",
    "box_coordinate_system": "original_image_pixels",
    "frame_sequence": 123,
    "source_received_at": 1793275200.0,
    "published_at": 1793275200.2,
    "source_header_stamp": {"sec": 1793275200, "nanosec": 0},
    "image_width": 640,
    "image_height": 480,
    "detections": [{"label": "person", "confidence": 0.9,
                    "box_xyxy": [200, 50, 380, 450]}]
  }
}
```

Timestamps above are illustrative. Receipt-to-use age must be ≤1.5 seconds and
frame identifiers must advance. This validates local freshness, not hardware
clock synchronization. See `client.pose()` for the required driver feedback
contract and `terminal()` for completion checks.

## Run

No Python packages beyond the standard library are required (Python 3.9+).
From this directory on the Jetson:

```bash
python3 server.py
```

Or build/run this directory as a **separate service**:

```bash
docker build -t tron2-head-visual-follow:0.1.2 limx/tron2/head_follow
docker run -d --name tron2-head-visual-follow --network host --read-only \
  --user 65532:65532 --cap-drop ALL --security-opt no-new-privileges \
  --pids-limit 64 --memory 128m --cpus 0.5 \
  tron2-head-visual-follow:0.1.2
```

The Docker build context above is relative to the repository root. The server
binds only `127.0.0.1:15793`. Register `http://127.0.0.1:15793/mcp` as an HTTP MCP
service in Motus to expose the card. It has no DDS input wiring of its own;
keep the existing head-camera→YOLO connection. `driver.yaml` describes this
standalone service; the parent TRON2 image/catalog is unchanged. Do not start a
second instance if the field-installed follow service already owns this port.

To check without movement, use `GET /health` or tool action `info`. To begin,
select `follow` and execute; to stop sending new targets, select `pause`.
After stopping the service, any command already accepted by the robot driver
can still complete. No automatic recenter is sent during shutdown.

## Validation

```bash
cd limx/tron2/head_follow
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
# Enable actual loopback HTTP tests where socket binding is permitted:
FOLLOW_HTTP_TEST=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s tests -v
```

The tests use fake joint feedback/detections and never contact hardware. Docker
builds run the real loopback HTTP checks as well. Tests cover full targets larger
than 4°, duration scaling, initial stationarity, exact-action proof reuse,
external-control conflicts, unknown send outcomes, fresh-frame gating,
ambiguous matches, person loss/reacquisition and lifecycle startup remaining idle.

The operator has confirmed that head following works on the physical TRON2 and
requested the final full-target variant for field testing. This is operator
acceptance, not a measured latency, servo smoothness or tracking-accuracy claim.
Site-specific update archives, container replacement journals, personal paths
and deployment backups are intentionally excluded from this contribution.
