# PiPER dual color cameras

One PiPER service can register `piper_camera_live` (On camera) and the optional
`piper_camera_live_b` (On camera B). Both reuse the same camera supervisor,
each with its own worker, cancellation token, device lease and ROS publisher.
The existing PiPER runtime owns the single CAN interface. The live camera tools
have start, stop and info operations. A single-camera configuration still
registers the original tool and topic.

## Operator configuration

1. Before mapping devices, identify the two physical units by serial and run
   `v4l2-ctl --list-devices` and
   `v4l2-ctl --device "$PIPER_CAMERA_A" --list-formats-ext` for each candidate.
   Verify actual color frames and the symlink targets with `readlink -f`.
   Use two different physical serials and the corresponding
   `/dev/v4l/by-id/*-video-indexN` color paths.
2. Copy the shipped `config.yaml` to a site-specific file. Set `camera.camera_id`,
   `camera.device`, `camera.live.enabled: true` and the corresponding
   `camera.secondary.camera_id`, `camera.secondary.device`,
   `camera.secondary.live.enabled: true`. The default config documents the
   dimensions, quality and FPS options. Set the
   `PIPER_CAMERA_A`, `PIPER_CAMERA_B` and `PIPER_DUAL_CAMERA_CONFIG` Compose
   variables. The device strings in YAML and Compose must match exactly.
3. Render the site Compose with `deploy/dual-camera.compose.yml` and inspect
   the resulting mounts and environment before starting. Apply this override
   in place of the single-camera override. It maps exactly the two selected
   device nodes and mounts the supplied config read-only. On re-enumeration
   that changes the device node, recreate the container after validating
   the by-id targets on the host. Keep the motion opt-in disabled for camera
   acceptance.
4. On the Dashboard add the two On camera cards and inspect
   `…/camera/live` and `…/camera/live_b`. The ROS node names are derived
   from distinct topics. The service uses the existing HTTP/MCP port.
   The camera lease uses Linux character-device major/minor where available,
   including when different paths alias the same active device.

## Message contract

The image is `sensor_msgs/msg/CompressedImage` with `format=jpeg`.
With `camera_id` configured, `header.frame_id` is the versioned provenance
tag below, carried atomically with the JPEG and its stamp.

```text
motus.capture/1|camera_id|optical_frame_id|capture_session_id|frame_sequence|host_read|device_id
```

- `camera_id` is the configured ID, and the optical frame label is
  `<camera_id>_color_optical_frame`. A consumer using TF must extract the
  optical label from the tag.
- `capture_session_id` is a random 32-hex UUID generated each time this
  individual capture worker opens. Sequence numbers are strictly increasing
  within that session, with gaps possible when the mailbox drops frames.
- `device_id` is the first 32 lowercase hex characters of SHA-256 over the
  exact by-id path string. This binds calibration to an operator-verified
  device selection. The fingerprint describes the path binding.
- The header stamp is host Unix wall time immediately after OpenCV returns
  from V4L2 read, before JPEG encoding. The driver can measure publication
  time separately through MCP `info`. OpenCV supplies no verified hardware
  exposure timestamp in this path. Measure host-read error at the installed
  frame rate before relying on a synchronization tolerance.
- The capture-tagged topics publish valid images only. During loss the topic
  goes quiet and MCP `info` reports the offline state. For legacy config
  without a camera ID, Monitor still receives the offline JPEG once per retry
  with zero capture stamp and `motus.capture/1|offline` in `frame_id`.
  Consumers can exclude it by metadata without inspecting pixels.
- The single-camera JPEG path provides RGB. Registered depth, camera intrinsics
  and workbench extrinsics come from separately verified sources.

Test from the Driver root on Linux with Python 3.10 or newer and the
existing camera dependencies.

```sh
python3 -m unittest discover -s tests -p test_piper_camera_live.py
python3 -m unittest discover -s tests -p test_piper_unified.py
```
