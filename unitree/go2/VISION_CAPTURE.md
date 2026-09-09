# Photo and video capture

`vision_capture` saves RGB photos as JPG and records silent H.264 MP4 videos.
It follows the Q5 capture card's asynchronous action contract and shares the
existing Go2 camera publishers, so it does not open a camera device twice.

## Select a camera

Select `front` for the built-in front camera, or `external` for an external
camera. RGB-only support is explained in the field description; no Core
dropdown-rendering change is needed. The card exposes no `external_instance_id` input.

Start the source camera first. For an external camera, configure and start an
[`ext_camera`](EXT_CAMERA.md) instance with `channel: rgb`. The capture card
automatically selects the only running RGB instance. Depth and infrared are
excluded; they may keep running alongside RGB. If multiple RGB instances are
running, keep only the intended RGB instance active before capture.

Selecting an unavailable external source returns an error and does not fall
back to the front camera. Photo and video requests retain their selected source
throughout the operation. Changing saved camera configuration during recording
is rejected; repeating the same configuration is accepted.

## Actions

| Action | Result |
| --- | --- |
| `capture_photo` | Wait for a new JPEG frame, save it, return `file_path` and source metadata. |
| `record_video` | Queue a 1–30-second recording, default 5 seconds, and return `action_id`. |
| `list_cameras` | List the built-in source and running external RGB instances. |
| `info` | Report source freshness, output directories, active recording and latest terminal result. |
| `start` | Subscribe to the configured source; report readiness based on actual frames. |
| `stop` | Cancel recording, remove incomplete output and report cancellation through ACP. |

Example MCP arguments for tool `vision_capture`:

```json
{"action": "capture_photo", "camera": "front"}
```

```json
{"action": "record_video", "camera": "external", "duration_s": 5}
```

Omit `camera` to use the saved card configuration. `queued` is an admission
response, not completion. The terminal result is posted to Core's
`/api/acp/complete` and remains available in `info.last_recording` until restart.
Only one recording may be active. Camera previews keep running when recording
is cancelled. Missing, stale or stalled input produces an error.

A successful video result includes `file_name`, the full `file_path`, and a
human-readable `message` naming the saved file. These fields are added only
after encoding and file validation succeed; cancellation does not advertise an
incomplete file as a saved result.

The queued response includes `queued_at`. A successful ACP completion is sent
only after FFmpeg exits successfully and ffprobe verifies the completed MP4's
duration and frame count. The result separates media duration from elapsed time:

| Field | Meaning |
| --- | --- |
| `requested_duration_s` | Requested video length. |
| `recorded_duration_s` | Completed MP4 duration measured by ffprobe. |
| `capture_elapsed_s` | Actual time spent collecting source frames, measured with a monotonic clock. |
| `finalize_elapsed_s` | Time spent finalizing and validating the file after collection ends. |
| `elapsed_s` | Total time from queue admission to terminal result, excluding callback network delivery. |
| `queued_at`, `capture_started_at`, `capture_finished_at`, `completed_at` | Millisecond ISO timestamps with explicit timezone offsets. |

For a 5-second recording, the MP4 must measure 5 seconds; the total elapsed time
can be longer due to waiting for the first frame and finalizing the file.
Failed/cancelled results still include request, queue, completion and total-time
fields. The single ACP terminal callback follows validation or failure cleanup.

Video timestamps preserve capture timing. Output uses the configured frame
rate and extends the final frame to the requested endpoint. At 15 fps, a
5-second video contains 75 encoded frames. `frames` counts fresh source frames;
`encoded_frames` includes repeats used to preserve timing. Padding does not
replace camera freshness checks or make a stalled capture succeed.

## Files and deployment

Default storage is `/opt/phanthy-motus/data/vision_capture/photos` and `videos`.
The existing Go2 Compose data mount preserves completed files across container
replacements. A custom `output_dir` needs its own persistent mount.

The Dockerfile includes FFmpeg and copies the plugin. Configuration under
`plugins.vision_capture` controls `enabled`, default `camera`, `output_dir`,
`fps` (1–15) and `max_duration_s` (1–30).

As with Q5, the card returns saved paths; it does not add inline media previews.
Download files to inspect them, for example:

```bash
mkdir -p ~/Downloads/go2-photos
scp 'unitree@GO2_IP:/opt/phanthy-motus/data/vision_capture/photos/*.jpg' ~/Downloads/go2-photos/
```

No Core modification is required for the driver's completion protocol or timing
fields: it uses the existing ACP endpoint and `info` result. Core's optional
activity-log forwarding is a separate observability fix, not a prerequisite for
recording or correct terminal signaling.
For automatic user-facing file receipts, Core must retain the file-result fields
when compacting ACP events. Legacy Core versions discard all `result` data and
cannot deliver the saved path. A separate minimal Core fix preserves file receipts
and sends their filename/path through the existing activity `trigger` renderer;
it does not modify frontend files or change the capture protocol.

## Validation

On 2026-09-08, Go2's built-in camera and an external D435i RGB source each
produced valid 1280x720 JPGs and 5.000000-second H.264 videos with 75 decoded
frames at 15 fps. Cancellation removed partial MP4s. Concurrent camera sampling
verified valid front/RGB/depth/infrared data and nonzero ROS publishers.
ACP completion and cancellation were also observed through the separately
updated Core activity stream and browser.

Local and ARM64-image checks covered selection, stale input, duration validation,
exact output timing at slow source rates, encoder failures and cancellation.
Local verification scripts are intentionally not included in this driver change.
On 2026-09-09, the timing metadata and ffprobe completion gate passed real Go2
verification on both sources. Each MP4 measured 5.000000 seconds and decoded to
75 frames. Measured collection time was 5.001 seconds for each source; queue
admission to file readiness took 5.335 seconds (front) and 5.328 seconds (external).
ACP reached Core 0.095 and 0.141 seconds after file readiness, respectively.
Cancellation also reported ordered timestamps and elapsed time. The Core dropdown
title patch was rolled back, and the temporary external test instance was stopped.
All 27 capture checks passed in the ARM64 image; 63 local Go2 checks passed before
deployment. Verification scripts remain local and are not part of this change.
The file receipt was subsequently verified with a real 5-second recording: the
existing browser log displayed the complete filename and path immediately from
the ACP callback, and frontend JavaScript matched the original Core image.
