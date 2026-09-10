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
| `capture_photo` | Wait for a new JPEG frame, save it, return `file_path` synchronously (no ACP). |
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

Omit `duration_s` to use the default of 5 seconds. `maximum` is 30 seconds.
Omit `camera` to use the saved card configuration.

`queued` is an admission response, not completion. The terminal result is posted
once to Core's `/api/acp/complete` and also remains in `info.last_recording`
until restart. Only one recording may be active. Camera previews keep running
when recording is cancelled. Missing, stale or stalled input produces an error.

The result contract matches the Q5 vision_capture card. A successful
`record_video` completion carries only the Q5 slim field set, and the honored
requested duration is returned after encoding:

| Field | Meaning |
| --- | --- |
| `ok` / `media_type` | Success flag and `video` media type. |
| `file_path` | Absolute path to the completed MP4 on the Go2 host. |
| `recorded_duration_s` | Completed MP4 duration measured by ffprobe. |
| `frames` | Fresh source frames encoded. |
| `captured_at` | Filesystem stamp ISO timestamp with timezone offset when encoding finished. |

Failure and cancellation result carry only `ok`, `code` and `message` — no extra
timing/display fields. `capture_photo` returns `file_path` synchronously and
never posts ACP, as on Q5. No extra display/verification fields are added so the
saved file is rendered by Core through the existing ACP `file_path` rendering
with zero Core changes.

Video output preserves capture timing and extends the last frame to the exact
requested endpoint, so a 5-second recording at 15 fps contains 75 encoded frames.
`frames` counts fresh source frames received. Padding does not replace camera
freshness checks or make a stalled capture succeed.

## Files and deployment

Default storage is `/opt/phanthy-motus/data/vision_capture/photos` and `videos`.
The existing Go2 Compose data mount preserves completed files across container
replacements. A custom `output_dir` needs its own persistent mount.

The Dockerfile includes FFmpeg and copies the plugin. Configuration under
`plugins.vision_capture` controls `enabled`, default `camera`, `output_dir`,
`fps` (1–15) and `max_duration_s` (1–30).

As with Q5, the card returns saved paths; it does not add inline media previews.
Download files to inspect them:

```bash
mkdir -p ~/Downloads/go2-photos
scp 'unitree@GO2_IP:/opt/phanthy-motus/data/vision_capture/photos/*.jpg' ~/Downloads/go2-photos/
```

No Core modification is required for the driver's completion protocol: single
ACP terminal callback, `file_path` in the result, and no display-only fields.

## Validation

On 2026-09-08, Go2's built-in camera and an external D435i RGB source each
produced valid 1280x720 JPGs and 5.000000-second H.264 videos with 75 decoded
frames. Cancellation removed partial MP4s. Concurrent sampling verified valid
front/RGB/depth/infrared data and nonzero ROS publishers.

On 2026-09-09 the contract was aligned to the Q5 card: `duration_s` default is
5 (maximum 30), `record_video` posts a single ACP `completed` whose result
carries only `file_path`/`recorded_duration_s`/`frames`/`captured_at`, and the
previous extra timing/display fields were removed. All 28 local capture checks
(including real FFmpeg/ffprobe media) pass; ARM64-image verification on the Go2
runs after the release image deploys.
'''