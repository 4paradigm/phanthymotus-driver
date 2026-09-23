# UBTECH U1 Pro driver

This driver exposes the U1 Pro capabilities used by Agent Core:

- `mic`: the vendor 16 kHz mono input stream as `audio/pcm-16k`.
- `speaker`: an Agent Core `audio/pcm-16k` input stream forwarded to the vendor output topic.
- `audio`: documented `play_action`, `play_text`, motion listing, interruption, and asynchronous completion.
- `expression`: a separate face/light motion card backed by the vendor command-motion list. The agent must call `list_actions` and use an exact returned `motion_id`; no firmware-dependent smile/blink aliases are invented.
- `camera_rgb`: the documented U1 video stream. It opens the vendor stream, reads the section 4.5 shared-memory ring, and publishes `/namespace/camera/rgb` as `image/jpeg`.
- `vision_capture`: photo/video capture built on the `camera_rgb` JPEG cache. It supports `capture_image`, timed `record_video`, continuous `start_recording`/`stop_recording`, `list`, `delete`, and `info`.
- `doa_event`: an opt-in JSON sound-direction event stream.

At startup the driver authorizes the vendor SDK from the read-only
`U1_PRO_AUTH_FILE` JSON mount (the JSON's relative `license_file` is read beside
it), or from the legacy protected environment variables, then disables the
vendor's built-in wake word. The authorization files must be provisioned on the
target host and must not be committed to the repository or image. Authentication
is a driver deployment concern rather than an Agent Core action. The playback event
topic remains an internal subscription used to complete `audio` actions; it is not
exposed as a separate Agent Core card.

The SDK document defines the event topics as `std_msgs/msg/String`. The `String.data`
field contains the vendor JSON envelope. The local `audio_msgs` package therefore only
contains the bridge audio messages and audio service definitions; it does not redefine the
vendor event topics, because a different DDS message type would not match the robot.

The camera card relies on the SDK video service response fields `path`,
`frame_payload_size`, and `max_frames`, and on video metadata fields `width`,
`height`, `step`, and `encoding`. It supports the documented packed RGB/BGR/RGBA/
BGRA/mono raw formats and rejects unknown encodings rather than publishing
corrupt images. The video shared-memory path must be visible inside the driver
container, as required by the vendor SDK deployment.

The deployment shares the Adapter's live `/tmp/robo/ipc` directory and
`/dev/shm` with the driver. These are runtime IPC resources, not persistent
data: the Adapter recreates `video.stream`, `audio.stream`, and its socket
after a restart. Do not copy these files to the data volume; keep the bind
mounts present while both containers are running.

Captured media is stored under `/opt/phanthy-motus/data/vision_capture/u1_pro`,
which is mounted from the host by the deployment. `capture_image` returns a JPG
path immediately after a fresh frame arrives. `record_video` returns an action ID
and completes through Agent Core ACP after ffmpeg finishes the MP4; `duration`
defaults to 5 seconds and is capped at 60 seconds. The card never records a
stale frame as the first frame of a request. `start_recording` is an explicitly
manual lifecycle action: it returns a `recording_id` immediately and keeps
recording until `stop_recording`; the final result is returned by
`stop_recording` and `info`, rather than being treated as a finite ACP task.

The official Adapter runtime currently exposes device-level microphone enable
and raw audio on ROS domain `2`, while its `/robo/...` SDK services use domain
`20`. The driver therefore uses a dedicated audio-device context for
`/sys/device/audio_in/enable` and `/audio/sense/audio_data_to_asr`; this is
separate from the SDK context and Agent Core context. Configure
`audio_device_domain_id` if the vendor Adapter deployment changes that domain.

The image installs `python3-pil` for the documented raw-video-to-JPEG conversion
and `ffmpeg` for MP4 capture,
and `python3-colcon-common-extensions`, `cmake`, and `build-essential`
only to build the local ROS interface packages during the image build. It installs the
CycloneDDS RMW used by the dual-domain runtime and `PyYAML` used by the shared driver
configuration loader. The official U1 SDK ROS2 runtime defaults to robot domain
`20` and binds CycloneDDS to host loopback. The deployment keeps Agent Core on
domain `42`; `ROS_LOCALHOST_ONLY` remains unset because the two contexts are
configured explicitly by the driver. `MaxAutoParticipantIndex` is increased to
200 because the U1 host already runs many ROS participants.

Local contract checks:

```text
python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 scripts/check_service_yml.py ubtrobot/u1_pro
```
