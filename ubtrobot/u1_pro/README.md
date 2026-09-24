# UBTECH U1 Pro driver

This driver exposes the U1 Pro capabilities used by Agent Core:

- `mic`: the vendor 16 kHz mono input stream as `audio/pcm-16k`.
- `speaker`: an Agent Core `audio/pcm-16k` input stream forwarded to the vendor output topic.
- `tts`: text-to-speech through the documented `play_text` service, with `interrupt`/`stop` and asynchronous completion. Raw audio and preset actions are intentionally not exposed by this card.
- `tts` also controls the shared U1 speaker volume with `set_volume` and `get_volume`; TTS and live speaker streams use the same device output volume.
- `expression`: face and gesture actions listed by readable names such as `smile` or `blink`; the driver maps these names to documented vendor IDs and only lists actions present in the robot's live command-motion list.
- `wakeup_control`: enables, disables, or queries the vendor's built-in wakeup and voice-interaction entry point. This is the documented `wakeup_enabled` switch, not a process/container lifecycle control.
- `visual_follow_control`: controls the documented visual system switch. Disabling it stops visual decisions, visual following, and visual idle actions, so it is the supported way to turn off built-in person-follow behavior.
- `head`: plays documented preset head motions (`nod`, `shake`, `tilt`, `look_up`, and `look_down`) by readable name. The SDK does not expose arbitrary head angles or low-level neck-joint control.
- `camera_left` and `camera_right`: the physical left- and right-eye RGB cameras exposed by the U1 perception runtime, published as separate JPEG topics.
- `vision_capture`: photo/video capture built on the left-eye JPEG cache. It supports `capture_image`, timed `record_video`, continuous `start_recording`/`stop_recording`, `list`, `delete`, and `info`.
- `doa_event`: an opt-in JSON sound-direction event stream.

Before exposing or registering any Agent Core cards, the driver authorizes the
vendor SDK from the read-only `U1_PRO_AUTH_FILE` JSON mount (the JSON's relative
`license_file` is read beside it), or from the legacy protected environment
variables. Startup fails if credentials are missing, the authorization request
fails, or the vendor response does not report `ok: true`, `code: "OK"`, and
`data.authorized: true`. Only after successful authorization does it disable the
vendor's built-in wake word. The authorization files must be provisioned on the
target host and must not be committed to the repository or image. Authentication
is a driver deployment concern rather than an Agent Core action. The playback event
topic remains an internal subscription used to complete `tts` actions; it is not
exposed as a separate Agent Core card.

The SDK document defines the event topics as `std_msgs/msg/String`. The `String.data`
field contains the vendor JSON envelope. The local `audio_msgs` package therefore only
contains the bridge audio messages and audio service definitions; it does not redefine the
vendor event topics, because a different DDS message type would not match the robot.

The camera card relies on the SDK video service response fields `path`,
`frame_payload_size`, and `max_frames`, and on video metadata fields `width`,
`height`, `step`, and `encoding`. It supports packed RGB/BGR/RGBA/BGRA/mono and
the U1 camera's `yuv422_yuy2` frames, converting them to JPEG; unknown encodings
are rejected rather than publishing corrupt images. The video shared-memory
path must be visible inside the driver container, as required by the vendor SDK
deployment.

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

Microphone input enables the Adapter's `/sys/device/audio_in/enable` service,
then subscribes to its `/sys/device/audio_in/raw` topic on domain `2` and
converts 16 kHz mono `AudioInData` messages to the Agent Core `audio/pcm-16k`
stream. Startup reports an error unless a supported PCM frame arrives. Speaker
output enables `/sys/device/audio_out/enable` when a connected stream starts and
publishes Agent Core PCM frames to
`/sys/device/audio_out/raw`; volume is read from `/sys/device/audio_out/current_volume`
and set through `/sys/device/audio_out/set_volume`. These device interfaces use
a dedicated domain `2` context selected by `audio_device_domain_id`.

The camera shared-memory ring uses a 64-byte ring header and 64-byte frame
headers, matching the SDK demo's cache-line-aligned `FrameHeader` definition.

The image installs `python3-pil` for the documented raw-video-to-JPEG conversion
and `ffmpeg` for MP4 capture,
and `python3-colcon-common-extensions`, `cmake`, and `build-essential`
only to build the local ROS interface packages during the image build. It installs the
CycloneDDS RMW used by the dual-domain runtime, `PyYAML` used by the shared driver
configuration loader, and NumPy 1.26.4 for vectorized conversion of the camera's
YUY2 frames. NumPy adds a Python wheel to this component image; the exact version
is pinned for reproducible ARM64/Python 3.10 builds. The official U1 SDK ROS2
runtime defaults to robot domain `20` and binds CycloneDDS to host loopback. The
deployment keeps Agent Core on domain `42`; `ROS_LOCALHOST_ONLY` remains unset
because the two contexts are configured explicitly by the driver.
`MaxAutoParticipantIndex` is increased to
200 because the U1 host already runs many ROS participants.

Local contract checks:

```text
python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 scripts/check_service_yml.py ubtrobot/u1_pro
```
