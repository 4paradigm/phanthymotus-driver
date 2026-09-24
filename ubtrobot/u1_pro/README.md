# UBTECH U1 Pro driver

This driver exposes the U1 Pro capabilities used by Agent Core:

- `mic`: the vendor 16 kHz mono input stream as `audio/pcm-16k`.
- `speaker`: an Agent Core `audio/pcm-16k` input stream forwarded to the vendor output topic.
- `tts`: text-to-speech through the documented `play_text` service, with `interrupt`/`stop` and asynchronous completion. Raw audio and preset actions are intentionally not exposed by this card.
- `tts` also controls the shared U1 speaker volume with `set_volume` and `get_volume`; TTS and live speaker streams use the same device output volume.
- `expression`: face and gesture actions exposed by declared readable names such as `smile` or `blink`; head motions are kept in the separate `head` card.
- `system_controls`: controls the three independent vendor switches `wakeup`, `wakeup_followup`, and `visual_behavior` from one card. Disabling `visual_behavior` stops vendor visual decisions, visual following, and visual idle actions; it does not stop explicitly requested expression/head motions.

The U1 SDK does not expose a documented switch for stopping or disabling the vendor's internal Agent process itself. Use `system_controls` to choose whether new wakeups, post-wakeup dialog continuation, and visual decisions/following/idle behavior are enabled; the driver does not change these settings during startup. It can interrupt current vendor playback/action through `tts.interrupt`. It cannot disable vendor ROS services, system processes, safety/control loops, or an already explicitly requested motion; those remain vendor-owned.
- `head`: plays the named preset head motions `look_down`, `look_up`, `nod`, `shake`, and `tilt` through the official typed motion service. The SDK does not expose arbitrary head angles or low-level neck-joint control.
- `camera_left` and `camera_right`: the physical left- and right-eye RGB cameras exposed by the U1 perception runtime, published as separate JPEG topics.
- `doa_event`: an opt-in JSON sound-direction event stream.

Before exposing or registering any Agent Core cards, the driver authorizes the
vendor SDK from the read-only `U1_PRO_AUTH_FILE` JSON mount (the JSON's relative
`license_file` is read beside it), or from the legacy protected environment
variables. Startup fails if credentials are missing, the authorization request
fails, or the vendor response does not report `ok: true`, `code: "OK"`, and
`data.authorized: true`. Startup intentionally does not change the vendor
wake-word, follow-up, or visual-behavior settings; use `system_controls` to change
them explicitly. The authorization files must be provisioned on the target host
and must not be committed to the repository or image. Authentication is a driver
deployment concern rather than an Agent Core action. The playback event topic
remains an internal subscription used to complete `tts` actions; it is not exposed
as a separate Agent Core card.

The U1 perception runtime publishes sound-direction events on
`/audio/sense/doa_event` as `audio_msgs/msg/DoaEvent`. The event card forwards
those events only while it is running; this is an event stream, not continuous audio.

The camera cards subscribe to the verified U1 perception runtime DDS topics
`/sensor/camera/left_eye/color/raw` and `/sensor/camera/right_eye/color/raw`,
converting each vendor `Image6m` frame to a JPEG output. The SDK
`open_stream` shared-memory service is a separate single-stream interface and is
not used for selecting the physical eyes.

Microphone input enables the Adapter's `/sys/device/audio_in/enable` service,
then subscribes to its `/sys/device/audio_in/raw` topic on domain `2` and
converts 16 kHz mono `AudioInData` messages to the Agent Core `audio/pcm-16k`
stream. Startup reports an error unless a supported PCM frame arrives. Speaker
output enables `/sys/device/audio_out/enable` when a connected stream starts and
publishes Agent Core PCM frames to
`/sys/device/audio_out/raw`; volume is read from `/sys/device/audio_out/current_volume`
and set through `/sys/device/audio_out/set_volume`. These device interfaces use
a dedicated domain `2` context selected by `audio_device_domain_id`.

The image installs `python3-pil` for the documented raw-video-to-JPEG conversion,
and `python3-colcon-common-extensions`, `cmake`, and `build-essential`
only to build the local ROS interface packages during the image build. It installs the
CycloneDDS RMW used by the dual-domain runtime, `PyYAML` used by the shared driver
configuration loader, and NumPy 1.26.4 for vectorized conversion of the camera's
YUY2 frames. NumPy adds a Python wheel to this component image; the exact version
is pinned for reproducible ARM64/Python 3.10 builds. The official U1 SDK ROS2
runtime exposes vendor control services on robot domain `20` and device streams on domain `2`, and binds CycloneDDS to host loopback. The
deployment keeps Agent Core on domain `42`; `ROS_LOCALHOST_ONLY` remains unset
because the two contexts are configured explicitly by the driver.
`MaxAutoParticipantIndex` is increased to
200 because the U1 host already runs many ROS participants.

Local contract checks:

```text
python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 scripts/check_service_yml.py ubtrobot/u1_pro
```
