# UBTECH U1 Pro driver

This driver exposes the U1 Pro capabilities used by Agent Core:

- `mic`: the vendor 16 kHz mono input stream as `audio/pcm-16k`.
- `speaker`: an Agent Core `audio/pcm-16k` input stream forwarded to the vendor output topic.
- `audio`: documented `play_action`, `play_text`, motion listing, interruption, and asynchronous completion.
- `doa_event`: an opt-in JSON sound-direction event stream.

At startup the driver authorizes the vendor SDK from protected configuration or
environment variables, then disables the vendor's built-in wake word. Authentication
is a driver deployment concern rather than an Agent Core action. The playback event
topic remains an internal subscription used to complete `audio` actions; it is not
exposed as a separate Agent Core card.

The SDK document defines the event topics as `std_msgs/msg/String`. The `String.data`
field contains the vendor JSON envelope. The local `audio_msgs` package therefore only
contains the bridge audio messages and audio service definitions; it does not redefine the
vendor event topics, because a different DDS message type would not match the robot.

The image installs `python3-colcon-common-extensions`, `cmake`, and `build-essential`
only to build the local ROS interface packages during the image build. It installs the
CycloneDDS RMW used by the dual-domain runtime and `PyYAML` used by the shared driver
configuration loader. The deployment uses host networking and loopback-bound
`CYCLONEDDS_URI`; the repository DDS checker reports this as a known FastDDS-checker gap.

Local contract checks:

```text
python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 scripts/check_service_yml.py ubtrobot/u1_pro
```
