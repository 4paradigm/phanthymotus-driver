# Tianyi 2.0 Pro Driver

Phanthy Motus driver bundle for the Tianyi 2.0 Pro humanoid robot. The driver
bridges robot-side ROS2 topics on domain 0 to Agent Core topics on domain 42 and
exposes the capabilities as MCP tools.

## Head camera snapshot card

The `vision_capture` card subscribes to `/ob_camera_head/color/image_raw` and
provides `capture_image`, `record_video`, `start_recording`, `stop_recording`,
`list`, and `delete` actions. Each capture or recording is saved under
`/opt/phanthy-motus/data/images` and returns a channel-visible path. The
driver derives the corresponding `/work/resource/...` path from the persistent
mount (or uses `PHANTHY_CHANNEL_OUTPUT_DIR` when a deployment uses another
mount). Use that returned path with `channel_reply(files=[{"path": "..."}])`
to send media through Feishu or Telegram. Use `image_name` (without `.jpg`)
for photos and `video_name` (without `.mp4`) for videos. To delete media, pass
Use `delete` with the complete filename in `name`, including `.jpg` or `.mp4`.

## Wrist camera card (`wrist_camera`, multiple instances)

Add `wrist_camera` to the canvas once per desired view. Each instance exposes
**RGB first, Depth second**; each output can be connected independently:

| Port | Output topic | Format |
| --- | --- | --- |
| RGB | `/<namespace>/wrist_camera/<instance_id>/<side>_rgb` | `image/jpeg` |
| Depth | `/<namespace>/wrist_camera/<instance_id>/<side>_depth` | `image/depth-zlib` |

Hyphens in a canvas instance ID become underscores in ROS topic names.
Conflicting normalized instance IDs are rejected. The final topic segment is
`left_rgb`, `left_depth`, `right_rgb` or `right_depth`, so existing Core monitor
panels distinguish wrist sides without frontend changes. Changing the device
keeps the topic; changing the wrist side changes its suffix. After first binding,
changing side, or upgrading from the previous `/rgb` and `/depth` suffixes, refresh
the canvas/monitor to resolve the current instance topics and connections.
The four former `camera_wrist_{left,right}_{rgb,depth}` cards are replaced by
this tool. Existing canvases using those names must replace the old cards,
select their devices, and reconnect RGB/depth to the corresponding ports.

### Select a device and wrist side

Use the card's instance settings (the same mechanism as `ext_mic`). This
change does not relocate settings to the resource sidebar or modify Agent Core.
An unconfigured newly dropped card must be bound before it can supply data:

| Setting | Values | Meaning |
| --- | --- | --- |
| `side` | `left`, `right` | Physical wrist label; required, never inferred |
| `device_id` | detected device or `auto` | Stable physical device identity |
| `fps` | `auto`, `15`, `30` | A supported native frame rate |
| `resolution` | `auto` or a discovered size | Native RGB resolution; depth stays 640x480 |

Confirm physical placement from a label, cable, or preview. If cameras are
installed on the opposite wrists, change the device selection or side labels.
No image rotation, flip, mirror, resizing, or depth-to-colour alignment is
performed. The card does not establish a wrist extrinsic calibration.

`auto` binds only when exactly one RGB-capable device is available. The resolved
identity is saved; subsequent configuration replays and hotplug events never
switch it to another camera. With multiple devices, select the intended camera.
RealSense identities use the SDK serial (without assuming a serial length or a
D405 model name). UVC uses `/dev/v4l/by-id` when available, otherwise its USB bus
path: cameras without unique serials should be rechecked if moved to another
USB port. Renumberable `/dev/videoN` indices are not saved as identities.

### Discovery and supported streams

Discovery runs in an isolated process and refreshes periodically (three seconds
between scans, with an eight-second scan deadline). Updated device choices are
advertised through `tools/list`; reopen the settings after Agent Core refreshes
the driver definition. `info` also reports `available_devices` and
`discovery_errors`. Querying `info` does not start or restart capture.

- RealSense: enumerate native RGB8/BGR8 and Z16 profiles using the installed
  `pyrealsense2` SDK, without a camera-model whitelist. This is not a guarantee
  that every RealSense model/firmware works with the pinned SDK.
- Standard USB/UVC: enumerate capture formats with `v4l2-ctl` and capture through
  OpenCV/V4L2. Metadata nodes and duplicate interfaces on one USB bus are
  excluded. UVC provides **RGB only**; the depth port remains in position 1 and
  reports `streams.depth.state: unavailable`. No synthetic depth is generated.
- SDK-owned RealSense/Orbbec nodes are excluded from generic UVC capture to
  avoid duplicate devices, IR/depth mistaken for colour, or stealing the head
  camera. Other vendors' RGB-D SDKs are not implemented by this change.

The container must see the camera's USB devices; the existing privileged
`/dev:/dev` mount supplies this. Cameras attached to another compute board need
capture/transport on that board and are not discovered by local enumeration.

The RGB resolution dropdown is built from the discovered native profiles, with
device names in the labels. It lists the union of usable sizes across devices;
existing Core does not dynamically filter it when the device field changes.
Reopen settings after the tool definition refreshes to see hotplug changes.
The driver validates the chosen device and frame rate when applying settings,
and retains saved offline sizes labelled as currently unavailable. Sizes that
cannot pair with native 640x480 depth at any common frame rate are excluded.

`auto` prefers 15 FPS and native 640x480 where supported, then selects a common
native frame rate and available RGB profile. Explicit RGB size selection does
not resize or change native depth: RGB may be 1280x720 while depth stays 640x480.
Unsupported requests fail with a clear error. **Native depth must be 640x480**, because the current Core
`image/depth-zlib` decoder assumes that size. Other native depth sizes are
reported as unsupported, not resized. Depth is the SDK Z16 data converted using
its actual depth scale to little-endian uint16 **millimetres**, then zlib
compressed; zero means invalid, and out-of-range values do not wrap.

### Preview, sharing, stop and reconnect

Saving a valid instance configuration starts its preview without **Start
Intelligent Control**. Instance bindings and requested preview states are saved
atomically under `/opt/phanthy-motus/data/wrist-camera.json` (the mounted data
directory), so an active preview can be restored after a driver restart.
Saving or replaying the same configuration keeps healthy capture running and
resumes a stopped preview without **Start Intelligent Control**.
Global `info()` advertises enabled instances' topics for Core's periodic
discovery and monitor registration. Allow the next driver heartbeat (up to
30 seconds) after saving a new instance for monitoring to discover its streams.
Removing a card or stopping its project sends `stop(instance_id)` and disables
its preview. Driver shutdown closes all capture resources while retaining the
saved requested state. Do not run two Tianyi driver services against one state
file or the same cameras.

Instances bound to the same physical camera share **one** capture process and
one native stream configuration, with separate topic pairs. Their `side`
labels may differ. Conflicting frame-rate/resolution requests report an error;
stop other users before changing shared capture settings. Stopping one instance
removes only its publication routes; stopping the last releases the device.

Route updates are acknowledged after any in-flight publication completes.
After a successful stop returns, that instance receives no new publications;
DDS messages already queued in transport/subscribers cannot be retracted. An
unresponsive worker is closed after a bounded route-update timeout, and the
remaining requested instances are restarted by the monitor. A native SDK crash
cannot terminate the body-controller process.

A requested instance reports `loading` until fresh frames arrive, `running`
when all supported channels are fresh, and `error` on capture failure or a
12-second first-frame timeout. `streams` reports per-channel freshness/errors;
a healthy RGB stream cannot hide a missing required depth stream. After the
startup grace, a living worker with no fresh supported stream is restarted,
including native startup/read hangs. A fresh sibling stream is preserved. A selected
camera's disappearance stops its session and reports an error; reinserting the
same device retries only instances that remain enabled. Discovery failure alone
does not tear down a healthy capture. Capture requires the loopback-only
`/opt/phanthy-motus/dds-local.xml` profile and publishes on domain 42, without
changing the vendor's domain-0 participant or head-camera services.

RGB/depth remain **unaligned**. ROS timestamps use the host clock, with raw SDK
timestamps reported separately when available; no hardware synchronization is
claimed. Actual per-stream SDK intrinsics are exposed through `camera_info`;
uncalibrated UVC optics remain unknown.

### Runtime dependencies

The image installs `v4l-utils` for UVC capability discovery and pins the prebuilt
Linux ARM64 / CPython 3.10 wheel `pyrealsense2==2.56.5.9235`; no SDK source build
or pip cache is included. All Python dependencies remain in `requirements.txt`.
The wheel is 4,338,854 bytes downloaded / 13,796,155 bytes unpacked (about
4.14 / 13.16 MiB); these are package sizes, not a measured image-size delta.
`libusb-1.0-0` remains an explicit runtime dependency because that wheel declares
`libusb-1.0.so.0` in ELF `DT_NEEDED` and the checked base image did not supply it.

## Raw arm joint card

`arm` directly commands seven joints per selected arm in this canonical order:

```text
[shoulder_pitch, shoulder_roll, shoulder_yaw,
 elbow_pitch, wrist_yaw, wrist_pitch, wrist_roll]
```

The card always exposes separate `left_positions` and `right_positions` arrays
in degrees and publishes both arrays in one 14-joint command, so the arms can
move to different poses simultaneously. There is no `side` selector. Raw arm
input is never mirrored automatically. To create a mirrored right-arm pose manually, negate
shoulder roll, shoulder yaw, wrist yaw, and wrist roll (indices 1, 2, 4, and 6)
from the left pose. Right-arm motor IDs are 21-27 and left-arm IDs are 11-17.
`move_pos` accepts speed [0.2, 1.5] rad/s. `move_ctrl` accepts seven-element
`kp` [10, 200] and `kd` [5, 50] arrays shared by both arms. Raw-control defaults
are `kp=50` and `kd=20` for every joint. The bounded tuning range includes the
tested lower-gain and original-default combinations while excluding zero
position stiffness and the previous unverified extreme upper bounds. These are
still not vendor-certified gains.

`move_pos` uses the rated-current limits from the Tianyi joint specification
table instead of one fixed current for every arm motor. In joint order, each
arm uses `[35, 23, 8, 8, 8, 5, 5]` A for shoulder pitch, shoulder roll,
shoulder yaw, elbow pitch, wrist yaw, wrist pitch, and wrist roll. The same
limits apply to left motor IDs 11-17 and right motor IDs 21-27. Raw and semantic
head position commands use 5 A for motor IDs 1-3. `move_ctrl` has no current
field and is unaffected by this mapping. Do not increase these values beyond
the specification; clear the workspace and keep the emergency stop available
when validating the higher-current shoulder joints.

`move_ctrl` sends a position step with target speed and feed-forward torque set
to zero. The vendor-side controller applies `kp` and `kd`; this driver does not
calculate the PD output or impose a trajectory speed limit. Use small target
changes, begin with low gains, keep the emergency stop available, and use
`move_pos` when bounded motion speed is required.

In practical terms, `move_ctrl` is intended for controller commissioning,
pose holding and already-validated compliant interaction where the operator
needs to tune joint stiffness and damping. Higher `kp` generally increases
position response and holding stiffness but can increase impact and overshoot;
lower `kp` is more compliant but can leave a larger load-dependent position
error. Higher `kd` generally adds damping, while insufficient `kd` can allow
overshoot or oscillation. The exact control law runs in the vendor controller,
so these effects are guidance rather than a driver-side torque calculation.
The same seven-element gain arrays are applied to corresponding joints on both
arms. Do not use `move_ctrl` as a substitute for `move_pos` for routine poses,
large position steps or semantic gestures.

Both modes reject malformed arrays and poses outside the checked-in URDF
limits. They check fresh `/arm/status`, selected motor faults, emergency stop,
and power state before publishing, then wait for newer feedback. Do not command
the raw `arm` and `arm_gesture` cards concurrently because both can publish to
`/arm/cmd_pos`.

For compatibility with dashboard array fields, `left_positions`,
`right_positions`, `kp`, and `kd` accept either native JSON arrays or strings
containing a JSON array. Both forms are decoded into seven numeric values before
the same range and URDF checks run. Direct callers using the former `positions`
field remain supported as a fallback, but its values are sent unchanged to both
arms and are no longer mirrored.

## Head gesture card

`head_gesture` turns safe, bounded head-position commands into cancellable
semantic sequences.

| Item | Value |
|---|---|
| Tool name | `head_gesture` |
| Tool type | `actuator` |
| Robot-side output | `/head/cmd_pos` |
| Message type | `bodyctrl_msgs/msg/CmdSetMotorPosition` |
| Actions | `nod`, `shake`, `scan`, `tilt`, `reset`, `stop` |

Yaw, pitch and roll are clamped to the limits already documented by the raw
`head` card. Starting a new gesture cancels the remaining frames of the previous
gesture; `stop` cancels future frames without issuing an additional pose. A nod
moves from neutral to positive pitch (down) and back to neutral; it never passes
through negative pitch (up).

Before publishing, the card checks fresh `/head/status` data, all three head
motors, motor error codes, emergency-stop state and power state. It then waits
up to two seconds for newer status and verifies that the head moved or was
already at the target. A verified call returns `feedback_verified: true`;
failures return `state: error`, a stable diagnostic `code`, details in the MCP
result, and protocol-level `isError: true`.

| Action | Dashboard defaults | Allowed range |
|---|---|---|
| `nod` | cycles 2, amplitude 12°, speed 30°/s | cycles 1–5, amplitude 5–20°, speed 5–60°/s |
| `shake` | cycles 2, amplitude 25°, speed 30°/s | cycles 1–5, amplitude 5–45°, speed 5–60°/s |
| `scan` | cycles 2, amplitude 25°, speed 30°/s, hold 1.0s/side | cycles 1–5, amplitude 5–45°, speed 5–60°/s, hold 0.2–3.0s/side |
| `tilt` | left, amplitude 12°, speed 30°/s, hold 0.8s | side left/right, amplitude 5–20°, speed 5–60°/s, hold 0.2–3.0s |
| `reset` | speed 30°/s | speed 5–60°/s |
| `stop` | no parameters | no parameters |

## Arm gesture card

`arm_gesture` provides semantic arm motions on top of the existing joint-level
`arm` card.

| Item | Value |
|---|---|
| Tool name | `arm_gesture` |
| Tool type | `actuator` |
| Robot-side output | `/arm/cmd_pos` |
| Message type | `bodyctrl_msgs/msg/CmdSetMotorPosition` |
| Actions | `salute`, `welcome`, `raise`, `shake_hands`, `high_five`, `reset`, `stop` |

The right-arm poses are mirrored from the left-arm definitions and remain
inside the checked-in URDF limits. The preset poses still require low-speed,
clear-area calibration on the target robot before production use. Do not run
the raw `arm` card and `arm_gesture` concurrently because both publish to the
same controller topic.

The semantic poses use a preparation frame before the final gesture. For
`welcome`, `salute`, and `high_five`, shoulder roll and shoulder yaw first
establish the elbow-flexion plane; elbow pitch can then raise the forearm
instead of leaving it horizontal in front of the torso. `salute` uses two
blended stages: the preparation frame bends the elbow and moves wrist yaw to
25 degrees; the second frame raises the upper arm laterally, flexes the elbow
to approximately 110 degrees, and completes wrist yaw at 50 degrees. In this
URDF chain, shoulder yaw rotates the downstream
elbow-pitch axis: positive left-arm yaw (mirrored negative on the right) makes
negative elbow pitch lift and fold the forearm inward, while the opposite yaw
direction drives the forearm downward. The preparation stage has no dwell and
hands off at 90% of its calculated transition time to avoid stop-start motion.
The action is intentionally limited to one arm at a time to avoid interference
near the head. The preparation frame bends the elbow before completing the
shoulder rotation, avoiding a fully extended sweep near the head. `welcome`
raises the hand beside and above the torso rather than in front of the chest.
It keeps shoulder yaw and all wrist axes fixed at their selected values, with
all three wrist angles remaining neutral throughout. It sweeps elbow pitch
between -110 and -90 degrees, which the checked-in URDF places mainly along the
lateral axis; changing shoulder yaw here would instead move the hand mainly
forward and backward.
`raise` lifts the upper arm close to overhead while keeping only a moderate
elbow bend, making its silhouette distinct from `welcome`.
`shake_hands` extends one arm with a smaller shoulder-pitch angle than the old
forward-reach pose and uses a small elbow sweep for the handshake.
`high_five` places the wrist approximately 0.37 m in front of the torso plane
and at approximately the shoulder-joint height in the checked-in URDF.

Wrist pitch stays neutral in every semantic frame. `salute` moves wrist yaw
progressively from 25 degrees in the preparation frame to 50 degrees in the
final frame. `welcome` keeps all wrist joints at zero for the entire sequence.
For the left-arm `high_five`, wrist roll moves from 10 degrees in the
preparation frame to 50 degrees in the final frame; the right arm uses the
mirrored negative angles.

The URDF defines joint axes and limits but contains no hand palm frame and no
arm visual/collision geometry. Palm-facing wrist values are therefore
conservative starting points, not geometrically proven orientations. Calibrate
`welcome`, `salute`, and `high_five` one arm at a time at low speed
with a clear workspace. Bilateral `salute` remains blocked because the hands
move near the head. Bilateral `high_five` is allowed because its mirrored paths
remain separated in front of the shoulders. Actions such as clapping, hugging,
and crossing arms are intentionally not provided until collision geometry or a
separate collision checker is available.

Before publishing an action, the card checks fresh `/arm/status` data, all
selected motor IDs, motor error codes, the physical/remote emergency stop and
power state. After publishing, it waits up to two seconds for a newer
`/arm/status` sample and verifies that a selected joint moved (or was already
at the requested target). The MCP result therefore distinguishes a scheduled
action from controller feedback with `feedback_verified: true`. Failures return
`state: error`, a stable `code`, the human-readable `error`, and relevant
diagnostic details.
MCP tool failures are also marked with the protocol-level `isError: true`, so
the dashboard/agent can distinguish them from successful results without
parsing the text payload first.

The SDK does not expose a dedicated "self-check completed" field in the message
types used here. A no-motion result can therefore identify an incomplete
self-check/Not Ready state as a likely cause, but cannot claim it conclusively.
Complete the documented startup self-check and confirm the whole-body status
light indicates Ready before running arm actions.

| Action | Dashboard defaults | Allowed range |
|---|---|---|
| `salute` | right arm, speed 0.5rad/s | side left/right, speed 0.2–1.5rad/s |
| `welcome` | right arm, cycles 2, speed 0.5rad/s | side left/right/both, cycles 1–5, speed 0.2–1.5rad/s |
| `raise` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `shake_hands` | right arm, cycles 2, speed 0.5rad/s | side left/right/both, cycles 1–5, speed 0.2–1.5rad/s |
| `high_five` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `reset` | right arm, speed 0.5rad/s | side left/right/both, speed 0.2–1.5rad/s |
| `stop` | no parameters | no parameters |

## External mic on PC1 (`ext_mic` network source)

A USB mic (e.g. a DJI Wireless Mic receiver) plugged into **PC1** is captured
there by `audio_sender.py` and streamed to the driver on PC2 as raw PCM-16k
over TCP (`tcp://192.168.41.1:9800/pcm16k`). PC1 is configured under
`plugins.ext_mic.network_sources` in `config.yaml`.

At startup, and again whenever the card's SSH settings are saved, the driver
prepares PC1 over SSH (`ext_devices.probe_remote_host`):

| Step | What it does | When it cannot |
|---|---|---|
| Devices | `arecord -l` as the SSH user; prefers a USB card | logs which of: SSH failed / no card in the kernel / no capture device |
| Permission | if the kernel has cards but the user is not in `audio`, runs `sudo usermod -aG audio <user>` with the SSH password | logs the exact command to run by hand |
| Sender | copies `audio_sender.py` when **its content** differs (md5), and restarts it | logs `deploy failed` |
| Streaming | ready only once bytes arrive on the port — silence counts, an open port does not | names the option `[not streaming]` and logs the tail of `/tmp/audio_sender.log` |

`audio_sender.py` needs `python3` and `numpy` on PC1 (Ubuntu 22.04 ships both).
It uses pyalsaaudio when a working one is installed and otherwise captures
through `arecord`, so PC1 needs no pip install and no internet. If its capture
fails, the process exits rather than keep an empty port open, and the next
health check restarts it.
