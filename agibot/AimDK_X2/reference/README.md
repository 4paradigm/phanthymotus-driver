# AimDK X2 driver — reference index

## Source material

- **SDK**: `https://x2-aimdk.agibot.com/downloads/aimdk-aarch64-a424add7-artifacts.zip`
  (revision `a424add7`). Only the `aimdk_msgs` ROS2 interface source
  (`.msg`/`.srv`/`CMakeLists.txt`/`package.xml`/`cmake/`, ~365KB) is vendored into this
  driver as `../aimdk_msgs-a424add7.zip`, SHA256-pinned in `../Dockerfile`. The zip's
  `prebuilt_aarch64/` binaries (~17MB) and the AimRT-based C++ runtime are **not** vendored —
  this driver talks plain `rclpy` directly, no proprietary middleware needed on our side.
  `py_examples/*.py` inside the SDK zip were read for wire idioms (request/response field
  layout not otherwise obvious from the `.srv` source, e.g. `set_mic_source.py`'s retry
  behavior, `get_map.py`'s `std_msgs/Header` usage) but were not vendored themselves.
- **URDF**: `https://x2-aimdk.agibot.com/zh-cn/latest/_downloads/2ffc9785259556f409e385974a7a0461/X2_URDF-v1.3.0.zip`
  (`X2_URDF-v1.3.0`). Only the 3 small `.urdf` text files for the interchangeable end
  effectors (`x2_fist.urdf`, `x2_hand.urdf`, `x2_ultra.urdf`, ~34-36KB each) are vendored into
  `../resource/`. The zip's mesh STLs and MuJoCo `.xml` variants (~48MB total) are excluded —
  see `../resource/SOURCE.md` for exclusion details and the original download URL/version.
- **BrainCo Revo2 doc**: `https://www.brainco-hz.com/docs/revolimb-hand/revo2/parameters.html`.
  This is used **only** as a generic dexterous-hand modeling reference (touch-sensor layout,
  5-finger joint naming convention) for shaping the `hand_command`/`hand_state` tool schemas.
  X2's own `HandType.msg` enum (`NONE / NIMBLE_HANDS / CLAW / LEISAI_NIMBLE_HANDS / ERROR`) has
  no BrainCo entry, so this doc is **not** evidence that X2 ships a BrainCo part — it's a
  naming-convention analogy only, not a vendor binding. See `message_schemas.md` for the
  actual native `HandCommand`/`HandState`/`HandType` schema this driver is wired to.

## Firmware caveat

The vendor SDK's own examples run under `$HOME=/agibot/data/home/agi` on-device — i.e. any
state the AimDK stack itself persists (logs, cached config) lives under that path on the
robot's filesystem, not under a container-relative path. Not directly relevant to this
driver's own state (which is in-memory only, per `device.py`'s `AimdkNodes`), but worth
knowing when debugging on real hardware alongside vendor-side logs.

## Other reference docs in this directory

- `topics_and_services.md` — full topic/service catalog transcribed from the vendor SDK,
  with a table mapping every entry to the corresponding tool in `device.py` (or to "not yet
  wired" / "explicitly unreleased").
- `message_schemas.md` — field-level `.msg`/`.srv` schemas for every message type this driver
  actually constructs or parses.
