# Q1 native motion-state message definitions

These 14 message files were copied without field changes from the Q1 firmware's
`/robot/software/mc/share/aimdk_msgs/msg/` on 2026-11-27. They supplement the
pinned public v1.1.8.0 SDK, which contains the motion services but omits
`McCommonState` and these dependencies. The Docker build generates them within
the existing `aimdk_msgs` package; it does not replace the public service types.

The driver subscribes read-only to `/aima/mc/common/state`. On the inspected
firmware, `GetMcAction` returns the correct mode and RUNNING status but leaves
`source` empty and `switch_ready=false`, while this topic supplies the live
switch flag and fall/lift/movement status. The flag is used only for a sample
no older than 0.5 seconds whose mode and status agree with the RPC. A mismatch
blocks a command. No data is treated as ready by default.

This is a firmware interface, not an additional public SDK promise. A firmware
with an incompatible layout must be checked against its installed definitions.
If the topic cannot be read, the driver retains the original RPC flag and does
not turn false into true. Do not substitute the older definitions bundled with
`aimrt_pulsecore`: its `McActionInfo` omits `source` and decodes the current
RPC incorrectly.
