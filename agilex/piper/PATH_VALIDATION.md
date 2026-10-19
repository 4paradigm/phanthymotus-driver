# PiPER path validation — 2026-10-15

## Verified source facts

Pinned SDK: 8d4dc29bd6e753066dedee235c008e90ecb2ab4a.
Official V2 example:
https://github.com/agilexrobotics/piper_sdk/blob/8d4dc29bd6e753066dedee235c008e90ecb2ab4a/piper_sdk/demo/V2/piper_ctrl_moveP.py
uses MotionCtrl_2(0x01,0x00,speed,0x00), followed by EndPoseCtrl.
MotionCtrl_2 documents MOVE P=0, MOVE J=1, MOVE L=2:
https://github.com/agilexrobotics/piper_sdk/blob/8d4dc29bd6e753066dedee235c008e90ecb2ab4a/piper_sdk/interface/piper_interface_v2.py
This establishes command encoding, not equivalence to RealMan joint planning.
No promise of a straight path, intermediate orientation or obstacle avoidance.

Official stop demo:
https://github.com/agilexrobotics/piper_sdk/blob/8d4dc29bd6e753066dedee235c008e90ecb2ab4a/piper_sdk/demo/V2/piper_ctrl_stop.py
uses MotionCtrl_1 emergency-stop and warns of subsequent reset/re-enable.
That sequence is incompatible with our torque-preserving Stop contract and is
not adopted. The existing measured linear hold must be observed during a
MOVE P trajectory before acceptance can be recorded.

## Current disposition

Candidate implemented and covered with a recording SDK; hardware path/Stop
acceptance pending. No real MOVE P command was sent during development or
software deployment. Installed TCP remains uncalibrated. Public Move stays
linear; public Pose and point-to-point remain hidden. Existing successful
linear trials do not validate MOVE P.

## Operator/engineer trial procedure

Schedule an attended arm-1-only trial with intelligent control stopped. Read
fresh feedback and confirm the adapter identity and one feedback source. Use
one SDK connection/owner, with the production task/cancellation/submission
checks, permission gate and event journal. A private engineering harness may
call user_motion.execute_target with an explicit supervised-trial permission
callback; it must not fabricate a production acceptance record. Public MCP has
no test action or bypass. Do not run the manufacturer's looping demo.

Start from a known reachable measured SDK pose. Compose one target exactly
1 mm along one measured SDK end axis, keep all three SDK orientation values,
use 1% speed. This validates the SDK reference only, not calibrated tip pose.
Record initial feedback, composed target, requested mode, final feedback,
controller completion and actual observation. Never recompute or retry the
same trial automatically. Use existing timeouts and frame freshness limits.

In separately arranged trials, request cancellation during motion and record
measured-hold command sequence, terminal feedback, stopping latency and observed
behavior. Controller rejection/fault invalidates the trial; do not clear or
reset. Only after arrival and Stop are physically verified may the candidate
be recorded as accepted. Calibrated absolute tip Pose still requires separate
TCP calibration and acceptance. A failed/unknown Stop keeps the feature closed.

## Acceptance format

Get status exposes current candidate binding values. Store a reviewed record in
intent_control.point_to_point_acceptance only after the above trials pass:

```yaml
point_to_point_acceptance:
  accepted: true
  device_serial: <observed USB-CAN serial>
  firmware: <exact firmware observed on this connection>
  driver_sha256: <current Python source digest>
  configuration_sha256: <current motion configuration digest>
  evidence: <reviewed trial record path and date>
```

Default is null. Unknown identity/firmware, empty evidence, source/config changes
or any mismatch keep point-to-point absent/rejected. Restart and refresh tool
metadata after configuration changes. The adapter binding cannot detect a
physical arm replacement on the same adapter: clear acceptance on any arm/tool
replacement and repeat verification. Never infer acceptance from SDK support.
