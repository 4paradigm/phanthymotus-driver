# S10 shared calibration and configurable loco_servo

Scope: driver code/tests only; no robot deployment, motion, mode changes or
configuration writes. User confirmed all intended S10Pro units share camera,
lens, front-facing upright mounting and coincident body/lidar_link axes.

1. Put actively used intrinsics and standard mounting extrinsics into packaged
   JSON. Default same-model loading must not require one robot's machine-id;
   explicit per-unit deployment overrides still win, invalid files never guess.
   Preserve existing numeric calibration and its measurement/operator provenance.
2. Match upstreamTianyi loco_servo (main b3e8e788): slow/normal/fast0.15/0.30/0.50m/s,
   dry_run defaulttrue and respected by start, rotate_only, min_vx planner-only,
   linear/angular acceleration and yaw cap. Retain S10 hardware ceilings and
   mandatory native standing gate. No copied wheel SDK/gait assumptions.
3. Implement zero-immediate/rebase semantics in an S10-local velocity sink;
   shared ControlSink and Tianyi remain unchanged: acceleration begins at confirmedzero, zero axes stop
   immediately, physical rebase never renews watchdog or clears sequences.
4. Validate configs atomically. Stop old real motion before settings changes,
   require fresh post-transition commands, preserve authorization/mutual exclusion,
   and never emit native commands in dry_run (including lifecycle paths).
5. RED/GREEN tests for packaging, default selection and override errors; servo
   defaults, limits, dry-run preservation, malformed configs, real->dry stop,
   replay barriers, watchdog/zero behavior and permission revocation. Then S10
   and shared-control regressions plus the repository test command. No deployment.

Calibration is implemented independently with file ownership; parent handles
S10 servo and combined documentation. Final integration reviews both.

Final ledger: user reinforced S10-only scope. All common/Tianyi edits are absent;
zero/rebase behavior lives in S10-local _VelocitySink. Standard profile reuse
is explicitly authorized for matching hardware/mounting. Removed obsolete
identityquery and its now-unused tests. Review P1 failed-stop/dryrun bypass was
reproduced and fixed; pending stop now survives disarm with actual reader retry.
Calibration integer overflow and explicit intrinsics error reporting have
RED/GREEN coverage. Final related421passed8skipped; repository80collection
errors match prior80line-for-line. No robotconnection/deployment/motion.
