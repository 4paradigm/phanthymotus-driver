# S10 locomotion interface

The current control, configuration, lifecycle and ACP contract is documented in
[the Driver README](../README.md). Keep that document as the single operational
reference rather than copying earlier posture/Prepare interfaces here.

Both `loco_move` and `relative_move` use one controller and physical resource
`base`. Pose transitions and movement have separate readiness checks. A canvas
`start` result of `ready` is a lifecycle marker; inspect the requested action's
current blockers before interpreting hardware readiness.

A successful DDS publication means local send only. Completion requires the
reported posture/velocity/odometry evidence and, for navigation, confirmed
stopping. Feedback errors cannot be replaced by a successful publish receipt.
