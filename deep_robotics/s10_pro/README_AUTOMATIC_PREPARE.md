# Driver Automatic Preparation

This additive driver-only update preserves the current entrypoint, bundle,
transport, protected policy, action scopes, Core and canvas contracts.
This requested driver build enables automatic preparation by default.
Set S10_AUTO_PREPARE=0 to opt out; the protected policy must still be valid.
The supervisor starts posture observation at process boot and calls the existing
prepare path only while idle and enabled. Faults, low battery, HES, advancing
joints and stationary feedback checks remain authoritative. Failed conditions
are rechecked every five seconds; an active task is never interrupted. A valid
session is retained until expiry, then preparation repeats only while idle.
Explicit card stop is respected; it is never automatically restarted. Command
failures latch for review. Policy edits/revocation cannot be bypassed. No posture,
velocity, prepare_test bypass, synthetic authority or task replay is introduced.
The canvas does not expose a manual prepare action. The existing internal
prepare path remains private to the driver supervisor for startup and session
renewal. Use driver logs for automatic-prepare waiting reasons. This is
application authorization, not proof of vendor ownership or physical stopping
guarantees.
