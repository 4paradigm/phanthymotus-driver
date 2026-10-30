# Stop confirmation diagnostics

A person observing a stationary robot and an application failing to confirm a
stop are different observations. `reported_stop_not_confirmed` records missing
application evidence; it does not establish that the robot is still moving.

Finite and relative movement terminal records include `stop_diagnostics`:

- `zero_commands_sent`, `elapsed_s`, and `available_budget_s`;
- `last_stationary_check`: reported state, velocity, freshness, sample count,
  stable duration and the unchanged tolerances;
- a bounded trace of the last 16 distinct feedback observations;
- `failure_kind` for stale feedback, state mismatch, velocity above tolerance,
  an insufficient stable window, an exhausted budget, or a command/read error.

The 1.2-second stop budget, 0.05 m/s planar and 0.06 rad/s yaw thresholds,
0.3-second stable window and original authorization checks are unchanged.
Diagnostics do not clear a failed worker or convert a failed task to success.
The trace is finite and contains no authorization permits or credentials.

After an unconfirmed stop, inspect the recorded evidence before deciding whether
feedback delivery or the bounded confirmation budget needs adjustment. Any
subsequent explicit posture recovery must still pass stationary, fault,
authorization and fresh-feedback checks. Do not replay the old request or erase
its terminal result. The existing `stand_down` path can recover a failed channel
when these conditions pass; faults or stale data still block publication.

`loco_move` supports forward/backward and in-place turns, not lateral movement.
`max_vy` is a shared/internal lateral cap (default 0.1 m/s, schema maximum
0.15 m/s); it is not the forward-speed or yaw-rate setting. Relative lateral
movement remains a separate interface with its existing limits. Changing the
finite forward/yaw maximum to 1 does not change this lateral contract.
