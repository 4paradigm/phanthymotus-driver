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

The normal path retains the 0.05 m/s planar and 0.06 rad/s yaw thresholds and
0.3-second stable window. The bounded stop-confirmation budget is now 2 seconds,
always capped by the request deadline; movement reserves 2.05 seconds for it.
Repeated zero commands and two distinct ready confirmations are still required.
Diagnostics do not clear a failed worker or convert a failed task to success.
The trace is finite and contains no authorization permits or credentials.

## Corroborating a bounded motion-feedback bias

An additional path accepts a stable raw planar reading below 0.20 m/s only when
independent odometry corroborates stopping. This is not a general 0.20 m/s stop
tolerance, and no offset is subtracted from recorded velocities. The exception
requires all of the following:

- Both streams are fresh (at most 0.35 seconds old), finite and in an allowed
  motion state. Each credited pair advances both source sequences and receive
  times; odometry's original header timestamp must also advance.
- At least four paired frames span 0.5 seconds in both streams.
- Raw motion yaw stays below 0.06 rad/s and the planar reading varies by at most
  0.02 m/s over the window.
- Odometry planar speed stays below 0.02 m/s and yaw rate below 0.03 rad/s.
- All positions in the window stay within 1 cm of each other and headings
  within 1 degree. An out-and-back excursion also invalidates the window.

Missing/stale/frozen feedback, changed posture, clock/sequence rewinds, excessive
raw speed, odometry movement or a pose jump cannot grant this exception. The
normal motion-only path remains available for small readings without odometry;
only the extra bias tolerance requires the second stream. Evidence is reported
as `confirmation_basis: motion_and_odometry` with `odometry_corroboration`.

The site observation motivating this path had approximately -0.157 m/s raw X
velocity while four seconds of odometry showed sub-millimetre net displacement
and near-zero speed; the operator also confirmed physical rest. This comparison
supports a bounded cross-check, not a claim that odometry is always correct.

## Reference implementations

The comparison uses upstream main commit `07f0e55c` in this repository:

- G1 relative locomotion (`unitree/g1/loco_relative_move.py`, `_confirm_stop`)
  repeatedly sends zero, uses odometry speed, requires 0.5 seconds of stability,
  and allows up to 3 seconds for confirmation. Its safety persistence uses new
  sensor frames, not repeated control-loop reads. A separate stop-and-confirm
  operation can clear its stop latch after fresh safety checks.
- G1 timed motion (`unitree/g1/timed_motion.py`) uses SDK stop acknowledgements;
  acknowledgement is kept distinct from physical feedback confirmation.
- M20 (`deep_robotics/lynx_m20/device.py`, `_stop_stream_locked` and
  `_motion_stream_loop`) serializes velocity writes, repeats zero three times
  and stops an expired/watchdog stream. Its returned stopped state is not the
  same feedback-based physical-stop proof required by this S10 path. Its README
  also records outstanding hardware validation.

S10 retains its own authorization, total deadline, state and feedback checks;
these robots' SDK acknowledgements, speed limits and recovery APIs are not
interchangeable. Core and the single-Driver architecture are unchanged.

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
