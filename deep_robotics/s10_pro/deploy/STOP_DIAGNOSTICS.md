# Stationary confirmation

Normal motion feedback must report planar speed below 0.05 m/s and absolute yaw
rate below 0.06 rad/s for at least three distinct samples over 0.3 seconds.

A small raw velocity bias below 0.20 m/s may instead be corroborated by fresh,
advancing odometry: planar speed below 0.02 m/s, yaw rate below 0.03 rad/s,
less than 1 cm position span and 1 degree heading span over at least 0.5 seconds
and four paired samples. Frozen/rewound timestamps, jumps, missing samples and
motion anywhere in the observation window invalidate that exception. Raw
telemetry is never rewritten to zero.

Finite navigation reserves 2.05 seconds inside its total action deadline, sends
zero at 10 Hz and grants up to two seconds for stop confirmation. At least two
zero publications and two consecutive ready observations are required. Failing
to confirm stop yields `reported_stop_not_confirmed`, never `completed`.

Current emergency-stop/fault checks remain independent from stationary feedback.
Application stop is not a hardware emergency stop or a manufacturer watchdog.
No failure installs a manual software latch or replays a task. Inspect current
telemetry before starting a new request.
