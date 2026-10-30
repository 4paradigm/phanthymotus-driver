# Current runtime update

Automatic preparation runs in the driver when enabled and the protected policy
is valid. The canvas no longer exposes a manual Prepare action. See
[automatic preparation](../README_AUTOMATIC_PREPARE.md). The older manual-start
notes below are historical; all motion requests still require healthy feedback
and a valid session.

# S10 Posture, Timed and Relative Movement Cards

The three tools belong to one S10 driver and share its controller, authorization,
task history and chassis lock. Starting a card observes state; it does not
open a motion session or switch the robot mode. A protected prepare policy
allows an explicit session of up to 60 minutes after readiness checks.

| Tool | Operations | Completion evidence |
| --- | --- | --- |
| motion | prepare, stand, lie, move_mode | Preparation sends no motion. Posture requests require fresh reported state; lie waits for final state 0. |
| timed_move | forward, backward, turn_left, turn_right | Direction-consistent velocity response and fresh stationary feedback after repeated zero commands. |
| relative_move | forward, backward, left, right, turn_left, turn_right | Valid /ODOM reaches the target, settles and confirms stopping. |

Use state 1 before requesting move_mode (17). The vendor state machine ignores
stand directly from 17: lie to final state 0 before stand. Navigation mode on
the handheld remains a site prerequisite. Do not infer vendor control ownership
from a DDS match, card state or reported motion state.

Timed movement uses speed and duration, never odometry distance. Relative
movement accepts distance for translation or angle for turns. Its speed default
is valid for both direction classes; backend limits remain authoritative.
Relative movement decelerates and stops on drift; full trajectory correction
has not been implemented. Physical relative movement is not accepted yet.

Each card owns separate status and event topics under /{namespace}/s10_pro:
posture_status/posture_events, timed_move_status/timed_move_events and
relative_move_status/relative_move_events. Do not connect them to a shared
legacy motion output. Lifecycle stop requests cancellation through the existing
shared controller. It is not a hardware e-stop or a guarantee that the robot
has stopped; completion must retain actual stop-confirmation evidence.

## Packaging and future provisioning

The install service and supervised overlay are bare mappings for the same
`deep-robotics-s10-pro` service. Merge their mounts and environment into the
existing full Compose services mapping. Do not create a second driver, run an
extra monitor container or add S10-specific code to the shared Core. Mount identity, policy, DDS profiles
and config read-only; keep driver task/outbox storage persistent and separate
from Core. Keep ACP HTTPS verification and protected credentials.

`init_prepare_policy.py` defaults to a dry run and rejects overwriting existing
policies. Its apply operation is a future site-provisioning step, outside this
local development run. No policy is created or activated by packaging the image.
The legacy forward_30cm/backward_30cm authorization names remain for compatibility;
they do not assert 30 cm of measured timed displacement.

## Acceptance and rollback

Use the exact built image and source hashes for any future acceptance. Preserve
request IDs and state databases on rollback. Do not replay failed or interrupted
actions, delete task history to bypass deduplication, or reuse an ID for a changed
target. A process restart invalidates live preparation and does not resume motion.

Existing physical observations from earlier versions are historical. Current
local tests cannot certify remote behavior, trustworthy robot odometry, UI
conditional rendering or the four missing aggregate-monitor implementations.
