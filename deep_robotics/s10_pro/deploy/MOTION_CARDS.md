# S10 motion cards

The current interface is `loco_move` plus the independent `relative_move` tool.
See [the current contract and migration](LOCO_MOVE.md) for actions, speed ranges,
completion and saved-canvas migration. Former `motion` and `timed_move` tools
are retired from the public catalogue. `loco_move` has no output ports; relative
movement retains its own ports. Read status through info/status and await ACP.
Automatic Prepare is unchanged and never selects a posture or sends velocity.

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
