# PR #451 deployment review follow-up

This change preserves the original posture and fixed two-second motion scope.
The separate variable-speed/duration/turning work is not included.

- Reader fd 1/2 remain unchanged. ROS console logging is forced to stderr with
  `RCUTILS_LOGGING_USE_STDOUT=0`; the parent rejects malformed NDJSON lines.
- The pinned public BSD-3-Clause message package is stored as reviewable text
  sources with its license. No S10 binary archive is committed or downloaded.
- The default config selects the read-only vendor DDS profile. The deployment
  requires that profile; an empty override or missing file prevents reader and
  command worker startup instead of falling back to builtin transports.
- Colcon and ROSIDL generators are confined to the interface build stage.
  Runtime dependencies are PyYAML (configuration), Fast DDS RMW (ROS transport)
  and ROSIDL runtime (generated messages). Pytest is no longer an image package,
  and PyYAML is installed only through apt.

## Deployment acceptance gate

Local Python 3.12 regression: **1268 passed, 1 skipped** across all
`tests/test_s10*.py`, Docker COPY contracts and common lifecycle tests.
This includes the deployed service/config profile selection and missing-profile
rejection regressions. Git staged whitespace checks passed.

The earlier `release.261027.7385f17` ARM64 image built successfully and was used
for supervised stand, move mode, forward/backward two-second, stop and lie
checks on 2026-10-28. Those observations apply to that image only.

This review-fix revision requires its own ARM64 image build, image-size comparison
with `release.261027.7385f17`, vendor profile verification and supervised physical
acceptance before enabling its motion authorization overlay. Local tests do not
satisfy this gate. Preserve the current deployment and task state for rollback.
