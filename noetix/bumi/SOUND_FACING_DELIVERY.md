# Bumi Sound Facing PR delivery — 2026-11-13

- Workspace: `/home/utopus/phanthy-work/phanthymotus-driver-bumi-sound-facing`.
- Branch: `feat/bumi-sound-facing`; target: `4paradigm/phanthymotus-driver:main`.
- Initial HEAD: `820a415730594f7d8904904435961c45905939f6`.
- Reviewed fix/tooling commit: `50092e51d883f8ba4b7d3d6d84dc72711ccb2645`. The final documentation commit follows it; use `git rev-parse HEAD` for the whole candidate. The final immutable HEAD, source patch and changed-file manifest are recorded in the task artifact RELEASE/DELIVERY records.
- Latest reviewed main: `18211f9902549ce931867a3622cbb96d4c79183b`; a disposable real trial merge passed without conflicts. Original branch history was retained, without reset/rebase or cross-Card commits.

Code Ready: YES for the reviewed software. Draft PR Ready: YES with the explicit external conditions. Regular PR Ready: NO: current ARM64 build/target startup and production/hardware acceptance are outstanding. There is no known unresolved introduced P0/P1 software defect in the reviewed paths; unavailable production key configuration remains an intentional safe denial. Docker availability, signer/admin isolation and physical checks are external gates, not switches this task enabled.

Materials: SOUND_FACING_PR_DRAFT.md (English PR), README.md and SOUND_FACING.md (architecture/contracts), SOUND_FACING_TESTING.md (actual commands/results), SOUND_FACING_REVIEW.md (findings/safety), SOUND_FACING_HARDWARE_ACCEPTANCE.md (operator stages and stop conditions).

Current evidence: `/home/utopus/phanthy-work/artifacts/dual-card-pr-ready-20261113/bumi/`. Important records: 181 directed tests; 697 passed / 4 baseline failures / 1 real-ROS skip in regression; 21 Core/API/ACP/Canvas scenarios; 160 cancellation cases; untouched-main failure reproduction; prepared ARM64 context, wheel hashes and explicit unavailable-Docker build receipt. The old ARM64 success report is historical, not current-image evidence.

No further large offline implementation is identified for the reviewed feature. A working authorized ARM64-capable Docker engine must still execute the prepared build/smoke; no source-only check can replace it. Physical work additionally requires real microphone calibration/time, ROS/SDK startup, body-yaw sign, feedback/stop/revoke/timeout and continuous tracking validation, plus independent signer/admin isolation. Review findings or target-specific results can still require changes.

Only local commits were made. No push, PR creation/merge, image publication, deployment, shared-service restart, real SDK connection or physical robot motion occurred. The source package and build helper are for review; they do not authorize enabling a robot.
