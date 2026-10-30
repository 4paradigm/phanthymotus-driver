# S10 Pro Container and Acceptance Evidence

This candidate is local development only. Do not deploy, upload, access the robot
or issue physical commands as part of these checks. Normalize Core to the
approved shared-platform baseline; keep S10-specific behavior in the driver.
Historical checkpoints remain in Git and the local pre-edit backup; their image
IDs and test totals are not evidence for this candidate.

## Local acceptance

1. Verify `G1_REFERENCE.json` against the exact local Git commit and confirm the
   reference G1 directory is unchanged. Record the S10 checkout HEAD and dirty
   files; HEAD alone does not identify a modified candidate.
2. Verify all Docker COPY inputs in the staged driver-plus-common context.
   Confirm `/deploy/service.yml` and its supervised overlay will be available
   inside the image. Check the one-service fragment and platform DDS isolation.
3. Run the S10 offline regression suite and the container-contract tests. Record
   exact pass, skip and failure counts, interpreter and any environment limits.
4. Check English source, comments, tool definitions and driver documentation.
   Upstream vendor sources retain their original attribution and licenses.
5. When baseline and candidate Compose documents exist locally, validate exactly
   one approved standard Core and one driver. Preserve the approved Core service
   and shared project resources. Do not invent an unknown Core configuration.

An unavailable Docker engine means an actual ARM64 image build and image-level
extraction have not passed. Parsing COPY instructions and staging files alone
cannot prove that an image builds or starts. An absent local Compose baseline
means deployed topology and Core equivalence have not been verified.

## Unresolved aggregate monitors

The old third monitor container used overview.py and groups.py. Those files are
not present locally and the user currently has no copy. The four existing
aggregate cards therefore cannot be claimed to have migrated to the single
driver. Obtain source, preserve their contracts and integrate them into the
existing bundle before changing the real canvas or removing the historical
container. Do not recreate their behavior from names alone.

## Future image and site acceptance

Future authorized work must build the exact candidate ARM64 image, inspect its
manifest and /deploy contents, and test in a network-isolated container before
site deployment. Compare vendor message fields, preserve task/outbox state,
and check actual mounted files and protected ACP trust configuration.

A separate site authorization is needed to change the current robot. Read-only
acceptance must establish the exact image/source hashes, two-container topology,
standard Core source and configuration, tools/list, independent card start/stop and fresh source data.
Preserve existing cards, LiDAR, positions and connections. The current local run
does not certify these properties on the robot.

Physical acceptance requires current on-site confirmation and takeover capability.
First verify real odometry and its consistency with motion feedback; never relax
jump limits or substitute speed integration to hide unreliable position data.
Treat relative movement, lateral movement, UI conditional rendering and reliable
completion/stop behavior as unaccepted until separately verified. Prior timed
movement observations do not establish relative-motion success.

Rollback preserves driver request identities and ACP outbox, restores only the
authorized driver artifacts, and retains the approved shared Core baseline. Restarted tasks stay
failed or interrupted and are never physically replayed.
