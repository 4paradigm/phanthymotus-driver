# Running Docker source synchronization — 2026-10-30

Source: the running `embodied-deep-robotics-s10-pro` container, image
`local/phanthy-motus/drivers/deep-robotics/s10-pro:auto-prepare-20261030-105214`.
The product-source stream SHA-256 was
`c216fe222a8737ff36b881d2836f1a4ca293382fa4731a4d600c0d1537b3ae99`.

The Python implementation and skeleton model were copied from that running
container, including automatic preparation, hidden manual Prepare entry,
bounded and relative movement, and the current G1-style monitor catalogue.
Runtime configuration, machine identity, authorization JSON, credentials,
state databases, logs and Core files were not imported.

At the owner's request, retain PR #451 deployment hardening: bridge/reader
logging and vendor-profile validation, default configuration, text vendor
sources, runtime requirements and multi-stage Docker build. The Docker COPY
list additionally includes automatic_prepare.py, relative_motion.py and both
skeleton files, plus /deploy. Service logging/restart formatting follows the
current container's install fragment while preserving required-profile checks.
README and motion-card notes identify current automatic preparation behavior.

Validation: 120 passed, 1 skipped across runtime automatic-preparation,
deployment-review, relative-controller, timed-motion and DDS tests. The service
configuration validator passes. A wider legacy suite was also run and was not
fully green: it still assumes the removed monitor catalogue/titles and manual
Prepare UI. Those failures are not claimed as passing; broader test migration
and an ARM64 rebuild of this combined revision remain outstanding. No physical
motion was sent and no running Core or Driver was changed during this sync.
