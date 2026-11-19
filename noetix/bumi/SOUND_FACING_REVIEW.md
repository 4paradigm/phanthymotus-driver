# Sound Facing review — 2026-11-13

Code Ready: YES for the reviewed offline software scope. Draft PR Ready: YES with the limitations below. Regular PR Ready: NO in this delivery: full current ARM64 build, target startup, production privilege isolation and physical acceptance remain unverified. This status supersedes the older readiness text; it does not certify a deployment.

## Findings and fixes

1. The configurable session lease could exceed the requested 30-second upper bound. Both parsing and advertised schemas now enforce 1–30 seconds. Explicit renewal still cannot extend the independent operator grant (maximum 300 seconds from challenge creation).
2. Source-probe invalidation discarded late results but did not wake waiting callers. Invalidation now signals that query's completion event immediately. Version/freshness checks still deny its late result, and a blocked external callback remains limited to one daemon. There is no permissive fallback.
3. Current production authorization already has a registered, initialized signature path. `operator_authorization_unavailable` is the intended denial for an unconfigured or unusable pinned key, not evidence that the API is absent. The real Core→MCP→Driver signed path and revoke/publish boundary passed current tests. Operational diagnosis and the compatibility contract are documented in `SOUND_FACING.md`.
4. The new authorization dependency group is pinned and wired into both pip requirements and Docker installation. An isolated build/context/smoke helper and a real-Core integration fallback are now reviewable and executable. The current Docker environment blocks a full build; no image success is inferred from old reports.

Each runtime fix has a regression that failed before the change. Existing authorization, scope/intent/replay, delayed preflight, publish-after-rate-limit, stop, ownership, feedback loss, settle/reobserve and clock tests remain in place. See `SOUND_FACING_TESTING.md` for actual results and retained failed runs.

## Authority and administrative boundary

The operator private key stays on an independent trusted workstation. Driver startup pins only a regular, trusted-owner, non-group/world-writable Ed25519 public-key file. MCP/config cannot replace it. A challenge binds device, process nonce, intent, session, chassis and deadline. The signature itself neither enables tracking nor sends motion. AI/Canvas may enable/renew only inside that independently approved interval. Public IDs, normal Core tokens, Canvas labels and dangerous flags grant no motion authority. Missing authority fails closed; stopping needs no authorization.

An administrator able to replace Driver code/key/container or inspect the signer remains outside this in-process verifier's threat boundary. This task did not inspect or alter the shared production environment. Before physical enable, independently verify Agent cannot administer those resources or read/call the signer. The prior document's statement about an operator-confirmed unrestricted deployment is historical context, not a newly verified production fact. No production policy was weakened, and no shared isolation change was performed.

Core's ACP is completion bookkeeping, not an independent operator approval service. The Driver's signed challenge is the compatible authorization mechanism; the existing Core simply transports its ordinary tool calls. No hidden Core patch or fake approval endpoint is required.

## Concurrency, motion and failure handling

The Card reuses one existing Loco instance, owner capability, rotation worker, publish limiter, IMU feedback and body arbitration. Stable independent acoustic windows are required before each turn. A matching terminal receipt, braking command and IMU settle precede post-turn acquisition; queued pre-turn windows cannot trigger another turn. Duplicate/early network ACP cannot complete internal owned turns.

Lease and operator deadlines are monotonic. The memory-only authorization predicate is checked after delayed preflight and immediately before nonzero publication, after the rate-limit wait. Stop/revoke/expiry invalidate the session and signal owner-specific cancellation independently of source health. Missing feedback/terminal/quiet confirmation retains the reservation and faults; it does not report a safe physical release. Another owner's actions are not stopped by the Card. Restart/config never implicitly enables follow.

This is not a hard-real-time controller. OS scheduling, permanently blocked vendor SDK calls, physical braking, slip and acoustic calibration require target validation and an independent physical emergency stop. Those limits do not authorize retrying retired sessions or relaxing a refusal.

## Scope and compatibility

Changes stay in the Bumi Card, its bundle registration/manifest, existing owned-motion integration, tests and documentation. `common` and upstream Core were read/tested without edits. Latest Driver main was trial-merged successfully in a disposable clone. Default registration remains off. Other developers' device upload/COPY failures were reproduced on untouched main and left unchanged.

## Remaining external gates

- Working ARM64-capable Docker/Buildx engine and a fresh full-image build/smoke for this code.
- Real ROS/DDS and target Python/SDK startup; no real SDK was imported here.
- Installation-specific microphone front/right calibration, time semantics, multiple-source/self-noise behavior and actual yaw sign.
- Permission isolation of the operator signer and Driver/admin plane.
- Real stop/revoke/timeout/feedback-loss behavior, physical stop distance and continuous tracking under laboratory supervision.

Use `SOUND_FACING_HARDWARE_ACCEPTANCE.md`; no robot operation is authorized by this review. No push, PR creation, image publication, shared-service changes or real motion occurred.
