# Policy-authorized automatic preparation

Automatic health observation is separate from permission to activate a motion
session. Merely installing a policy, setting `enabled` on a card, or setting
`S10_AUTO_PREPARE=1` does not grant automatic activation.

| Protected policy | Startup activation | Session renewal after expiry |
| --- | --- | --- |
| prepare_authorization_policy/1, explicit_prepare | Never automatic | Never automatic |
| prepare_authorization_policy/2, automatic_prepare | Only when allow_automatic_startup is true | Only when allow_automatic_renewal is true |

Version 2 requires both permission fields as literal booleans and provenance
`owner_approved_automatic_prepare`. Each permission is checked again against the
unchanged root-owned, machine-bound policy before creating a session. Permissions
are not canvas settings. `S10_AUTO_PREPARE=0` disables the supervisor;
`S10_AUTO_PREPARE=1` only lets the supervisor use already-approved permissions.

Loading either policy and reading status never activate it. The supervisor uses
a private `prepare_automatic` path, and the authorization object checks the
appropriate startup/renewal permission again at session creation. Explicit
trusted `prepare` remains distinct; neither operation is a public loco_move
motion action. Old version-1 policy files are never silently upgraded. With an
old policy, the merged canvas remains locked until deliberate trusted explicit
preparation or owner-approved migration to version 2.

When a version-2 policy is valid, the supervisor may start read-only observation
and check readiness. It creates a session only while idle/enabled and permitted.
Every activation still checks stationary telemetry, advancing enabled joints,
fresh clear HES and fault feedback. Active tasks are never interrupted for
renewal. An explicit card stop is respected. Command failures close the old
worker; current readiness is checked again on the next bounded supervisor tick.
Healthy authorized conditions allow channel reconstruction without a manual
unlock or session renewal. Preparation exceptions are retried on later ticks,
not permanently latched. This never replays a task, selects a posture, or
publishes velocity. Policy changes,
revocation or machine/boot mismatch remain fail-closed. A process restart loses
its in-memory session and must pass the separately approved startup permission.

## Provisioning and migration

`deploy/init_prepare_policy.py` defaults to a dry run. Without automatic approval
flags it produces only a legacy explicit policy. Preview the requested mode:

```bash
python3 /deploy/init_prepare_policy.py \
  --approve-automatic-startup --approve-automatic-renewal
```

After separate owner review of automatic startup and renewal, provision on the
NOS host into a new dedicated directory, with the existing three site checks:

```bash
sudo python3 /deploy/init_prepare_policy.py \
  --output-dir /opt/phanthy-motus/s10-pro-auto-v2 \
  --approve-automatic-startup --approve-automatic-renewal \
  --operator-present --handheld-estop-verified --no-competing-commands --apply
```

Omitting the renewal approval gives a startup-only policy; expiry then requires
explicit trusted preparation. A renewal-only policy cannot activate the first
session automatically. Existing policy/identity files are never overwritten.
Preserve the old files and update only the Driver's read-only identity/policy
mounts after approval; adjust the supervised overlay's host directory to match.
Do not copy active site policy files into Git or an image, change Core, or erase
task/outbox state. Merely deploying this source does not migrate an existing
robot policy or authorize automatic activation.

## Deployment switch and readiness checks

The service fragment explicitly sets `S10_AUTO_PREPARE=1`. Keep this setting in
the rendered site Compose service when using an owner-approved automatic policy.
It enables the supervisor, not the policy permissions; legacy explicit policies,
revocation, missing grants and feedback blockers still prevent activation.
Set `0` only as a deliberate site opt-out, with a separate trusted explicit
preparation path. The public locomotion card does not expose Prepare.

When replacing a container, apply the intended Compose environment rather than
copying the old container's effective environment unchanged. An inherited
`S10_AUTO_PREPARE=0` overrides the application's default and survives an ordinary
restart. Changing this environment variable requires recreating the Driver.
Preserve the old container/image, site files, policy scopes and task/outbox state.

Check both configuration and readiness before calling deployment successful:

1. Confirm the running Driver's `S10_AUTO_PREPARE` is `1` and the persisted Compose
   service has the same setting. Inspect only this value, never dump credentials.
2. Query `loco_move` with `action: info`. The detailed readiness must contain
   `automatic_preparation`; with healthy feedback and an approved automatic
   policy it reaches `session_active` and `operator_session.authorized: true`.
3. Check command-channel readiness and the actual `blocked_by` list. Missing
   telemetry, faults or denied policy permissions remain deployment blockers;
   do not clear them by relaxing thresholds or expanding permissions.
4. Verify Core was not changed/restarted and no motion task was created. Do not
   issue stand-up, sit-down or velocity commands as a deployment health check.

If the card is running with fresh feedback but reports `prepare_required`, no
command channel and `automatic_preparation: null`, check this switch first.
Clicking Start enables observation; it does not replace the disabled supervisor.
