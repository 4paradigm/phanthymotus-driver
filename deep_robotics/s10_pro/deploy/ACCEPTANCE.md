# Deployment acceptance

1. Build the reviewed commit for `linux/arm64`; record image ID and same-method
   size comparison with the deployed baseline.
2. Preserve old Driver containers/images, site configuration and task history.
   Install the shipped vendor UDP profile as a host file before container creation.
3. Preserve the standard Core, its configuration and shared Compose settings.
   Replace only the Driver image. Verify original mounts and restart/log limits.
4. Verify domain 42 is loopback-only while separate domain-0 reader/worker
   processes discover fresh telemetry and correct command receivers.
5. Confirm cards begin idle and lifecycle start/stop returns ready/idle without
   emitting posture or velocity. Verify host motion opt-out cannot be overridden.
6. Under separate on-site motion authorization, verify transitions, finite and
   relative movement, cancellation, hardware e-stop/fault handling and stopping.
7. Verify ACP completion reaches the local self-signed HTTPS Core once per action
   with the same unique ID; no extra pending action is created by terminal replay.

Local fake-telemetry tests are required but do not replace these hardware checks.
