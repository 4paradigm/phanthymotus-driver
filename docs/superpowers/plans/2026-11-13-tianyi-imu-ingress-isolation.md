# Tianyi IMU ingress isolation

Goal: fix the deployed IMU startup failure without relaxing freshness or changing
public topics, camera geometry, capture time or lifecycle ownership. User allows
code fixes/offline tests/PR519 push and read-only robot diagnosis; no deployment,
business restart, container replacement or robot motion.

Evidence: f8e51ab2 producer is deployed. Forwarded IMU had 20 samples in a 20 s
window, age median 504.8 ms and max 526.2 ms; first packet rejected before any
gravity initialization. Raw stream and the same production pairing/TF method in
an isolated diagnostic process delivered approximately 100 Hz, age median2 ms,
max10.7 ms. Increasing raw subscription depth alone was insufficient.

Design: retain the existing capture source and transport. The dedicated camera
executor now owns a distinct small IMU node and its TF listener; image/info
subscriptions stay on the existing vendor executor. An IMU-specific lock keeps
pairing and publication independent of image work. Lifecycle mutations take
source lock then IMU lock, so stop/restart still serialize publication and reset
sampling state. Heavy image processing remains outside lifecycle changes. Validate
freshness again immediately before IMU publication. No guessed transforms and no
new dependencies. A separate always-running process is unnecessary unless the
bounded shadow probe demonstrates that thread isolation is insufficient.

Execute inline, with TDD; user waived incremental confirmations:
1. Add failing tests: image lock/slow image work cannot block IMU forwarding;
   raw IMU subscriptions/TF live on the dedicated node, not image executor;
   stop/restart cannot leak samples; stale TF work cannot publish an aged packet.
2. Implement only x-humanoid/tianyi2.0/rgbd_camera.py; adapt existing test fixtures
   for the separate node/lock. Run Tianyi/shared control/lifecycle regressions.
3. Run bounded shadow diagnostic process using private output namespace and
   existing source subscriptions. Preserve deployed files/services/config.
   Confirm actual IMU intervals, age, consumer admission and cleanup. Failure of
   shadow experiment requires more investigation rather than a success claim.
4. Independent review; commit/push PR519 explicit ref after checking remote head.
   Keep offline, diagnostic and deployed acceptance results separate.

Verification: 278 Tianyi/shared/control/lifecycle tests passed. Independent review
found no Important/Critical issues (64 targeted tests independently passed).
A 150 s private-namespace source run forwarded14887 IMU samples, maximum capture
gap10.133 ms, median receive age27.65 ms and max444.73 ms; consumer stayed active
with no recorded IMU fault. Final image/IMU/executor workers exited cleanly.
Attitude-only production consumer ran35.31 s,217 distinct tracking captures,
zero restarts and final tracking. The all-enabled fusion trial kept IMU active
but restarted6times for modeled position uncertainty; it is not accepted.

Limits: the old source also passed a20s private standalone run; these windows
do not prove the modified full driver will match private-process throughput.
The controlled lock-blocking regression proves the specific contention fix.
The first45s harness mutated telemetry during callbacks and had cleanup races;
it is excluded. A later30s attitude attempt outlived its source and is excluded.
Clean final150s source and paired35s consumers are the acceptance evidence above.
No business config/file/container or robot motion was changed. Full-driver
deployment and ordinary-topic retest remain separately authorized work.
