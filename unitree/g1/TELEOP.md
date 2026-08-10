# Unitree G1_23 teleoperation

The root G1 Driver can expose `teleop_session` and `teleop_state` in the same
MCP process and port as its other tools. V1 accepts Quest/OpenXR head and two
controller poses, controls the ten G1_23 arm joints, and deliberately disables
base and hand output. It never starts a second Driver or a second
`rt/arm_sdk` publisher.

The default `config.yaml` keeps teleoperation disabled and the legacy arm tool
enabled. Copy `config.teleop-shadow.example.yaml` to the configured
`CONFIG_PATH` to opt in. Shadow performs the real controller transform,
Pinocchio/CasADi IK, and `rt/lowstate` comparison, but constructs no publisher.
`plugins.arm.enabled` must remain false whenever teleoperation is enabled;
startup rejects conflicting arm authorities.

Live output is a separate explicit gate: set `teleop.mode: live` and
`teleop.live.enabled: true` only after `docker build unitree/g1` (or
`./build.sh unitree/g1`) passes on linux/amd64 or linux/arm64 and the robot reports
`mode_machine=4`. The image build cold-loads IPOPT and solves the bundled
G1_23 model; startup repeats a current-pose warm-up before constructing the
sole publisher. A failed warm-up leaves the Driver unavailable rather than
moving the solve into the first 150 ms control frame.

The image owns one numerical ABI under `/opt/g1-teleop`, created from
`environment.teleop.yml` with `conda-forge` plus `nodefaults`: Python 3.10,
Pinocchio 3.1.0, CasADi 3.6.7, and NumPy 1.26.4. Pinocchio and CasADi are not
mixed across ROS apt and pip. Miniforge 24.7.1-2 installers are checksum-pinned
for linux/amd64 and linux/arm64; every other target architecture fails the
build. The target-image preflight verifies all three numerical versions,
imports `pinocchio.casadi`, and completes a real solve before the image can be
published.

The V1 joint-speed limit is intentionally fixed at `0.5 rad/s`. This can be a
visible source of following delay; changing it requires a new calibrated
profile rather than an untracked runtime tweak. Status exposes target,
measured position, error, publisher weight, transport counters, and segmented
receive/admit/mailbox/IK/publish/follow latency.

When teleoperation is enabled, runtime secrets are supplied through
`MOTUS_DRIVER_TOKEN` and `MOTUS_TELEOP_TICKET_SECRET`. Registration uses that
Driver Bearer and the pinned Core CA mounted at
`/etc/motus-core-certs/cert.pem`; TLS verification cannot be disabled. Legacy
deployments may leave the secrets empty while teleoperation remains disabled.

Offline fake validation from this directory:

```sh
PYTHONDONTWRITEBYTECODE=1 uv run --offline --python 3.10 \
  --with 'aiohttp>=3.13,<3.15' --with 'aiortc>=1.14,<1.16' \
  --with 'cryptography>=44,<47' --with 'PyYAML>=6.0' \
  --with 'numpy==1.26.4' python -m unittest discover -s tests -v
```
