# Official SDK provenance

- Upstream: https://github.com/Dobot-Arm/TCP-IP-Python-V4
- Commit: `55ec1ec82201aaf2e6d54aa47b6272bbaa14c815`
- Python distribution: `dobot_sdk`, version `2.0.0`
- License: MIT; upstream source archive includes `LICENSE`.
- Source artifact: https://codeload.github.com/Dobot-Arm/TCP-IP-Python-V4/zip/55ec1ec82201aaf2e6d54aa47b6272bbaa14c815
- SHA-256: `b7bccfc2b6970441f3a73e7f2252a1d15a7225e23bdf4606973457a9789903b1`

`requirements.txt` pins both the commit and the artifact hash. Docker installs
the official package into the driver image. No SDK installation on the Jetson
host, shared-object mount, or controller-side installation is required. The SDK
is Python/numpy/requests, with no x86-only native robot library.

The SDK source declares an unbounded `setuptools_scm>=6.2` build requirement.
`build-requirements.txt` pins the build toolchain before pip installs the SDK
with `--no-build-isolation`. This prevents an isolated build from resolving a
new scm/vcs-versioning stack against Ubuntu's older system `packaging`.
All required build dependencies are installed explicitly, as required by
[pip's build isolation contract](https://pip.pypa.io/en/stable/reference/build-system/#disabling-build-isolation).
The upstream SDK source and artifact checksum remain unchanged.

The adapter imports upstream `DobotConnection`, `FeedbackParser`, `MyType` and
the SDK version. It deliberately does not instantiate `DobotRobot`, whose
connection lifecycle also opens Dashboard. A single receive-only feedback
connection supplies the joints skeleton and joint-angle JSON, even outside
TCP control mode. joints and joint_move are visible cards in this release; the model
resource supplies background URDF geometry. The joint_move actuator lazily opens one serialized Dashboard connection.
No systemcontrol or skin card is registered.

Compatibility baseline: CRA six-axis V4 feedback layout, 1440 bytes, little
endian, test magic `0x0123456789abcdef`, as documented by the supplied TCP/IP
V4.6.6 manual. Framing and protocol-offset tests use that specification rather
than generating their input from the SDK dtype. Other robot families and
firmware layouts are not claimed supported.

Upstream is unmodified. The adapter handles:

- TCP packet splitting/coalescing, malformed-byte resynchronization and bounded
  buffering, reconnects, freshness, and NaN/Inf rejection.
- `BrakeStatus` as a per-joint bitmask, not the SDK parser's collapsed boolean.
  The decoded array is ordered J1..J6 (wire bits 5..0, respectively).
- `CRRobotType` as an unsigned byte, preserving values above 127; type 113 is
  identified as CR3A for the bundled official skeleton model.
- Mode 11 as collision per the interface manual, rather than upstream's JOG
  label. Unknown mode values remain unknown rather than becoming INIT.
- `SafeState`, dynamically assigned by the SDK parser but omitted by its
  dataclass serialization; both safety bytes retain their unsigned high bits.
- Guaranteed socket close after upstream disconnect, including failed-connect
  sockets for which shutdown can fail before the SDK reaches its close call.
  Cleanup exceptions are contained by the client so reconnect and stop/join
  still complete; diagnostics retain them separately as `cleanup_error`.
- Receive and freshness timeouts allow at least three nominal feedback periods,
  including after port changes from the joints configuration. Port 30006 therefore
  gets at least 3 seconds instead of timing out at its 1-second packet interval.

The joints sensor publishes through output topics without business actions.
The reserved info action returns static card metadata (description, schemas
and ports). Removed systemcontrol actions are rejected as unknown tools.

Joint streams are intentionally compact: the skeleton carries timestamp,
availability and name/q pairs in radians; the numeric JSON carries timestamp,
availability and six independent degree fields, joint1_deg through joint6_deg,
one per line. Unavailable angles are null. Transport diagnostics, model metadata,
velocities and duplicate angle representations are not streamed. The model
resource still validates robot identity and exposes geometry errors; unmatched
models never publish skeleton joint positions.

For an SDK update, download the new immutable commit artifact, compute its
SHA-256, update requirements and this file plus `SDK_COMMIT`, and run the
protocol and MCP tests before building a new ARM64 image.

Joint motion uses SDK MovJ with joint coordinates and per-command v/a, cp=0.
After confirming enabled feedback, move_j sends EnableSafeSkin(0), then ClearError; resume sends
EnableSafeSkin(0) before Continue. All acknowledgements are checked, with no
automatic retry or skin re-enable. Successful replies are not verified state readback.

Before disabling skin or sending MovJ/Continue, disabled feedback triggers exactly one EnableRobot()
without load arguments. Motion waits for enabled feedback (default timeout 5s);
ACK alone is insufficient. Resume must still be paused after enabling.
