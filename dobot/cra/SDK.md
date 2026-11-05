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
connection supplies joints and the safe_skin_control JSON output, even outside
TCP control mode. There is no standalone safe_skin sensor. The safe_skin_control and systemcontrol actuators share one
RobotController and a lazily created, serialized DashboardConnection. They call
the official RobotControl methods for skin settings, operating state, diagnostics
and explicit system actions. Startup never transmits control commands. SDK
formatting is retained; the adapter uses sendall and complete semicolon framing
to handle partial TCP writes/reads. No second Dashboard client is created.

Compatibility baseline: CRA six-axis V4 feedback layout, 1440 bytes, little
endian, test magic `0x0123456789abcdef`, as documented by the supplied TCP/IP
V4.6.6 manual. Framing and protocol-offset tests use that specification rather
than generating their input from the SDK dtype. Other robot families and
firmware layouts are not claimed supported.

Upstream is unmodified. The adapter handles:

- TCP packet splitting/coalescing, malformed-byte resynchronization and bounded
  buffering, reconnects, freshness, and NaN/Inf rejection.
- `BrakeStatus` as a per-joint bitmask, not the SDK parser's collapsed boolean.
  The published array is ordered J1..J6 (wire bits 5..0, respectively).
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

All writes require explicit confirmation. No automatic takeover, enable, alarm
clearing, motion resumption, IO write, arbitrary command endpoint or invented
safe-skin reset is exposed. The system card calls RequestControl, PowerOn,
EnableRobot, DisableRobot, ClearError, Stop, Pause, Continue, SpeedFactor,
StartDrag and StopDrag; its explicit read-only GetErrorID query parses the
documented flat JSON array of alarm codes. State guards are action-specific;
stale feedback does not block stopping/disable/exit-drag requests or error
queries. EnableRobot is called without load arguments, preserving the
controller's configured payload and center parameters.

Acknowledgements are not verified physical state changes. Skin switch and
sensitivity readback are absent; the skin actuator's compact JSON output only reports trigger flags,
and request records live in the actuator get_status response. Each card keeps its own
request/error history. Writes are never automatically replayed.

All three visible cards expose get_status as a public action. The platform
filters the reserved info action out of its selector; info now returns static
card metadata only (description, schemas and ports). Status queries use fresh
feedback without opening Dashboard or echoing a previous write as actual state.
System status includes explicit enabled/running/paused/dragging/collision/error
flags and nullable inferred power state. Unavailable feedback yields null values.

For an SDK update, download the new immutable commit artifact, compute its
SHA-256, update requirements and this file plus `SDK_COMMIT`, and run the
protocol and MCP tests before building a new ARM64 image.
