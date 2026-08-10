# Generic Teleop Shadow Driver

`generic/teleop_shadow` is a deployable, robot-free teleoperation endpoint for validating the Quest/WebRTC path before any robot adapter is connected. It implements MCP session control, strict Frame v1 validation, short-lived one-time RTC tickets, peer-bound browser frames that never disclose the session fence, the required WebRTC data channels, latest-only Pose storage, watchdog diagnostics, and a recording-only final-dispatch arbiter.

This Driver is permanently constrained to:

- `mode: shadow`
- `actuation_enabled: false`
- no robot SDK, ROS command publisher, motor API, or hardware output
- bounded `would_apply` / `would_stop` records instead of device commands

It is useful on its own: a team can negotiate a real local `aiortc` connection, stream Quest poses, observe freshness and rejection counters through `teleop_state`, and verify exactly which safe intents would reach a robot adapter without connecting a robot.

## Interfaces

| Interface | Purpose |
|---|---|
| `POST /mcp` | MCP `initialize`, `tools/list`, and `tools/call` |
| `POST /offer` | Core-internal Bearer + one-time-ticket WebRTC SDP exchange; never called by browser JavaScript |
| `GET /health` | Sanitized service, RTC, authentication, and registration status |
| `teleop_session` | `start`, lifecycle `stop`, `prepare_shadow`, `heartbeat`, `pause`, `release`, `soft_stop`, `status`, and diagnostic `submit_shadow_frame` |
| `teleop_state` | Callable read-only session/lease/Pose/RTC/final-dispatch snapshot |

Only the authenticated MCP `teleop_session.heartbeat` action renews the Core-owned lease. RTC ping, RTC traffic, and Pose arrival never renew it. A stale Core lease, stale Pose, released deadman, tracking loss, or transport disconnect enters `HOLD`. Recovery from a recoverable HOLD requires both current-generation RTC channels plus a strictly higher `clutch_sequence`; `soft_stop`, `pause`, lease expiry, and `release` require release/newer `prepare_shadow` as applicable.

The browser RTC wire frame contains pose and input data only. After a one-time
offer ticket is verified, the Driver binds that peer to an immutable runtime
generation and injects the private boot/session/epoch/fence identity internally.
Client-supplied authority fields are rejected. Session mutations remain on the
authenticated Core REST → MCP path; RTC control supports diagnostics only.

## Recording final dispatch

Accepted motion uses a one-element mailbox: if frames arrive faster than the
recording adapter consumes them, only the newest pending frame survives. Before
each `would_apply`, the owner thread rechecks authority, session generation,
dispatch generation, deadline, deadman, and tracking. The adapter never receives
the fence.

Every prepare, HOLD, pause, release, watchdog trip, and graceful shutdown uses a
separate non-droppable stop path. Control RPCs that promise a safety transition
wait for its `would_stop` acknowledgement. `teleop_state.dispatch` exposes the
mailbox depth, generations, last admitted/applied sequence, acknowledgement,
fault latch, counters, and a bounded record list. These records are executable
evidence for later live adapters; they are not evidence of physical motion.

Python cannot safely interrupt a vendor SDK call that ignores its timeout. The
arbiter revokes authority and reports not-ready when such a call exceeds its
budget, but the process must then be restarted to reclaim the stuck owner
thread. Live adapters must enforce the timeout in the SDK/transport itself.

The full wire contract and state behavior are in [TELEOP.md](TELEOP.md).

## Run locally

Python 3.10 or newer is required (3.12 is used by the image).

```bash
cd generic/teleop_shadow
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt

export MOTUS_DRIVER_TOKEN='replace-with-a-long-random-driver-token'
export MOTUS_TELEOP_TICKET_SECRET='replace-with-at-least-32-random-bytes'
export MOTUS_AGENT_CORE_VERIFY_TLS=0  # local development only
python main.py
```

Check the independently usable service:

```bash
curl http://127.0.0.1:15711/health

curl -H "Authorization: Bearer $MOTUS_DRIVER_TOKEN" \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}' \
  http://127.0.0.1:15711/mcp
```

`start` is intentionally passive: it reports readiness and does not create a session. Agent Core must call `prepare_shadow` with a new UUID `session_id`, a strictly increasing `epoch`, and a 24–128 character URL-safe random `fence`. Every subsequent protected call and Frame must match `boot_id + session_id + epoch + fence`.

The first adapter event after every process start is `would_stop/startup_safe`.
If that acknowledgement fails, `/health` reports `ready=false`, registration is
inhibited, and prepare cannot arm a session.

For a one-command, robot-free proof of the final boundary:

```bash
python smoke_recording.py
```

The command exits non-zero unless one valid Frame becomes `would_apply`,
`soft_stop` is acknowledged as `would_stop`, release revokes the session, and
the printed JSON remains permanently `actuation_enabled=false`.

## Production deployment

Build through the repository's normal discovery path:

```bash
./build.sh generic/teleop_shadow
```

The service fragment explicitly uses `network_mode: host`, because the Dashboard
deploy path installs the fragment as-is. Its HTTP server binds only
`127.0.0.1`, so Agent Core can reach MCP and `/offer` through
`localhost:15711`, while Quest JavaScript cannot reach either private endpoint.
The `aiortc` peer still gathers host-network ICE candidates for the direct
DTLS/SCTP data path. The fragment does not request host IPC, host PID, device
mounts, or privileged mode. It mounts the host Core
certificate directory read-only:

```text
/opt/phanthy-motus/data/certs -> /etc/motus-core-certs
```

Registration pins `/etc/motus-core-certs/cert.pem` with `CERT_REQUIRED`. Hostname checking is disabled because the deployed certificate is currently issued to `phanthy-motus` while host-network registration uses `https://localhost:15678`; certificate trust verification remains enabled. A missing or invalid certificate never silently downgrades TLS: registration reports `tls_error` in `/health` and retries while MCP and shadow diagnostics remain available.

`MOTUS_AGENT_CORE_VERIFY_TLS=0` is the only supported insecure override and is for explicit local development only.

The repository's real `aiortc` scenario currently uses a local test signer. The
Driver validates and consumes tickets but does not mint them. Production Quest
JavaScript sends only its SDP offer to authenticated Core; Core signs a ticket,
calls the pinned loopback Driver `/offer` with its Driver Bearer credential, and
returns only the sanitized SDP answer. Neither the ticket nor fence is exposed
to the browser.
An absent `MOTUS_TELEOP_TICKET_SECRET` is allowed only when registration is
explicitly disabled and HTTP is bound to loopback for local MCP recording
diagnostics. In that mode `/health.ready=false`, `rtc_enabled=false`, and the MCP
descriptor omits signaling, so it cannot be mistaken for a deployable teleop
instance. Any registering or non-loopback service requires the secret; absent,
empty, too-short, or invalid UTF-8 values fail process startup instead of
advertising partial teleoperation readiness. The deployed image health check
also remains unhealthy until RTC verification is enabled.

## Configuration

| Environment variable | Default | Meaning |
|---|---|---|
| `MOTUS_BIND_HOST` | `127.0.0.1` | HTTP bind address; non-loopback requires Driver Bearer authentication and is not used by the bundled deployment |
| `MOTUS_MCP_PORT` | `15711` | MCP/signaling port |
| `MOTUS_MCP_URL` | `http://localhost:15711/mcp` | URL advertised to Core |
| `MOTUS_DRIVER_ID` | `teleop-shadow-driver` | Stable unique 1–64 character instance ID; the default is only suitable for one local instance |
| `MOTUS_DRIVER_NAME` | `Generic Teleop Shadow Diagnostics` | Human-readable instance name |
| `MOTUS_ROBOT_ID` | unset | Optional stable 1–64 character robot authority ID; matches Agent Core's authority-domain boundary |
| `AGENT_CORE_URL` | `https://localhost:15678` | Core base URL |
| `MOTUS_AGENT_CORE_CA_FILE` | `/etc/motus-core-certs/cert.pem` | Pinned Core certificate |
| `MOTUS_AGENT_CORE_VERIFY_TLS` | `1` | Set to `0` only for local development |
| `MOTUS_DRIVER_TOKEN` | unset | Bearer protection for MCP, `/offer`, and registration forwarding; required whenever registration is enabled or HTTP binds beyond loopback; must use 24–4096 restricted ASCII Bearer characters (`A-Z a-z 0-9 . _ ~ + / = -`) |
| `MOTUS_TELEOP_TICKET_SECRET` | unset | HMAC secret of at least 32 UTF-8 bytes; required for registration or non-loopback binding. Absence is allowed only for registration-disabled loopback diagnostics, whose health is not ready and whose descriptor omits signaling |
| `CONFIG_PATH` | bundled `config.yaml` | Alternate configuration file |

Do not put either secret into SDP, browser responses, Frames, logs, or status. The ticket is also confidential because its signed payload contains the fence. Public snapshots intentionally omit the fence and remove it from the retained latest Frame. The only unauthenticated mode is an explicitly registration-disabled service bound to loopback for local diagnostics.

### Render multiple instances on one host

[`deploy/instances.example.yml`](deploy/instances.example.yml) is a complete
two-instance, standalone Shadow input for the strict deployment renderer. Copy it and edit only the
non-secret instance data:

```yaml
schema_version: 2
core_ca_file: /opt/phanthy-motus/data/certs/cert.pem
instances:
  - service: teleop-shadow-lab-a
    container: motus-teleop-shadow-lab-a
    driver_id: teleop-shadow-lab-a
    driver_name: Lab A Quest Teleop Shadow
    robot_id: teleop-shadow-lab-a
    mcp_port: 15711
    driver_token_env: MOTUS_TELEOP_SHADOW_LAB_A_DRIVER_TOKEN
    ticket_secret_env: MOTUS_TELEOP_SHADOW_LAB_A_TICKET_SECRET
```

Every instance must have a unique `service`, `container`, `driver_id`,
`robot_id`, and `mcp_port`. Schema v2 also requires `driver_token_env` and
`ticket_secret_env`: these are non-secret names of host environment variables,
must match the dedicated `MOTUS_TELEOP_*_DRIVER_TOKEN` or
`MOTUS_TELEOP_*_TICKET_SECRET` namespace for their role, and must be globally
unique across every credential role and instance. Generic process variables
such as `PATH`/`HOME` and the legacy shared secret names are rejected, so an
already-populated host variable cannot silently satisfy secret preflight. Both
identity fields are stable 1–64 character
IDs that match Agent Core's authority boundary; ports are restricted to the
Driver allocation range `15700-15799`. Unknown fields, duplicate YAML keys,
unsafe identifiers, non-normalized CA paths, Compose interpolation, and
identity, port, or credential-reference conflicts are rejected.
The renderer fixes `network_mode: host`, binds each HTTP endpoint to
`127.0.0.1`, and mounts only the selected Core CA file read-only. Before any
output is produced, `core_ca_file` must resolve to an existing regular X.509
certificate/bundle inside the real `/opt/phanthy-motus` host deployment tree.
An escaping symlink, device/FIFO, invalid certificate, oversized file, or any
private-key PEM block is rejected. The canonical public-certificate path is
written to Compose; the Core private key is never mounted.

Run the renderer on the deployment host, where it can validate that CA file.
If the bundled single instance is installed, release its session and stop it
before reusing port `15711`. Confirm every selected port is free with
`ss -ltnp`; another process can race this check, so Docker health and identity
checks below remain mandatory. Then build or select an image, inspect the exact
output, and atomically create a Compose file:

```bash
docker build -t teleop-shadow:local generic/teleop_shadow

python generic/teleop_shadow/render_instances.py \
  --instances generic/teleop_shadow/deploy/instances.example.yml \
  --image teleop-shadow:local \
  --dry-run

python generic/teleop_shadow/render_instances.py \
  --instances generic/teleop_shadow/deploy/instances.example.yml \
  --image teleop-shadow:local \
  --output generic/teleop_shadow/deploy/teleop-shadow.compose.yml

docker compose \
  -f /opt/phanthy-motus/docker-compose.yml \
  stop generic-teleop-shadow  # only when the old bundled service exists
```

`--dry-run` and `--stdout` emit canonical Compose to stdout and never write a
file. Normal output uses a same-directory temporary file plus atomic replace,
so an interrupted render cannot leave a partial Compose document.

#### Migrate a schema v1 instances file

Schema v1 used the same host `MOTUS_DRIVER_TOKEN` and
`MOTUS_TELEOP_TICKET_SECRET` for every generated service, placing all instances
in one host trust zone. The v2 renderer deliberately rejects v1 instead of
silently preserving that credential alias. Migration is a coordinated
maintenance window, not a zero-downtime credential rotation:

1. Release every active schema v1 session through Core. Do not continue while
   Core reports an active session or a restart-recovery authority guard.
2. Stop every service produced by the v1 Compose file. Keep the old services
   stopped for the remainder of the migration.
3. Generate a different Driver token using 24–4096 restricted ASCII Bearer
   characters and a different ticket secret of at least 32 UTF-8 bytes for every
   exact Driver ID. Provision each value under its new dedicated host
   environment name.
4. Put the same values into Core's exact-ID `MOTUS_DRIVER_TOKENS` and
   `MOTUS_TELEOP_TICKET_SECRETS` maps, set
   `MOTUS_ENFORCE_DRIVER_AUTH=true`, and restart Core. A mapped ID does not fall
   back to the legacy shared credential; retain legacy fallbacks only for
   unrelated Drivers that have not yet migrated.
5. Change `schema_version` to `2`, add a distinct `driver_token_env` and
   `ticket_secret_env` to every instance, re-render, run
   `docker compose config --quiet`, and recreate the stopped services. Editing
   an already generated Compose file is not a supported migration.
6. Complete the health, registration, and real `/offer` checks below for every
   instance before admitting a session.
7. After every Driver has migrated, remove the legacy Core fallbacks and restart
   Core once more.

There is no old/new credential overlap for one mapped ID. Do not roll back by
starting a v1 service against v2 Core maps. If rollback is unavoidable, keep
the Driver stopped, restore one reviewed matching Core/Driver credential set
during a maintenance window, restart Core, and repeat all acceptance checks.

The bundled `deploy/service.yml` remains a compatible single-instance fragment
for the Dashboard deployment path. It still passes the container-standard
`MOTUS_DRIVER_TOKEN` and `MOTUS_TELEOP_TICKET_SECRET` variables directly. Use
the schema v2 renderer whenever two or more independently authenticated Shadow
instances run on one host.

The instances YAML and generated Compose contain no secret values. For each
service, Compose maps its unique host references into the unchanged container
variables `MOTUS_DRIVER_TOKEN` and `MOTUS_TELEOP_TICKET_SECRET`; a missing or
empty value makes `docker compose config` fail before a container starts. The
renderer never reads these host values, so Compose checks only presence; Driver
and Core enforce the contract of 24–4096 restricted ASCII Bearer characters and
the 32-byte ticket minimum at startup. Agent Core must be configured through
`MOTUS_DRIVER_TOKENS` and `MOTUS_TELEOP_TICKET_SECRETS` with the matching exact
Driver-ID-specific values before that instance can register and negotiate RTC.
Core must also reject duplicate secret values, because a renderer can prove
references are distinct but cannot inspect secret-manager values.

Supply both values from a secret manager or read them without putting literal
values into shell history, then validate and start the independently named
services:

```bash
read -rsp 'Lab A Driver token (24–4096 restricted ASCII Bearer characters): ' \
  MOTUS_TELEOP_SHADOW_LAB_A_DRIVER_TOKEN \
  && export MOTUS_TELEOP_SHADOW_LAB_A_DRIVER_TOKEN
printf '\n'
read -rsp 'Lab A ticket secret (at least 32 UTF-8 bytes): ' \
  MOTUS_TELEOP_SHADOW_LAB_A_TICKET_SECRET \
  && export MOTUS_TELEOP_SHADOW_LAB_A_TICKET_SECRET
printf '\n'
read -rsp 'Lab B Driver token (24–4096 restricted ASCII Bearer characters): ' \
  MOTUS_TELEOP_SHADOW_LAB_B_DRIVER_TOKEN \
  && export MOTUS_TELEOP_SHADOW_LAB_B_DRIVER_TOKEN
printf '\n'
read -rsp 'Lab B ticket secret (at least 32 UTF-8 bytes): ' \
  MOTUS_TELEOP_SHADOW_LAB_B_TICKET_SECRET \
  && export MOTUS_TELEOP_SHADOW_LAB_B_TICKET_SECRET
printf '\n'

docker compose \
  -f generic/teleop_shadow/deploy/teleop-shadow.compose.yml \
  config --quiet

docker compose \
  -f generic/teleop_shadow/deploy/teleop-shadow.compose.yml \
  up -d
```

Use `docker compose config --quiet` for validation so resolved environment
values are not printed. Do not use plain `docker compose config` in logs or
review artifacts because it resolves the references. The example deliberately
sets `robot_id == driver_id`:
each service is therefore a directly usable, standalone Shadow authority
domain and needs no physical-robot binding. The registration payload, tool
descriptors, `teleop_state`, and `/health` report each configured identity;
`boot_id` remains unique per process start. The image health check reads each
service's `MOTUS_MCP_PORT`, so it follows the rendered per-instance port rather
than the single-instance default.

Verify both independent services instead of accepting only a container state:

```bash
docker compose \
  -f generic/teleop_shadow/deploy/teleop-shadow.compose.yml \
  ps
curl -fsS http://127.0.0.1:15711/health | python3 -m json.tool
curl -fsS http://127.0.0.1:15712/health | python3 -m json.tool
```

Each health response must show its exact `driver_id == robot_id`, the expected
port's identity, `ready=true`, `rtc_enabled=true`, `mode=shadow`, and
`actuation_enabled=false`. Then query Core's authenticated
`GET /api/teleop/robots`; both entries must report `teleop_ready=true` and
`reason=ready` before Acquire is considered usable. These checks prove local
configuration, not that Core and Driver hold the same ticket secret.

For each instance, acquire a fresh Shadow session in Core's `/teleop` console
and click **Connect WebRTC Shadow**. This generates a real browser SDP offer,
posts it through Core's authenticated
`/api/teleop/sessions/{session_id}/signaling/offer` route, and exercises the
private Driver `/offer` endpoint. Accept the instance only after the browser
reports a connected peer, `teleop-control` and `teleop-pose` are both open, and
Driver `teleop_state.rtc.connected` is true. Disconnect and release that
session before checking the next instance. A Driver response of
`401 invalid_signature` means its `ticket_secret_env` value does not match the
Core map; fix the mapping instead of enabling a shared fallback. Browser code
must never call the Driver `/offer` endpoint directly because the one-time
ticket and authority fence remain server-side.

#### Bind a Shadow adapter to an existing physical robot

To share one command authority domain with a real robot, configure the Shadow
instance's `robot_id` to the exact ID of an existing trusted HTTP root Driver
*before its first registration*. That root must expose at least one ordinary
actuator and must not itself be an authority alias. The Shadow Driver cannot
approve this relationship and the renderer never accepts an owner token.

After both Drivers have registered as trusted, an owner stages the binding:

```bash
curl --fail-with-body \
  --cacert /opt/phanthy-motus/data/certs/cert.pem \
  -H "Authorization: Bearer $ACCESS_TOKEN" \
  -H 'Content-Type: application/json' \
  -X PUT \
  -d '{"root_mcp_id":"g1-driver"}' \
  https://localhost:15678/api/mcp/teleop-shadow-lab-a/authority-domain
```

The response must be `202` with `restart_required=true`. Release every teleop
session, restart Core through the normal deployment path, and query
`GET /api/mcp` plus `GET /api/teleop/robots`. Accept the binding only when the
active `authority_domain`, descriptor `robot_id`, and root ID are identical and
the teleop directory reports `reason=ready`. There is no automatic session,
heartbeat, RTC, or authority recovery across that restart; Acquire again with
a new epoch. To remove a binding, stop the Shadow instance, use the owner-only
`DELETE /api/mcp/{driver_id}/authority-domain`, restart Core, and keep the
instance stopped until its immutable registered robot identity is deliberately
re-provisioned under a new reviewed Driver ID.

The CA mount is canonicalized at render time. After Core certificate/CA
rotation, re-run the renderer, recreate every generated service, and repeat
registration TLS plus `/health` identity checks; do not assume an old bind
mount followed a certificate-manager symlink.

## Tests

```bash
cd generic/teleop_shadow
python -m unittest discover -s tests -v
```

The suite covers pure protocol/runtime logic, the standalone recording smoke
proof, authenticated MCP over local HTTP, certificate-pinned local TLS, and an
actual local `aiortc` offer with both required data channels and visible
`would_apply`/disconnect `would_stop` evidence. It also validates deterministic
schema v2 multi-instance Compose rendering, strict identity and credential
reference conflicts, 23/24-byte Bearer and 31/32-byte ticket boundaries,
cross-instance Bearer rejection, A ticket acceptance on A plus
`invalid_signature` rejection on B, secret absence from rendered output and
health, atomic output, and the dynamic RTC-required container health check.

## Third-party use

- `aiohttp` provides HTTP/MCP signaling and outbound registration (Apache-2.0/MIT).
- `aiortc` provides the standards-based ICE/DTLS/SCTP/WebRTC implementation (BSD-3-Clause). This Driver does not implement or fork WebRTC.
- `cryptography` is the bounded TLS/DTLS crypto backend used by `aiortc` (Apache-2.0/BSD-3-Clause).
- `PyYAML` loads deployment configuration (MIT).

See `requirements.txt` for the bounded supported version ranges.
