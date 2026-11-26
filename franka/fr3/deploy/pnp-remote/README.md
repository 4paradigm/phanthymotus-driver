# PNP FR3 teleoperation

The Orin card offers `start_trajectory`, `stop_trajectory`, `stop_motion`, and `status`. The PNP service starts
the four installed vendor launchers for each selected side in order. The vendor initializer
moves the arm to its configured pose and initializes the Hand during alignment.

On the PNP workstation, keep the vendor tree at `~/frankateleop` and its
`polymetis` Conda environment. Run `sh install-pnp.sh` from this directory as
the `pnp` user. The installer generates the runtime copy of the vendor follow
script with corrected gripper units. The unit uses the host's realtime kernel
and an 8 GiB memory-lock limit.

On PNP, install the Orin service key for the `pnp` user with SSH
`restrict,command="/home/pnp/.local/lib/fr3-pnp/pnp-card-ctl"`. The PNP user
needs passwordless permission for just `/usr/bin/systemctl start
pnp-fr3-right-teleop.service` and its `stop` counterpart. Store existing service credentials under the persistent state directory as
`id_ed25519` and `known_hosts`, or let canvas initialization provision them.

Build the standard FR3 image using the parent driver documentation and deploy
it with `../service.yml`. The Teleop card shares the FR3 driver's registered
MCP endpoint on port 15723 with the other FR3 cards. The standard service
persists the PNP SSH identity and verified host keys under `/var/lib/fr3-pnp`.
Stop any older FR3 driver container before starting this service.

Set `FRANKA_PNP_SSH_HOST` to the right workstation and
`FRANKA_PNP_LEFT_SSH_HOST` to the left workstation (for example `pnp@<left-host>`).
Run the PNP installer and install the restricted Orin key on each host. Leave
the left variable empty when both USB devices are attached to one workstation.
In that mode the card starts the sides that are ready on that workstation.
The card keeps per-host and per-arm status, and stops all configured services
when you execute `stop_trajectory` or `stop_motion`. If an SSH start request fails, it attempts to
stop each selected service and reports any stop that could not be confirmed.

Right and left master identities come from the installed vendor side launchers.
Right ports are 50051, 50053, 6001; left ports are 50052, 50054, 6002. Use the workstation SSH
address for the host variables. The service unit retains its original identifier
on both machines for compatibility with the restricted SSH command.


## Install from the driver image

All active PNP support sources are embedded under `/deploy/pnp-remote`.
On Orin, export the package from the image:

```bash
docker run --rm --entrypoint sh IMAGE /deploy/pnp-remote/export-pnp.sh > pnp-support.tar.gz
scp pnp-support.tar.gz USER@PNP_HOST:/tmp/
```

On the PNP host, extract into an empty directory and run `sh install-pnp.sh`.
The installer checks the vendor runtime, renders the service for the login
account, and installs the restricted start/stop sudo rule. Administrative
authentication is required once. Configure the Orin public SSH key with the
forced command described above and verify the host key before using the card.
Set `PNP_VENDOR_ROOT` and `PNP_CONDA_BASE` for alternative installation paths.
The vendor runtime and libfranka executables must match the PNP host CPU and
robot system version. The export contains our support sources and requires the
vendor runtime already installed on the PNP host.

## Wired Orin to PNP connection

The validated workstation uses wired 172.16.0.200/24 and Orin uses
172.16.0.184/32. Orin's NetworkManager profile `franka-fr3` has a persistent
host route to 172.16.0.200 on eno1. The card's default SSH destination is now
`pnp@172.16.0.200`. Use `FRANKA_PNP_SSH_HOST` for another workstation.
Validate its SSH host key and check `ip route get <PNP_IP>` before changing
the destination. FCI runs locally between PNP and the Franka controllers.


## Initialize from the canvas

Fill in **PNP Host Username** and **PNP Host Password**. When the card loads,
the driver checks the wired IPv4 configuration, discovers the PNP workstation,
and starts initialization automatically. The installer discovers the vendor tree
and Polymetis Conda base on the remote host. Loading the card always starts an asynchronous install. `status` returns `initialization.state`
and the current stage. It checks the package version, installs the image's
support sources, configures the service and limited sudo rule, authorizes a
restricted service key, and verifies key login. Installation never starts the
robot service. Execute `start_trajectory` after initialization reports `ready`. The card
waits for alignment, requests Follow, and completes ACP after Follow is confirmed.

The driver keeps the private service key and known host keys in the writable
`/var/lib/fr3-pnp` volume. Keep this volume across image replacements.
The first connection records the SSH host key (OpenSSH accept-new). A changed
key is rejected and needs verification before updating the trust file.
Passwords are marked sensitive in the card schema, excluded from returned
configuration and logs, and cleared from driver memory after an attempt.
Canvas-side persistence and request logging depend on the Core implementation.
The initial login must permit SSH password authentication and sudo using the
same password. Initialization requires the PNP vendor runtime already installed.


## Wired discovery and slave IP configuration

PNP host discovery is automatic. Orin sends ARP requests on
active wired interfaces within the /24 networks derived from the slave IPs.
It excludes its own addresses and both slave controllers, checks SSH, and
verifies both known USB masters after login. A unique matching host is selected.
Credentials are required for a new host. Discovery identifies USB connectivity;
the masters do not stream joint telemetry over Ethernet before teleoperation.
Multiple matches stop initialization. Connect only the intended PNP workstation
to the robot network before retrying.

Host networking, NET_RAW and NET_ADMIN are required for ARP discovery and
per-candidate wired host routes. The installer stores distinct right and left
slave IPs in the PNP runtime and passes them to the vendor launchers.


During initialization, vendor right/left IP assignments are read from each
robot_client configuration and cross-checked against its gripper configuration.
Use `status` to see the resolved IPs and their sources. Missing or ambiguous
assignments retain the current values and produce a warning. Configure the
controller addresses on the PNP workstation.


## Master USB identities

The driver enumerates /dev/serial/by-id and reads each side's teleop port from
the manufacturer's 4_run_env.sh launcher. It checks that the port has an entry
in the manufacturer's PORT_CONFIG_MAP before exposing it as a ready master.
USB serial numbers are read from these paths at runtime. No machine-specific
master serial number is embedded in the driver. The manufacturer must provide
a consistent launcher and calibration entry for each master. Unmapped USB
devices are reported without assigning a left/right role by enumeration order.


## Teach session

`start_trajectory` runs the vendor alignment sequence and releases Follow after
fresh alignment feedback. `stop_trajectory` ends the session. The canvas
interrupt hooks use `stop_motion`, which requests the same verified PNP
shutdown. `status` reports the service and execution stage.

During Follow, the card records PNP telemetry received on UDP port 18642.
The installed vendor runtime must provide this telemetry. Recordings default
to `/var/lib/fr3-pnp/trajectories` and use UTC timestamp filenames.
Confirmed stops with valid samples produce JSONL files. Interrupted or empty
captures remain partial files.

Use `list_trajectories` to find a filename and pass it to
`delete_trajectory`. The driver sends ACP results for start, stop, interrupt,
and delete operations. Playback remains unimplemented.
