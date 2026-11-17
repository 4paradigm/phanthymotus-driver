# PNP FR3 teleoperation

The Orin card offers `follow`, `pause`, and `status`. The PNP service starts
the four installed vendor launchers for each selected side in order. The vendor initializer
moves the arm to its configured pose and initializes the Hand during `follow`.

On the PNP workstation, keep the vendor tree at `~/frankateleop` and its
`polymetis` Conda environment. Run `sh install-pnp.sh` from this directory as
the `pnp` user. The installer generates the runtime copy of the vendor follow
script with corrected gripper units. The unit uses the host's realtime kernel
and an 8 GiB memory-lock limit.

On PNP, install the Orin service key for the `pnp` user with SSH
`restrict,command="/home/pnp/.local/lib/fr3-pnp/pnp-card-ctl"`. The PNP user
needs passwordless permission for just `/usr/bin/systemctl start
pnp-fr3-right-teleop.service` and its `stop` counterpart. Store the private key
and verified PNP host key in the Orin SSH directory mounted by `service.yml`.

Build the base Franka image using the parent driver documentation. Create the same build context as the parent build, including `common/`.
Build the PNP variant inside that context with
`docker build -f deploy/pnp-remote/Dockerfile --build-arg BASE_IMAGE=franka-fr3:local --build-arg SOURCE_REVISION=<git-commit> -t franka-fr3:pnp-remote .`
and deploy with the service template in this directory. Keep the direct-FCI
container stopped while the PNP service controls the same arm.

Set `FRANKA_PNP_SSH_HOST` to the right workstation and
`FRANKA_PNP_LEFT_SSH_HOST` to the left workstation (for example `pnp@<left-host>`).
Run the PNP installer and install the restricted Orin key on each host. Leave
the left variable empty when both USB devices are attached to one workstation.
In that mode the card starts the sides that are ready on that workstation.
The card keeps per-host and per-arm status, and stops all configured services
on pause. If an SSH start request fails, it attempts to
stop each selected service and reports any stop that could not be confirmed.

Right uses FTBMDRZS, FR3 172.16.0.2 and ports 50051, 50053, 6001. Left uses
FTB57FAR, FR3 172.16.0.3 and ports 50052, 50054, 6002. Use the workstation SSH
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
