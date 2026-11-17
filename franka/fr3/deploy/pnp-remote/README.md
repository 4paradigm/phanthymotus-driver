# PNP FR3 right arm

The Orin card offers `follow`, `pause`, and `status`. The PNP service starts
the four installed right-arm vendor launchers in order. The vendor initializer
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
