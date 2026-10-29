# L20 Lite two-hand visualization model

The two simplified URDFs come from the [official LinkerHand URDF repository](https://github.com/linker-bot/linkerhand-urdf/tree/075cc7d42cc1e756bdcbece0fc069a0779fc5237/L20lite).
That repository is Apache-2.0 licensed; its license is preserved here as
`LICENSE.linkerhand-urdf`.

The models retain link names, joint origins, axes, limits, and mimic relations.
Meshes, collision, and inertial data were removed. Origin translations were
scaled four times so the hands fit the dashboard's shared skeleton camera.
This is a display model, not a metric, collision, or control model.

The glove broadcast contains 0..255 motor targets rather than radians. The
preview linearly maps the five flexion motors between the operator-confirmed
open/fist endpoints in `config.yaml`. Uncalibrated sway, roll, and distal thumb
joints remain neutral. These approximate angles are never used to control the
physical hands. The two URDFs are combined into one model because the current
dashboard fetches one `model` resource per driver.
