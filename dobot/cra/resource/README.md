# Official skeleton model

`cr3_robot.urdf` is copied with trailing whitespace normalized from:

- Repository: https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4
- Commit: `ec201c40d5db63185a3593ec9c39f80fe25aa527`
- Path: `dobot_rviz/urdf/cr3_robot.urdf`
- SHA-256: `e79a0f39c0c06f2a645031f3a3d92bc111e2968844bb30a59846ddc838376bc7`
- License: MIT, retained as `LICENSE.dobot`.

The V4 CRA ROS model uses joint1..joint6. The model resource derives a display-only
URDF from the original file: both long rods follow their offset center planes,
with two right-angle connectors per rod (four fixed corners). Compensating fixed
frames preserve endpoint transforms. Compound RPY origins use single-axis rotations so
the existing platform renderer and standard URDF FK agree. All six original
actuated joints and all physical link transforms are preserved. No STL meshes,
RViz, MoveIt or platform modifications are required.

The supplied TCP/IP V4.6.6 manual maps feedback `CRRobotType=113` to CR3A.
That type was also observed directly from the target controller. This driver
selects the official V4 `cr3` chain for that type only. It does not select it for
other payload/geometry variants based on the CC262 controller model or a name
prefix. The model is for state visualization, not motion planning or calibration.

The upstream publisher sends SDK joint positions under joint1..joint6; our
feedback adapter supplies the same order with degree-to-radian conversion.
No arbitrary joint-angle offsets, direction changes or IMU orientation are added.

## Rigid rod centerlines

URDF joint origins alone do not encode the shape between them. In particular,
the upper arm has equal joint-origin Z values but the actual rod is offset in Z,
so a single straight origin-to-origin bone passes outside the physical rod.

The center planes come from the same commit's `dobot_rviz/meshes/cr3` files:

| Mesh | Mid-length section | Z bounds at section (m) | Center plane (m) | SHA-256 |
|---|---|---|---|---|
| J2.STL | X=-0.137m | 0.0887877062 .. 0.1688122957 | 0.1288 | `1fe1454681fc145c33bcd870995541134635ff604aeb471c047d9b976a707c26` |
| J3.STL | X=-0.115m | -0.0227884789 .. 0.0473884753 | 0.0123 | `24768f47c1ccddedd22bb75f417d2a14503688b6b2416752acd7e5602b52a674` |

Intersect mesh triangle edges with the stated plane, take the midpoint of the
section's Z bounds, and round to 0.1mm. The upper-arm route in Link2 is
`(0,0,0) -> (0,0,0.1288) -> (-0.274,0,0.1288) -> (-0.274,0,0)`.
The forearm route in Link3 is
`(0,0,0) -> (0,0,0.0123) -> (-0.23,0,0.0123) -> (-0.23,0,0.1283)`.
These are schematic rod centerlines, not full CAD shell geometry. Do not lock
real wrist joints at 90 degrees simply because a particular pose looks bent.
