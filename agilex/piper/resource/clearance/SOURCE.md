# Bundled PiPER 100 mm gripper profile

- Upstream: https://github.com/agilexrobotics/agx_arm_urdf
- Source revision: 680fa925cd00db7e2628563d2edc5540963045de
- License: MIT; see LICENSE.agx_arm_urdf, retained verbatim from that revision.
- Profile SHA-256: 549be81f8571e355f9f671fff2559a0a90b76b3eaaece6b769dc9a288a15d0a7
- JSON size: 67,334 bytes.

Derived from the base PiPER URDF and gripper Xacro. Joint origins are converted
to homogeneous transforms; collision STL vertices are transformed into their
link frames and reduced to convex hull vertices offline. The minimum of a
linear plane projection is preserved by this reduction. Runtime uses NumPy,
without loading STL, running SciPy or downloading model resources.

The profile's source_hashes records all seven URDF/Xacro/collision-STL inputs.
Each source file was retrieved at the pinned upstream revision and its SHA-256
checked against this manifest before bundling. The profile matches the existing
site preview model; it covers flange_link, gripper_base, gripper_link1 and
gripper_link2. It does not model all arm links, carried objects or cables.

The 0–100 mm opening range describes this model. Retained joint reference ranges
are observational input limits, not changes to motor or control limits.

The default table_z_mm=0 assumes a horizontal table at the model mounting plane.
No installed-tool measurement or physical calibration is implied. Outputs remain
estimated with calibration_verified=false; collision protection is inactive.
