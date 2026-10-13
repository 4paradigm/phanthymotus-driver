# PiPER monitoring model provenance

- Upstream: https://github.com/agilexrobotics/piper_ros
- Branch at acquisition: `noetic`
- Pinned commit: `ac41fcbcdda598f01b51cf6175ed9a24d0dacadc`
- Model: `src/piper_description/urdf/piper_description.urdf`
- Model SHA256 (unaltered upstream bytes): `c1a5c29ae9b91160557ed02bddd47365a71f0eb1653abb4997eb34f124021486`
- License: MIT; verbatim upstream license is `LICENSE.piper_ros`.
- License SHA256: `e32d78fd3761052ab0b2bb42fef8a7a9ad3f298d315d305333c349d2aff1f134`

`upstream/piper_description.urdf` retains the original bytes and exporter notice.
No mesh assets are bundled or fetched at runtime. `monitor.renderer_urdf`
exports links and joints only, removing inertia/visual/collision elements that
the Core skeleton renderer does not use. It freezes the two prismatic finger
joints at q=0 because that renderer handles revolute joints only. Gripper opening
is measured and displayed as a parameter, not drawn as finger motion.

Mixed RPY origins are factored into fixed yaw, fixed pitch, and the original
joint's roll, preserving translation and the six original arm joint names.
URDF uses `Rz(yaw) Ry(pitch) Rx(roll)`; Core's Three.js `Euler('XYZ')` uses the
opposite composition. Single-axis factors avoid changing Core and preserve the
original forward kinematics, including joint2's mixed RPY origin. Unit tests
compare every original link transform at zero and two nonzero poses using
independent URDF and renderer matrix compositions.

This is an arm visualization model, not a controller or a command-limit source.
The injected snapshot supplies the actual SDK/deployment limit intersection.
The original model variant and zero/axis convention still require physical
verification when integrated on the site; simulated kinematic agreement is not
a claim of real-hardware model calibration.
