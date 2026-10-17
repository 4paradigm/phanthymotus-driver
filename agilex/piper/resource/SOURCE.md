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
exports the motion chain, removing upstream inertia/mesh/collision elements.
`monitor.load_model()` then adds simple URDF box/cylinder visuals from
`monitor_shapes.json`: wider base and joint housings, slimmer grey arm links,
a smaller wrist and two fingers. Dimensions/colors are approximate display
shells based on the user's side and gripper reference photos, not calibrated
collision geometry. The photos themselves are not bundled. Core remains
unchanged and currently ignores these `<visual>` elements. To show modest body
volume in that renderer, the export also adds `display_*` fixed branches outlining
the major boxes and hexagonal cylinder cages. They attach to the existing links,
never replace the six measured joints or alter the original link transforms.
Core renders these branches with its existing bone cylinders and endpoint spheres.
They are approximate skeletal housings rather than filled mesh surfaces. Fine
decorative strips are omitted; fingers use the existing renderer's slimmer bones.

The export freezes the two prismatic finger
joints at q=0 because that renderer handles revolute joints only. Gripper opening
is measured and displayed as a parameter, not drawn as finger motion; the new
double-finger shell has a fixed illustrative opening and no camera/accessories.

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
