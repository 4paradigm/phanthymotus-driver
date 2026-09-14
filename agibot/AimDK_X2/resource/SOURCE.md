# URDF provenance

Source: `X2_URDF-v1.3.0.zip`
https://x2-aimdk.agibot.com/zh-cn/latest/_downloads/2ffc9785259556f409e385974a7a0461/X2_URDF-v1.3.0.zip

The upstream zip is ~48MB, almost entirely mesh STLs plus MuJoCo `.xml` scene variants. Only
the three `.urdf` text files needed by the dashboard's `sensor/skeleton` renderer (joint names,
limits, kinematic tree) are vendored here:

- `x2_fist.urdf` — end-effector variant: fist (no fingers)
- `x2_hand.urdf` — end-effector variant: hand (basic gripper)
- `x2_ultra.urdf` — end-effector variant: ultra (full sensor package)

**Not vendored** (fetch from the URL above if needed):
- `meshes/*.STL` — visual/collision meshes referenced by `<mesh filename="./meshes/...">` in
  each URDF. The skeleton renderer only needs the joint tree, so missing meshes do not break
  it; if 3D mesh rendering is ever added, download the full zip and place `meshes/` alongside
  the `.urdf` files.
- `x2_fist.xml`, `x2_hand.xml`, `x2_ultra.xml`, `scene.xml`, `package.xml` — MuJoCo variants,
  not used by this driver.
- `x2_ultra_simple_collision.urdf` — simplified-collision variant, not currently used.

`config.yaml`'s `end_effector` key selects which of the three `.urdf` files the `model`
resource tool serves.
