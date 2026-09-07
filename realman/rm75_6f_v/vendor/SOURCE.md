# Official RealMan Python API2 SDK

- Upstream: https://github.com/RealManRobot/RM_API2
- Archive: `RM_API2-main.zip`, downloaded 2026-09-03
- Archive SHA-256: `04475322beaa25b7af22d5b59a5edaacab01394bd35c8eb631634ca80f92ef47`
- SDK library version: API2 1.1.6
- Included in Git: `Python/Robotic_Arm` Python modules.
- Host-provided artifact: Linux ARM64 `libapi_c.so`, mounted read-only from
  `/opt/realman/rm_api2/libs/linux_arm/` with SHA-256
  `5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9`.
- Excluded: demos, Windows, x86, debug, C++, images, data, and motion examples

The Python-only vendored subset is about 590 KB; the 1.5 MB shared library is
neither stored in Git nor redistributed in the image. It must be provisioned
on each robot host from the operator's licensed SDK copy.
