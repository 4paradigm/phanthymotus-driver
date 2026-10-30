# Build inputs and image-size evidence

## ROS base pin

Both build stages default to the same immutable multi-platform image:

`ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617`

Source: Docker Hub's official `library/ros` tag metadata, checked 2026-10-30:
https://hub.docker.com/v2/repositories/library/ros/tags/jazzy-ros-base

The tag metadata reports last update 2026-10-27 and ARM64/v8 manifest
`sha256:1db94fb5490d0a9c5485e27624c9cf26206c6ff4b26136291628ce8de14f7a6c`.
The index pin retains support for the architectures provided by upstream;
production S10 builds must target linux/arm64. A deliberate ROS_BASE_IMAGE
override is a new build input and must be recorded and tested separately.
The pin prevents tag drift, but apt package resolution is not snapshot-pinned,
so it does not promise bit-for-bit identical rebuilds.

This local edit has not been rebuilt or deployed. Registry manifest access was
unavailable, so equality between the newly pinned base and the previously
built image's base layers has not been established. Rebuild and validate the
ARM64 image before replacing the current deployment.

## Why the infrastructure files are needed

- Dockerfile: generate the S10-specific drdds messages in a separate stage.
  Colcon and ROS interface generators stay in that stage. The runtime installs
  YAML parsing, Fast DDS RMW and generated-interface support; no pytest or
  duplicate pip packages are installed.
- deploy/service.yml: preserve host networking, domain 42, separate vendor
  DDS profile selection, read-only configuration, persistent driver task data
  and bounded local logs. This fragment does not define or modify Core.
- driver.yaml: supply driver identity, fixed MCP port, the shared common/
  build context and the current public tool catalogue. Internal reader sources
  are not advertised as extra cards. The model tool is a resource.
- requirements.txt: document the image-provided dependencies; it contains no
  additional pip requirements.

## Size evidence and acceptance boundary

The existing deployed build `release.261030.0d84ce6`, associated by the build bot
with PR commit 67603552, has image/config ID
`sha256:ae62dff403ff8120801b1956343b741fdf9375d4a281e579c442321a03017dd3`.
Its registry manifest is
`sha256:ee364f1a353634b5424830c633f5a916c4f9c58d94fc965791ac80f99264ac61`.
The downloaded registry layers total 287,716,875 compressed bytes. This is
neither an incremental pull size nor Docker's unpacked image size.
The offline transfer archive size is not used for image-size comparisons.

No same-method before/after size measurement exists yet for this local edit.
After rebuilding, record both image IDs and docker image inspect Size values
for the current and candidate ARM64 images, then report the exact byte delta.
Do not claim a no-growth result from shared disk usage or compressed-vs-unpacked
comparisons. The merged-card revision adds loco_move.py and diagnostic documentation to the
COPY payload, with no new runtime package. This does not establish an unchanged
image size; record the actual before/after sizes after a complete rebuild.
