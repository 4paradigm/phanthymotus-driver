ARG BASE_IMAGE=franka-fr3:local
FROM ${BASE_IMAGE}
ARG SOURCE_REVISION=unknown
LABEL org.opencontainers.image.revision=${SOURCE_REVISION}
COPY common/ /work/common/
COPY *.py *.yaml /work/franka/fr3/
COPY resource/ /work/franka/fr3/resource/
COPY deploy/pnp-remote/ /deploy/pnp-remote/
ENV CONFIG_PATH=/deploy/pnp-remote/config-pnp-remote.yaml FRANKA_TELEOP_BACKEND=pnp_remote
RUN command -v ssh
