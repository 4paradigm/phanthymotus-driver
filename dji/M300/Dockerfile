
ARG PSDK_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/psdk-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${PSDK_BASE_IMAGE}
ARG PYPI_MIRROR

COPY third_party/psdk_lib-3.8.0-m300.tar.gz /tmp/
RUN echo "5cd52fc65b05baa1826a16d138136eec68a456aea3b0cf4ade78cc3e54f089bd  /tmp/psdk_lib-3.8.0-m300.tar.gz" | sha256sum -c - && \
    rm -rf /opt/Payload-SDK && mkdir -p /opt/Payload-SDK && \
    tar -xzf /tmp/psdk_lib-3.8.0-m300.tar.gz -C /opt/Payload-SDK && \
    rm -f /tmp/psdk_lib-3.8.0-m300.tar.gz

WORKDIR /work

# M300 liveview uses the aircraft's USB vendor interfaces through libusb.
RUN apt-get update && apt-get install -y --no-install-recommends libusb-1.0-0-dev && \
    rm -rf /var/lib/apt/lists/*

# ── Python dependencies ─────────────────────────────────────────────
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml netifaces

# ── ROS2 environment ────────────────────────────────────────────────
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ── Build psdk_bridge (links against pre-built PSDK in base image) ──
COPY psdk_bridge/ /work/psdk_bridge/
RUN mkdir -p /work/psdk_bridge/build && \
    cd /work/psdk_bridge/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DPSDK_DIR=/opt/Payload-SDK \
             -DCMAKE_C_FLAGS="-DPSDK_ENABLED -DPSDK_ARCH_SYS_LINUX=1" && \
    make -j$(nproc) && \
    cp psdk_bridge /usr/local/bin/psdk_bridge

# ── Application code (changes frequently) ──────────────────────────
COPY common/ /work/common/
COPY main.py         /work/main.py
COPY device.py       /work/device.py
COPY bridge_client.py /work/bridge_client.py
COPY config.yaml     /work/config.yaml
COPY resource/       /work/resource/
COPY deploy/         /deploy/

RUN chmod 755 /deploy/entrypoint.sh

# ── udev rules for E-Port device auto-permission ───────────────────
COPY deploy/99-eport.rules /etc/udev/rules.d/99-eport.rules

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1

# Pre-build GStreamer plugin registry (so avdec_h264 etc. are available at runtime)
RUN rm -f /root/.cache/gstreamer-1.0/registry.aarch64.bin && \
    gst-inspect-1.0 avdec_h264 > /dev/null 2>&1 || true

EXPOSE 15702

CMD ["/deploy/entrypoint.sh"]
