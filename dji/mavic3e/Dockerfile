# ── DJI Mavic 3E Driver Image ───────────────────────────────────────
# FROM psdk-base (pre-built PSDK library), adds:
#   - psdk_bridge C binary (compiled with PSDK_ENABLED)
#   - Python MCP server + plugins
#
# Build:
#   ./build.sh dji/mavic3e
# ────────────────────────────────────────────────────────────────────

ARG PSDK_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/psdk-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${PSDK_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

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
             -DCMAKE_C_FLAGS="-DPSDK_ENABLED -DPSDK_ARCH_SYS_LINUX=1 -DCONFIG_HARDWARE_CONNECTION=DJI_USE_UART_AND_NETWORK_DEVICE" && \
    make -j$(nproc) && \
    cp psdk_bridge /usr/local/bin/psdk_bridge && \
    cp startup_bulk /usr/local/bin/startup_bulk

# ── Application code (changes frequently) ──────────────────────────
COPY main.py         /work/main.py
COPY device.py       /work/device.py
COPY bridge_client.py /work/bridge_client.py
COPY config.yaml     /work/config.yaml
COPY resource/       /work/resource/
COPY deploy/         /deploy/

# ── udev rules for E-Port device auto-permission ───────────────────
COPY deploy/99-eport.rules /etc/udev/rules.d/99-eport.rules

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1

EXPOSE 15702

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
