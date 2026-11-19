ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG UBUNTU_APT_MIRROR=

# ros-base provides pip; only install missing ROS2 runtime packages.
# ROS1 geometry_msgs is encoded as JSON by rosbridge, not imported locally.
RUN set -eu; missing=""; \
    for package in ros-humble-rmw-fastrtps-cpp ros-humble-std-msgs ros-humble-sensor-msgs openssh-client sshpass; do \
        if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then \
            missing="$missing $package"; \
        fi; \
    done; \
    if [ -n "$missing" ]; then \
        if [ -n "${UBUNTU_APT_MIRROR}" ]; then \
            sed -i "s|http://mirrors.tencentyun.com/ubuntu-ports|${UBUNTU_APT_MIRROR}|g" /etc/apt/sources.list; \
        fi; \
        apt-get update && apt-get install -y --no-install-recommends $missing && \
        rm -rf /var/lib/apt/lists/*; \
    fi
COPY requirements.txt /work/requirements.txt
# ROS base revisions may already provide distro PyYAML without pip metadata.
# Reuse an importable copy rather than layering a redundant pip installation.
# Keep all dependency declarations (including websocket-client) in requirements.
RUN set -eu; \
    if python3 -c 'import yaml'; then \
        grep -v '^pyyaml$' /work/requirements.txt > /tmp/kuavo-requirements.txt; \
    else \
        cp /work/requirements.txt /tmp/kuavo-requirements.txt; \
    fi; \
    pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/kuavo-requirements.txt; \
    rm /tmp/kuavo-requirements.txt; \
    python3 -c 'import yaml, websocket'
# The existing ros-base audio_msgs overlay is shared with Agent Core; reuse it.
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 -c "from audio_msgs.msg import AudioChunk"'
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py rosbridge_client.py kuavo_sensors.py peripherals.py speaker.py startup.py bootstrap.py connection_defaults.py controlled_spatial.py spatial_client.py config.yaml driver.yaml /work/
COPY host/ /work/host/
COPY resource/ /work/resource/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42
EXPOSE 15747
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && exec python3 /work/main.py"]
