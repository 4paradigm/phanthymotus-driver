ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
ARG UBUNTU_MIRROR=http://ports.ubuntu.com/ubuntu-ports
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG UBUNTU_MIRROR
ARG PYPI_MIRROR

# The robot runs CycloneDDS, Agent Core runs FastDDS: both RMWs must be present, and the
# standard message packages this driver subscribes to. Only what is missing is installed.
RUN set -eu; missing=""; \
    for package in ca-certificates python3-pip \
        ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-fastrtps-cpp \
        ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-diagnostic-msgs; do \
        if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
            missing="$missing $package"; \
        fi; \
    done; \
    if [ -n "$missing" ]; then \
        find /etc/apt \( -name '*.list' -o -name '*.sources' \) -type f -exec sed -i \
            -e "s|http://mirrors.tencentyun.com/ubuntu-ports|${UBUNTU_MIRROR}|g" \
            -e "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_MIRROR}|g" {} + && \
        apt-get update && apt-get install -y --no-install-recommends $missing && rm -rf /var/lib/apt/lists/*; \
    fi

COPY requirements.txt /tmp/panther-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/panther-requirements.txt && rm /tmp/panther-requirements.txt

WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py panther_payloads.py panther_bridge.py config.yaml driver.yaml /work/unixai/panther/
COPY resource/panther.urdf /work/unixai/panther/resource/panther.urdf
COPY deploy/ /deploy/

# The main process reads the robot on CycloneDDS; panther_bridge starts its child processes on
# FastDDS (domain 42, loopback profile) after overriding these in the child.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15747
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/unixai/panther/main.py"]
