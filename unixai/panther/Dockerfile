ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
ARG UBUNTU_APT_MIRROR=http://ports.ubuntu.com/ubuntu-ports
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG ROS_APT_MIRROR
ARG UBUNTU_APT_MIRROR

# The robot runs CycloneDDS, Agent Core runs FastDDS: both RMWs must be present, with the
# standard message packages this driver subscribes to. This ros-base revision keeps the ROS
# installation and its signing key but not the ROS 2 apt source, so a plain `apt-get install
# ros-humble-*` fails with "no installation candidate". Restore the signed source first (the
# same step engineai/t800 needs), and install only what is missing.
RUN test -s /usr/share/keyrings/ros2-archive-keyring.gpg || \
        { echo "ERROR: base image is missing /usr/share/keyrings/ros2-archive-keyring.gpg"; exit 1; } && \
    . /etc/os-release && \
    printf 'deb %s %s main restricted universe multiverse\ndeb %s %s-updates main restricted universe multiverse\ndeb %s %s-backports main restricted universe multiverse\ndeb %s %s-security main restricted universe multiverse\n' \
        "${UBUNTU_APT_MIRROR}" "${UBUNTU_CODENAME}" \
        "${UBUNTU_APT_MIRROR}" "${UBUNTU_CODENAME}" \
        "${UBUNTU_APT_MIRROR}" "${UBUNTU_CODENAME}" \
        "${UBUNTU_APT_MIRROR}" "${UBUNTU_CODENAME}" \
        > /etc/apt/sources.list && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] ${ROS_APT_MIRROR} ${UBUNTU_CODENAME} main" \
        > /etc/apt/sources.list.d/ros2.list && \
    missing="" && \
    for package in ca-certificates python3-pip \
        ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-fastrtps-cpp \
        ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-diagnostic-msgs; do \
        if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
            missing="$missing $package"; \
        fi; \
    done && \
    if [ -n "$missing" ]; then \
        apt-get update && apt-get install -y --no-install-recommends $missing && rm -rf /var/lib/apt/lists/*; \
    fi

COPY requirements.txt /tmp/panther-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/panther-requirements.txt && rm /tmp/panther-requirements.txt

WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py panther_payloads.py panther_bridge.py config.yaml driver.yaml /work/unixai/panther/
COPY resource/panther.urdf /work/unixai/panther/resource/panther.urdf
COPY deploy/ /deploy/

# Fail the build here, not on the robot, if the pieces the driver needs cannot be imported.
RUN . /opt/ros/humble/setup.sh && \
    ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null && \
    ros2 pkg prefix rmw_fastrtps_cpp >/dev/null && \
    python3 -c "import numpy, cv2, yaml; from sensor_msgs.msg import BatteryState, Image, Imu, JointState, LaserScan, CompressedImage; from nav_msgs.msg import Odometry; from diagnostic_msgs.msg import DiagnosticArray; print('panther imports ok', numpy.__version__, cv2.__version__)"

# The main process reads the robot on CycloneDDS; panther_bridge starts its child processes on
# FastDDS (domain 42, loopback profile) after overriding these in the child.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15747
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/unixai/panther/main.py"]
