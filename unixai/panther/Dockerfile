ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
ARG UBUNTU_APT_MIRROR=http://ports.ubuntu.com/ubuntu-ports

# ---- stage 1: build the one UniX message type the gripper topics use -------------------------
# Build tools stay in this stage; only /panther_ws/install is copied into the runtime image.
FROM ${ROS_BASE_IMAGE} AS interface-builder
ARG ROS_APT_MIRROR
ARG UBUNTU_APT_MIRROR

# This ros-base revision keeps the ROS installation and its signing key but not the ROS 2 apt
# source, so a plain `apt-get install ros-humble-*` fails with "no installation candidate".
# Restore the signed source first (the same step engineai/t800 needs).
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
    apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions cmake build-essential \
        ros-humble-rosidl-default-generators ros-humble-sensor-msgs && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /panther_ws
COPY vendor/unix_msgs/ /panther_ws/src/unix_msgs/
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select unix_msgs --merge-install

# ---- stage 2: runtime ------------------------------------------------------------------------
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG ROS_APT_MIRROR
ARG UBUNTU_APT_MIRROR

# The robot runs CycloneDDS, Agent Core runs FastDDS: both RMWs must be present, with the message
# packages this driver reads and the control interfaces it commands through. Only what the base
# image lacks is installed. util-linux is for `nsenter` (host_camera.py).
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
    for package in ca-certificates python3-pip util-linux \
        ros-humble-rmw-cyclonedds-cpp ros-humble-rmw-fastrtps-cpp \
        ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-diagnostic-msgs \
        ros-humble-geometry-msgs ros-humble-tf2-msgs ros-humble-trajectory-msgs \
        ros-humble-control-msgs ros-humble-controller-manager-msgs; do \
        if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
            missing="$missing $package"; \
        fi; \
    done && \
    if [ -n "$missing" ]; then \
        echo "[panther] installing packages the base image lacks:$missing"; \
        apt-get update && apt-get install -y --no-install-recommends $missing && rm -rf /var/lib/apt/lists/*; \
    else \
        echo "[panther] the base image already has every runtime package; nothing installed"; \
    fi

COPY requirements.txt /tmp/panther-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/panther-requirements.txt && rm /tmp/panther-requirements.txt

COPY --from=interface-builder /panther_ws/install /panther_ws/install

WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py contracts.py host_camera.py motion_switch.py panther_motion.py panther_payloads.py panther_bridge.py config.yaml driver.yaml /work/unixai/panther/
COPY resource/panther.urdf /work/unixai/panther/resource/panther.urdf
COPY deploy/ /deploy/

# Fail the build here, not on the robot, if the pieces the driver needs cannot be imported.
RUN . /opt/ros/humble/setup.sh && . /panther_ws/install/setup.sh && \
    command -v nsenter >/dev/null && \
    ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null && \
    ros2 pkg prefix rmw_fastrtps_cpp >/dev/null && \
    python3 -c "import numpy, cv2, yaml; from sensor_msgs.msg import BatteryState, Image, Imu, JointState, LaserScan, CompressedImage; from nav_msgs.msg import Odometry; from diagnostic_msgs.msg import DiagnosticArray; from tf2_msgs.msg import TFMessage; from geometry_msgs.msg import Twist; from control_msgs.action import FollowJointTrajectory; from controller_manager_msgs.srv import ListControllers, SwitchController; from unix_msgs.msg import GripperCommand; print('panther imports ok', numpy.__version__, cv2.__version__)"

# The main process reads and commands the robot on CycloneDDS; panther_bridge starts its child
# processes on FastDDS (domain 42, loopback profile) after overriding these in the child.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15747
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /panther_ws/install/setup.bash && python3 /work/unixai/panther/main.py"]
