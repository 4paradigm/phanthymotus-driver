ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
ARG UBUNTU_APT_MIRROR=http://ports.ubuntu.com/ubuntu-ports
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG ROS_APT_MIRROR
ARG UBUNTU_APT_MIRROR

WORKDIR /work

# Some ros-base revisions retain the ROS installation and signing key but not
# the ROS 2 apt source. Restore the signed source before installing the RMW.
RUN test -s /usr/share/keyrings/ros2-archive-keyring.gpg || \
        { echo "ERROR: base image is missing /usr/share/keyrings/ros2-archive-keyring.gpg"; exit 1; } && \
    . /etc/os-release && \
    printf 'deb %s %s main restricted universe multiverse\ndeb %s %s-updates main restricted universe multiverse\ndeb %s %s-backports main restricted universe multiverse\ndeb %s %s-security main restricted universe multiverse\n' \
        "${UBUNTU_APT_MIRROR}" "${UBUNTU_CODENAME}" \
        "${UBUNTU_APT_MIRROR}" "${UBUNTU_CODENAME}" \
        "${UBUNTU_APT_MIRROR}" "${UBUNTU_CODENAME}" \
        "${UBUNTU_APT_MIRROR}" "${UBUNTU_CODENAME}" \
        > /etc/apt/sources.list && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] ${ROS_APT_MIRROR} $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-nav-msgs \
        ros-humble-sensor-msgs \
        pulseaudio-utils \
        build-essential \
        cmake && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/t800-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/t800-requirements.txt

# The base image's audio workspace intentionally narrows CMAKE_PREFIX_PATH.
# Restore the ROS underlay after sourcing it so colcon can discover both trees.
COPY msgs/ /t800_ws/src/
RUN echo "[t800] checking ROS interface build dependencies" && \
    find /t800_ws/src -name '._*' -delete && \
    ldconfig && \
    . /opt/ros/humble/setup.sh && \
    . /ros_ws/install/setup.sh && \
    export CMAKE_PREFIX_PATH="/ros_ws/install/audio_msgs:/opt/ros/humble" && \
    echo "[t800] ROS interface dependency diagnostics" && \
    dpkg-query -W -f='[t800] ${Package} ${Version}\n' \
        ros-humble-ament-cmake \
        ros-humble-builtin-interfaces \
        ros-humble-geometry-msgs \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime \
        ros-humble-rosidl-generator-c \
        ros-humble-std-msgs && \
    test -f /opt/ros/humble/share/builtin_interfaces/cmake/ament_cmake_export_libraries-extras.cmake && \
    test -e /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so && \
    printf '%s\n' \
        'find_library(BUILTIN_GENERATOR_C builtin_interfaces__rosidl_generator_c PATHS /opt/ros/humble/lib NO_DEFAULT_PATH)' \
        'message(STATUS "BUILTIN_GENERATOR_C=${BUILTIN_GENERATOR_C}")' \
        'if(NOT BUILTIN_GENERATOR_C)' \
        '  message(FATAL_ERROR "builtin_interfaces__rosidl_generator_c not found by CMake")' \
        'endif()' > /tmp/t800_check_builtin.cmake && \
    cmake -P /tmp/t800_check_builtin.cmake && \
    cd /t800_ws && \
    colcon build --packages-select interface_protocol && \
    . /t800_ws/install/setup.sh && \
    python3 -c "from interface_protocol.msg import BodyVelCmd, Heartbeat, LinkInfo, MotionStateRequest; from nav_msgs.msg import Odometry; from sensor_msgs.msg import CompressedImage, Image, PointCloud2; print('T800 ROS imports ok')"

COPY common/ /work/common/
COPY main.py device.py control.py native_sdk.py virtual_gamepad.py config.yaml driver.yaml controlled_spatial_map.py /work/
COPY resource/ /work/resource/
COPY deploy/ /deploy/
RUN . /opt/ros/humble/setup.sh && \
    . /ros_ws/install/setup.sh && \
    . /t800_ws/install/setup.sh && \
    command -v pactl >/dev/null && \
    command -v parec >/dev/null && \
    ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null && \
    python3 -c "import main; from audio_msgs.msg import AudioChunk; import lcm, numpy, yaml; print('T800 runtime imports ok')"

ENV T800_PREFERRED_RMW=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15708

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /t800_ws/install/setup.bash && if [ -n \"${RMW_IMPLEMENTATION:-}\" ] && ! ros2 pkg prefix \"${RMW_IMPLEMENTATION}\" >/dev/null 2>&1; then echo \"[t800] ignoring unavailable RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}\"; unset RMW_IMPLEMENTATION; fi && if [ -n \"${T800_PREFERRED_RMW:-}\" ] && ros2 pkg prefix \"${T800_PREFERRED_RMW}\" >/dev/null 2>&1; then export RMW_IMPLEMENTATION=\"${T800_PREFERRED_RMW}\"; fi && if [ -z \"${CYCLONEDDS_URI:-}\" ]; then export CYCLONEDDS_URI=\"<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='${NETWORK_INTERFACE:-eth1}'/></Interfaces></General></Domain></CycloneDDS>\"; fi && exec python3 /work/main.py"]
