ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG ROS_APT_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-colcon-common-extensions \
        python3-pip \
        build-essential \
        cmake && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/t800-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/t800-requirements.txt

COPY msgs/ /t800_ws/src/
RUN . /etc/os-release && \
    echo "[t800] installing ROS interface build dependencies" && \
    find /t800_ws/src -name '._*' -delete && \
    echo "deb [arch=$(dpkg --print-architecture) trusted=yes] ${ROS_APT_MIRROR} ${UBUNTU_CODENAME} main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated --reinstall \
        ros-humble-ament-cmake \
        ros-humble-builtin-interfaces \
        ros-humble-geometry-msgs \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime \
        ros-humble-rosidl-generator-c \
        ros-humble-std-msgs && \
    ldconfig && \
    . /opt/ros/humble/setup.sh && \
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
    python3 -c "from interface_protocol.msg import BodyVelCmd, Heartbeat, LinkInfo, MotionStateRequest; print('interface_protocol import ok')" && \
    rm -rf /var/lib/apt/lists/*

COPY main.py device.py control.py native_sdk.py virtual_gamepad.py config.yaml driver.yaml /work/
COPY resource/ /work/resource/
COPY deploy/ /deploy/

ENV T800_PREFERRED_RMW=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15708

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /t800_ws/install/setup.bash && if [ -n \"${RMW_IMPLEMENTATION:-}\" ] && ! ros2 pkg prefix \"${RMW_IMPLEMENTATION}\" >/dev/null 2>&1; then echo \"[t800] ignoring unavailable RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}\"; unset RMW_IMPLEMENTATION; fi && if [ -n \"${T800_PREFERRED_RMW:-}\" ] && ros2 pkg prefix \"${T800_PREFERRED_RMW}\" >/dev/null 2>&1; then export RMW_IMPLEMENTATION=\"${T800_PREFERRED_RMW}\"; fi && export CYCLONEDDS_URI=\"<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='${NETWORK_INTERFACE:-eth0}'/></Interfaces></General></Domain></CycloneDDS>\" && python3 /work/main.py"]
