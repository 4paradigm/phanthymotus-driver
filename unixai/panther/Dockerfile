ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu

FROM ${ROS_BASE_IMAGE} AS interface-builder
ARG ROS_APT_MIRROR
RUN . /etc/os-release && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] ${ROS_APT_MIRROR} ${UBUNTU_CODENAME} main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions cmake build-essential ros-humble-sensor-msgs && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /panther_ws
COPY vendor/unix_msgs/ /panther_ws/src/unix_msgs/
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select unix_msgs --merge-install

FROM ${ROS_BASE_IMAGE}
ARG ROS_APT_MIRROR
RUN . /etc/os-release && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] ${ROS_APT_MIRROR} ${UBUNTU_CODENAME} main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-yaml \
      python3-numpy \
      python3-opencv \
      systemd \
      util-linux \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-control-msgs \
      ros-humble-controller-manager-msgs \
      ros-humble-diagnostic-msgs \
      ros-humble-geometry-msgs \
      ros-humble-nav-msgs \
      ros-humble-sensor-msgs \
      ros-humble-std-msgs \
      ros-humble-trajectory-msgs && \
    rm -rf /var/lib/apt/lists/*

# Runtime dependencies deliberately come from Ubuntu/ROS apt packages. This
# avoids duplicate pip NumPy/OpenCV wheels and keeps their ABI aligned with ROS.
# nav-msgs is required for the /odom Odometry subscription; removing the
# navigation card did not remove that message dependency. util-linux supplies
# nsenter; systemd supplies the systemctl/systemd-run clients used to manage the
# owned transient camera unit after entering the host namespaces.

COPY --from=interface-builder /panther_ws/install /panther_ws/install
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py plugins.py contracts.py core_bridge.py camera.py config.yaml driver.yaml /work/
COPY resource/panther.urdf /work/resource/panther.urdf
COPY entrypoint.sh /work/entrypoint.sh
COPY deploy/ /deploy/
RUN chmod 0755 /work/entrypoint.sh

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1
ENV RCUTILS_COLORIZED_OUTPUT=0
EXPOSE 15792
CMD ["/work/entrypoint.sh"]

