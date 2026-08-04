ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PND_ROS2_REV=29d92afbf417b32afc26c9450f27d211abf1b259
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN apt-get update && apt-get install -y --no-install-recommends git python3-pip python3-colcon-common-extensions ros-humble-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} pyyaml
RUN git clone https://github.com/pndbotics/pnd_ros2.git /tmp/pnd_ros2 && \
    git -C /tmp/pnd_ros2 checkout ${PND_ROS2_REV} && \
    mkdir -p /adam_ws/src && cp -r /tmp/pnd_ros2/cyclonedds_ws/src/pnd_adam /adam_ws/src/ && \
    . /opt/ros/humble/setup.sh && cd /adam_ws && colcon build --packages-select pnd_adam && rm -rf /tmp/pnd_ros2
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15712
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /adam_ws/install/setup.bash && python3 /work/main.py"]
