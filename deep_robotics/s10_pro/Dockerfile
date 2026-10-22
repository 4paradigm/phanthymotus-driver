# Vendor DDS and platform DDS run in separate processes, both on Jazzy/Python3.12.
ARG ROS_BASE_IMAGE=ros:jazzy-ros-base
FROM ${ROS_BASE_IMAGE}
ARG DEEP_ROBOTICS_MSG_REV=a0d1a29eec5c4db5a9107595bb51e3be8122b86c
ARG DEEP_ROBOTICS_MSG_SHA256=1d268a76e80af8ea5aa3dc28de0c236de87bce55a5d397fdad1e23515f02a537
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-yaml python3-colcon-common-extensions python3-pytest \
    ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*
COPY vendor/deep-robotics-msg-${DEEP_ROBOTICS_MSG_REV}.zip /tmp/drdds.zip
RUN echo "${DEEP_ROBOTICS_MSG_SHA256}  /tmp/drdds.zip" | sha256sum -c - && \
    mkdir -p /s10_ws/src && python3 -m zipfile -e /tmp/drdds.zip /s10_ws/src && \
    . /opt/ros/jazzy/setup.sh && cd /s10_ws && \
    colcon build --packages-select drdds --cmake-args -DBUILD_TESTING=OFF && \
    rm -rf /tmp/drdds.zip /s10_ws/build /s10_ws/log
WORKDIR /work
COPY common/ /work/common/
COPY main.py bridge.py device.py reader.py pointcloud.py preflight.py motion.py motion_transport.py motion_dds.py motion_backend.py motion_feedback.py motion_process.py motion_worker.py motion_preflight.py config.yaml driver.yaml requirements.txt /work/deep_robotics/s10_pro/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 RCUTILS_COLORIZED_OUTPUT=0
EXPOSE 15745
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /s10_ws/install/setup.bash && exec python3 /work/deep_robotics/s10_pro/main.py"]
