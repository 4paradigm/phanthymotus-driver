# Vendor DDS and platform DDS run in separate processes, both on Jazzy/Python3.12.
# Immutable multi-platform ROS Jazzy base; provenance and size notes are in
# README.md. Refresh only with a reviewed image rebuild.
ARG ROS_BASE_IMAGE=ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617
FROM ${ROS_BASE_IMAGE} AS interfaces
ARG DEEP_ROBOTICS_MSG_REV=a0d1a29eec5c4db5a9107595bb51e3be8122b86c
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions ros-jazzy-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*
COPY vendor/deep-robotics-msg-${DEEP_ROBOTICS_MSG_REV}/ /s10_ws/src/drdds/
RUN . /opt/ros/jazzy/setup.sh && cd /s10_ws && \
    colcon build --packages-select drdds --cmake-args -DBUILD_TESTING=OFF && \
    rm -rf /s10_ws/build /s10_ws/log
FROM ${ROS_BASE_IMAGE}
# Runtime only: YAML config parser, Fast DDS RMW and generated-interface runtime.
# Colcon, code generators and pytest stay out of the final image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-yaml ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*
COPY --from=interfaces /s10_ws/install/ /s10_ws/install/
WORKDIR /work
COPY common/ /work/common/
COPY deploy/ /deploy/
COPY main.py registration.py bridge.py device.py reader.py pointcloud.py s10_skeleton.py s10_skeleton.urdf parameters.py config.yaml driver.yaml requirements.txt /work/deep_robotics/s10_pro/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 RCUTILS_COLORIZED_OUTPUT=0
EXPOSE 15745
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /s10_ws/install/setup.bash && exec python3 /work/deep_robotics/s10_pro/main.py"]
