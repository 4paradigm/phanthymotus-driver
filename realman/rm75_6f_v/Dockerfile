ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG APT_MIRROR=http://mirrors.tencentyun.com/ubuntu-ports
ARG PYPI_MIRROR=https://pypi.org/simple/

RUN sed -i "s|http://mirrors.tencentyun.com/ubuntu-ports|${APT_MIRROR}|g" /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip python3-yaml ros-humble-rmw-fastrtps-cpp \
        libusb-1.0-0 v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Keep the same ARM64/Python 3.10 SDK version as the repository's verified
# RealSense implementation. Canvas renders the published frames, so OpenCV
# does not need a local GUI backend.
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} --only-binary=pyrealsense2 \
    numpy opencv-python-headless pyrealsense2==2.56.5.9235

WORKDIR /work
COPY common/ /work/common/
COPY vendor/Robotic_Arm/ /work/Robotic_Arm/
COPY main.py device.py camera.py realsense.py config.yaml driver.yaml gripper_bridge.py /work/realman/rm75_6f_v/
COPY resource/ /work/realman/rm75_6f_v/resource/
COPY deploy/ /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_LOCALHOST_ONLY=0 \
    PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    PYTHONPATH=/work \
    RM_DRIVER_ENABLED=0

EXPOSE 15718
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/realman/rm75_6f_v/main.py"]
