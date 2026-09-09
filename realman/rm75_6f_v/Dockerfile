ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG APT_MIRROR=http://mirrors.tencentyun.com/ubuntu-ports
ARG PYPI_MIRROR=https://pypi.org/simple/

RUN sed -i "s|http://mirrors.tencentyun.com/ubuntu-ports|${APT_MIRROR}|g" /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip python3-yaml ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Some x86 CI builders run this ARM64 image through QEMU versions whose
# ldconfig crashes while dpkg processes the final libc-bin trigger.  The camera
# packages have already been configured at that point.  Tolerate that emulator
# failure only when every required package is actually installed; download or
# package failures must still fail the layer.  Native ARM64 builds take the
# normal successful apt path.
RUN apt-get update \
    && (apt-get install -y --no-install-recommends libusb-1.0-0 v4l-utils \
        || { \
            for package in libusb-1.0-0 libv4l-0 libv4l2rds0 libv4lconvert0 v4l-utils; do \
                test "$(dpkg-query -W -f='${db:Status-Status}' "$package" 2>/dev/null)" = installed \
                    || exit 1; \
            done; \
        }) \
    && rm -rf /var/lib/apt/lists/*

# Keep the same ARM64/Python 3.10 SDK version as the repository's verified
# RealSense implementation. Canvas renders the published frames, so OpenCV
# does not need a local GUI backend.
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} --only-binary=pyrealsense2 \
    numpy opencv-python-headless pyrealsense2==2.56.5.9235

WORKDIR /work
COPY common/ /work/common/
COPY vendor/Robotic_Arm/ /work/Robotic_Arm/
COPY main.py device.py camera.py realsense.py config.yaml driver.yaml /work/realman/rm75_6f_v/
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
