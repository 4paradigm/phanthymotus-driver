ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG APT_MIRROR=http://mirrors.tencentyun.com/ubuntu-ports
ARG PYPI_MIRROR=https://pypi.org/simple/

RUN sed -i "s|http://mirrors.tencentyun.com/ubuntu-ports|${APT_MIRROR}|g" /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip python3-yaml ros-humble-rmw-fastrtps-cpp \
    && cd /tmp \
    && apt-get download libusb-1.0-0 \
    && mkdir -p /opt/realman/libusb /tmp/libusb-root \
    && dpkg-deb --extract ./libusb-1.0-0_*_arm64.deb /tmp/libusb-root \
    && cp -a /tmp/libusb-root/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0* /opt/realman/libusb/ \
    && test -e /opt/realman/libusb/libusb-1.0.so.0 \
    && test -z "$(dpkg --audit)" \
    && rm -rf /tmp/libusb-root /tmp/libusb-1.0-0_*_arm64.deb \
    && rm -rf /var/lib/apt/lists/*

# pyrealsense2 dynamically loads libusb. Only that shared object is extracted
# from Ubuntu's signed runtime package above; no target-architecture maintainer
# script or libc trigger is bypassed. RealSense discovery and all three streams
# use the SDK USB backend, so V4L2 utilities and a compiler are unnecessary.
# OpenCV only encodes JPEG frames; it does not need a GUI backend.
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} --only-binary=:all: \
    numpy==1.23.5 opencv-python-headless==4.11.0.86 pyrealsense2==2.56.5.9235

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
    LD_LIBRARY_PATH=/opt/realman/libusb \
    RM_DRIVER_ENABLED=0

EXPOSE 15718
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/realman/rm75_6f_v/main.py"]
