ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://mirrors.cloud.tencent.com/pypi/simple/
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu

# Build the exact Livox SDK/ROS overlay used by Adam's directly attached
# MID-360. Pin commits instead of following mutable master branches.
FROM ${ROS_BASE_IMAGE} AS livox-builder
ARG LIVOX_SDK2_REV=c0796f04c143143899c87a773d9f6b7136453c0b
ARG LIVOX_ROS_DRIVER2_REV=21445540f0d100dc86a7e6df312dd70bbdb4afdf
ARG ROS_APT_MIRROR
RUN sed -i 's|http://mirrors.tencentyun.com/ubuntu-ports|http://ports.ubuntu.com/ubuntu-ports|g' /etc/apt/sources.list && \
    test -s /usr/share/keyrings/ros2-archive-keyring.gpg && \
    . /etc/os-release && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] ${ROS_APT_MIRROR} ${UBUNTU_CODENAME} main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential python3-colcon-common-extensions \
    libapr1-dev libpcl-dev \
    ros-humble-ament-cmake-auto ros-humble-pcl-conversions \
    ros-humble-rclcpp-components ros-humble-rosidl-default-generators && \
    rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && git checkout --detach ${LIVOX_SDK2_REV} && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel 2 && \
    cmake --install build && \
    ldconfig && \
    mkdir -p /livox-sdk-runtime/lib && \
    cp -a /usr/local/lib/liblivox_lidar_sdk_* /livox-sdk-runtime/lib/ && \
    rm -rf /tmp/Livox-SDK2
RUN mkdir -p /livox_ws/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git /livox_ws/src/livox_ros_driver2 && \
    cd /livox_ws/src/livox_ros_driver2 && \
    git checkout --detach ${LIVOX_ROS_DRIVER2_REV} && \
    cp package_ROS2.xml package.xml && cp -a launch_ROS2 launch && \
    . /opt/ros/humble/setup.sh && \
    cd /livox_ws && \
    colcon build --packages-select livox_ros_driver2 --cmake-args \
        -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=OFF && \
    rm -rf /livox_ws/build /livox_ws/log /livox_ws/src

FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG ROS_APT_MIRROR

WORKDIR /work

COPY --from=livox-builder /livox-sdk-runtime/lib/ /usr/local/lib/
COPY --from=livox-builder /livox_ws/install/ /livox_ws/install/
RUN ldconfig

RUN sed -i 's|http://mirrors.tencentyun.com/ubuntu-ports|http://ports.ubuntu.com/ubuntu-ports|g' /etc/apt/sources.list && \
    test -s /usr/share/keyrings/ros2-archive-keyring.gpg && \
    . /etc/os-release && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] ${ROS_APT_MIRROR} ${UBUNTU_CODENAME} main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-pip cmake build-essential curl git ffmpeg \
    libturbojpeg libusb-1.0-0 libegl1 libgles2 libv4l-0 \
    libx11-6 libxext6 libbsd0 python3-pil \
    libapr1-dev libpcl-dev \
    ros-humble-pcl-conversions ros-humble-rclcpp-components && \
    rm -rf /var/lib/apt/lists/*

# Resolve the installed package and fail the image build if a native Livox/PCL
# dependency was omitted from the runtime stage.
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    source /livox_ws/install/setup.bash && \
    driver="$(ros2 pkg prefix livox_ros_driver2)/lib/livox_ros_driver2/livox_ros_driver2_node" && \
    test -x "$driver" && \
    if ldd "$driver" | grep -q "not found"; then ldd "$driver"; exit 1; fi'

# Python dependencies. Keep the deployed runtime identical to local installs;
# generated protobuf/gRPC modules enforce the minimum versions listed here.
COPY requirements.txt /tmp/adam-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    -r /tmp/adam-requirements.txt && \
    rm -f /tmp/adam-requirements.txt

# CycloneDDS 0.10.2 (required by pnd_sdk_python)
ARG CYCLONEDDS_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/cyclonedds-0.10.5.tar.gz
RUN curl -fsSL ${CYCLONEDDS_URL} | tar xz -C /tmp && \
    mkdir -p /tmp/cyclonedds-0.10.5/build && \
    cd /tmp/cyclonedds-0.10.5/build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds && \
    cmake --build . --config RelWithDebInfo --parallel --target install && \
    rm -rf /tmp/cyclonedds-0.10.5

ENV CYCLONEDDS_HOME=/opt/cyclonedds
ENV LD_LIBRARY_PATH=/usr/local/zed/lib:/opt/cyclonedds/lib:${LD_LIBRARY_PATH}

RUN pip3 install --no-cache-dir --no-binary cyclonedds -i ${PYPI_MIRROR} cyclonedds==0.10.2

# pnd_sdk_python (Adam DDS SDK)
COPY pnd_sdk_python/ /tmp/pnd_sdk_python/
RUN pip3 install --no-cache-dir --no-deps /tmp/pnd_sdk_python && \
    rm -rf /tmp/pnd_sdk_python

# Application files
COPY proto/         /work/proto/
COPY resource/      /work/resource/
COPY common/ /work/common/
COPY main.py        /work/main.py
COPY device.py      /work/device.py
COPY lidar.py       /work/lidar.py
COPY estop.py       /work/estop.py
COPY battery_status.py /work/battery_status.py
COPY grpc_client.py /work/grpc_client.py
COPY config.yaml    /work/config.yaml
COPY deploy/        /deploy/

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONPATH=/work:/opt/pyzed
ENV PYTHONUNBUFFERED=1
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'
ENV ZED_PYTHON_PATH=/opt/pyzed

EXPOSE 15722

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /livox_ws/install/setup.bash && exec python3 /work/main.py"]
