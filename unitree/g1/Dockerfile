ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl \
        git \
        unzip \
        libusb-1.0-0 && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml netifaces opencv-python-headless numpy pyrealsense2

# Build CycloneDDS 0.10.5 from source
ARG CYCLONEDDS_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/cyclonedds-0.10.5.tar.gz
RUN curl -fsSL ${CYCLONEDDS_URL} \
    | tar xz -C /tmp && \
    mkdir -p /tmp/cyclonedds-0.10.5/build && \
    cd /tmp/cyclonedds-0.10.5/build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds && \
    cmake --build . --config RelWithDebInfo --parallel --target install && \
    rm -rf /tmp/cyclonedds-0.10.5

ENV CYCLONEDDS_HOME=/opt/cyclonedds
ENV LD_LIBRARY_PATH=/opt/cyclonedds/lib:${LD_LIBRARY_PATH}

RUN pip3 install --no-cache-dir --no-binary cyclonedds -i ${PYPI_MIRROR} cyclonedds==0.10.5


# Build FAST-LIO2 (LiDAR-Inertial Odometry)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libeigen3-dev \
        libpcl-dev \
        ros-humble-tf2-ros \
        ros-humble-tf2-eigen && \
    rm -rf /var/lib/apt/lists/*

ARG FAST_LIO_URL=https://github.com/hku-mars/FAST_LIO.git
ARG FAST_LIO_BRANCH=main
RUN bash -c "source /opt/ros/humble/setup.bash && \
    mkdir -p /tmp/fastlio_ws/src && \
    cd /tmp/fastlio_ws/src && \
    git clone --depth 1 -b ${FAST_LIO_BRANCH} ${FAST_LIO_URL} fast_lio && \
    cd /tmp/fastlio_ws && \
    colcon build --packages-select fast_lio \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    cp -r /tmp/fastlio_ws/install/fast_lio /opt/ros/humble/ && \
    rm -rf /tmp/fastlio_ws"

# Isolate rclpy topics (domain 42) from unitree_sdk2py's domain 0.
# Force FastDDS to match agent-core (which has no CycloneDDS rmw installed).
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Copy source files last (most frequently changed → least cache invalidation)
COPY unitree_sdk2py/ /work/unitree_sdk2py/
COPY resource/       /work/resource/
COPY main.py   /work/main.py
COPY device.py /work/device.py
COPY scan_context.py /work/scan_context.py
COPY config.yaml /work/config.yaml
COPY fastlio_config/ /work/fastlio_config/

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1

EXPOSE 15701

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-eth0}"]
