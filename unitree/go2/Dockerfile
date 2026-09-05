ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG UBUNTU_PORTS_MIRROR=https://mirrors.aliyun.com/ubuntu-ports
ARG PYPI_MIRROR=https://mirrors.aliyun.com/pypi/simple/
FROM ${ROS_BASE_IMAGE}
ARG UBUNTU_PORTS_MIRROR
ARG PYPI_MIRROR

WORKDIR /work

RUN find /etc/apt -type f \( -name '*.list' -o -name '*.sources' \) \
        -exec sed -i \
            -e "s|http://mirrors.tencentyun.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" \
            -e "s|https://mirrors.tencentyun.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" \
            -e "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" \
            -e "s|https://ports.ubuntu.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" \
            {} + && \
    apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl \
        git \
        libusb-1.0-0 && \
    rm -rf /var/lib/apt/lists/*

# GStreamer runtime (for Go2 front camera H264 UDP multicast)
RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-libav || true && \
    ldconfig && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        libasound2-dev \
        libportaudio2 \
        v4l-utils && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml netifaces numpy opencv-python-headless pyalsaaudio sounddevice

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

# Isolate rclpy topics (domain 42) from unitree_sdk2py's domain 0.
# Force FastDDS to match agent-core (which has no CycloneDDS rmw installed).
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT

COPY unitree_sdk2py/ /work/unitree_sdk2py/
COPY resource/ /work/resource/
COPY common/ /work/common/
COPY main.py   /work/main.py
COPY device.py /work/device.py
COPY rpc_proxy.py /work/rpc_proxy.py
COPY controlled_spatial.py /work/controlled_spatial.py
COPY spatial.py /work/spatial.py
COPY scan_context.py /work/scan_context.py
COPY path_planner.py /work/path_planner.py
COPY icp.py /work/icp.py
COPY global_registration.py /work/global_registration.py
COPY ext_devices.py /work/ext_devices.py
COPY lidar.py /work/lidar.py
COPY pointcloud_utils.py /work/pointcloud_utils.py
COPY velocity_proposal.py /work/velocity_proposal.py
COPY proposal_controller.py /work/proposal_controller.py
COPY navigation_pointcloud.py /work/navigation_pointcloud.py
COPY navigation_time.py /work/navigation_time.py
COPY navigation_sensor_bridge.py /work/navigation_sensor_bridge.py
COPY navigation_sensor_bridge_main.py /work/navigation_sensor_bridge_main.py
COPY config.yaml /work/config.yaml
COPY deploy/     /deploy/

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1
# ANSI colour codes are control bytes; in a non-tty container log they are
# exactly the "garbage characters" that make `docker logs` unreadable.
ENV RCUTILS_COLORIZED_OUTPUT=0

# Suppress CycloneDDS tracing to stdout (prevent oversized log messages)
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'

EXPOSE 15703

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-eth0}"]
