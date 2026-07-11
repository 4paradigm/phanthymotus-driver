ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        cmake \
        build-essential \
        curl \
        git \
        libusb-1.0-0-dev \
        libssl-dev \
        libopus-dev \
        ffmpeg \
        udev \
        usbutils && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml netifaces

# ROS2 environment
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Copy Python driver
COPY main.py         /work/main.py
COPY device.py       /work/device.py
COPY bridge_client.py /work/bridge_client.py
COPY config.yaml     /work/config.yaml
COPY resource/       /work/resource/
COPY deploy/         /deploy/

# Install udev rule for E-Port device auto-permission
COPY deploy/99-eport.rules /etc/udev/rules.d/99-eport.rules

# Copy and build C bridge (stub mode unless PSDK_DIR is provided at build time)
COPY psdk_bridge/ /work/psdk_bridge/
RUN mkdir -p /work/psdk_bridge/build && \
    cd /work/psdk_bridge/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    cp psdk_bridge /usr/local/bin/psdk_bridge

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1

EXPOSE 15702

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
