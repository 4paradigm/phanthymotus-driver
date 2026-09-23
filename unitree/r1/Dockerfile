ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl \
        git \
        libusb-1.0-0 \
        libportaudio2 \
        libasound2-dev && \
    rm -rf /var/lib/apt/lists/*

# GStreamer runtime (separate layer to avoid QEMU libc-bin segfault)
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
# Sibling of device.py — the camera cards' motus.camera/1 declaration lives here
# rather than in device.py so it can be tested without rclpy. Missing it makes
# every camera tool fail on the first info().
COPY camera_specs.py /work/camera_specs.py
COPY rpc_proxy.py /work/rpc_proxy.py
COPY ext_devices.py /work/ext_devices.py
COPY loco_servo.py /work/loco_servo.py
COPY config.yaml /work/config.yaml
COPY deploy/     /deploy/

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1
# ANSI colour codes are control bytes; in a non-tty container log they are
# exactly the "garbage characters" that make `docker logs` unreadable.
ENV RCUTILS_COLORIZED_OUTPUT=0

# Suppress CycloneDDS tracing to stdout (prevent oversized log messages)
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'

EXPOSE 15702

# The image deliberately does **not** try to set the Jetson power mode or lock
# its clocks. It used to: the CMD began with
#
#   if command -v nvpmodel >/dev/null 2>&1; then nvpmodel -m 0; jetson_clocks; fi
#
# and on r1_sz that branch had never once been taken, because there is no
# `nvpmodel` inside the container — it is part of the host's L4T tooling, and
# both commands write host-wide state that a container has no business owning
# anyway. The line read as "this is handled" while the GPU sat at 612 MHz of its
# 918 MHz ceiling and the memory clock at two thirds of its own.
#
# It belongs to the host installer, which runs as root outside any container:
# `resource-center/public/install.sh`, step 1.55. That one also installs a
# systemd unit, because `jetson_clocks` is not persistent across a reboot while
# `nvpmodel` is — so a rebooted machine and a fresh one would otherwise run at
# different speeds with nothing anywhere saying why.
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-eth10}"]
