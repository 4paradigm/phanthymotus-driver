ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE} AS audio_msgs_builder
COPY audio_msgs/ /bumi_ros_ws/src/audio_msgs/
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && cd /bumi_ros_ws && colcon build --packages-select audio_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash && python3 -c "from audio_msgs.msg import AudioChunk, SoundDirection"'

FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG UBUNTU_MIRROR=https://ports.ubuntu.com/ubuntu-ports
# Keep only the generated interface; source and build artifacts stay in the builder.
COPY --from=audio_msgs_builder /bumi_ros_ws/install/ /bumi_ros_ws/install/
RUN /bin/bash -c 'source /ros_ws/install/setup.bash && source /bumi_ros_ws/install/setup.bash && python3 -c "from audio_msgs.msg import AudioChunk, SoundDirection"'

WORKDIR /work

# The shared ROS base image points apt at Tencent's private mirror, which is
# unavailable on ordinary networks and on the Bumi host. Replace it with the
# official Ubuntu HTTPS archive before the first apt invocation; packages remain signed
# and subject to apt's normal verification.
RUN find /etc/apt \( -name '*.list' -o -name '*.sources' \) \
        -exec sed -i "s|http://mirrors.tencentyun.com/ubuntu-ports|${UBUNTU_MIRROR}|g" {} + && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-dev \
        ffmpeg \
        cmake \
        build-essential \
        curl \
        git \
        libusb-1.0-0 \
        libportaudio2 \
        libasound2-dev \
        libssl-dev \
        libeigen3-dev \
        libboost-dev && \
    rm -rf /var/lib/apt/lists/*

# Realsense SDK (D435i camera)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        librealsense2-dev \
        librealsense2-utils || true && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml netifaces numpy opencv-python-headless pyrealsense2 pybind11

# Noetix Bumi SDK — copy source and download pre-built libs from COS
COPY noetix_sdk_bumi/ /work/noetix_sdk_bumi/

# Download pre-built libs (libdriver.a, DDS .so, etc.) from COS
ARG SDK_LIBS_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/noetix/noetix_sdk_bumi_libs.tar.gz
RUN curl -fsSL ${SDK_LIBS_URL} | tar xz -C /work/noetix_sdk_bumi/

# Patch CMakeLists: remove onnxruntime dep and executables (we only need pybind .so)
RUN cd /work/noetix_sdk_bumi && \
    sed -i 's/find_package(onnxruntime REQUIRED)/#find_package(onnxruntime REQUIRED)/' CMakeLists.txt && \
    sed -i '/add_executable/,/^)/s/^/#/' CMakeLists.txt && \
    sed -i '/target_include_directories(example/,/^)/s/^/#/' CMakeLists.txt && \
    sed -i '/target_link_libraries(example/,/^)/s/^/#/' CMakeLists.txt

# Build the pybind modules used by the current cards
RUN cd /work/noetix_sdk_bumi && \
    mkdir -p build && cd build && \
    cmake .. \
        -DCMAKE_CXX_STANDARD=17 \
        -Dpybind11_DIR=$(python3 -c "import pybind11; print(pybind11.get_cmake_dir())") && \
    make -j$(nproc) highcontrol_py mediacontrol_py && \
    echo "[build] pybind modules built successfully"

# Verify .so files exist
RUN ls /work/noetix_sdk_bumi/build/highcontrol_py*.so && \
    ls /work/noetix_sdk_bumi/build/mediacontrol_py*.so

# ROS2 domain isolation
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY resource/ /work/resource/
COPY common/ /work/common/
COPY main.py   /work/main.py
COPY device.py /work/device.py
# Navi support modules are independent from device.py for ROS-free tests.
COPY loco_servo.py /work/loco_servo.py
COPY imu_distance_control.py /work/imu_distance_control.py
COPY leg_odometry.py /work/leg_odometry.py
COPY odometry_inputs.py /work/odometry_inputs.py
COPY odometry_feedback.py /work/odometry_feedback.py
COPY camera_runtime.py /work/camera_runtime.py
COPY camera_specs.py /work/camera_specs.py
COPY odom_spec.py /work/odom_spec.py
COPY sound_direction.py /work/sound_direction.py
COPY direction_card.py /work/direction_card.py
COPY config.yaml /work/config.yaml
COPY deploy/     /deploy/

# Check direct capture imports without opening USB or starting ROS nodes.
# No extra ROS camera, RTAB-Map or EKF packages are needed.
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -c "import camera_runtime, imu_distance_control, leg_odometry, odometry_inputs, odometry_feedback, cv2, numpy, pyrealsense2, yaml; from sensor_msgs.msg import CompressedImage, Imu"'

# Add SDK DDS libs + pybind modules to paths
ENV LD_LIBRARY_PATH=/work/noetix_sdk_bumi/dds/aarch64/lib:${LD_LIBRARY_PATH}
ENV PYTHONPATH=/work:/work/noetix_sdk_bumi/build
ENV PYTHONUNBUFFERED=1
# rclpy logs bypass the Python stdout guard; keep container logs plain text.
ENV RCUTILS_COLORIZED_OUTPUT=0

# Vendor HighController/MediaController use CycloneDDS to discover the motion
# board (192.168.55.102). This driver's ROS 2 bridge to Agent Core uses
# rmw_fastrtps_cpp and the mounted FASTRTPS_DEFAULT_PROFILES_FILE instead.
ENV CYCLONEDDS_URI='file:///work/noetix_sdk_bumi/config/dds.xml'

EXPOSE 15704

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
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /bumi_ros_ws/install/setup.bash && python3 /work/main.py"]
