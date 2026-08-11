ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        python3-dev \
        cmake \
        build-essential \
        curl \
        git \
        libusb-1.0-0 \
        libportaudio2 \
        libasound2-dev \
        libssl-dev \
        libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*

# Realsense SDK (D435i camera)
RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        librealsense2-dev \
        librealsense2-utils || true && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml netifaces numpy opencv-python-headless pyrealsense2 pybind11

# Noetix Bumi SDK — copy source and build pybind modules
COPY noetix_sdk_bumi/ /work/noetix_sdk_bumi/

# Build only the pybind modules (highcontrol_py, mediacontrol_py)
# Skip onnxruntime-dependent executables via custom CMakeLists target selection
RUN cd /work/noetix_sdk_bumi && \
    mkdir -p build && cd build && \
    cmake .. \
        -DCMAKE_CXX_STANDARD=17 \
        -Dpybind11_DIR=$(python3 -c "import pybind11; print(pybind11.get_cmake_dir())") \
    2>&1 || true && \
    make -j$(nproc) highcontrol_py mediacontrol_py 2>&1 && \
    echo "[build] pybind modules built successfully"

# Verify .so files exist
RUN ls /work/noetix_sdk_bumi/build/highcontrol_py*.so && \
    ls /work/noetix_sdk_bumi/build/mediacontrol_py*.so

# ROS2 domain isolation
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT

COPY resource/ /work/resource/
COPY main.py   /work/main.py
COPY device.py /work/device.py
COPY config.yaml /work/config.yaml
COPY deploy/     /deploy/

# Add SDK DDS libs + pybind modules to paths
ENV LD_LIBRARY_PATH=/work/noetix_sdk_bumi/dds/aarch64/lib:${LD_LIBRARY_PATH}
ENV PYTHONPATH=/work:/work/noetix_sdk_bumi/build
ENV PYTHONUNBUFFERED=1

# Suppress CycloneDDS tracing
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'

EXPOSE 15704

CMD ["/bin/bash", "-c", "if command -v nvpmodel >/dev/null 2>&1; then nvpmodel -m 0 2>/dev/null; jetson_clocks 2>/dev/null; echo '[init] Jetson MAXN + clocks locked'; fi && source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
