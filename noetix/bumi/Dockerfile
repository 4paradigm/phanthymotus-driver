ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN sed -i "s#mirrors.tencentyun.com/ubuntu-ports#ports.ubuntu.com/ubuntu-ports#g" /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null || true

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
        libeigen3-dev \
        libboost-dev && \
    rm -rf /var/lib/apt/lists/*

# Realsense SDK (D435i camera)
RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
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

# Use SDK's DDS config for peer discovery (unicast to motion control board 192.168.55.102)
ENV CYCLONEDDS_URI='file:///work/noetix_sdk_bumi/config/dds.xml'

EXPOSE 15704

CMD ["/bin/bash", "-c", "if command -v nvpmodel >/dev/null 2>&1; then nvpmodel -m 0 2>/dev/null; jetson_clocks 2>/dev/null; echo '[init] Jetson MAXN + clocks locked'; fi && source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
