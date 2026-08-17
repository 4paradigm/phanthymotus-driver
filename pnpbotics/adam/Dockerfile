ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://mirrors.cloud.tencent.com/pypi/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip cmake build-essential curl git \
    libturbojpeg libusb-1.0-0 libegl1 libgles2 libv4l-0 \
    libx11-6 libxext6 libbsd0 python3-pil && \
    rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml numpy grpcio protobuf

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
COPY main.py        /work/main.py
COPY device.py      /work/device.py
COPY grpc_client.py /work/grpc_client.py
COPY config.yaml    /work/config.yaml
COPY deploy/        /deploy/

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT
ENV PYTHONPATH=/work:/opt/pyzed
ENV PYTHONUNBUFFERED=1
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'
ENV ZED_PYTHON_PATH=/opt/pyzed

EXPOSE 15702

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
