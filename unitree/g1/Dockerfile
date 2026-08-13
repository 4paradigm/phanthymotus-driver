ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG TARGETARCH
ARG MINIFORGE_VERSION=24.7.1-2
ARG MINIFORGE_BASE_URL=https://github.com/conda-forge/miniforge/releases/download

WORKDIR /work

# The privileged G1 bundle currently runs as root; keep the persisted Capture
# credential digest private and writable by that actual runtime identity.
RUN install -d -m 0700 /var/lib/motus-g1-teleop

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl \
        git \
        libusb-1.0-0 \
        libportaudio2 \
        libasound2-dev \
        v4l-utils && \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    case "${TARGETARCH}" in \
        amd64) \
            miniforge_arch="x86_64"; \
            miniforge_sha256="636f7faca2d51ee42b4640ce160c751a46d57621ef4bf14378704c87c5db4fe3" \
            ;; \
        arm64) \
            miniforge_arch="aarch64"; \
            miniforge_sha256="7bf60bce50f57af7ea4500b45eeb401d9350011ab34c9c45f736647d8dba9021" \
            ;; \
        *) \
            echo "unsupported TARGETARCH: ${TARGETARCH}" >&2; \
            exit 64 \
            ;; \
    esac; \
    installer="/tmp/Miniforge3-${MINIFORGE_VERSION}-Linux-${miniforge_arch}.sh"; \
    curl -fsSL \
        "${MINIFORGE_BASE_URL}/${MINIFORGE_VERSION}/Miniforge3-${MINIFORGE_VERSION}-Linux-${miniforge_arch}.sh" \
        -o "${installer}"; \
    echo "${miniforge_sha256}  ${installer}" | sha256sum -c -; \
    /bin/bash "${installer}" -b -p /opt/miniforge3; \
    rm -f "${installer}"

COPY environment.teleop.yml /work/environment.teleop.yml
COPY requirements.txt /work/requirements.txt
RUN /opt/miniforge3/bin/conda env create \
        --prefix /opt/g1-teleop \
        --file /work/environment.teleop.yml && \
    /opt/miniforge3/bin/conda clean --all --yes

ENV PATH=/opt/g1-teleop/bin:${PATH}

RUN /opt/g1-teleop/bin/python -m pip install --no-cache-dir -i ${PYPI_MIRROR} \
    -r /work/requirements.txt pyalsaaudio

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

RUN /opt/g1-teleop/bin/python -m pip install \
    --no-cache-dir --no-binary cyclonedds \
    -i ${PYPI_MIRROR} cyclonedds==0.10.5

# Isolate rclpy topics (domain 42) from unitree_sdk2py's domain 0.
# Force FastDDS to match agent-core (which has no CycloneDDS rmw installed).
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT

COPY unitree_sdk2py/ /work/unitree_sdk2py/
COPY resource/       /work/resource/
COPY teleop/         /work/teleop/
COPY main.py   /work/main.py
COPY device.py /work/device.py
COPY rpc_proxy.py /work/rpc_proxy.py
COPY ext_devices.py /work/ext_devices.py
COPY safety_harness.py /work/safety_harness.py
COPY scan_context.py /work/scan_context.py
COPY controlled_spatial.py /work/controlled_spatial.py
COPY controlled_spatial_map.py /work/controlled_spatial_map.py
COPY pointcloud_utils.py /work/pointcloud_utils.py
COPY test_slam_rpc.py /work/test_slam_rpc.py
COPY config.yaml /work/config.yaml
COPY config.teleop-shadow.example.yaml /work/config.teleop-shadow.example.yaml
COPY deploy/     /deploy/

# Fail the target-architecture image build unless the exact G1_23 dependency
# path can cold-load IPOPT and solve the bundled neutral end-effector target.
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    export PYTHONPATH=/opt/g1-teleop/lib/python3.10/site-packages:/work:\${PYTHONPATH:-} && \
    export LD_LIBRARY_PATH=/opt/cyclonedds/lib:\${LD_LIBRARY_PATH:-} && \
    PYTHONDONTWRITEBYTECODE=1 /opt/g1-teleop/bin/python -m teleop.preflight image \
        --urdf /work/resource/g1_body23.urdf"

ENV PYTHONPATH=/opt/g1-teleop/lib/python3.10/site-packages:/work
ENV PYTHONUNBUFFERED=1

# Suppress CycloneDDS tracing to stdout (prevent oversized log messages)
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'

EXPOSE 15701 15702

CMD ["/bin/bash", "-c", "if command -v nvpmodel >/dev/null 2>&1; then nvpmodel -m 0 2>/dev/null; jetson_clocks 2>/dev/null; echo '[init] Jetson MAXN + clocks locked'; fi && source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && export PYTHONPATH=/opt/g1-teleop/lib/python3.10/site-packages:/work:${PYTHONPATH:-} && export LD_LIBRARY_PATH=/opt/cyclonedds/lib:${LD_LIBRARY_PATH:-} && exec /opt/g1-teleop/bin/python /work/main.py ${NETWORK_INTERFACE:-eth0}"]
