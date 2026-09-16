ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
WORKDIR /work
ARG PYPI_MIRROR
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip cmake build-essential curl && rm -rf /var/lib/apt/lists/*
ARG CYCLONEDDS_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/cyclonedds-0.10.5.tar.gz
RUN curl -fsSL ${CYCLONEDDS_URL} | tar xz -C /tmp && \
    cmake -S /tmp/cyclonedds-0.10.5 -B /tmp/cyclonedds-0.10.5/build -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds && \
    cmake --build /tmp/cyclonedds-0.10.5/build --parallel 2 --target install && \
    rm -rf /tmp/cyclonedds-0.10.5
ENV CYCLONEDDS_HOME=/opt/cyclonedds
ENV LD_LIBRARY_PATH=/opt/cyclonedds/lib:${LD_LIBRARY_PATH}
COPY requirements.txt /work/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r requirements.txt && \
    pip3 install --no-cache-dir --no-binary cyclonedds -i ${PYPI_MIRROR} cyclonedds==0.10.5
COPY unitree_sdk2py/ /work/unitree_sdk2py/
ARG TARGETARCH
ARG UNITREE_SDK_COMMIT=65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5
RUN set -eu; \
    case "${TARGETARCH}" in \
      amd64) crc_file=crc_amd64.so; crc_sha=b136a91a7e99c5cf914f465e131f63c897a17107939184a6f85d1a57cf315ade ;; \
      arm64) crc_file=crc_aarch64.so; crc_sha=a4db103653db78540d141ff4dc55216b83e5298280e639b2ad54a84ac2fae9a5 ;; \
      *) echo "unsupported target architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac; \
    crc_dir=/work/unitree_sdk2py/utils/lib; \
    mkdir -p "${crc_dir}"; \
    curl -fsSL --retry 5 --retry-delay 2 --connect-timeout 20 --max-time 120 \
      "https://cdn.jsdelivr.net/gh/unitreerobotics/unitree_sdk2_python@${UNITREE_SDK_COMMIT}/unitree_sdk2py/utils/lib/${crc_file}" \
      -o "${crc_dir}/${crc_file}"; \
    echo "${crc_sha}  ${crc_dir}/${crc_file}" | sha256sum -c -
COPY main.py device.py lidar.py controlled_spatial.py motion_tools.py rpc_proxy.py config.yaml /work/
COPY resource/ /work/resource/
COPY common/ /work/common/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    RCUTILS_COLORIZED_OUTPUT=0 \
    CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'
EXPOSE 15709
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-}"]
