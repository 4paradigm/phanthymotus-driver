ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        libusb-1.0-0 \
        can-utils && \
    rm -rf /var/lib/apt/lists/*

# bc-stark-sdk is a compiled Rust/pyo3 wheel (cp39-abi3, manylinux_2_34) — a
# normal pip dependency, not something to git-clone or vendor from COS.
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml bc-stark-sdk==2.0.2

COPY common/     /work/common/
COPY resource/    /work/resource/
COPY main.py      /work/main.py
COPY device.py    /work/device.py
COPY config.yaml  /work/config.yaml
COPY deploy/      /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15706

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
