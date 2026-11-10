ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

WORKDIR /work
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential kmod iproute2 systemd python3-pip util-linux && \
    rm -rf /var/lib/apt/lists/*
COPY requirements.txt /work/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /work/requirements.txt
COPY common /work/common
COPY main.py device.py protocol.py can_runtime.py dds_bridge.py config.yaml /work/agilex/bunker-mini/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work/agilex/bunker-mini:/work
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml
ENV PYTHONUNBUFFERED=1
ENV RCUTILS_COLORIZED_OUTPUT=0
EXPOSE 15744
ENTRYPOINT ["/bin/bash", "/deploy/entrypoint.sh"]
