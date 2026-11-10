ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

WORKDIR /work
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential kmod iproute2 can-utils systemd python3-pip && \
    rm -rf /var/lib/apt/lists/*
COPY requirements.txt /work/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /work/requirements.txt
COPY common /work/common
COPY . /work/agilex/bunker-pro
COPY deploy/ /deploy/
COPY deploy/entrypoint.sh /usr/local/bin/bunker-entrypoint
RUN chmod 0755 /usr/local/bin/bunker-entrypoint
ENV PYTHONPATH=/work/agilex/bunker-pro:/work
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml
ENV PYTHONUNBUFFERED=1
EXPOSE 15742
ENTRYPOINT ["/usr/local/bin/bunker-entrypoint"]
