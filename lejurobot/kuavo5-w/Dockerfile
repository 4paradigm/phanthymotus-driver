ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/

# ros-base provides pip; only install missing ROS2 runtime packages.
# ROS1 geometry_msgs is encoded as JSON by rosbridge, not imported locally.
RUN set -eu; missing=""; \
    for package in ros-humble-rmw-fastrtps-cpp ros-humble-std-msgs; do \
        if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then \
            missing="$missing $package"; \
        fi; \
    done; \
    if [ -n "$missing" ]; then \
        apt-get update && apt-get install -y --no-install-recommends $missing && \
        rm -rf /var/lib/apt/lists/*; \
    fi
COPY requirements.txt /work/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /work/requirements.txt
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py rosbridge_client.py kuavo_sensors.py config.yaml driver.yaml /work/
COPY resource/ /work/resource/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42
EXPOSE 15747
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /work/main.py"]
