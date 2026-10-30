ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE} AS drdds-builder
RUN set -eu; package=python3-colcon-common-extensions; \
    if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
        apt-get update && apt-get install -y --no-install-recommends "$package" && rm -rf /var/lib/apt/lists/*; \
    fi
WORKDIR /s10_pro_ws
COPY resource/deep-robotics-msg/ /s10_pro_ws/src/deep-robotics-msg/
RUN . /opt/ros/humble/setup.sh && cd /s10_pro_ws && colcon build --packages-select drdds && rm -rf build log src

FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN set -eu; missing=""; \
    for package in ca-certificates python3-pip ros-humble-rmw-fastrtps-cpp; do \
        if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
            missing="$missing $package"; \
        fi; \
    done; \
    if [ -n "$missing" ]; then \
        apt-get update && apt-get install -y --no-install-recommends $missing && rm -rf /var/lib/apt/lists/*; \
    fi
COPY --from=drdds-builder /s10_pro_ws/install/ /s10_pro_ws/install/
COPY requirements.txt /tmp/s10-pro-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/s10-pro-requirements.txt && rm /tmp/s10-pro-requirements.txt
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py basic_server.py domain_bridge.py media_workers.py config.yaml driver.yaml /work/deep_robotics/s10_pro/
COPY resource/S10.urdf /work/deep_robotics/s10_pro/resource/S10.urdf
COPY deploy/ /deploy/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15745
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /s10_pro_ws/install/setup.bash && python3 /work/deep_robotics/s10_pro/main.py"]
