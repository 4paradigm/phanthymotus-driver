ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
ARG UBUNTU_MIRROR=http://ports.ubuntu.com/ubuntu-ports
FROM ${ROS_BASE_IMAGE} AS drdds-builder
ARG UBUNTU_MIRROR
RUN set -eu; package=python3-colcon-common-extensions; \
    if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
        find /etc/apt \( -name '*.list' -o -name '*.sources' \) -type f -exec sed -i \
            -e "s|http://mirrors.tencentyun.com/ubuntu-ports|${UBUNTU_MIRROR}|g" \
            -e "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_MIRROR}|g" {} + && \
        apt-get update && apt-get install -y --no-install-recommends "$package" && rm -rf /var/lib/apt/lists/*; \
    fi
WORKDIR /s10_pro_ws
COPY resource/deep-robotics-msg/ /s10_pro_ws/src/deep-robotics-msg/
COPY resource/audio_msgs/ /s10_pro_ws/src/audio_msgs/
RUN . /opt/ros/humble/setup.sh && cd /s10_pro_ws && colcon build --packages-select drdds audio_msgs && rm -rf build log src

FROM ${ROS_BASE_IMAGE}
ARG UBUNTU_MIRROR
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN set -eu; missing=""; \
    for package in alsa-utils ca-certificates ffmpeg openssh-client python3-pip ros-humble-rmw-fastrtps-cpp sshpass; do \
        if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
            missing="$missing $package"; \
        fi; \
    done; \
    if [ -n "$missing" ]; then \
        find /etc/apt \( -name '*.list' -o -name '*.sources' \) -type f -exec sed -i \
            -e "s|http://mirrors.tencentyun.com/ubuntu-ports|${UBUNTU_MIRROR}|g" \
            -e "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_MIRROR}|g" {} + && \
        apt-get update && apt-get install -y --no-install-recommends $missing && rm -rf /var/lib/apt/lists/*; \
    fi
COPY --from=drdds-builder /s10_pro_ws/install/ /s10_pro_ws/install/
COPY requirements.txt /tmp/s10-pro-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/s10-pro-requirements.txt && rm /tmp/s10-pro-requirements.txt
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py basic_server.py domain_bridge.py loco_servo.py spatial.py media_workers.py undistortion.py audio.py config.yaml driver.yaml /work/deep_robotics/s10_pro/
COPY tools/ /work/deep_robotics/s10_pro/tools/
COPY resource/S10_kinematic.urdf /work/deep_robotics/s10_pro/resource/S10_kinematic.urdf
COPY resource/ssh_known_hosts /work/deep_robotics/s10_pro/resource/ssh_known_hosts
COPY deploy/ /deploy/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15745
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /s10_pro_ws/install/setup.bash && python3 /work/deep_robotics/s10_pro/main.py"]
