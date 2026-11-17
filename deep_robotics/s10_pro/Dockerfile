ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
ARG UBUNTU_MIRROR=http://ports.ubuntu.com/ubuntu-ports
# Verified source model from release.261117.bb0a602; no code is inherited.
ARG S10_DEPTH_MODEL_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/drivers/deep-robotics/s10-pro@sha256:f608a94272a1b6b386a80d59dd7bc335389997998ba4d8dd778f86b6ef57cda3
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
COPY depth_kernels.cpp /tmp/s10_depth_kernels.cpp
# The pinned ROS base installs build-essential, including g++.
RUN g++ -std=c++17 -O3 -shared -fPIC /tmp/s10_depth_kernels.cpp -o /tmp/_depth_kernels.so

FROM ${ROS_BASE_IMAGE} AS runtime-dependencies
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

FROM ${S10_DEPTH_MODEL_IMAGE} AS published-model

FROM runtime-dependencies AS model-source
COPY model_artifact.py /tmp/s10_model_artifact.py
COPY --from=published-model /work/deep_robotics/s10_pro/resource/models/yolo26n-depth.onnx /tmp/published-model.onnx
ARG S10_DEPTH_MODEL_URL=file:///tmp/published-model.onnx
RUN python3 -S /tmp/s10_model_artifact.py --model-url "${S10_DEPTH_MODEL_URL}" --output /model/source.onnx

FROM runtime-dependencies AS model-builder
COPY monocular_depth.py model_artifact.py /work/deep_robotics/s10_pro/
COPY requirements-build.txt /tmp/s10-pro-build-requirements.txt
COPY --from=model-source /model/source.onnx /tmp/source-model.onnx
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/s10-pro-build-requirements.txt \
    && python3 -m deep_robotics.s10_pro.monocular_depth --source-model /tmp/source-model.onnx

FROM runtime-dependencies
COPY --from=model-builder /work/deep_robotics/s10_pro/resource/models/ /work/deep_robotics/s10_pro/resource/models/
COPY common/ /work/common/
COPY main.py device.py basic_server.py domain_bridge.py loco_servo.py spatial.py media_workers.py undistortion.py front_depth.py depth_projection.py depth_fusion.py monocular_depth.py model_artifact.py cpu_kernels.py cloud_relay.py audio.py config.yaml driver.yaml /work/deep_robotics/s10_pro/
COPY --from=drdds-builder /tmp/_depth_kernels.so /work/deep_robotics/s10_pro/_depth_kernels.so
RUN python3 -c 'from deep_robotics.s10_pro.cpu_kernels import AVAILABLE; assert AVAILABLE, "native depth kernels failed to load"'
COPY resource/S10_kinematic.urdf /work/deep_robotics/s10_pro/resource/S10_kinematic.urdf
COPY resource/ssh_known_hosts /work/deep_robotics/s10_pro/resource/ssh_known_hosts
COPY resource/calibration/ /work/deep_robotics/s10_pro/resource/calibration/
COPY deploy/ /deploy/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15745
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /s10_pro_ws/install/setup.bash && python3 /work/deep_robotics/s10_pro/main.py"]
