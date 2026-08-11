# Q5 外挂 PC 驱动构建上下文
# SDK 全部为本地 vendor，构建时不访问网络拉取
#
# 构建参数：
#   ROS_BASE_IMAGE  — 基础镜像（默认 ros:humble，需预先 docker pull）
#   PYPI_MIRROR     — pip 镜像（默认 https://pypi.tuna.tsinghua.edu.cn/simple/）
#   APT_MIRROR      — apt 镜像站点（默认 mirrors.tuna.tsinghua.edu.cn，留空则用默认源）

ARG ROS_BASE_IMAGE=ros:humble
ARG PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple/
ARG APT_MIRROR=mirrors.tuna.tsinghua.edu.cn
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG APT_MIRROR

WORKDIR /work

# 替换 apt 源为国内镜像（arm64 用 ubuntu-ports）
RUN if [ -n "$APT_MIRROR" ]; then \
        sed -i "s|ports.ubuntu.com|${APT_MIRROR}|g" /etc/apt/sources.list && \
        sed -i "s|archive.ubuntu.com|${APT_MIRROR}|g" /etc/apt/sources.list; \
    fi

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        libportaudio2 \
        portaudio19-dev \
        libasound2-dev \
        alsa-utils \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-control-msgs \
        ros-humble-nav-msgs \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-std-srvs \
        ros-humble-diagnostic-msgs \
        ros-humble-action-msgs && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --timeout=300 --retries=5 -i ${PYPI_MIRROR} \
    pyaudio && \
    pip3 install --no-cache-dir --timeout=300 --retries=5 -i ${PYPI_MIRROR} \
    sherpa-onnx || echo "[WARN] sherpa-onnx install failed — SpeakerPlugin will not work"

# 本地 vendor 的 ROS2 消息包：xbot_common_interfaces + era_nav_msgs
COPY vendor/xbot_common_interfaces /q5_ws/src/xbot_common_interfaces
COPY vendor/era_nav_msgs /q5_ws/src/era_nav_msgs
RUN cd /q5_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select xbot_common_interfaces era_nav_msgs

# 本地 vendor 的纯 Python SDK（非 colcon 包，直接 import）
COPY vendor/xbot_sdk_api /opt/xbot_sdk_api

COPY main.py device.py config.yaml driver.yaml /work/

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=211
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work:/opt/xbot_sdk_api

EXPOSE 15708

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
    source /q5_ws/install/setup.bash && \
    python3 /work/main.py"]
