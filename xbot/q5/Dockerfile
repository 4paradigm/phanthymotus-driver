# Q5 外挂 PC 驱动构建上下文
# SDK 全部为本地 vendor，构建时不访问网络拉取

ARG ROS_BASE_IMAGE=ros:humble
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        libportaudio2 \
        portaudio19-dev \
        libasound2-dev \
        alsa-utils && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml pyaudio

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
