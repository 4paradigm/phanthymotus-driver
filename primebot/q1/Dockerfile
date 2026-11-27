ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG DEBIAN_FRONTEND=noninteractive
RUN if ! command -v colcon >/dev/null || ! dpkg -s curl python3-yaml ros-humble-rosidl-default-generators ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-rmw-fastrtps-cpp >/dev/null 2>&1; then \
      apt-get update && apt-get install -y --no-install-recommends curl python3-colcon-common-extensions python3-yaml ros-humble-rosidl-default-generators ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-rmw-fastrtps-cpp && rm -rf /var/lib/apt/lists/*; \
    fi
WORKDIR /q1_ws
ARG Q1_SDK_URL=https://csdn-tos.primebot.com/qiyuan/2026/11/17/85d49c0e7740409ba067b09bcba57c94.zip
RUN curl -fsSL --retry 2 --connect-timeout 10 --max-time 300 "$Q1_SDK_URL" -o /tmp/q1-sdk.zip && \
    echo "29d14627da375b30da308fc82c9fdd454e9393ab6b97ca8b51aa6229afffcae8  /tmp/q1-sdk.zip" | sha256sum -c - && \
    python3 -m zipfile -e /tmp/q1-sdk.zip /tmp/q1-sdk && mkdir -p src && \
    mv /tmp/q1-sdk/q1-sdk-v1.1.8.0/aimdk_msgs src/ && rm -rf /tmp/q1-sdk /tmp/q1-sdk.zip
RUN . /opt/ros/humble/setup.sh && MAKEFLAGS=-j3 colcon build --packages-select aimdk_msgs --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF && rm -rf build log
WORKDIR /work
COPY deploy/service.yml /deploy/service.yml
COPY common/ common/
COPY main.py battery.py robot.py preset_postures.py preset_robot.py config.yaml primebot/q1/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EXPOSE 15748
ENTRYPOINT []
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /q1_ws/install/setup.bash && exec /usr/bin/python3 /work/primebot/q1/main.py"]
