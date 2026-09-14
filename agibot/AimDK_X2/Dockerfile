ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG AIMDK_MSGS_REV=a424add7
ARG AIMDK_MSGS_SHA256=4577528a5f4812e06e538ca759a7ef33f535fb70a0a20991b55a2b8b59c5d5a6
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-colcon-common-extensions ros-humble-rmw-fastrtps-cpp ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/agibot-x2-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/agibot-x2-requirements.txt
COPY aimdk_msgs-${AIMDK_MSGS_REV}.zip /tmp/aimdk_msgs.zip
RUN echo "${AIMDK_MSGS_SHA256}  /tmp/aimdk_msgs.zip" | sha256sum -c - && \
    mkdir -p /aimdk_x2_ws/src/aimdk_msgs && \
    python3 -m zipfile -e /tmp/aimdk_msgs.zip /aimdk_x2_ws/src/aimdk_msgs && \
    . /opt/ros/humble/setup.sh && cd /aimdk_x2_ws && colcon build --packages-select aimdk_msgs && \
    rm -rf /tmp/aimdk_msgs.zip
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/agibot/AimDK_X2/
COPY resource/ /work/agibot/AimDK_X2/resource/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15717
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /aimdk_x2_ws/install/setup.bash && python3 /work/agibot/AimDK_X2/main.py"]
