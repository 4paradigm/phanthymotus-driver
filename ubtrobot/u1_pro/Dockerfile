ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG ROS_APT_MIRROR

WORKDIR /work
RUN test -s /usr/share/keyrings/ros2-archive-keyring.gpg || \
        { echo "ERROR: base image is missing /usr/share/keyrings/ros2-archive-keyring.gpg"; exit 1; } && \
    . /etc/os-release && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] ${ROS_APT_MIRROR} ${UBUNTU_CODENAME} main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-pil python3-colcon-common-extensions cmake build-essential \
    ros-humble-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/u1-requirements.txt
RUN pip3 install --no-cache-dir -i "${PYPI_MIRROR}" -r /tmp/u1-requirements.txt && rm /tmp/u1-requirements.txt
WORKDIR /u1_ws
COPY vendor/ /u1_ws/src/
RUN . /opt/ros/humble/setup.sh && colcon build --merge-install
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/
COPY deploy/ /deploy/

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/u1_ws/install/audio_msgs/lib/python3.10/site-packages:/u1_ws/install/robo_sdk/lib/python3.10/site-packages:/work

EXPOSE 15740
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /u1_ws/install/setup.bash && python3 /work/main.py"]
