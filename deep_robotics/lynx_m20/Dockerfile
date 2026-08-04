ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG DEEP_ROBOTICS_MSG_REV=a0d1a29eec5c4db5a9107595bb51e3be8122b86c
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN apt-get update && apt-get install -y --no-install-recommends git python3-pip python3-colcon-common-extensions ros-humble-rmw-fastrtps-cpp && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} pyyaml
RUN git clone https://github.com/DeepRoboticsLab/deep-robotics-msg.git /tmp/deep-robotics-msg && \
    git -C /tmp/deep-robotics-msg checkout ${DEEP_ROBOTICS_MSG_REV} && \
    mkdir -p /lynx_m20_ws/src && cp -r /tmp/deep-robotics-msg /lynx_m20_ws/src/deep-robotics-msg && \
    . /opt/ros/humble/setup.sh && cd /lynx_m20_ws && colcon build --packages-select drdds && \
    rm -rf /tmp/deep-robotics-msg /lynx_m20_ws/src/deep-robotics-msg/.git
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py basic_server.py config.yaml driver.yaml /work/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15716
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /lynx_m20_ws/install/setup.bash && python3 /work/main.py"]
