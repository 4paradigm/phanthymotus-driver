ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG DEEP_ROBOTICS_MSG_REV=a0d1a29eec5c4db5a9107595bb51e3be8122b86c
ARG DEEP_ROBOTICS_MSG_SHA256=1d268a76e80af8ea5aa3dc28de0c236de87bce55a5d397fdad1e23515f02a537
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-colcon-common-extensions ros-humble-rmw-fastrtps-cpp && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} pyyaml
COPY deep-robotics-msg-${DEEP_ROBOTICS_MSG_REV}.zip /tmp/deep-robotics-msg.zip
RUN echo "${DEEP_ROBOTICS_MSG_SHA256}  /tmp/deep-robotics-msg.zip" | sha256sum -c - && \
    mkdir -p /lynx_m20_ws/src && \
    python3 -m zipfile -e /tmp/deep-robotics-msg.zip /tmp/deep-robotics-msg-unpacked && \
    mv /tmp/deep-robotics-msg-unpacked/deep-robotics-msg-${DEEP_ROBOTICS_MSG_REV} /lynx_m20_ws/src/deep-robotics-msg && \
    . /opt/ros/humble/setup.sh && cd /lynx_m20_ws && colcon build --packages-select drdds && \
    rm -rf /tmp/deep-robotics-msg.zip /tmp/deep-robotics-msg-unpacked
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py basic_server.py config.yaml driver.yaml /work/deep_robotics/lynx_m20/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15716
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /lynx_m20_ws/install/setup.bash && python3 /work/deep_robotics/lynx_m20/main.py"]
