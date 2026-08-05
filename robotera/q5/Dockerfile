ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

ARG XBOT_INTERFACES_REV=578353e11a9b46a87356fb994982f726051ba6ce
ARG ERA_NAV_REV=86a415493d34af309e1c8c007fdea467d33d51e4
ARG PYPI_MIRROR=https://pypi.org/simple/

RUN apt-get update && apt-get install -y --no-install-recommends git python3-pip python3-colcon-common-extensions ros-humble-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} pyyaml

RUN mkdir -p /q5_ws/src && \
    git clone https://github.com/roboterax/teleop_client.git /tmp/teleop_client && \
    git -C /tmp/teleop_client checkout ${XBOT_INTERFACES_REV} && \
    cp -r /tmp/teleop_client/xbot_common_interfaces /q5_ws/src/ && \
    git clone https://github.com/roboterax/era_nav_msgs.git /q5_ws/src/era_nav_msgs && \
    git -C /q5_ws/src/era_nav_msgs checkout ${ERA_NAV_REV} && \
    . /opt/ros/humble/setup.sh && cd /q5_ws && colcon build --packages-select xbot_common_interfaces era_nav_msgs && \
    rm -rf /tmp/teleop_client /q5_ws/src/*/.git

WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15711
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /q5_ws/install/setup.bash && python3 /work/main.py"]
