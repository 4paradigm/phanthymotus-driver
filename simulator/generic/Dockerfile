ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG APT_MIRROR=http://mirrors.tencentyun.com/ubuntu-ports
ARG PYPI_MIRROR=https://pypi.org/simple/

# Deliberately thin. rclpy and std_msgs are needed because `topic_out` is the
# only route to the dashboard — ros2_bridge._resolve_msg_type maps
# sensor/mapping to UInt8MultiArray and data/json to String. Everything the
# vendor bundles add on top is not: no colcon, no cmake or build-essential, no
# vendor message packages, no espeak-ng or alsa-utils, no sshpass, no opencv and
# no CycloneDDS built from source. There is no hardware to talk to.
RUN sed -i "s|http://mirrors.tencentyun.com/ubuntu-ports|${APT_MIRROR}|g" /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip python3-yaml ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY common/ /work/common/

# The build context is this bundle's own directory, so `simulator/__init__.py`
# (one level up in the repo) is not in it. Written here rather than left to
# Python's implicit namespace packages: that would work, silently, right up
# until someone puts another `simulator` directory on the path.
RUN mkdir -p /work/simulator && \
    printf '"""Virtual driver bundles."""\n' > /work/simulator/__init__.py

# Explicit rather than a glob, as the other bundles do — a glob would make
# tests/test_dockerfile_copies.py's import check vacuous, and that check is the
# only thing standing between a forgotten module and a container that builds
# cleanly and then dies on startup with ModuleNotFoundError.
COPY __init__.py acp.py card_base.py cards_audio.py cards_motion.py \
     cards_scenario.py cards_sensors.py clock.py geometry.py main.py plugins.py \
     maps.py scenario.py sensors.py world.py config.yaml driver.yaml \
     /work/simulator/generic/
COPY backend/ /work/simulator/generic/backend/
COPY tools/ /work/simulator/generic/tools/
COPY maps/ /work/simulator/generic/maps/
COPY scenarios/ /work/simulator/generic/scenarios/
COPY deploy/ /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_LOCALHOST_ONLY=0 \
    PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    PYTHONPATH=/work

EXPOSE 15711
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/simulator/generic/main.py"]
