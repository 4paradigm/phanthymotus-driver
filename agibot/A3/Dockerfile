ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/

# FastDDS in the Humble base image fails to reassemble the A3 Jazzy Image and
# PointCloud2 samples ("sequence size exceeds remaining buffer").  Keep
# FastDDS for the isolated domain-42 bridge, but use CycloneDDS in the main
# robot participant for robust large-sample reception.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg lsb-release && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# A3 has no vendored msgs package to colcon-build (unlike X2's aimdk_msgs zip):
# The official AimDK v3.2 wheel is vendored below (the vendor package does not
# require a device-specific runtime).  ros2_plugin_proto remains sourced from
# the mounted dev-kit because it is an AArch64 ROS interface package.
COPY requirements.txt /tmp/agibot-a3-requirements.txt
COPY vendor/a3_aimdk-3.2.0-py3-none-any.whl /tmp/a3_aimdk-3.2.0-py3-none-any.whl
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/agibot-a3-requirements.txt /tmp/a3_aimdk-3.2.0-py3-none-any.whl && rm -f /tmp/agibot-a3-requirements.txt /tmp/a3_aimdk-3.2.0-py3-none-any.whl
COPY audio_msgs/ /tmp/a3_ws/src/audio_msgs/
COPY ros2_plugin_proto/ /tmp/a3_ws/src/ros2_plugin_proto/
RUN cd /tmp/a3_ws && \
    if [ -f /opt/ros/jazzy/setup.sh ]; then . /opt/ros/jazzy/setup.sh; else . /opt/ros/humble/setup.sh; fi && \
    colcon build --packages-select audio_msgs ros2_plugin_proto --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf src build log

WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py domain_bridge.py config.yaml driver.yaml /work/agibot/A3/
COPY resource/ /work/agibot/A3/resource/
# service.yml must be INSIDE the image (12th PR review): Agent Core extracts the
# deployment fragment from the driver image and merges it into the host's unified
# /opt/phanthy-motus/docker-compose.yml (README_dev deployment contract). Without
# this COPY the deployment silently falls back to legacy `docker run` — no host
# networking, no /agibot-devkit mount (so the wheel/ros2_plugin_proto CMD paths
# never run and the command cards stay withheld/degraded in production), no log
# rotation, no DDS profile.
COPY deploy/ /deploy/
COPY cyclonedds.xml /work/agibot/A3/cyclonedds.xml
# RCUTILS_COLORIZED_OUTPUT=0: rclpy logging must not emit ANSI color codes —
# logsafe scrubs them anyway, but the fleet contract (README_dev, 11 drivers)
# is to keep raw rclpy output plain so every consumer sees identical records.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///work/agibot/A3/cyclonedds.xml ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 PYTHONPATH=/work
EXPOSE 15719
CMD ["/bin/bash", "-c", "set -e; if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi; source /tmp/a3_ws/install/setup.bash; for setup in /opt/agibot/share/ros2_package/aimrt_protocol_ros2_package/share/ros2_plugin_proto/local_setup.bash /opt/agibot/share/ros2_package/aima_protocol_ros2_package/share/audio_msgs/local_setup.bash /agibot-devkit/prebuilt/ros2_plugin_proto_aarch64/share/ros2_plugin_proto/local_setup.bash /agibot-devkit/prebuilt/audio_msgs_proto_aarch64/share/audio_msgs/local_setup.bash; do if [ -f \"$setup\" ]; then source \"$setup\"; fi; done; python3 -c 'from audio_msgs.msg import AudioCapture, AudioPlayback; print(\"[audio] audio_msgs AudioCapture/AudioPlayback available\", flush=True)' || { echo '[warn] A3 audio_msgs unavailable — mic/speaker degraded'; }; python3 -c 'from ros2_plugin_proto.msg import RosMsgWrapper; print(\"[ros2_plugin_proto] RosMsgWrapper available\", flush=True)' || { echo '[warn] ros2_plugin_proto unavailable — wrapper command cards degraded'; }; python3 -c 'from aimdk.protocol.hal.bms.hal_bms_channel_pb2 import BmsStateChannel; from aimdk.protocol.hal.state.hal_state_channel_pb2 import EmergencyStateChannel; print(\"[a3_aimdk] 3.2.0 protobuf available\", flush=True)' || { echo '[warn] a3_aimdk protobuf import failed — battery/estop degraded'; }; python3 /work/agibot/A3/main.py"]
