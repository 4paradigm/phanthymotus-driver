ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/

# Download at image-build time (the build worker has external network access;
# ADU does not). The official arm64 standalone archive contains the bridge
# executable and embeds its DDS implementation.
ADD --checksum=sha256:fdb64d942d4b6beccbe9b1f8a359a01a2fca95a023c27535bf97b233603fad5f \
    https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.10.1/zenoh-plugin-ros2dds-1.10.1-aarch64-unknown-linux-gnu-standalone.zip \
    /tmp/a3-zenoh.zip
RUN python3 -c "import zipfile; z=zipfile.ZipFile('/tmp/a3-zenoh.zip'); z.extract('zenoh-bridge-ros2dds', '/usr/local/libexec/a3-zenoh');" && \
    chmod 0755 /usr/local/libexec/a3-zenoh/zenoh-bridge-ros2dds && \
    rm -f /tmp/a3-zenoh.zip

# The robot publishes Jazzy ROS types. Keep the main participant on FastDDS so
# its type-hash negotiation matches the robot; the isolated domain-42 bridge
# also uses FastDDS in its child workers. The base image already provides the
# FastDDS RMW, so no extra apt package is needed here.

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
COPY zenoh_bridge.sh /work/agibot/A3/zenoh_bridge.sh
RUN chmod 0755 /work/agibot/A3/zenoh_bridge.sh
COPY resource/ /work/agibot/A3/resource/
# service.yml must be INSIDE the image (12th PR review): Agent Core extracts the
# deployment fragment from the driver image and merges it into the host's unified
# /opt/phanthy-motus/docker-compose.yml (README_dev deployment contract). Without
# this COPY the deployment silently falls back to legacy `docker run` — no host
# networking, no /agibot-devkit mount (so the wheel/ros2_plugin_proto CMD paths
# never run and the command cards stay withheld/degraded in production), no log
# rotation, no DDS profile.
COPY deploy/ /deploy/
# RCUTILS_COLORIZED_OUTPUT=0: rclpy logging must not emit ANSI color codes —
# logsafe scrubs them anyway, but the fleet contract (README_dev, 11 drivers)
# is to keep raw rclpy output plain so every consumer sees identical records.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 PYTHONPATH=/work
EXPOSE 15719
CMD ["/bin/bash", "-c", "set -e; source /opt/ros/humble/setup.bash; source /tmp/a3_ws/install/setup.bash; if [ \"${A3_ZENOH_BRIDGE:-0}\" = 1 ]; then /work/agibot/A3/zenoh_bridge.sh start; trap '/work/agibot/A3/zenoh_bridge.sh stop' EXIT; fi; python3 -c 'from audio_msgs.msg import AudioCapture, AudioPlayback; print(\"[audio] audio_msgs AudioCapture/AudioPlayback available\", flush=True)' || { echo '[warn] A3 audio_msgs unavailable — mic/speaker degraded'; }; python3 -c 'from ros2_plugin_proto.msg import RosMsgWrapper; print(\"[ros2_plugin_proto] RosMsgWrapper available\", flush=True)' || { echo '[warn] ros2_plugin_proto unavailable — wrapper command cards degraded'; }; python3 -c 'from aimdk.protocol.hal.bms.hal_bms_channel_pb2 import BmsStateChannel; from aimdk.protocol.hal.state.hal_state_channel_pb2 import EmergencyStateChannel; print(\"[a3_aimdk] 3.2.0 protobuf available\", flush=True)' || { echo '[warn] a3_aimdk protobuf import failed — battery/estop degraded'; }; python3 /work/agibot/A3/main.py"]
