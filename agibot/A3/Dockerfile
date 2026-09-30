ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/

# A3 has no vendored msgs package to colcon-build (unlike X2's aimdk_msgs zip):
# the aimdk.protocol_pb2 wheel and the ros2_plugin_proto prebuilt both ship in
# the robot's AimDK 开发包 prebuilt/ directory instead of this repo. The CMD
# pip-installs the wheel and sources ros2_plugin_proto's local_setup.bash when
# the dev-kit directory is mounted at /agibot-devkit; device.py degrades
# gracefully when they are absent (pb stream tools withheld, JSON payload
# fallback), so the image also runs on any host for development — with
# loco/waist_control/face_play withheld too, since their RosMsgWrapper
# publishers cannot exist without ros2_plugin_proto.
COPY requirements.txt /tmp/agibot-a3-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/agibot-a3-requirements.txt && rm -f /tmp/agibot-a3-requirements.txt

WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/agibot/A3/
COPY resource/ /work/agibot/A3/resource/
# RCUTILS_COLORIZED_OUTPUT=0: rclpy logging must not emit ANSI color codes —
# logsafe scrubs them anyway, but the fleet contract (README_dev, 11 drivers)
# is to keep raw rclpy output plain so every consumer sees identical records.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 PYTHONPATH=/work
EXPOSE 15719
CMD ["/bin/bash", "-c", "set -e; source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; if [ -d /agibot-devkit/prebuilt ]; then pip3 install --no-cache-dir /agibot-devkit/prebuilt/a3_aimdk-3.1.0-py3-none-any.whl || echo '[warn] a3_aimdk wheel install failed — running degraded (pb stream cards withheld, RosMsgWrapper payloads fall back to raw JSON bytes)'; if [ -f /agibot-devkit/prebuilt/ros2_plugin_proto_aarch64/share/ros2_plugin_proto/local_setup.bash ]; then source /agibot-devkit/prebuilt/ros2_plugin_proto_aarch64/share/ros2_plugin_proto/local_setup.bash; else echo '[warn] ros2_plugin_proto local_setup.bash not found — command wrappers will not be publishable'; fi; else echo '[warn] /agibot-devkit not mounted — running degraded (no a3_aimdk wheel, no ros2_plugin_proto)'; fi; python3 /work/agibot/A3/main.py"]
