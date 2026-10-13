ARG G1_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/teleop/base:release.261012.4674297
FROM ${G1_BASE_IMAGE}

# Isolate rclpy topics (domain 42) from unitree_sdk2py's domain 0.
# Force FastDDS to match agent-core (which has no CycloneDDS rmw installed).
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY unitree_sdk2py/ /work/unitree_sdk2py/
COPY resource/       /work/resource/
COPY common/ /work/common/
COPY main.py   /work/main.py
COPY device.py /work/device.py
COPY led_control.py /work/led_control.py
COPY rpc_proxy.py /work/rpc_proxy.py
COPY ext_devices.py /work/ext_devices.py
COPY safety_harness.py /work/safety_harness.py
COPY sport_mode_state.py /work/sport_mode_state.py
COPY scan_context.py /work/scan_context.py
COPY controlled_spatial.py /work/controlled_spatial.py
COPY controlled_spatial_map.py /work/controlled_spatial_map.py
COPY pointcloud_utils.py /work/pointcloud_utils.py
COPY servo.py /work/servo.py
COPY arm_sdk.py /work/arm_sdk.py
COPY arm_stream.py motion_control.py motion_bus.py teleop_control.py teleop_bus.py ros_spin.py /work/
COPY motion/ /work/motion/
RUN python3 -c 'import sys, numpy, casadi, pinocchio; from pinocchio import casadi as cpin; assert sys.executable == "/usr/bin/python3"; assert numpy.__version__ == "2.2.6"; assert casadi.__version__ == "3.8.1"; assert pinocchio.__version__ == "4.1.0"; cpin.Model(pinocchio.buildSampleModelManipulator())' && \
    python3 /work/motion/fetch_assets.py && python3 /work/motion/fetch_assets.py --check
COPY servo_eef.py /work/servo_eef.py
COPY test_slam_rpc.py /work/test_slam_rpc.py
COPY config.yaml /work/config.yaml
COPY deploy/     /deploy/

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1
# ANSI colour codes are control bytes; in a non-tty container log they are
# exactly the "garbage characters" that make `docker logs` unreadable.
ENV RCUTILS_COLORIZED_OUTPUT=0

# Suppress CycloneDDS tracing to stdout (prevent oversized log messages)
ENV CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'

EXPOSE 15701

# The image deliberately does **not** try to set the Jetson power mode or lock
# its clocks. It used to: the CMD began with
#
#   if command -v nvpmodel >/dev/null 2>&1; then nvpmodel -m 0; jetson_clocks; fi
#
# and on r1_sz that branch had never once been taken, because there is no
# `nvpmodel` inside the container — it is part of the host's L4T tooling, and
# both commands write host-wide state that a container has no business owning
# anyway. The line read as "this is handled" while the GPU sat at 612 MHz of its
# 918 MHz ceiling and the memory clock at two thirds of its own.
#
# It belongs to the host installer, which runs as root outside any container:
# `resource-center/public/install.sh`, step 1.55. That one also installs a
# systemd unit, because `jetson_clocks` is not persistent across a reboot while
# `nvpmodel` is — so a rebooted machine and a fresh one would otherwise run at
# different speeds with nothing anywhere saying why.
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-eth0}"]
