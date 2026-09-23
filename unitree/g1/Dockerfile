ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM mambaorg/micromamba:2.3.2 AS motion-numeric-installer
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl \
        git \
        libusb-1.0-0 \
        libportaudio2 \
        libasound2-dev \
        ffmpeg \
        v4l-utils && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml netifaces opencv-python-headless numpy pyrealsense2 sounddevice pyalsaaudio

# Build CycloneDDS 0.10.5 from source
ARG CYCLONEDDS_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/cyclonedds-0.10.5.tar.gz
RUN curl -fsSL ${CYCLONEDDS_URL} \
    | tar xz -C /tmp && \
    mkdir -p /tmp/cyclonedds-0.10.5/build && \
    cd /tmp/cyclonedds-0.10.5/build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds && \
    cmake --build . --config RelWithDebInfo --parallel --target install && \
    rm -rf /tmp/cyclonedds-0.10.5

ENV CYCLONEDDS_HOME=/opt/cyclonedds
ENV LD_LIBRARY_PATH=/opt/cyclonedds/lib:${LD_LIBRARY_PATH}

RUN pip3 install --no-cache-dir --no-binary cyclonedds -i ${PYPI_MIRROR} cyclonedds==0.10.5

# Isolate rclpy topics (domain 42) from unitree_sdk2py's domain 0.
# Force FastDDS to match agent-core (which has no CycloneDDS rmw installed).
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT

COPY unitree_sdk2py/ /work/unitree_sdk2py/
COPY resource/       /work/resource/
COPY common/ /work/common/
COPY main.py   /work/main.py
COPY device.py /work/device.py
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
COPY arm_stream.py motion_control.py motion_bus.py teleop_control.py teleop_bus.py /work/
COPY g1_motion/ /work/g1_motion/
COPY dds-local.xml /work/dds-local.xml
COPY --from=motion-numeric-installer /bin/micromamba /usr/local/bin/micromamba
RUN micromamba create --yes --prefix /opt/g1-motion --file /work/g1_motion/requirements.numeric-linux-aarch64.lock && \
    micromamba clean --all --yes && \
    /opt/g1-motion/bin/python -c 'import numpy, casadi, pinocchio; from pinocchio import casadi as cpin; assert numpy.__version__ == "1.26.4"; assert casadi.__version__ == "3.6.7"; assert pinocchio.__version__ == "3.1.0"' && \
    python3 /work/g1_motion/fetch_assets.py && python3 /work/g1_motion/fetch_assets.py --check
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

CMD ["/bin/bash", "-c", "if command -v nvpmodel >/dev/null 2>&1; then nvpmodel -m 0 2>/dev/null; jetson_clocks 2>/dev/null; echo '[init] Jetson MAXN + clocks locked'; fi && source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-eth0}"]

# ── IK：pinocchio ───────────────────────────────────────────────────────────
#
# `servo_eef` 解逆运动学要它。**加在文件末尾**是有意的：在开头插一个 ARG 或一条
# RUN 会让它后面的每一层都失效，整个镜像重建一遍。
#
# 约 200 MB。只有 servo_eef 用得到，而那张卡默认关闭 —— 但装它而不是让 import 在
# 运行时失败：一张声明了 eef_pose 却解不了 IK 的卡片，会收下位姿然后什么都不做，
# 而画布上它看起来是在跑的。缺了它 `ArmChain` 会拒绝启动并写出包名。
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} pin
