ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG UBUNTU_PORTS_MIRROR=

WORKDIR /work

RUN if [ -n "$UBUNTU_PORTS_MIRROR" ]; then \
      UBUNTU_PORTS_MIRROR="$UBUNTU_PORTS_MIRROR" python3 -c 'import os; from pathlib import Path; p=Path("/etc/apt/sources.list"); s=p.read_text(); p.write_text(s.replace("http://mirrors.tencentyun.com/ubuntu-ports", os.environ["UBUNTU_PORTS_MIRROR"]))'; \
    fi && apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl \
        espeak-ng \
        alsa-utils \
        libasound2-dev \
        sshpass \
        openssh-client && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml numpy opencv-python-headless phonemizer pyalsaaudio sounddevice

COPY tianyi_motion/requirements.lock /tmp/tianyi-motion-requirements.lock
RUN pip3 install --no-cache-dir --require-hashes --only-binary=:all: \
    -i ${PYPI_MIRROR} -r /tmp/tianyi-motion-requirements.lock

# audio_msgs is inherited from ros-base's /ros_ws overlay, not rebuilt here.
COPY msgs/ /tianyi_ws/src/
COPY deploy/cmake_failure_diagnostics.py /tmp/tianyi-cmake-failure.py
COPY deploy/cmake_readdir_errno.c /tmp/tianyi-cmake-errno.c
RUN cd /tianyi_ws && \
    . /opt/ros/humble/setup.sh && \
    . /ros_ws/install/setup.sh && \
    (trap 'rm -f /tmp/tianyi-cmake-errno.so /tmp/tianyi-cmake-errno.c' EXIT; \
     cc -shared -fPIC -Wall -Wextra -Werror /tmp/tianyi-cmake-errno.c -ldl -o /tmp/tianyi-cmake-errno.so && \
     LD_PRELOAD=/tmp/tianyi-cmake-errno.so colcon build --packages-select bodyctrl_msgs lyre_msgs; \
     build_status=$?; \
     if [ "$build_status" -ne 0 ]; then \
       python3 /tmp/tianyi-cmake-failure.py || true; \
     fi; \
     exit "$build_status")

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONUNBUFFERED=1
ENV RCUTILS_COLORIZED_OUTPUT=0

# Ensure tianyi msg packages are on PYTHONPATH (base image entrypoint won't source them)
ENV PYTHONPATH=/tianyi_ws/install/lyre_msgs/local/lib/python3.10/dist-packages:/tianyi_ws/install/bodyctrl_msgs/local/lib/python3.10/dist-packages:${PYTHONPATH}
ENV AMENT_PREFIX_PATH=/tianyi_ws/install/bodyctrl_msgs:/tianyi_ws/install/lyre_msgs:${AMENT_PREFIX_PATH}
ENV LD_LIBRARY_PATH=/tianyi_ws/install/bodyctrl_msgs/lib:/tianyi_ws/install/lyre_msgs/lib:${LD_LIBRARY_PATH}

COPY resource/ /work/resource/
COPY common/ /work/common/
COPY main.py device.py servo.py joints_bridge.py light.py nav_client.py ext_devices.py config.yaml driver.yaml controlled_spatial.py controlled_spatial_map.py socket_bridge.py bridge_integration.py bridged_publisher.py /work/
COPY motion_stream.py teleop_executor.py motion_control.py dds-local.xml /work/
COPY tianyi_motion/ /work/tianyi_motion/
COPY dds-local.xml /opt/phanthy-motus/
COPY deploy/     /deploy/
RUN bash -c 'source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /tianyi_ws/install/setup.bash && python3 -c "from audio_msgs.msg import AudioChunk; from bodyctrl_msgs.msg import MotorStatusMsg, PowerBoardKeyStatus, CmdSetMotorPosition; from teleop_executor import TeleopExecutor; from motion_control import MotionControl; import pinocchio, scipy; from main import TianyiDeviceBundle"'
RUN mkdir -p /opt/phanthy-motus/data/maps

EXPOSE 15707

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && source /tianyi_ws/install/setup.bash && python3 /work/main.py"]
