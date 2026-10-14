# Official ROS image supports amd64 (tested controller) and arm64.
ARG ROS_BASE_IMAGE=ros:humble-ros-base@sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG PIPER_SDK_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/piper-sdk-8d4dc29bd6e753066dedee235c008e90ecb2ab4a.zip
ARG PIPER_SDK_SHA256=89dfa296331abc04b8d7ca5ebbc4dbe21bed44e521e85e8c4716a81e41305c29
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip ros-humble-rmw-fastrtps-cpp ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY requirements.txt /work/agilex/piper/requirements.txt
# COS mirrors the unchanged official SDK archive; its license stays bundled.
# pip verifies its hash before installation and removes its temporary download;
# --no-cache-dir prevents the ZIP/wheel cache from becoming an image layer.
RUN sed '/^piper-sdk @ /d' /work/agilex/piper/requirements.txt > /tmp/piper-dependencies.txt \
    && pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/piper-dependencies.txt \
       "${PIPER_SDK_URL}#sha256=${PIPER_SDK_SHA256}" \
    && rm /tmp/piper-dependencies.txt
COPY common/ /work/common/
COPY main.py device.py monitor.py monitor_bundle.py status_card.py arm_control.py runtime.py cards.py arm.py control.py mac_can.py camera_capture.py camera_live.py camera_owner.py camera_worker.py dual.py paired_motion.py trajectory.py teleop_bus.py teaching.py config.yaml config.dual.example.yaml driver.yaml /work/agilex/piper/
COPY motion/ /work/agilex/piper/motion/
COPY resource/ /work/agilex/piper/resource/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PIPER_MOTION_ENABLED=0 PIPER_CAMERA_ENABLED=0
EXPOSE 15741
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/agilex/piper/main.py"]
