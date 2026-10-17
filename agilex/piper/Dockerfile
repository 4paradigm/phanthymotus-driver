# Reuse the project's verified ARM64 ROS/Python dependencies by immutable digest.
# build-local.sh selects the pinned official multiarch base for amd64 builds.
ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG PIPER_SDK_URL=https://github.com/agilexrobotics/piper_sdk/archive/8d4dc29bd6e753066dedee235c008e90ecb2ab4a.zip
ARG PIPER_SDK_SHA256=89dfa296331abc04b8d7ca5ebbc4dbe21bed44e521e85e8c4716a81e41305c29
# The project base already contains these packages. Only an explicit alternate
# base (for example amd64) needs to install missing packages.
RUN missing=""; \
    for package in python3-pip ros-humble-rmw-fastrtps-cpp ros-humble-sensor-msgs; do \
        dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -qx 'install ok installed' \
            || missing="$missing $package"; \
    done; \
    if [ -n "$missing" ]; then \
        apt-get update && apt-get install -y --no-install-recommends $missing \
        && rm -rf /var/lib/apt/lists/*; \
    fi
WORKDIR /work
COPY requirements.txt /work/agilex/piper/requirements.txt
# COS publication/access dependency and exact pinned artifact: SDK_ARTIFACT.md.
# The SDK URL can be replaced by an approved COS artifact without editing code.
# pip verifies its hash before installation and removes its temporary download;
# --no-cache-dir prevents the ZIP/wheel cache from becoming an image layer.
RUN sed '/^piper-sdk @ /d' /work/agilex/piper/requirements.txt > /tmp/piper-dependencies.txt \
    && pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/piper-dependencies.txt \
       "${PIPER_SDK_URL}#sha256=${PIPER_SDK_SHA256}" \
    && rm /tmp/piper-dependencies.txt
COPY common/ /work/common/
COPY main.py device.py runtime.py cards.py arm.py arm_control.py control.py mac_can.py camera_capture.py config.yaml driver.yaml /work/agilex/piper/
COPY monitor.py monitor_bundle.py /work/agilex/piper/
COPY status_card.py direct_control.py /work/agilex/piper/
COPY intent_control.py intent_motion.py intent_monitor.py intent_teaching.py /work/agilex/piper/
COPY unified_control.py user_motion.py path_capability.py preset.py /work/agilex/piper/
COPY completion.py /work/agilex/piper/
COPY camera_live.py camera_worker.py camera_owner.py /work/agilex/piper/
COPY resource/ /work/agilex/piper/resource/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PIPER_MOTION_ENABLED=0 PIPER_CAMERA_ENABLED=0
EXPOSE 15741
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/agilex/piper/main.py"]
