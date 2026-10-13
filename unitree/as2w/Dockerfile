ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/

# Build the exact AudioChunk interface consumed by this component.  Keeping it
# in a dedicated stage makes audio_msgs an explicit AS2W dependency instead of
# relying on an undocumented /ros_ws overlay in the shared base image.
FROM ${ROS_BASE_IMAGE} AS audio-interface-builder
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-dev python3-colcon-common-extensions cmake build-essential \
    ros-humble-rosidl-default-generators ros-humble-std-msgs && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /audio_ws
COPY vendor/audio_msgs/ /audio_ws/src/audio_msgs/
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    colcon build --packages-select audio_msgs && \
    echo "Generated audio_msgs overlay size:" && du -sh /audio_ws/install'

FROM ${ROS_BASE_IMAGE}
WORKDIR /work
ARG PYPI_MIRROR
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip cmake build-essential curl && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
# FFmpeg is a runtime dependency of record_video, not a test-only tool. Measure
# its architecture-specific uncompressed filesystem delta while installing the
# smallest distro-supported package set; the build log makes the cost visible.
RUN set -eux; \
    before_bytes="$(du -sx -B1 / | cut -f1)"; \
    apt-get update; \
    apt-get install -y --no-install-recommends ffmpeg; \
    command -v ffmpeg; \
    command -v ffprobe; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/*; \
    after_bytes="$(du -sx -B1 / | cut -f1)"; \
    echo "FFmpeg runtime filesystem increase: $((after_bytes - before_bytes)) bytes"
ARG CYCLONEDDS_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/cyclonedds-0.10.5.tar.gz
ARG CYCLONEDDS_SHA256=ec3ec898c52b02f939a969cd1a276e219420e5e8419b21cea276db35b4821848
RUN curl --proto '=https' --tlsv1.2 -fsSL ${CYCLONEDDS_URL} -o /tmp/cyclonedds-0.10.5.tar.gz && \
    echo "${CYCLONEDDS_SHA256}  /tmp/cyclonedds-0.10.5.tar.gz" | sha256sum -c - && \
    tar xzf /tmp/cyclonedds-0.10.5.tar.gz -C /tmp && \
    cmake -S /tmp/cyclonedds-0.10.5 -B /tmp/cyclonedds-0.10.5/build -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds && \
    cmake --build /tmp/cyclonedds-0.10.5/build --parallel --target install && \
    rm -rf /tmp/cyclonedds-0.10.5 /tmp/cyclonedds-0.10.5.tar.gz
ENV CYCLONEDDS_HOME=/opt/cyclonedds
ENV LD_LIBRARY_PATH=/opt/cyclonedds/lib:${LD_LIBRARY_PATH}
COPY requirements.txt /work/requirements.txt
# Keep the ROS base intact after installing CycloneDDS; purging the build
# packages also purges ros-humble-rclpy under this image's APT dependency graph.
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r requirements.txt && \
    pip3 install --no-cache-dir --no-binary cyclonedds -i ${PYPI_MIRROR} cyclonedds==0.10.5
COPY --from=audio-interface-builder /audio_ws/install/ /as2w_ws/install/
# Validate the component-owned overlay before copying the application.  This
# catches an interface build or runtime-link mismatch during image creation.
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
    test -f /as2w_ws/install/setup.bash && \
    source /as2w_ws/install/setup.bash && \
    python3 -c "from audio_msgs.msg import AudioChunk" && \
    echo "Runtime audio_msgs overlay size:" && du -sh /as2w_ws/install'
COPY unitree_sdk2py/ /work/unitree_sdk2py/
COPY main.py device.py multimedia.py vision_capture.py lidar.py lidar_backend.py sensor_worker.py controlled_spatial.py slam_mapping.py rpc_proxy.py config.yaml /work/
COPY resource/ /work/resource/
COPY common/ /work/common/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    RCUTILS_COLORIZED_OUTPUT=0 \
    CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'
EXPOSE 15709
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /as2w_ws/install/setup.bash && exec python3 /work/main.py"]
