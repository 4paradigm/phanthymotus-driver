ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
WORKDIR /work
ARG PYPI_MIRROR
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip cmake build-essential curl && rm -rf /var/lib/apt/lists/*
ARG CYCLONEDDS_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/cyclonedds-0.10.5.tar.gz
RUN curl -fsSL ${CYCLONEDDS_URL} | tar xz -C /tmp && \
    cmake -S /tmp/cyclonedds-0.10.5 -B /tmp/cyclonedds-0.10.5/build -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds && \
    cmake --build /tmp/cyclonedds-0.10.5/build --parallel --target install && \
    rm -rf /tmp/cyclonedds-0.10.5
ENV CYCLONEDDS_HOME=/opt/cyclonedds
ENV LD_LIBRARY_PATH=/opt/cyclonedds/lib:${LD_LIBRARY_PATH}
COPY requirements.txt /work/requirements.txt
# Keep the ROS base intact after installing CycloneDDS; purging the build
# packages also purges ros-humble-rclpy under this image's APT dependency graph.
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r requirements.txt && \
    pip3 install --no-cache-dir --no-binary cyclonedds -i ${PYPI_MIRROR} cyclonedds==0.10.5
COPY unitree_sdk2py/ /work/unitree_sdk2py/
COPY main.py device.py lidar.py controlled_spatial.py rpc_proxy.py config.yaml /work/
COPY resource/ /work/resource/
COPY common/ /work/common/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    RCUTILS_COLORIZED_OUTPUT=0 \
    CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity><OutputFile>/dev/null</OutputFile></Tracing></Domain></CycloneDDS>'
EXPOSE 15709
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-}"]
