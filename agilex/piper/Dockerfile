# Official ROS image supports amd64 (tested controller) and arm64.
ARG ROS_BASE_IMAGE=ros:humble-ros-base
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip ros-humble-rmw-fastrtps-cpp ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY requirements.txt /work/agilex/piper/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /work/agilex/piper/requirements.txt
COPY common/ /work/common/
COPY main.py device.py arm.py camera_capture.py config.yaml driver.yaml /work/agilex/piper/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PIPER_MOTION_ENABLED=0
EXPOSE 15741
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/agilex/piper/main.py"]
