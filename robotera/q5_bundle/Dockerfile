ARG ROS_BASE_IMAGE=ros:humble-ros-base
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

ENV ROS_DOMAIN_ID=211
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-opencv \
    openssh-client \
    sshpass \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-geometry-msgs \
    ros-humble-lifecycle-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY requirements.txt /work/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r requirements.txt

COPY main.py /work/main.py
COPY q5_bus_bridge.py /work/q5_bus_bridge.py
COPY q5_bundle_entrypoint.sh /work/q5_bundle_entrypoint.sh
COPY sensor_contract.py /work/sensor_contract.py
COPY control_contract.py /work/control_contract.py
COPY q5_sdk_client.py /work/q5_sdk_client.py
COPY body_command.py /work/body_command.py
COPY arm_control.py /work/arm_control.py
COPY joint_limits.py /work/joint_limits.py
COPY mic.py speaker.py audio.py camera_rgb.py camera_depth.py q5_control_mode.py /work/
COPY legacy_device.py legacy_direct_control.py q5_media_bridge.py /work/
COPY config.yaml /work/config.yaml
COPY resource/q5_model.urdf /work/resource/q5_model.urdf
COPY resource/fastdds_udp_only.xml /work/resource/fastdds_udp_only.xml
COPY deploy/ /deploy/

RUN chmod +x /work/q5_bundle_entrypoint.sh

EXPOSE 15793

CMD ["bash", "/work/q5_bundle_entrypoint.sh"]
