ARG ROS_BASE_IMAGE=ros:humble-ros-base
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

ENV ROS_DOMAIN_ID=211
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
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
COPY base_drive.py /work/base_drive.py
COPY joints_state.py /work/joints_state.py
COPY joints.py /work/joints.py
COPY robot_ready.py /work/robot_ready.py
COPY battery.py /work/battery.py
COPY system_health.py /work/system_health.py
COPY hand_state.py /work/hand_state.py
COPY estop.py /work/estop.py
COPY diagnostics.py /work/diagnostics.py
COPY simple_action.py /work/simple_action.py
COPY body_command.py /work/body_command.py
COPY arm_control.py /work/arm_control.py
COPY hand_command.py /work/hand_command.py
COPY hand_control.py /work/hand_control.py
COPY hand_gesture.py /work/hand_gesture.py
COPY head_control.py /work/head_control.py
COPY joint_limits.py /work/joint_limits.py
COPY config.yaml /work/config.yaml
COPY resource/q5_model.urdf /work/resource/q5_model.urdf
COPY resource/fastdds_udp_only.xml /work/resource/fastdds_udp_only.xml
COPY deploy/ /deploy/

RUN chmod +x /work/q5_bundle_entrypoint.sh

EXPOSE 15793

CMD ["bash", "/work/q5_bundle_entrypoint.sh"]
