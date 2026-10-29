ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

COPY requirements.txt /work/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /work/requirements.txt

COPY common/       /work/common/
COPY main.py       /work/main.py
COPY modbus_client.py /work/modbus_client.py
COPY modbus_device.py /work/modbus_device.py
COPY gesture_device.py /work/gesture_device.py
COPY glove_teleop_device.py /work/glove_teleop_device.py
COPY glove_direct_reader.py /work/glove_direct_reader.py
COPY glove_protocol.py /work/glove_protocol.py
COPY glove_skeleton.py /work/glove_skeleton.py
COPY check_runtime.py /work/check_runtime.py
COPY resource/ /work/resource/
COPY config.yaml   /work/config.yaml
COPY deploy/       /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15733

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
