ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} PyYAML==6.0.2 pyserial==3.5

COPY common/       /work/common/
COPY main.py       /work/main.py
COPY modbus_client.py /work/modbus_client.py
COPY modbus_device.py /work/modbus_device.py
COPY config.yaml   /work/config.yaml
COPY deploy/       /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15723

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
