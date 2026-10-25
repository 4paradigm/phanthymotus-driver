ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

COPY requirements.txt /work/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /work/requirements.txt

COPY common/           /work/common/
COPY main.py           /work/main.py
COPY can_client.py     /work/can_client.py
COPY hand_device.py    /work/hand_device.py
COPY gesture_device.py /work/gesture_device.py
COPY grasp_device.py   /work/grasp_device.py
COPY tactile_device.py /work/tactile_device.py
COPY tactile_heatmap.py /work/tactile_heatmap.py
COPY config.yaml       /work/config.yaml
COPY deploy/           /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15734

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
