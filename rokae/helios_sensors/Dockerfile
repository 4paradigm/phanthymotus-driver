ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
WORKDIR /driver
COPY requirements.txt /driver/rokae/helios_sensors/requirements.txt
RUN python3 -m pip install --no-cache-dir --only-binary=:all: --no-deps \
    -r /driver/rokae/helios_sensors/requirements.txt
COPY common/__init__.py common/camera_info.py common/vendor_runtime.py common/logsafe.py /driver/common/
COPY __init__.py main.py orbbec.py orientation.py depth_socket.py depth_preview.py driver.yaml /driver/rokae/helios_sensors/
COPY deploy/ /deploy/
ENV PYTHONPATH=/driver PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 RCUTILS_COLORIZED_OUTPUT=0 \
    ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH=/opt/orbbec-sdk
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -c "from rokae.helios_sensors import main; from sensor_msgs.msg import CompressedImage; from std_msgs.msg import String"'
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 -m rokae.helios_sensors.main"]
