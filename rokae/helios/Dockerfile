# Build with driver files + common/ in the context (the build.sh convention).
# Use the established driver base; do not carry Core or its virtualenv.
ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
WORKDIR /driver
COPY requirements.txt /driver/rokae/helios/requirements.txt
ARG PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r /driver/rokae/helios/requirements.txt
COPY common/__init__.py common/vendor_runtime.py common/logsafe.py /driver/common/
COPY __init__.py main.py device.py model.py feedback.py battery.py chassis_battery_pb2.py spatial_map.py spatial_map_worker.py spatial_map_preview.py config.yaml driver.yaml /driver/rokae/helios/
COPY resource/ /driver/rokae/helios/resource/
COPY deploy/ /deploy/
ENV PYTHONPATH=/driver PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -c "import rclpy; from rokae.helios.battery import HeliosBattery; from rokae.helios.device import HeliosJoints"'
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 -m rokae.helios.main"]
