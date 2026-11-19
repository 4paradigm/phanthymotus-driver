# Build with driver files + common/ in the context (the build.sh convention).
# The pinned base supplies ROS Humble and the Python runtime.
# The build worker needs registry access to this image.
ARG BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/core:release.261111.2b00f1d
FROM ${BASE_IMAGE}
WORKDIR /driver
COPY common/__init__.py common/vendor_runtime.py common/logsafe.py /driver/common/
COPY __init__.py main.py device.py model.py feedback.py battery.py chassis_battery_pb2.py config.yaml driver.yaml /driver/rokae/helios/
COPY resource/ /driver/rokae/helios/resource/
RUN /work/.venv/bin/python -c 'import aiohttp, yaml'
COPY requirements.txt /driver/rokae/helios/requirements.txt
ARG PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple
RUN /work/.venv/bin/python -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r /driver/rokae/helios/requirements.txt
COPY deploy/ /deploy/
RUN /work/.venv/bin/python -c 'from rokae.helios.battery import HeliosBattery; from rokae.helios.device import HeliosJoints'
ENV PYTHONPATH=/driver PYTHONUNBUFFERED=1 ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec /work/.venv/bin/python -m rokae.helios.main"]
