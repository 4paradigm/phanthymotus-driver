ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple
WORKDIR /driver
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} 'opencv-python-headless>=4.8,<5' 'PyYAML>=6,<7'
COPY common/__init__.py common/vendor_runtime.py /driver/common/
COPY main.py tracking.py config.yaml driver.yaml /driver/
COPY deploy/ /deploy/
ENV PYTHONPATH=/driver PYTHONUNBUFFERED=1 ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /driver/main.py"]
