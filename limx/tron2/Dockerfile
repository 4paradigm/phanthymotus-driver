ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
WORKDIR /work
USER root
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip ca-certificates && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/tron2-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/tron2-requirements.txt
COPY common/ /work/common/
COPY main.py device.py vendor.py state.py ros2.py check.py config.yaml driver.yaml /work/
COPY deploy/ /deploy/
ENV PYTHONUNBUFFERED=1
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONPATH=/work
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Import from the actual /work image layout; no robot or DDS participant is opened.
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && python3 -c 'import main, device, vendor, state, ros2, rclpy; from std_msgs.msg import String'"
EXPOSE 15791
STOPSIGNAL SIGTERM
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /work/main.py"]
