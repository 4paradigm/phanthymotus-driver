ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
WORKDIR /work
USER root
RUN set -eu; missing=""; \
    for package in python3-pip ca-certificates; do \
        if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
            missing="$missing $package"; \
        fi; \
    done; \
    if [ -n "$missing" ]; then \
        apt-get update && apt-get install -y --no-install-recommends $missing && rm -rf /var/lib/apt/lists/*; \
    fi
COPY requirements.txt /tmp/tron2-requirements.txt
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/tron2-requirements.txt
COPY common/ /work/common/
COPY main.py device.py vendor.py state.py reset.py skeleton.py completion.py pose.py head.py gripper.py camera.py ros2.py check.py config.yaml driver.yaml /work/
COPY resource/ /work/resource/
COPY deploy/ /deploy/
ENV PYTHONUNBUFFERED=1
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONPATH=/work
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOG_DIR=/tmp/ros-log
# Import from the actual /work image layout; no robot or DDS participant is opened.
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && python3 -c 'import main, device, vendor, state, ros2, rclpy; from std_msgs.msg import String; from sensor_msgs.msg import CompressedImage'"
EXPOSE 15791
STOPSIGNAL SIGTERM
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /work/main.py"]
