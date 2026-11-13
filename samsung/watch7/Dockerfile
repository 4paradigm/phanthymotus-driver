ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
WORKDIR /work
# ROS Humble base already bundles rclpy, std_msgs and python3-yaml.
# Avoid adding pip and network resolution for a read-only two-card Driver.
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && python3 -c 'import yaml, rclpy; from std_msgs.msg import String'"
COPY common/ /work/common/
COPY main.py device.py protocol_v1.py protocol_v2.py transport.py sensors.py /work/
COPY config.yaml /work/config.yaml
COPY deploy/ /deploy/
ENV PYTHONUNBUFFERED=1
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONPATH=/work
EXPOSE 15750 15751
CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && exec python3 /work/main.py"]
