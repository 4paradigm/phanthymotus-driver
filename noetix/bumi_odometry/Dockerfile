# Build --target dependencies once; reuse its registry digest as FUSION_BASE_IMAGE.
ARG FUSION_BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${FUSION_BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-c"]
RUN if [ ! -f /opt/bumi-fusion-packages.txt ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rtabmap-odom ros-humble-robot-localization \
        ros-humble-tf2-ros ros-humble-message-filters \
        ros-humble-rclpy ros-humble-sensor-msgs ros-humble-nav-msgs \
        python3-opencv python3-numpy python3-yaml && \
      dpkg-query -W > /opt/bumi-fusion-packages.txt && \
      rm -rf /var/lib/apt/lists/*; fi
RUN source /opt/ros/humble/setup.bash && python3 -c \
    'import rclpy, cv2, numpy, yaml, message_filters; from rtabmap_msgs.msg import OdomInfo; from nav_msgs.msg import Odometry; from tf2_ros import StaticTransformBroadcaster' && \
    test -x /opt/ros/humble/lib/rtabmap_odom/rgbd_odometry && \
    test -x /opt/ros/humble/lib/robot_localization/ekf_node

FROM dependencies AS runtime
WORKDIR /work
COPY common/ /work/common/
COPY pipeline.py runtime.py service.py ros_probe.py timing.py bag_export.py /work/
ENV ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONPATH=/work \
    PYTHONUNBUFFERED=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 RCUTILS_COLORIZED_OUTPUT=0
EXPOSE 15714
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /work/service.py"]

# Optional integration/recording image; production cards do not inherit bag tools.
FROM runtime AS validation
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros2bag ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins && \
    rm -rf /var/lib/apt/lists/*

FROM runtime AS release
