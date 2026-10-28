# Standalone read-only RGB-D visual odometry sidecar.
#
# Visual odometry only: this image runs RTAB-Map `rgbd_odometry` and nothing
# else estimates pose. There is no inertial input and no state estimator on top
# of the visual output, so the dependency list stops at what the vision node
# itself needs.
#
# Build --target dependencies once; reuse its registry digest as
# VISUAL_BASE_IMAGE. The installed package set is frozen into
# /opt/bumi-visual-packages.txt so a published image can be reproduced from the
# same apt resolution instead of a moving `ros:humble-ros-base`.
ARG VISUAL_BASE_IMAGE=ros:humble-ros-base-jammy
FROM ${VISUAL_BASE_IMAGE} AS dependencies
SHELL ["/bin/bash", "-c"]
RUN if [ ! -f /opt/bumi-visual-packages.txt ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rtabmap-odom ros-humble-rtabmap-msgs \
        ros-humble-tf2-ros ros-humble-message-filters \
        ros-humble-rclpy ros-humble-sensor-msgs ros-humble-nav-msgs \
        ros-humble-geometry-msgs \
        python3-opencv python3-numpy python3-yaml && \
      dpkg-query -W > /opt/bumi-visual-packages.txt && \
      rm -rf /var/lib/apt/lists/*; fi
# Only the RGB-D odometry nodelet is required and checked. If it is missing the
# image is useless, so fail the build here rather than at first capture.
RUN source /opt/ros/humble/setup.bash && python3 -c \
    'import rclpy, cv2, numpy, yaml, message_filters; from rtabmap_msgs.msg import OdomInfo; from nav_msgs.msg import Odometry; from geometry_msgs.msg import Pose, Transform; from tf2_ros import StaticTransformBroadcaster' && \
    test -x /opt/ros/humble/lib/rtabmap_odom/rgbd_odometry

FROM dependencies AS runtime
WORKDIR /work
COPY common/ /work/common/
COPY pipeline.py runtime.py service.py ros_probe.py /work/
# Agent Core extracts the compose fragment from the image at deploy time.
COPY deploy/ /deploy/
ENV ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONPATH=/work \
    PYTHONUNBUFFERED=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 RCUTILS_COLORIZED_OUTPUT=0
EXPOSE 15720
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /work/service.py"]

# Optional integration/recording image; production cards do not inherit bag tools.
FROM runtime AS validation
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros2bag ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins && \
    rm -rf /var/lib/apt/lists/*

FROM runtime AS release
