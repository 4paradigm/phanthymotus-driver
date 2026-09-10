ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

WORKDIR /work

RUN apt-get update && apt-get install -y --no-install-recommends python3-yaml && rm -rf /var/lib/apt/lists/*

COPY common/     /work/common/
COPY main.py     /work/main.py
COPY device.py   /work/device.py
COPY config.yaml /work/config.yaml
COPY deploy/     /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONPATH=/work

EXPOSE 15719

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
