ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

WORKDIR /work
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions cmake build-essential \
    ros-humble-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/u1-requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/u1-requirements.txt && rm /tmp/u1-requirements.txt
WORKDIR /u1_ws
COPY vendor/ /u1_ws/src/
RUN . /opt/ros/humble/setup.sh && colcon build --merge-install
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/u1_ws/install/audio_msgs/lib/python3.10/site-packages:/u1_ws/install/uworld_action_msgs/lib/python3.10/site-packages:/u1_ws/install/coze_msgs/lib/python3.10/site-packages:/work

EXPOSE 15723
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /u1_ws/install/setup.bash && python3 /work/main.py"]
