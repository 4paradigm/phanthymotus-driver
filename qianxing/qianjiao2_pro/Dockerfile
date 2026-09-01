ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
ARG UBUNTU_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
WORKDIR /work
RUN grep -RIl 'mirrors.tencentyun.com/ubuntu-ports' /etc/apt 2>/dev/null | \
    xargs -r sed -i "s#http://mirrors.tencentyun.com/ubuntu-ports#${UBUNTU_MIRROR}#g" && \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-pip ffmpeg ros-humble-rclpy ros-humble-sensor-msgs ros-humble-std-msgs && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir -i "${PYPI_MIRROR}" 'pymavlink>=2.4.41,<3' 'PyYAML>=6'
COPY main.py device.py config.yaml driver.yaml ./
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work
ENV PYTHONPATH=/work:/opt/ros/humble/lib/python3.10/site-packages:/usr/lib/python3/dist-packages:/usr/local/lib/python3.10/dist-packages
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0
EXPOSE 15719
CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && exec /usr/bin/python3 main.py"]
