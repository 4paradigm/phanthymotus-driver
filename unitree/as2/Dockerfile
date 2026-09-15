ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
WORKDIR /work
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir pyyaml
COPY unitree_sdk2py/ /work/unitree_sdk2py/
COPY main.py device.py lidar.py rpc_proxy.py config.yaml /work/
COPY resource/ /work/resource/
COPY common/ /work/common/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1
EXPOSE 15704
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-eth0}"]
