ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip ros-humble-rmw-fastrtps-cpp && rm -rf /var/lib/apt/lists/*
# The shared runtime loads config.yaml via PyYAML; keep this explicit because
# the ROS base image does not guarantee it.
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} pyyaml
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/realman/rm75_6f_v/
COPY resource/ /work/realman/rm75_6f_v/resource/
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 PYTHONPATH=/work
EXPOSE 15718
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && if [ -f /rm_ws/install/setup.bash ]; then source /rm_ws/install/setup.bash; fi && python3 /work/realman/rm75_6f_v/main.py"]
