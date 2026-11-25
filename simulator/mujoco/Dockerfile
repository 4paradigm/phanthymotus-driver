ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
RUN apt-get update && apt-get install -y --no-install-recommends python3-yaml ros-humble-rmw-fastrtps-cpp && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY common/ /work/common/
RUN mkdir -p /work/simulator && printf '"""Simulator bundles."""\n' > /work/simulator/__init__.py
COPY __init__.py main.py plugin.py client.py workbench.html config.yaml driver.yaml /work/simulator/mujoco/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EXPOSE 15712
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/simulator/mujoco/main.py"]
