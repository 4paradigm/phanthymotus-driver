ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip ros-humble-rmw-fastrtps-cpp ros-humble-geometry-msgs \
        ros-humble-std-msgs && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /work/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /work/requirements.txt
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py rosbridge_client.py kuavo_sensors.py config.yaml driver.yaml /work/
COPY deploy/ /deploy/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42
EXPOSE 15747
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /work/main.py"]
