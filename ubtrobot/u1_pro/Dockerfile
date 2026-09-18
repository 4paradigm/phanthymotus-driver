ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

WORKDIR /work
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/u1-requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/u1-requirements.txt && rm /tmp/u1-requirements.txt
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONUNBUFFERED=1

EXPOSE 15723
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/main.py"]
