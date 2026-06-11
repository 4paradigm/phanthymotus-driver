ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    aiohttp pyyaml

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

COPY main.py    /work/main.py
COPY plugins.py /work/plugins.py
COPY config.yaml /work/config.yaml

ENV PYTHONPATH=/work

EXPOSE 15710 15711

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
