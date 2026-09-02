ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        curl && \
    rm -rf /var/lib/apt/lists/*

# boosteros pulls in booster-robotics-sdk-python (its low-level RPC/DDS
# binding) as a normal transitive pip dependency — no vendoring needed, unlike
# unitree_sdk2py. Pinned <1.1.3: newer releases require Python >=3.12, which
# the ROS humble base (Ubuntu 22.04, Python 3.10) does not have.
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    boosteros==1.1.1 pyyaml netifaces opencv-python-headless numpy

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT

COPY resource/ /work/resource/
COPY common/   /work/common/
COPY main.py   /work/main.py
COPY device.py /work/device.py
COPY config.yaml /work/config.yaml
COPY deploy/   /deploy/

ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1
# ANSI colour codes are control bytes; in a non-tty container log they are
# exactly the "garbage characters" that make `docker logs` unreadable.
ENV RCUTILS_COLORIZED_OUTPUT=0

EXPOSE 15705

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && python3 /work/main.py"]
