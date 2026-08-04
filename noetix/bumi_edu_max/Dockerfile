ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

ARG BUMI_SDK_REV=052ea95bf2d503cc2174b68a015b374591d3d54e
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN apt-get update && apt-get install -y --no-install-recommends git cmake g++ python3-dev python3-pip pybind11-dev libeigen3-dev ros-humble-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} pyyaml numpy
RUN git clone https://github.com/Noetix-Robotics/noetix_sdk_bumi.git /opt/noetix-bumi-sdk && \
    git -C /opt/noetix-bumi-sdk checkout ${BUMI_SDK_REV} && \
    cd /opt/noetix-bumi-sdk && ./build.sh && rm -rf .git

WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py config.yaml driver.yaml /work/
ENV BUMI_SDK_PATH=/opt/noetix-bumi-sdk RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work:/opt/noetix-bumi-sdk/build
EXPOSE 15713
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/main.py"]
