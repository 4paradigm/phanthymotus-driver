ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

# Some ros-base revisions retain the ROS installation and signing key but not
# the ROS 2 apt source. Restore the signed source before installing the RMW.
RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    test -s /usr/share/keyrings/ros2-archive-keyring.gpg || \
        { echo "ERROR: base image is missing /usr/share/keyrings/ros2-archive-keyring.gpg"; exit 1; } && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-colcon-common-extensions \
        python3-pip \
        ros-humble-rmw-cyclonedds-cpp \
        pulseaudio-utils \
        build-essential \
        cmake && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/t800-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/t800-requirements.txt

COPY msgs/ /t800_ws/src/
RUN . /opt/ros/humble/setup.sh && \
    cd /t800_ws && \
    colcon build --packages-select interface_protocol

COPY main.py device.py control.py native_sdk.py virtual_gamepad.py config.yaml driver.yaml /work/
COPY resource/ /work/resource/
COPY deploy/ /deploy/

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15708

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /t800_ws/install/setup.bash && export CYCLONEDDS_URI=\"<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='${NETWORK_INTERFACE:-eth0}'/></Interfaces></General></Domain></CycloneDDS>\" && python3 /work/main.py"]
