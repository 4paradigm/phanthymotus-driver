ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

# 系统依赖: portaudio (sounddevice/pyaudio), ffmpeg (音频处理)
RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl \
        libportaudio2 \
        portaudio19-dev \
        ffmpeg && \
    rm -rf /var/lib/apt/lists/*

# Python依赖: sounddevice替代pyaudio(手册用sounddevice直访声卡)
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml numpy opencv-python-headless sounddevice requests

# Build Q5 custom ROS2 message packages (q5_msgs)
COPY msgs/ /q5_ws/src/
RUN cd /q5_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select q5_msgs || \
    echo "WARNING: q5_msgs build failed, will run in stub mode"

# 手册要求: cyclonedds + domain 211
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=211
ENV PYTHONUNBUFFERED=1

# Ensure Q5 msg packages are on PYTHONPATH
ENV PYTHONPATH=/q5_ws/install/q5_msgs/local/lib/python3.10/dist-packages:${PYTHONPATH}
ENV AMENT_PREFIX_PATH=/q5_ws/install/q5_msgs:${AMENT_PREFIX_PATH}
ENV LD_LIBRARY_PATH=/q5_ws/install/q5_msgs/lib:${LD_LIBRARY_PATH}

# 运行时文件
COPY resource/ /work/resource/
COPY main.py device.py q5_pub_client.py config.yaml driver.yaml /work/

EXPOSE 15708

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && (source /q5_ws/install/setup.bash 2>/dev/null || true) && python3 /work/main.py"]