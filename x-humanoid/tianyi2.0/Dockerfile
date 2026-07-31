ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml numpy opencv-python-headless

# Build tianyi ROS2 message packages (bodyctrl_msgs + lyre_msgs)
COPY msgs/ /tianyi_ws/src/
RUN cd /tianyi_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select bodyctrl_msgs lyre_msgs || \
    echo "WARNING: msg build failed, will run in stub mode"

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV PYTHONUNBUFFERED=1

# Ensure tianyi msg packages are on PYTHONPATH (base image entrypoint won't source them)
ENV PYTHONPATH=/tianyi_ws/install/lyre_msgs/local/lib/python3.10/dist-packages:/tianyi_ws/install/bodyctrl_msgs/local/lib/python3.10/dist-packages:${PYTHONPATH}
ENV AMENT_PREFIX_PATH=/tianyi_ws/install/bodyctrl_msgs:/tianyi_ws/install/lyre_msgs:${AMENT_PREFIX_PATH}
ENV LD_LIBRARY_PATH=/tianyi_ws/install/bodyctrl_msgs/lib:/tianyi_ws/install/lyre_msgs/lib:${LD_LIBRARY_PATH}

COPY resource/ /work/resource/
COPY main.py device.py nav_client.py config.yaml driver.yaml controlled_spatial.py /work/
RUN mkdir -p /work/maps

EXPOSE 15707

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && (source /tianyi_ws/install/setup.bash 2>/dev/null || true) && python3 /work/main.py"]
