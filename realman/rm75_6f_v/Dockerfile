ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-yaml ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY common/ /work/common/
COPY vendor/Robotic_Arm/ /work/Robotic_Arm/
ARG RM_API2_LIB_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/realman-api2-1.1.6-linux-arm64-libapi_c.so
ARG RM_API2_LIB_SHA256=5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9
ADD ${RM_API2_LIB_URL} /tmp/libapi_c.so
RUN echo "${RM_API2_LIB_SHA256}  /tmp/libapi_c.so" | sha256sum -c - \
    && install -D -m 0755 /tmp/libapi_c.so /work/Robotic_Arm/libs/linux_arm/libapi_c.so \
    && rm /tmp/libapi_c.so
COPY main.py device.py config.yaml driver.yaml /work/realman/rm75_6f_v/
COPY resource/ /work/realman/rm75_6f_v/resource/
COPY deploy/ /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_LOCALHOST_ONLY=0 \
    PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    PYTHONPATH=/work \
    RM_DRIVER_ENABLED=0

EXPOSE 15718
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/realman/rm75_6f_v/main.py"]
