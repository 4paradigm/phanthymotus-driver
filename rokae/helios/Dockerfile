# Build with driver files + common/ in the context (the build.sh convention).
# Use the established driver base; do not carry Core or its virtualenv.
ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
WORKDIR /driver
# The base image's apt source (mirrors.tencentyun.com) only resolves inside Tencent Cloud;
# same override as deep_robotics/s10_pro and chasing/qianjiao_p200_pro.
ARG UBUNTU_MIRROR=http://ports.ubuntu.com/ubuntu-ports
RUN find /etc/apt -type f \( -name '*.list' -o -name '*.sources' \) \
    -exec sed -i -e "s|https\?://mirrors.tencentyun.com/ubuntu-ports|${UBUNTU_MIRROR}|g" {} + \
    && apt-get -o Acquire::Retries=1 -o Acquire::http::Timeout=20 \
       -o Acquire::https::Timeout=20 -o APT::Update::Error-Mode=any update \
    && apt-get install -y --no-install-recommends \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /driver/rokae/helios/requirements.txt
ARG PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r /driver/rokae/helios/requirements.txt
COPY common/__init__.py common/vendor_runtime.py common/logsafe.py common/camera_info.py /driver/common/
COPY __init__.py main.py device.py model.py feedback.py battery.py camera.py camera_capture.py chassis_battery_pb2.py config.yaml driver.yaml /driver/rokae/helios/
COPY resource/ /driver/rokae/helios/resource/
COPY deploy/ /deploy/
ENV PYTHONPATH=/driver PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN gst-inspect-1.0 v4l2src >/dev/null && gst-inspect-1.0 jpegenc >/dev/null \
    && gst-inspect-1.0 videorate >/dev/null \
    && /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -c "import rclpy; from rokae.helios.battery import HeliosBattery; from rokae.helios.device import HeliosJoints; from rokae.helios.camera import HeliosCamera"'
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 -m rokae.helios.main"]
