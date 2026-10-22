ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
WORKDIR /work
USER root
RUN set -eu; missing=""; \
    for package in python3-pip ca-certificates; do \
        if [ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != "install ok installed" ]; then \
            missing="$missing $package"; \
        fi; \
    done; \
    if [ -n "$missing" ]; then \
        apt-get update && apt-get install -y --no-install-recommends $missing && rm -rf /var/lib/apt/lists/*; \
    fi
COPY requirements.txt /tmp/tron2-teleop-requirements.txt
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/tron2-teleop-requirements.txt
COPY common/logsafe.py common/teleop_contract.py /work/common/
COPY common/motion/__init__.py common/motion/worker.py /work/common/motion/
COPY main.py model.py mapping.py kinematics.py vendor.py control.py bus.py config.yaml driver.yaml /work/limx/tron2_teleop/
COPY resource/ /work/limx/tron2_teleop/resource/
COPY deploy/service.yml /deploy/service.yml
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/work
ENV RCUTILS_COLORIZED_OUTPUT=0 ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOG_DIR=/tmp/ros-log
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && python3 -c 'from limx.tron2_teleop import main, kinematics; import rclpy; from std_msgs.msg import String'"
EXPOSE 15794
STOPSIGNAL SIGTERM
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 -m limx.tron2_teleop.main"]
