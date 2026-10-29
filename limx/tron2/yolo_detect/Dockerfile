ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
USER root
WORKDIR /app
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip ca-certificates && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /app/
RUN python3 -m pip install --no-cache-dir -r requirements.txt
COPY engine.py server.py probe.py smoke_dds.py /app/
COPY tests/ /app/tests/
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR=/tmp/ros-log
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -c "import engine, server, rclpy" && YOLO_HTTP_TEST=1 python3 -m unittest discover -s tests -v'
USER 65532:65532
EXPOSE 15792
STOPSIGNAL SIGTERM
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /app/server.py"]
