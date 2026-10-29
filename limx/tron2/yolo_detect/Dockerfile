# BASE_IMAGE must already contain the field-verified ROS2/Pillow runtime.
ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS verify
USER root
WORKDIR /opt/tron2-yolo-card
COPY engine.py server.py probe.py smoke_dds.py Dockerfile ./
COPY tests/ ./tests/
ENV PYTHONDONTWRITEBYTECODE=1
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -c "import PIL, rclpy" && YOLO_HTTP_TEST=1 python3 -m unittest discover -s tests -v'

FROM ${BASE_IMAGE} AS runtime
WORKDIR /opt/tron2-yolo-card
COPY --from=verify /opt/tron2-yolo-card/engine.py /opt/tron2-yolo-card/server.py /opt/tron2-yolo-card/probe.py /opt/tron2-yolo-card/smoke_dds.py ./
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR=/tmp/ros-log
USER 65532:65532
EXPOSE 15792
STOPSIGNAL SIGTERM
ENTRYPOINT []
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /opt/tron2-yolo-card/server.py"]
