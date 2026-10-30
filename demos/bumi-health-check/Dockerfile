ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
WORKDIR /work
COPY health_service/ /work/health_service/
COPY config.json /work/config.json
ENV PYTHONUNBUFFERED=1
EXPOSE 15741
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 -m health_service.server"]
