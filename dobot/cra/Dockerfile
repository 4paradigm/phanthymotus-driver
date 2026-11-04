ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/

WORKDIR /work
COPY build-requirements.txt requirements.txt /work/dobot/cra/
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r /work/dobot/cra/build-requirements.txt \
    && python3 -m pip install --no-cache-dir --no-build-isolation -i ${PYPI_MIRROR} -r /work/dobot/cra/requirements.txt \
    && python3 -c 'from dobot_sdk.protocol.data_types import MyType; assert MyType.itemsize == 1440'
COPY common/ /work/common/
COPY main.py device.py feedback.py config.yaml driver.yaml SDK.md /work/dobot/cra/
COPY deploy/ /deploy/

ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 \
    ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
EXPOSE 15746
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/dobot/cra/main.py"]
