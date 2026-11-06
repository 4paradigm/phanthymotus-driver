# syntax=docker/dockerfile:1
ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE} AS sdk-builder
ARG PYPI_MIRROR=https://pypi.org/simple/
WORKDIR /build
COPY build-requirements.txt requirements.txt ./
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r build-requirements.txt \
    && python3 -m pip wheel --no-cache-dir --no-build-isolation -i ${PYPI_MIRROR} -r requirements.txt -w /build/wheels

FROM ${ROS_BASE_IMAGE}
WORKDIR /work
RUN --mount=type=bind,from=sdk-builder,source=/build/wheels,target=/tmp/dobot-wheels \
    python3 -m pip install --no-cache-dir --no-index /tmp/dobot-wheels/*.whl \
    && python3 -c 'from dobot_sdk.protocol.data_types import MyType; assert MyType.itemsize == 1440'
COPY common/ /work/common/
COPY main.py device.py feedback.py control.py completion.py model.py config.yaml driver.yaml /work/dobot/cra/
COPY resource/ /work/dobot/cra/resource/
COPY deploy/ /deploy/

ENV PYTHONPATH=/work PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 \
    ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
EXPOSE 15746
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/dobot/cra/main.py"]
