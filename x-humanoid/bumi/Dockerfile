ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        cmake \
        build-essential \
        curl && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/requirements.txt

# Bumi SDK (noetix_sdk_bumi) — 需在算力板上编译 pybind11 库
# 编译产物 highcontrol_py.so / lowcontrol_py.so / mediacontrol_py.so 放入 build/
COPY build/ /work/build/
COPY config/ /work/config/
COPY resource/ /work/resource/
COPY main.py device.py media_plugins.py config.yaml driver.yaml /work/

ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work/build:${PYTHONPATH}

EXPOSE 15709

CMD ["python3", "/work/main.py"]
