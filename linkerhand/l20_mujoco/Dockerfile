ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
WORKDIR /opt/g20-simulation

RUN pip3 install --no-cache-dir --index-url "${PYPI_MIRROR}" --target=/opt/g20-python \
    mujoco==3.2.7 numpy==1.26.4 Pillow==11.3.0 python-can==4.6.1
RUN python3 -c "from urllib.request import urlretrieve; urlretrieve('https://urdf-view.linkerbot.cn/api/v1/urdf-models/10/download', 'vendor.zip')" \
    && echo 'f5558bc0aa0d70503cd21611efc9e446592aac535dd68c9e61f71754004b2327  vendor.zip' | sha256sum -c -

COPY common/ /work/common/
COPY l20/config.yaml l20/can_client.py l20/hand_device.py l20/joints_model.py l20/gesture_device.py l20/grasp_feedback.py l20/adaptive_grasp_device.py /work/
COPY l20/resource/ /work/resource/
COPY l20/mujoco_card/*.py l20/mujoco_card/workbench.html l20/mujoco_card/dds-udp.xml l20/mujoco_card/sim-launch ./
COPY deploy/ /deploy/
ENV PYTHONPATH=/opt/g20-python:/work MUJOCO_GL=disable PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=42 ROS_HOME=/tmp/ros ROS_LOG_DIR=/tmp/ros/log FASTRTPS_DEFAULT_PROFILES_FILE=/opt/g20-simulation/dds-udp.xml
EXPOSE 15769
ENTRYPOINT ["/bin/bash", "/opt/g20-simulation/sim-launch"]
