ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
WORKDIR /opt/g20-simulation

RUN pip3 install --no-cache-dir --index-url "${PYPI_MIRROR}" \
    mujoco==3.2.7 Pillow==11.3.0
ARG L20_MODEL_URL=https://urdf-view.linkerbot.cn/api/v1/urdf-models/10/download
RUN python3 -c "from urllib.request import urlretrieve; import sys; urlretrieve(sys.argv[1], 'vendor.zip')" "${L20_MODEL_URL}" \
    && echo 'f5558bc0aa0d70503cd21611efc9e446592aac535dd68c9e61f71754004b2327  vendor.zip' | sha256sum -c -

COPY common/ /work/common/
COPY l20/config.yaml l20/can_client.py l20/hand_device.py l20/joints_model.py l20/gesture_device.py l20/grasp_feedback.py l20/adaptive_grasp_device.py /work/
COPY l20/resource/LICENSE l20/resource/SOURCE.md /usr/share/doc/linkerhand-l20-mujoco/
COPY l20/mujoco_card/*.py l20/mujoco_card/workbench.html l20/mujoco_card/sim-launch ./
COPY deploy/ /deploy/
ENV PYTHONPATH=/work MUJOCO_GL=disable PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=42 ROS_HOME=/tmp/ros ROS_LOG_DIR=/tmp/ros/log FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml
EXPOSE 15769
ENTRYPOINT ["/bin/bash", "/opt/g20-simulation/sim-launch"]
