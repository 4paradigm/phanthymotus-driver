ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base@sha256:82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

COPY requirements.txt /work/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /work/requirements.txt

ARG L20_MODEL_URL=https://urdf-view.linkerbot.cn/api/v1/urdf-models/10/download
RUN mkdir -p /work/mujoco_card \
    && python3 -c "from urllib.request import urlretrieve; import sys; urlretrieve(sys.argv[1], '/work/mujoco_card/vendor.zip')" "${L20_MODEL_URL}" \
    && echo 'f5558bc0aa0d70503cd21611efc9e446592aac535dd68c9e61f71754004b2327  /work/mujoco_card/vendor.zip' | sha256sum -c -

COPY mujoco_card/*.py mujoco_card/workbench.html mujoco_card/sim-launch /work/mujoco_card/
COPY common/           /work/common/
COPY main.py           /work/main.py
COPY can_client.py     /work/can_client.py
COPY qemu_compat.py    /work/qemu_compat.py
COPY hand_device.py    /work/hand_device.py
COPY gesture_device.py simulated_motion_device.py /work/
COPY tactile_device.py /work/tactile_device.py
COPY grasp_feedback.py /work/grasp_feedback.py
COPY adaptive_grasp_device.py    /work/adaptive_grasp_device.py
COPY joints_device.py  /work/joints_device.py
COPY joints_model.py   /work/joints_model.py
COPY resource/         /work/resource/
COPY config.yaml       /work/config.yaml
COPY deploy/           /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15734

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && exec python3 /work/main.py"]
