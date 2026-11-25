ARG L20_DRIVER_IMAGE
FROM ${L20_DRIVER_IMAGE}
WORKDIR /opt/g20-simulation
COPY wheels/ /tmp/g20-wheels/
RUN pip3 install --no-index --find-links=/tmp/g20-wheels --target=/opt/g20-python mujoco==3.2.7 numpy==1.26.4 Pillow==11.3.0 && rm -r /tmp/g20-wheels
COPY vendor.zip simulation.py trajectories.py playback.py hand_model.py sim_bus.py workbench.html sim-launch dds-udp.xml ./
ENV PYTHONPATH=/opt/g20-python:/work MUJOCO_GL=disable PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=42 ROS_HOME=/tmp/ros ROS_LOG_DIR=/tmp/ros/log FASTRTPS_DEFAULT_PROFILES_FILE=/opt/g20-simulation/dds-udp.xml
EXPOSE 15769
ENTRYPOINT ["/bin/bash","/opt/g20-simulation/sim-launch"]
