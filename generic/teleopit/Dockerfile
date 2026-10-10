ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

WORKDIR /work
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-pip python3-venv libosmesa6 libgl1 \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir PyYAML==6.0.3

COPY common/ /work/common/
COPY main.py plugin.py manager.py worker.py backend.py upstream.py /work/
COPY setup_teleopit.py requirements*.txt prepare_runtime.sh /work/
COPY config.yaml driver.yaml /work/
COPY deploy/ /deploy/

# Source, checkpoints and sample data are intentionally not redistributed in the
# image. An operator explicitly provisions this persistent runtime after reviewing
# third-party terms. The MCP server remains usable for preflight before setup.
ENV PYTHONPATH=/work \
    PYTHONUNBUFFERED=1 \
    TELEOPIT_ROOT=/opt/teleopit-runtime/source \
    TELEOPIT_PYTHON=/opt/teleopit-runtime/venv/bin/python \
    MUJOCO_GL=osmesa \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EXPOSE 15719
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/main.py"]
