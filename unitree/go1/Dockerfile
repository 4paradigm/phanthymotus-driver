# drivers/unitree/go1/Dockerfile — Go1 只读传感器卡驱动(loco_state / imu / feet)
#
# 仅 3 张只读状态卡,不含运动 → 无需 py3.7 运动子进程;py3.10 主环境跑 MCP + rclpy 数据面。
# 数据源后端:mt.backend=factory(狗上工厂高层 .so 读真实 HighState);Mac/无 .so 自动降 fake。

ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

# 依赖极简:配置解析用 pyyaml;rclpy 来自基础镜像
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} pyyaml

# ROS2 数据面:与 agent-core 对齐(FastDDS + domain 42)
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT

# ── 拷贝驱动代码(只含这 3 张卡所需) ────────────────────────────────────────
COPY resource/  /work/resource/
COPY plugins/   /work/plugins/
COPY main.py go1_ctrl.py go1_hl.py ros_bridge.py config.yaml driver.yaml requirements.txt /work/

ENV PYTHONUNBUFFERED=1
EXPOSE 15704

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /work/main.py ${NETWORK_INTERFACE:-eth0}"]
