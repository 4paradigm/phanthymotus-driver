# 众擎 T800 开发版 driver 镜像（ARM64，部署于 T800 应用计算单元 Jetson Orin NX）。
#
# 构建上下文约定（与 unitree/g1 一致，driver 目录为上下文；build.sh 自动处理）：
#   - 本目录（engineai/t800/）下的代码
#   - interface_protocol/（官方消息源码随 driver 入库，构建容器内编译 interface_protocol 消息包；
#     若编译失败 colcon build 会打印 WARNING，driver 以 stub 模式运行——订阅将降级）
#
# 双 domain 环境：
#   - 容器默认 env 为 ctx_t800 用（domain 69 + rmw_cyclonedds_cpp + ROS_LOCALHOST_ONLY=0）
#   - ctx_core 由 driver 进程内临时切换回 domain 42 + rmw_fastrtps_cpp

ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR

WORKDIR /work

RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip \
        python3-colcon-common-extensions \
        cmake \
        build-essential \
        curl \
        git \
        libportaudio2 \
        libasound2-dev && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} \
    pyyaml numpy sounddevice

# ── rmw_cyclonedds_cpp（ctx_t800 需要）──────────────────────────────────────
# 优先从 apt 源安装 ros-humble-rmw-cyclonedds-cpp；
# 若镜像 apt 源没有该包，则从源码构建 cyclonedds + rmw_cyclonedds_cpp 到 /ros2_ws。
RUN (apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp 2>/dev/null || true) && \
    bash -c 'if ! dpkg -s ros-humble-rmw-cyclonedds-cpp >/dev/null 2>&1; then \
        echo "[cyclonedds] apt 无 ros-humble-rmw-cyclonedds-cpp，从源码构建"; \
        mkdir -p /tmp/cyclone_src && cd /tmp/cyclone_src && \
        git clone --depth 1 -b 0.10.5 https://github.com/eclipse-cyclonedds/cyclonedds.git && \
        cd cyclonedds && mkdir -p build && cd build && \
        cmake .. -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -DBUILD_TESTING=OFF && \
        cmake --build . --target install --parallel && \
        cd /tmp/cyclone_src && \
        git clone --depth 1 -b humble https://github.com/ros2/rmw_cyclonedds_cpp.git && \
        mkdir -p /ros2_ws/src && mv rmw_cyclonedds_cpp /ros2_ws/src/ && \
        cd /ros2_ws && . /opt/ros/humble/setup.sh && \
        export CYCLONEDDS_HOME=/opt/cyclonedds && \
        colcon build --packages-select rmw_cyclonedds_cpp || true; \
    fi'

# ── 官方消息包（interface_protocol）──────────────────────────────────────────
# 源码随 driver 入库（interface_protocol/，取自官方 engineai_ros2_workspace 的
# src/interface_protocol）；interface_example 仅为示例脚本且 C++ 侧依赖
# engineai_robotics_third_party，不参与编译，driver 只依赖 interface_protocol 的 msg 定义。
COPY interface_protocol/ /ros2_ws/src/interface_protocol
RUN cd /ros2_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select interface_protocol || \
    echo "WARNING: interface_protocol build failed, will run in stub mode"

# ── 双域环境（容器默认给 ctx_t800 用；ctx_core 在进程内临时切回 42+fastrtps）──
ENV ROS_DOMAIN_ID=69
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1

# 确保容器内可直接 import interface_protocol 消息包（base 镜像 entrypoint 不会 source 工作空间）
ENV PYTHONPATH=/ros2_ws/install/interface_protocol/local/lib/python3.10/dist-packages:${PYTHONPATH}
ENV AMENT_PREFIX_PATH=/ros2_ws/install/interface_protocol:${AMENT_PREFIX_PATH}
ENV LD_LIBRARY_PATH=/ros2_ws/install/interface_protocol/lib:${LD_LIBRARY_PATH}

COPY main.py device.py ros2.py config.yaml driver.yaml /work/
COPY plugins/  /work/plugins/
COPY deploy/   /deploy/

EXPOSE 15708

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && (source /ros2_ws/install/setup.bash 2>/dev/null || true) && python3 /work/main.py"]
