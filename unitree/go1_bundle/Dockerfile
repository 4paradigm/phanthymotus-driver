ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
ARG PYPI_MIRROR=https://pypi.org/simple/
# 基础镜像里的 apt 源指向腾讯云内网 mirrors.tencentyun.com（Pi 解析不到）→ 换成可达源。
# 默认官方 ports.ubuntu.com；国内网慢可 --build-arg APT_MIRROR_HOST=mirrors.tuna.tsinghua.edu.cn
ARG APT_MIRROR_HOST=ports.ubuntu.com
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR
ARG APT_MIRROR_HOST

WORKDIR /work

# ① 把不可达的腾讯内网 apt 源替换成可达源；② 只装 SDK 构建真正需要的包
#    (unitree_legged_sdk python_wrapper 只需 cmake/g++/python3-dev + msgpack)；
#    ③ sshpass + openssh-client 供 deploy/nano_bootstrap.sh 首启时自动 SSH 到 Nano 部署 beep/camera 后端
#       （sshpass 只是包装器，真正连接要 openssh-client 提供的 ssh/scp；ROS base 默认不带，须显式装）。
RUN find /etc/apt \( -name '*.list' -o -name '*.sources' \) -exec sed -i "s|mirrors.tencentyun.com|${APT_MIRROR_HOST}|g" {} + ; \
    apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        python3-pip python3-dev cmake build-essential git libmsgpack-dev sshpass openssh-client && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /work/requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /work/requirements.txt

# ── unitree_legged_sdk (Go1 EDU) → robot_interface*.so ──────────────────────────
#   1) 根 CMakeLists 里 PYTHON_BUILD 默认 OFF → 必须 -DPYTHON_BUILD=ON 才编 python wrapper；
#   2) python_wrapper/python_interface.cpp #include <msgpack.hpp> → 需 libmsgpack-dev；
#   3) wrapper 用自带 third-party/pybind11，按容器 python 版本编出匹配的 .so；
#   4) 产物落在 lib/python/${ARCH}/robot_interface*.so。
# 构建失败不阻断镜像（运行期靠 STUB 兜底），但正常应成功。
ARG UNITREE_SDK_REPO=https://github.com/unitreerobotics/unitree_legged_sdk.git
ARG UNITREE_SDK_REF=go1
RUN git clone --depth 1 -b ${UNITREE_SDK_REF} ${UNITREE_SDK_REPO} /opt/unitree_legged_sdk && \
    cd /opt/unitree_legged_sdk && mkdir -p build && cd build && \
    cmake -DPYTHON_BUILD=ON .. && make -j"$(nproc)" && \
    find /opt/unitree_legged_sdk/lib/python -name 'robot_interface*.so' -exec cp -v {} /work/ \; || true

COPY main.py           /work/main.py
COPY go1_sdk_client.py /work/go1_sdk_client.py
COPY loco_state.py     /work/loco_state.py
COPY battery.py        /work/battery.py
COPY spin.py           /work/spin.py
COPY beep.py           /work/beep.py
COPY camera_rgb.py     /work/camera_rgb.py
COPY config.yaml       /work/config.yaml
COPY deploy/           /deploy/
# Nano 侧后端源码放进 /deploy/，供 nano_bootstrap.sh 首启时分发/编译到 Head Nano：
#   beep_adapter.py（beep，纯 Python 免编译）/ camera_adapter/（camera_rgb，Nano 上现编 C++）。
COPY beep_adapter.py   /deploy/beep_adapter.py
COPY camera_adapter/   /deploy/camera_adapter/
# ★ 新增卡：加一行 `COPY <卡名>.py /work/<卡名>.py`（main.py 会按 config 里的卡名自动 import）。

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_BUILTIN_TRANSPORTS=DEFAULT
ENV PYTHONPATH=/work
ENV PYTHONUNBUFFERED=1

EXPOSE 15717

# 首启时后台自动 provision Nano（部署 beep_adapter + 编译部署 camera_adapter），失败不阻塞驱动主进程；
# 随后前台起 MCP server。容器须 --network host 才够得到 Nano 内网（service.yml 已配）。
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && (bash /deploy/nano_bootstrap.sh || true) & python3 /work/main.py ${NETWORK_INTERFACE:-eth0}"]
