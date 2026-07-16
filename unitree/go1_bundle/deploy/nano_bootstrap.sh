#!/bin/bash
# nano_bootstrap.sh —— 容器首启时由 Dockerfile CMD 自动后台跑一次，把 camera_rgb / beep 的
# Nano 端自动布好。目标：Pi 上 clone → build → run 镜像后，无需任何手动步骤，
# 就能在 15678 上直接调用 beep / camera_rgb。
#
# 在容器里跑（容器须 --network host 才够得到 Nano 内网 192.168.123.x）：
#   bash /deploy/nano_bootstrap.sh
# CMD 里以后台方式点火：  bash /deploy/nano_bootstrap.sh & python3 /work/main.py
#
# 幂等：已装好（服务 active + 二进制在）则跳过重装；每次都确保占用自启被禁 + 设备当前空闲。
# 依赖：容器内有 sshpass；/deploy/ 下有 camera_adapter/camera_adapter.cpp、beep_adapter.py、
#       go1-camera-adapter.service、go1-beep-adapter.service、go1-beep-adapter.example.json。
# Nano 前置（本狗都已具备）：~/UnitreecameraSDK + g++ + opencv4（编 camera_adapter）；alsa-utils（beep 出声）。
set +e

NANO="${NANO_IP:-192.168.123.13}"
PW="${NANO_PW:-123}"
DEPLOY="${DEPLOY_DIR:-/deploy}"
SSH="sshpass -p $PW ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8"
SCP="sshpass -p $PW scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8"
R="unitree@$NANO"

log(){ echo "[nano_bootstrap] $*"; }

command -v sshpass >/dev/null 2>&1 || { log "✗ 容器内无 sshpass（Dockerfile 应 apt 装）；跳过 provision。"; exit 0; }

# 0) 可达性（连不上不阻塞主进程；主进程照常起，只是 beep/camera 暂不可用）
if ! $SSH $R 'echo ok' >/dev/null 2>&1; then
  log "✗ 连不上 Nano $NANO（容器需 --network host + Nano 在线 + ssh unitree/123）；跳过 provision。"
  exit 0
fi
log "Nano $NANO 可达，开始 provision camera_rgb + beep。"

# ── 1) camera_rgb 端：camera_adapter（front / /dev/video1 / 控制口 9301）─────────────
if $SSH $R '[ -x ~/camera_adapter/camera_adapter ]' 2>/dev/null; then
  log "camera_adapter 已在 Nano 上（跳过编译）。"
elif [ -f "$DEPLOY/camera_adapter/camera_adapter.cpp" ]; then
  log "在 Nano 上现编 camera_adapter（需 opencv4 + UnitreecameraSDK，静态库在 lib/arm64）…"
  $SSH $R 'mkdir -p ~/camera_adapter' 2>/dev/null
  $SCP "$DEPLOY/camera_adapter/camera_adapter.cpp" "$R:~/camera_adapter/camera_adapter.cpp" 2>/dev/null
  $SSH $R 'cd ~/camera_adapter; SDK=$HOME/UnitreecameraSDK; \
    pkg-config --exists opencv4 || { echo NO_OPENCV4; exit 1; }; \
    [ -d "$SDK/include" ] || { echo NO_SDK; exit 1; }; \
    g++ -O2 -std=c++14 -pthread camera_adapter.cpp -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 \
      -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \
      $(pkg-config --cflags --libs opencv4) -o camera_adapter && echo BUILD_OK' 2>&1 | tail -4
else
  log "✗ /deploy 下无 camera_adapter/camera_adapter.cpp → camera 端跳过。"
fi
# 装/启用 camera 服务（front）
if [ -f "$DEPLOY/go1-camera-adapter.service" ]; then
  $SCP "$DEPLOY/go1-camera-adapter.service" "$R:/tmp/go1-camera-adapter.service" 2>/dev/null
  $SSH $R "echo $PW | sudo -S cp /tmp/go1-camera-adapter.service /etc/systemd/system/go1-camera-adapter.service" 2>/dev/null
  $SSH $R "echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl enable --now go1-camera-adapter" 2>/dev/null
  log "camera 服务已装/启用（控制口 9301，JSON-TCP + JPEG-over-UDP）。"
fi

# ── 2) beep 端：beep_adapter（:18082 /v1/beep/actions，纯 Python 免编译）──────────────
if [ -f "$DEPLOY/beep_adapter.py" ]; then
  $SCP "$DEPLOY/beep_adapter.py" "$R:~/beep_adapter.py" 2>/dev/null
  # 首次没有配置就下发示例配置（audio_device=auto / mixer=Speaker）。
  if ! $SSH $R '[ -f ~/go1-beep-adapter.json ]' 2>/dev/null && [ -f "$DEPLOY/go1-beep-adapter.example.json" ]; then
    $SCP "$DEPLOY/go1-beep-adapter.example.json" "$R:~/go1-beep-adapter.json" 2>/dev/null
  fi
  if [ -f "$DEPLOY/go1-beep-adapter.service" ]; then
    $SCP "$DEPLOY/go1-beep-adapter.service" "$R:/tmp/go1-beep-adapter.service" 2>/dev/null
    $SSH $R "echo $PW | sudo -S cp /tmp/go1-beep-adapter.service /etc/systemd/system/go1-beep-adapter.service" 2>/dev/null
    $SSH $R "echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl restart go1-beep-adapter" 2>/dev/null
    log "beep_adapter 服务已装/启用（:18082 /v1/beep/actions）。"
  fi
fi

# ── 3) 持久禁用抢设备的 autostart（belt-and-suspenders；adapter 也能每次调用自愈腾设备）──
# 音频：.startlist.sh 里注释 wsaudio（抢 USB 扬声器 → beep 打不开设备）。
$SSH $R 'SL=$HOME/Unitree/autostart/.startlist.sh; [ -f "$SL" ] && grep -qE "^wsaudio" "$SL" && { cp "$SL" "$SL.bak-bootstrap"; sed -i "s/^wsaudio/#wsaudio/" "$SL"; echo wsaudio_disabled; }' 2>/dev/null
# 相机 front：startNode.sh 里注释引用 stereo_camera_config1.yaml 的 rosrun（占 /dev/video1）。
$SSH $R 'CN=$HOME/Unitree/autostart/camerarosnode/cameraRosNode/startNode.sh; [ -f "$CN" ] && ! grep -q "camera_rgb-disabled" "$CN" && { cp "$CN" "$CN.bak-bootstrap"; sed -i "/stereo_camera_config1.yaml/ s/^\\([^#]\\)/#camera_rgb-disabled \\1/" "$CN"; echo front_autostart_disabled; }' 2>/dev/null

# ── 4) 当次立刻腾设备（graceful，不用 -9）：让首次调用无需等下次重启 ─────────────────────
$SSH $R 'echo '"$PW"' | sudo -S pkill -TERM -f example_putImagetrans 2>/dev/null; echo '"$PW"' | sudo -S pkill -TERM -f point_cloud_node 2>/dev/null; pkill -TERM -f wsaudio 2>/dev/null; true' 2>/dev/null

# ── 5) point cloud 端:pointcloud_stream(每路一个常驻服务;连上才开相机 → 免重启热切)──────
#   镜像 depth_stream 的已验证路径:同一 SDK 目录 ~/Unitree/sdk/UnitreeCameraSdk(CMake 构建)。
#   config 命名约定:/dev/video0=stereo_camera_config.yaml,/dev/video1=stereo_camera_config1.yaml。
#   逐块板:可达且有该 SDK 才编,每块只编一次;然后每路装一个 systemd 服务(空闲不占相机)。
#   板不就绪(不可达/无 SDK)则跳过,不阻塞主进程,也不影响已工作的 depth/RGB。
PCL_SDK="/home/unitree/Unitree/sdk/UnitreeCameraSdk"
# 每行: position board_ip config port  (与 Pi 侧 test_camera_pointcloud.positions 对齐)
PCL_ROWS=(
  "front 192.168.123.13 stereo_camera_config1.yaml 9401"
  "chin  192.168.123.13 stereo_camera_config.yaml  9402"
  "left  192.168.123.14 stereo_camera_config.yaml  9403"
  "right 192.168.123.14 stereo_camera_config1.yaml 9404"
  "belly 192.168.123.15 stereo_camera_config.yaml  9405"
)
bssh(){ sshpass -p "$PW" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "unitree@$1" "$2"; }
bscp(){ sshpass -p "$PW" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "$2" "unitree@$1:$3"; }

if [ -f "$DEPLOY/camera/pointcloud_stream.cc" ]; then
  # 5a) 每块用到的板:编译一次(mirror depth_stream)+ 释放出厂点云节点
  PCL_BOARDS=$(printf '%s\n' "${PCL_ROWS[@]}" | awk '{print $2}' | sort -u)
  for B in $PCL_BOARDS; do
    if ! bssh "$B" 'echo ok' >/dev/null 2>&1; then log "point cloud: 板 $B 不可达 → 跳过"; continue; fi
    if bssh "$B" "[ -x $PCL_SDK/bins/pointcloud_stream ]" 2>/dev/null; then
      log "point cloud: $B 上 pointcloud_stream 已存在(跳过编译)。"
    elif bssh "$B" "[ -d $PCL_SDK/build ]" 2>/dev/null; then
      log "point cloud: 在 $B 现编 pointcloud_stream(mirror depth_stream)…"
      bscp "$B" "$DEPLOY/camera/pointcloud_stream.cc" "$PCL_SDK/examples/pointcloud_stream.cc" 2>/dev/null
      bssh "$B" "cd $PCL_SDK/examples && grep -q 'add_executable(pointcloud_stream' CMakeLists.txt || printf '\nadd_executable(pointcloud_stream ./pointcloud_stream.cc)\ntarget_link_libraries(pointcloud_stream \${SDKLIBS})\n' >> CMakeLists.txt" 2>/dev/null
      bssh "$B" "cd $PCL_SDK/build && cmake .. >/dev/null 2>&1 && make pointcloud_stream 2>&1 | tail -3" 2>&1 | tail -3
    else
      log "point cloud: $B 无 $PCL_SDK(depth 用的 SDK)→ 跳过该板(其上机位不可用)。"
      continue
    fi
    bssh "$B" "echo $PW | sudo -S pkill -TERM -f point_cloud_node 2>/dev/null; true" 2>/dev/null
  done
  # 5b) 每路装 systemd 服务(空闲不占相机;Restart=always 常驻,等 Pi 卡连上才开相机)
  for row in "${PCL_ROWS[@]}"; do
    set -- $row; POS="$1"; B="$2"; CFG="$3"; PORT="$4"
    bssh "$B" "[ -x $PCL_SDK/bins/pointcloud_stream ]" 2>/dev/null || { log "point cloud: $POS@$B 无二进制 → 跳过服务"; continue; }
    SVC="go1-pointcloud-$POS"
    bssh "$B" "cat > /tmp/$SVC.service <<'EOF'
[Unit]
Description=Go1 pointcloud_stream $POS (:$PORT)
After=network.target
[Service]
Type=simple
User=unitree
WorkingDirectory=$PCL_SDK
ExecStart=$PCL_SDK/bins/pointcloud_stream $CFG $PORT 4
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
EOF" 2>/dev/null
    bssh "$B" "echo $PW | sudo -S cp /tmp/$SVC.service /etc/systemd/system/$SVC.service; echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl enable --now $SVC" 2>/dev/null
    log "point cloud: $POS@$B → 服务 $SVC 已装/启用(端口 $PORT, config $CFG)。"
  done
else
  log "✗ /deploy/camera/pointcloud_stream.cc 不存在(Dockerfile 应 COPY camera/)→ point cloud 端跳过。"
fi

log "=== provision 完成 ==="
$SSH $R "echo -n '  cam_svc='; echo $PW|sudo -S systemctl is-active go1-camera-adapter 2>/dev/null; echo -n '  beep_svc='; echo $PW|sudo -S systemctl is-active go1-beep-adapter 2>/dev/null" 2>/dev/null | grep -vE 'password|sudo'
exit 0
