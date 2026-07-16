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

# 按板 IP 的 ssh/scp（camera/pointcloud 多板 provision 用；单行短连接）。
bssh(){ sshpass -p "$PW" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "unitree@$1" "$2"; }
bscp(){ sshpass -p "$PW" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "$2" "unitree@$1:$3"; }
# 板上探测 SDK 目录（含 include/UnitreeCameraSDK.hpp 的第一个候选）；单引号 → $HOME 在板上求值。
CAM_DETECT='for d in $HOME/UnitreecameraSDK $HOME/Unitree/sdk/UnitreeCameraSdk; do [ -f "$d/include/UnitreeCameraSDK.hpp" ] && echo "$d" && break; done'

command -v sshpass >/dev/null 2>&1 || { log "✗ 容器内无 sshpass（Dockerfile 应 apt 装）；跳过 provision。"; exit 0; }

# 0) 可达性（连不上不阻塞主进程；主进程照常起，只是 beep/camera 暂不可用）
if ! $SSH $R 'echo ok' >/dev/null 2>&1; then
  log "✗ 连不上 Nano $NANO（容器需 --network host + Nano 在线 + ssh unitree/123）；跳过 provision。"
  exit 0
fi
log "Nano $NANO 可达，开始 provision camera_rgb + beep。"

# ── 1) camera_rgb 端：camera_adapter × 五机位（三块板；SDK/编法/opencv fallback 同 point cloud 自动探测）──
#   每行: position board_ip device_id device_node control_port  （与 Pi 侧 camera_rgb.positions 对齐）
#   adapter 平时只监听控制口、probe/start 时才开相机（不占相机）→ 可与 pointcloud 服务共存，谁 start 谁抢设备。
#   .14 若无 opencv/编译失败 → 该板跳过（left/right 暂不可用），不阻塞其它板。calib 尽力用 output_camCalibParams.yaml
#   （front 匹配→去畸变；其它机位缺 per-device 标定→adapter 降级鱼眼直通，仍出图）。
CAM_ROWS=(
  "front 192.168.123.13 1 /dev/video1 9301"
  "chin  192.168.123.13 0 /dev/video0 9302"
  "left  192.168.123.14 0 /dev/video0 9303"
  "right 192.168.123.14 1 /dev/video1 9304"
  "belly 192.168.123.15 0 /dev/video0 9305"
)
if [ -f "$DEPLOY/camera_adapter/camera_adapter.cpp" ]; then
  CAM_DONE=""   # 已处理过的板（每板只编译/禁 autostart/腾设备一次）
  for row in "${CAM_ROWS[@]}"; do
    set -- $row; POS="$1"; B="$2"; DEVID="$3"; NODE="$4"; CTRL="$5"
    if ! bssh "$B" 'echo ok' >/dev/null 2>&1; then log "camera: 板 $B 不可达 → 跳过 $POS"; continue; fi
    SDK=$(bssh "$B" "$CAM_DETECT" 2>/dev/null | tr -d '\r' | head -1)
    if [ -z "$SDK" ]; then log "camera: $B 无 UnitreeCameraSDK → 跳过 $POS"; continue; fi
    case " $CAM_DONE " in
      *" $B "*) : ;;
      *)
        # 首见该板：① 持久禁出厂相机 autostart（point_cloud_node 独占相机）；② 当次腾设备；③ 编 camera_adapter
        bssh "$B" 'SN=$HOME/Unitree/autostart/camerarosnode/cameraRosNode/startNode.sh; [ -f "$SN" ] && ! grep -q "camrgb-disabled" "$SN" && { cp "$SN" "$SN.bak-camrgb"; sed -i "/stereo_camera_config.*\.yaml/ s/^\([^#]\)/#camrgb-disabled \1/" "$SN"; echo autostart_disabled; }' 2>/dev/null
        bssh "$B" "echo $PW | sudo -S pkill -TERM -f point_cloud_node 2>/dev/null; echo $PW | sudo -S pkill -TERM -f example_putImagetrans 2>/dev/null; true" 2>/dev/null
        log "camera: 在 $B 编 camera_adapter(SDK=$SDK)…"
        bssh "$B" "rm -f $SDK/bins/camera_adapter" 2>/dev/null
        bscp "$B" "$DEPLOY/camera_adapter/camera_adapter.cpp" "/tmp/camera_adapter.cpp" 2>/dev/null
        bssh "$B" "mkdir -p $SDK/bins; OCV=\$(pkg-config --cflags --libs opencv4 2>/dev/null); [ -z \"\$OCV\" ] && OCV=\$(pkg-config --cflags --libs opencv 2>/dev/null); [ -z \"\$OCV\" ] && OCV=\"-I/usr/include/opencv4 -I/usr/local/include/opencv4 -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d\"; g++ -O2 -std=c++14 -pthread /tmp/camera_adapter.cpp -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \$OCV -o $SDK/bins/camera_adapter 2>&1 | tail -4" 2>&1 | tail -4
        CAM_DONE="$CAM_DONE $B"
        ;;
    esac
    if ! bssh "$B" "[ -x $SDK/bins/camera_adapter ]" 2>/dev/null; then
      log "camera: $POS@$B 无有效二进制(编译失败?可能缺 opencv)→ 跳过服务"; continue
    fi
    # per-position 标定：有 output_camCalibParams.yaml 就作为该机位 --calib（front 匹配→去畸变；
    # 其它机位用它是近似/或文件不存在→adapter 降级鱼眼）。TODO: 用 example_getCalibParamsFile 按 device 生成。
    CAL="$SDK/calib_$POS.yaml"
    bssh "$B" "[ -f $SDK/output_camCalibParams.yaml ] && cp -f $SDK/output_camCalibParams.yaml $CAL 2>/dev/null; true" 2>/dev/null
    SVC="go1-camera-adapter-$POS"
    bssh "$B" "cat > /tmp/$SVC.service <<EOF
[Unit]
Description=Go1 camera_adapter $POS (dev$DEVID $NODE :$CTRL)
After=network.target
[Service]
Type=simple
User=unitree
WorkingDirectory=$SDK/bins
ExecStart=$SDK/bins/camera_adapter --device-id $DEVID --device-node $NODE --control-port $CTRL --calib $CAL
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
EOF" 2>/dev/null
    bssh "$B" "echo $PW | sudo -S cp /tmp/$SVC.service /etc/systemd/system/$SVC.service; echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl enable $SVC >/dev/null 2>&1; echo $PW | sudo -S systemctl restart $SVC" 2>/dev/null
    log "camera: $POS@$B → 服务 $SVC 已装/重启(dev$DEVID $NODE, 控制口 $CTRL, calib $CAL, SDK $SDK)。"
  done
else
  log "✗ /deploy 下无 camera_adapter/camera_adapter.cpp → camera 端跳过。"
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

# ── 2b) speaker 端：speaker_adapter（:18083 /v1/speaker/actions，播放 remote_mic 音频流，纯 Python 免编译）──
if [ -f "$DEPLOY/speaker_adapter.py" ]; then
  $SCP "$DEPLOY/speaker_adapter.py" "$R:~/speaker_adapter.py" 2>/dev/null
  # 首次没有配置就下发示例配置（audio_device=auto / mixer=Speaker / idle_timeout）。
  if ! $SSH $R '[ -f ~/go1-speaker-adapter.json ]' 2>/dev/null && [ -f "$DEPLOY/go1-speaker-adapter.example.json" ]; then
    $SCP "$DEPLOY/go1-speaker-adapter.example.json" "$R:~/go1-speaker-adapter.json" 2>/dev/null
  fi
  if [ -f "$DEPLOY/go1-speaker-adapter.service" ]; then
    $SCP "$DEPLOY/go1-speaker-adapter.service" "$R:/tmp/go1-speaker-adapter.service" 2>/dev/null
    $SSH $R "echo $PW | sudo -S cp /tmp/go1-speaker-adapter.service /etc/systemd/system/go1-speaker-adapter.service" 2>/dev/null
    $SSH $R "echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl restart go1-speaker-adapter" 2>/dev/null
    log "speaker_adapter 服务已装/启用（:18083 /v1/speaker/actions）。"
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
#   各板 SDK 路径不同(.13/.14=~/UnitreecameraSDK,.15=~/Unitree/sdk/UnitreeCameraSdk)→ 自动探测。
#   用 raw g++ 编译(镜像 camera_adapter 的编法,已在 .13 验证),产物落 $SDK/bins/pointcloud_stream。
#   pointcloud_stream 按 device_id 开相机(不依赖 per-device config)。每路装一个 systemd 服务(空闲不占相机)。
#   板不就绪(不可达/无 SDK)则跳过,不阻塞主进程,也不影响已工作的 depth/RGB。
# 每行: position board_ip device_id port  (与 Pi 侧 test_camera_pointcloud.positions 对齐)
PCL_ROWS=(
  "front 192.168.123.13 1 9401"
  "chin  192.168.123.13 0 9402"
  "left  192.168.123.14 0 9403"
  "right 192.168.123.14 1 9404"
  "belly 192.168.123.15 0 9405"
)
bssh(){ sshpass -p "$PW" ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "unitree@$1" "$2"; }
bscp(){ sshpass -p "$PW" scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=8 "$2" "unitree@$1:$3"; }
# 板上探测 SDK 目录(含 include/UnitreeCameraSDK.hpp 的第一个候选);单引号 → $HOME 在板上求值。
PCL_DETECT='for d in $HOME/UnitreecameraSDK $HOME/Unitree/sdk/UnitreeCameraSdk $HOME/UnitreeCameraSDK; do [ -f "$d/include/UnitreeCameraSDK.hpp" ] && echo "$d" && break; done'

if [ -f "$DEPLOY/camera/pointcloud_stream.cc" ]; then
  PCL_DONE=""   # 本次已重编过的板(每板只编一次,但强制重编以匹配当前 .cc + 服务参数)
  for row in "${PCL_ROWS[@]}"; do
    set -- $row; POS="$1"; B="$2"; DEVID="$3"; PORT="$4"
    if ! bssh "$B" 'echo ok' >/dev/null 2>&1; then log "point cloud: 板 $B 不可达 → 跳过 $POS"; continue; fi
    SDK=$(bssh "$B" "$PCL_DETECT" 2>/dev/null | tr -d '\r' | head -1)
    if [ -z "$SDK" ]; then log "point cloud: $B 无 UnitreeCameraSDK → 跳过 $POS"; continue; fi
    # 本板首次:删旧二进制→强制重编。按 SDK 布局选编法:有 CMake build 用 CMake(SDK 自处理 opencv,
    # .14/.15 的 ~/Unitree/sdk/UnitreeCameraSdk 用这个),否则 raw g++(.13 的 ~/UnitreecameraSDK)。
    # 编失败则无二进制→下方跳过,绝不用陈旧/不匹配的二进制。
    case " $PCL_DONE " in
      *" $B "*) : ;;
      *)
        log "point cloud: 在 $B 编 pointcloud_stream(SDK=$SDK)…"
        bssh "$B" "rm -f $SDK/bins/pointcloud_stream" 2>/dev/null
        if bssh "$B" "[ -d $SDK/build ] && [ -f $SDK/examples/CMakeLists.txt ]" 2>/dev/null; then
          bscp "$B" "$DEPLOY/camera/pointcloud_stream.cc" "$SDK/examples/pointcloud_stream.cc" 2>/dev/null
          bssh "$B" "cd $SDK/examples && grep -q 'add_executable(pointcloud_stream' CMakeLists.txt || printf '\nadd_executable(pointcloud_stream ./pointcloud_stream.cc)\ntarget_link_libraries(pointcloud_stream \${SDKLIBS})\n' >> CMakeLists.txt; cd $SDK/build && cmake .. >/dev/null 2>&1 && make pointcloud_stream 2>&1 | tail -4" 2>&1 | tail -4
        else
          bscp "$B" "$DEPLOY/camera/pointcloud_stream.cc" "$SDK/pointcloud_stream.cc" 2>/dev/null
          bssh "$B" "cd $SDK && mkdir -p bins; OCV=\$(pkg-config --cflags --libs opencv4 2>/dev/null); [ -z \"\$OCV\" ] && OCV=\$(pkg-config --cflags --libs opencv 2>/dev/null); [ -z \"\$OCV\" ] && OCV=\"-I/usr/include/opencv4 -I/usr/local/include/opencv4 -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d -lopencv_video\"; g++ -O2 -std=c++14 -pthread pointcloud_stream.cc -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \$OCV -o bins/pointcloud_stream 2>&1 | tail -4" 2>&1 | tail -4
        fi
        PCL_DONE="$PCL_DONE $B"
        ;;
    esac
    if ! bssh "$B" "[ -x $SDK/bins/pointcloud_stream ]" 2>/dev/null; then
      log "point cloud: $POS@$B 无有效二进制(编译失败?)→ 跳过服务"; continue
    fi
    # 装 systemd 服务(空闲不占相机;连上才开 dev$DEVID;fuser 抢占由 pointcloud_stream 自己做)
    SVC="go1-pointcloud-$POS"
    bssh "$B" "cat > /tmp/$SVC.service <<EOF
[Unit]
Description=Go1 pointcloud_stream $POS (dev$DEVID :$PORT)
After=network.target
[Service]
Type=simple
User=unitree
WorkingDirectory=$SDK
ExecStart=$SDK/bins/pointcloud_stream $PORT $DEVID 4
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
EOF" 2>/dev/null
    bssh "$B" "echo $PW | sudo -S cp /tmp/$SVC.service /etc/systemd/system/$SVC.service; echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl enable $SVC >/dev/null 2>&1; echo $PW | sudo -S systemctl restart $SVC" 2>/dev/null
    log "point cloud: $POS@$B → 服务 $SVC 已装/重启(dev$DEVID, 端口 $PORT, SDK $SDK)。"
  done
else
  log "✗ /deploy/camera/pointcloud_stream.cc 不存在(Dockerfile 应 COPY camera/)→ point cloud 端跳过。"
fi

# ── 7) depth 端:depth_stream(5 路机位;连上才开相机、断开即释放 → 不常占)────────────────────
#   与 pointcloud 同套路:每路一个常驻服务(空闲不占相机);自动探测 SDK 路径 + raw g++ 编译。
#   depth_stream 按 device_id 现场生成 config 开相机(加载立体标定,同 pointcloud);Pi 侧 test_camera_depth 卡 start 才连、stop 就断。
#   与点云指向同一相机 → 二者互斥(谁连上谁 fuser 顶掉对方),属预期。
#   端口 91xx 与点云 94xx / RGB 92xx-93xx 错开。每行: position board_ip device_id port
#   (与 config.yaml test_camera_depth.positions 及 Pi 侧 depth_port 对齐)
DEPTH_ROWS=(
  "front 192.168.123.13 1 9101"
  "chin  192.168.123.13 0 9102"
  "left  192.168.123.14 0 9103"
  "right 192.168.123.14 1 9104"
  "belly 192.168.123.15 0 9105"
)
if [ -f "$DEPLOY/camera/depth_stream.cc" ]; then
  for row in "${DEPTH_ROWS[@]}"; do
    set -- $row; POS="$1"; B="$2"; DEVID="$3"; PORT="$4"
    if ! bssh "$B" 'echo ok' >/dev/null 2>&1; then log "depth: 板 $B 不可达 → 跳过 $POS"; continue; fi
    SDK=$(bssh "$B" "$PCL_DETECT" 2>/dev/null | tr -d '\r' | head -1)
    if [ -z "$SDK" ]; then log "depth: $B 无 UnitreeCameraSDK → 跳过 $POS"; continue; fi
    # 编译一次(该板二进制不存在才编;raw g++ 镜像 pointcloud_stream)
    if ! bssh "$B" "[ -x $SDK/bins/depth_stream ]" 2>/dev/null; then
      log "depth: 在 $B 编 depth_stream(SDK=$SDK)…"
      bscp "$B" "$DEPLOY/camera/depth_stream.cc" "$SDK/depth_stream.cc" 2>/dev/null
      bssh "$B" "cd $SDK && mkdir -p bins && g++ -O2 -std=c++14 -pthread depth_stream.cc -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \$(pkg-config --cflags --libs opencv4) -o bins/depth_stream 2>&1 | tail -3" 2>&1 | tail -3
    fi
    if ! bssh "$B" "[ -x $SDK/bins/depth_stream ]" 2>/dev/null; then
      log "depth: $POS@$B 编译失败/无二进制 → 跳过服务"; continue
    fi
    # 装 systemd 服务(空闲不占相机;连上才开 dev$DEVID;fuser 抢占由 depth_stream 自己做)
    SVC="go1-depth-$POS"
    bssh "$B" "cat > /tmp/$SVC.service <<EOF
[Unit]
Description=Go1 depth_stream $POS (dev$DEVID :$PORT)
After=network.target
[Service]
Type=simple
User=unitree
WorkingDirectory=$SDK
ExecStart=$SDK/bins/depth_stream $PORT $DEVID
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
EOF" 2>/dev/null
    bssh "$B" "echo $PW | sudo -S cp /tmp/$SVC.service /etc/systemd/system/$SVC.service; echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl enable --now $SVC" 2>/dev/null
    log "depth: $POS@$B → 服务 $SVC 已装/启用(dev$DEVID, 端口 $PORT, SDK $SDK)。"
  done
else
  log "✗ /deploy/camera/depth_stream.cc 不存在(Dockerfile 应 COPY camera/)→ depth 端跳过。"
fi

log "=== provision 完成 ==="
for row in "${CAM_ROWS[@]}"; do
  set -- $row; POS="$1"; B="$2"
  st=$(bssh "$B" "echo $PW|sudo -S systemctl is-active go1-camera-adapter-$POS 2>/dev/null" 2>/dev/null | grep -vE 'password|sudo' | tr -d '\r')
  log "  cam_$POS@$B=${st:-unknown}"
done
$SSH $R "echo -n '  beep_svc='; echo $PW|sudo -S systemctl is-active go1-beep-adapter 2>/dev/null; echo -n '  speaker_svc='; echo $PW|sudo -S systemctl is-active go1-speaker-adapter 2>/dev/null" 2>/dev/null | grep -vE 'password|sudo'
exit 0
