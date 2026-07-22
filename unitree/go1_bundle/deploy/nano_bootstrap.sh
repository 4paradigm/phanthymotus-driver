#!/bin/bash
# nano_bootstrap.sh —— 容器首启时由 Dockerfile CMD 自动后台跑一次，把 camera / beep 的
# Nano 端自动布好。目标：Pi 上 clone → build → run 镜像后，无需任何手动步骤，
# 就能在 15678 上直接调用 beep / camera。
#
# 在容器里跑（容器须 --network host 才够得到 Nano 内网 192.168.123.x）：
#   bash /deploy/nano_bootstrap.sh
# CMD 里以后台方式点火：  bash /deploy/nano_bootstrap.sh & python3 /work/main.py
#
# 幂等：已装好（服务 active + 二进制在）则跳过重装；每次都确保占用自启被禁 + 设备当前空闲。
# 依赖：容器内有 sshpass；/deploy/camera/ 下有 rgb_stream.cc、nvjpeg_worker.cc、
# depth_stream.cc、pointcloud_stream.cc，以及 beep/speaker adapter。
# Nano 前置：UnitreeCameraSDK、g++、OpenCV；RGB 硬件 JPEG 还需要 GStreamer 的 nvjpegenc。
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
log "Nano $NANO 可达，开始 provision camera + beep。"

# ── 1) camera(type=rgb) 端：rgb_stream × 五机位（三块板；每路一个常驻 TCP 服务）──
#   每行: position board_ip device_id image_port  （与 Pi 侧 camera.positions 对齐）
#   rgb_stream 空闲时只监听 image_port(9201~9205)、不占相机；Pi 侧 camera 卡 start 连上才开相机，
#   断开即 _exit(0) 释放相机(systemd 重启回待命) → 与同机位的 pointcloud/depth 互斥(谁连上谁占,属预期)。
#   opencv4：.13/.15 在 /usr/pkg-config；.14 装在 /usr/local(头+库+rpath)。下面 OCV 探测全覆盖。
#   rgb_stream 使用 CMei 去鱼眼、翻正与自动裁黑边；JPEG 由独立 nvjpeg_worker 进程硬件编码，
#   避免 Unitree SDK/OpenCV 与 GStreamer nvjpegenc 的 libjpeg ABI 冲突。
CAM_ROWS=(
  "front 192.168.123.13 1 9201"
  "chin  192.168.123.13 0 9202"
  "left  192.168.123.14 0 9203"
  "right 192.168.123.14 1 9204"
  "belly 192.168.123.15 0 9205"
)
if [ -f "$DEPLOY/camera/rgb_stream.cc" ] && [ -f "$DEPLOY/camera/nvjpeg_worker.cc" ]; then
  CAM_DONE=""   # 已处理过的板（每板只编译/禁 autostart/腾设备一次）
  for row in "${CAM_ROWS[@]}"; do
    set -- $row; POS="$1"; B="$2"; DEVID="$3"; PORT="$4"
    if ! bssh "$B" 'echo ok' >/dev/null 2>&1; then log "camera: 板 $B 不可达 → 跳过 $POS"; continue; fi
    SDK=$(bssh "$B" "$CAM_DETECT" 2>/dev/null | tr -d '\r' | head -1)
    if [ -z "$SDK" ]; then log "camera: $B 无 UnitreeCameraSDK → 跳过 $POS"; continue; fi
    case " $CAM_DONE " in
      *" $B "*) : ;;
      *)
        # 首见该板：① 持久禁出厂相机 autostart（point_cloud_node/example_point 独占相机）。
        bssh "$B" 'SN=$HOME/Unitree/autostart/camerarosnode/cameraRosNode/startNode.sh; [ -f "$SN" ] && ! grep -q "camrgb-disabled" "$SN" && { cp "$SN" "$SN.bak-camrgb"; sed -i "/stereo_camera_config.*\.yaml/ s/^\([^#]\)/#camrgb-disabled \1/" "$SN"; echo autostart_disabled; }' 2>/dev/null
        # ② 当次腾设备：杀掉正在独占 /dev/videoN 的出厂进程（graceful SIGTERM，不用 -9 避免 V4L2 卡死）。
        bssh "$B" "echo $PW | sudo -S pkill -TERM -f point_cloud_node 2>/dev/null; echo $PW | sudo -S pkill -TERM -f example_putImagetrans 2>/dev/null; echo $PW | sudo -S pkill -TERM -f example_point 2>/dev/null; true" 2>/dev/null
        # ③ 停掉旧 camera_adapter 路线的服务（若之前装过 go1-camera-adapter-*，避免抢 9201~9205 没用、
        #   但抢相机/端口）。新路线服务名是 go1-rgb-<pos>，与旧名不冲突，这里只清旧的。
        bssh "$B" "echo $PW | sudo -S systemctl stop go1-camera-adapter.service go1-camera-adapter-front go1-camera-adapter-chin go1-camera-adapter-left go1-camera-adapter-right go1-camera-adapter-belly 2>/dev/null; echo $PW | sudo -S systemctl disable go1-camera-adapter.service go1-camera-adapter-front go1-camera-adapter-chin go1-camera-adapter-left go1-camera-adapter-right go1-camera-adapter-belly 2>/dev/null; true" 2>/dev/null
        log "camera: 在 $B 编 rgb_stream + 隔离 nvjpeg_worker(SDK=$SDK)…"
        bssh "$B" "rm -f $SDK/bins/rgb_stream $SDK/bins/nvjpeg_worker" 2>/dev/null
        bscp "$B" "$DEPLOY/camera/rgb_stream.cc" "$SDK/rgb_stream.cc" 2>/dev/null
        bscp "$B" "$DEPLOY/camera/nvjpeg_worker.cc" "$SDK/nvjpeg_worker.cc" 2>/dev/null
        # 编法镜像 depth_stream：opencv4 用 pkg-config；.14 无 pkg-config → 显式 /usr/local + rpath。
        # 🔴 /usr/local 回退必须链全 opencv 模块:SDK 的 libunitree_camera.a(StereoCameraCommon) 内部用
        #   cv::VideoWriter(videoio)、cv::StereoSGBM/StereoBM(calib3d),只链 core/imgproc/imgcodecs 会
        #   undefined reference → .14 编不过(left/right inactive)。pkg-config opencv4 自带全模块不受影响。
        # rgb_stream 不链接 GStreamer，避免与 Unitree SDK 的 libjpeg ABI 冲突；独立 worker 才链接 nvjpegenc。
        bssh "$B" "cd $SDK && mkdir -p bins && OCV=\$(pkg-config --cflags --libs opencv4 2>/dev/null); if [ -z \"\$OCV\" ]; then OCV=\$(pkg-config --cflags --libs opencv 2>/dev/null); fi; if [ -z \"\$OCV\" ] && [ -f /usr/local/include/opencv4/opencv2/opencv.hpp ]; then OCV=\"-I/usr/local/include/opencv4 -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_highgui -lopencv_features2d -lopencv_flann -Wl,-rpath,/usr/local/lib\"; fi; if [ -z \"\$OCV\" ] && [ -f /usr/include/opencv4/opencv2/opencv.hpp ]; then OCV=\"-I/usr/include/opencv4 -L/usr/lib/aarch64-linux-gnu -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -Wl,-rpath,/usr/lib/aarch64-linux-gnu\"; fi; GST=\$(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 2>/dev/null); if [ -z \"\$GST\" ]; then echo missing-gstreamer-dev; exit 1; fi; g++ -O2 -std=c++14 -pthread rgb_stream.cc -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \$OCV -o bins/rgb_stream && g++ -O2 -std=c++14 -pthread nvjpeg_worker.cc \$GST -o bins/nvjpeg_worker 2>&1 | tail -8" 2>&1 | tail -8
        CAM_DONE="$CAM_DONE $B"
        ;;
    esac
    if ! bssh "$B" "[ -x $SDK/bins/rgb_stream ] && [ -x $SDK/bins/nvjpeg_worker ]" 2>/dev/null; then
      log "camera: $POS@$B 无有效 rgb_stream/nvjpeg_worker(编译失败?可能缺依赖)→ 跳过服务"; continue
    fi
    SVC="go1-rgb-$POS"
    bssh "$B" "cat > /tmp/$SVC.service <<EOF
[Unit]
Description=Go1 rgb_stream $POS (dev$DEVID :$PORT)
After=network.target
[Service]
Type=simple
User=unitree
WorkingDirectory=$SDK
ExecStart=$SDK/bins/rgb_stream $PORT $DEVID
Restart=always
RestartSec=3
[Install]
WantedBy=multi-user.target
EOF" 2>/dev/null
    bssh "$B" "echo $PW | sudo -S cp /tmp/$SVC.service /etc/systemd/system/$SVC.service; echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl enable $SVC >/dev/null 2>&1; echo $PW | sudo -S systemctl restart $SVC" 2>/dev/null
    log "camera: $POS@$B → 服务 $SVC 已装/重启(dev$DEVID, 端口 $PORT, SDK $SDK)。"
  done
else
  log "✗ /deploy 下无 camera/rgb_stream.cc 或 nvjpeg_worker.cc → camera 端跳过。"
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
# 相机 autostart：三块板逐个注释 startNode.sh 里引用 stereo_camera_config*.yaml 的 rosrun（独占 /dev/videoN）。
#   .13/.14/.15 的 startNode.sh 同一段脚本，单独注释其正则行（已带 camrgb-disabled 标记则跳过，幂等）。
#   仅连 .13(R) 不够 → 这里对三块板各自 bssh 一遍，确保 left/right/belly 所在 .14/.15 也被持久放开。
for B in 192.168.123.13 192.168.123.14 192.168.123.15; do
  bssh "$B" 'CN=$HOME/Unitree/autostart/camerarosnode/cameraRosNode/startNode.sh; [ -f "$CN" ] && ! grep -q "camera_rgb-disabled" "$CN" && { cp "$CN" "$CN.bak-bootstrap"; sed -i "/stereo_camera_config.*\.yaml/ s/^\\([^#]\\)/#camera_rgb-disabled \\1/" "$CN"; echo front_autostart_disabled@$B; }' 2>/dev/null
done

# ── 4) 当次立刻腾设备（graceful，不用 -9）：让首次调用无需等下次重启 ─────────────────────
# 三块板逐个腾（仅 .13 的 $R 不够）：杀 point_cloud_node / example_putImagetrans / example_point。
for B in 192.168.123.13 192.168.123.14 192.168.123.15; do
  bssh "$B" "echo $PW | sudo -S pkill -TERM -f example_putImagetrans 2>/dev/null; echo $PW | sudo -S pkill -TERM -f point_cloud_node 2>/dev/null; echo $PW | sudo -S pkill -TERM -f example_point 2>/dev/null; pkill -TERM -f wsaudio 2>/dev/null; true" 2>/dev/null
done

# ── 5) point cloud 端:pointcloud_stream(每路一个常驻服务;连上才开相机 → 免重启热切)──────
#   各板 SDK 路径不同(.13=~/UnitreecameraSDK,.14/.15=~/Unitree/sdk/UnitreeCameraSdk)→ 自动探测。
#   ⚠ 与 camera(type=rgb) 同设备互斥:同一机位的 pointcloud 与 RGB 服务不能同时开相机,
#   谁先 start 谁占设备(另一路 fuser 抢)。要 15678 五路 RGB 都出图,本段默认**不启用**
#   pointcloud 服务(CAM 之外的点云由 test_camera_pointcloud 卡按需起,不靠这里常驻)。
#   保留代码但用 ${PCL_ENABLE:-0} 开关;除非显式 PCL_ENABLE=1 否则跳过,避免和 belly(.15 dev0)等撞设备。
PCL_ENABLE="${PCL_ENABLE:-0}"
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
PCL_DETECT='for d in $HOME/Unitree/sdk/UnitreeCameraSdk $HOME/UnitreecameraSDK $HOME/UnitreeCameraSDK; do [ -f "$d/include/UnitreeCameraSDK.hpp" ] && echo "$d" && break; done'

if [ "${PCL_ENABLE}" = "1" ] && [ -f "$DEPLOY/camera/pointcloud_stream.cc" ]; then
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
          bssh "$B" "cd $SDK && mkdir -p bins && OCV=\$(pkg-config --cflags --libs opencv4 2>/dev/null); if [ -z \"\$OCV\" ]; then OCV=\$(pkg-config --cflags --libs opencv 2>/dev/null); fi; if [ -z \"\$OCV\" ] && [ -f /usr/local/include/opencv4/opencv2/opencv.hpp ]; then OCV=\"-I/usr/local/include/opencv4 -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_highgui -lopencv_features2d -lopencv_flann -Wl,-rpath,/usr/local/lib\"; fi; if [ -z \"\$OCV\" ] && [ -f /usr/include/opencv4/opencv2/opencv.hpp ]; then OCV=\"-I/usr/include/opencv4 -L/usr/lib/aarch64-linux-gnu -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lopencv_highgui -Wl,-rpath,/usr/lib/aarch64-linux-gnu\"; fi; g++ -O2 -std=c++14 -pthread pointcloud_stream.cc -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \$OCV -o bins/pointcloud_stream 2>&1 | tail -6" 2>&1 | tail -6
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
  if [ "${PCL_ENABLE}" != "1" ]; then
    log "point cloud: PCL_ENABLE!=1 → 跳过(默认不与 camera RGB 抢设备;按需 PCL_ENABLE=1 启用)。"
  elif [ ! -f "$DEPLOY/camera/pointcloud_stream.cc" ]; then
    log "✗ /deploy/camera/pointcloud_stream.cc 不存在(Dockerfile 应 COPY camera/)→ point cloud 端跳过。"
  fi
fi

# ── 7) depth 端:depth_stream(5 路机位;连上才开相机、断开即释放 → 不常占)────────────────────
#   与 pointcloud 同套路:每路一个常驻服务(空闲不占相机);自动探测 SDK 路径 + raw g++ 编译。
#   depth_stream 按 device_id 现场生成 config 开相机(加载立体标定,同 pointcloud);Pi 侧 test_camera_depth 卡 start 才连、stop 就断。
#   与点云/RGB 指向同一相机 → 三者互斥(谁连上谁 fuser 顶掉对方),属预期。
#   ⚠ 与 camera RGB 抢同一设备 → 默认**不启用**(DEPTH_ENABLE 默认 0),除非显式要常驻深度,
#     否则交给 test_camera_depth 卡按需起,避免和 belly(.15 dev0)/left/right 等抢相机。
#   端口 91xx 与点云 94xx / RGB 92xx-93xx 错开。每行: position board_ip device_id port
#   (与 config.yaml test_camera_depth.positions 及 Pi 侧 depth_port 对齐)
DEPTH_ENABLE="${DEPTH_ENABLE:-0}"
DEPTH_ROWS=(
  "front 192.168.123.13 1 9101"
  "chin  192.168.123.13 0 9102"
  "left  192.168.123.14 0 9103"
  "right 192.168.123.14 1 9104"
  "belly 192.168.123.15 0 9105"
)
DEPTH_DONE=""
if [ "${DEPTH_ENABLE}" = "1" ] && [ -f "$DEPLOY/camera/depth_stream.cc" ]; then
  for row in "${DEPTH_ROWS[@]}"; do
    set -- $row; POS="$1"; B="$2"; DEVID="$3"; PORT="$4"
    if ! bssh "$B" 'echo ok' >/dev/null 2>&1; then log "depth: 板 $B 不可达 → 跳过 $POS"; continue; fi
    SDK=$(bssh "$B" "$PCL_DETECT" 2>/dev/null | tr -d '\r' | head -1)
    if [ -z "$SDK" ]; then log "depth: $B 无 UnitreeCameraSDK → 跳过 $POS"; continue; fi
    # 每块板首次:删旧二进制 → 强制重编(否则旧的崩溃版二进制会被沿用,修复装不上)。
    # opencv 探测:pkg-config opencv4 → opencv → 回退 /usr/local 并链该目录下全部 opencv 模块
    # (含 videoio,SDK 的 VideoWriter 需要;.14 无 opencv4.pc,靠这条兜底)。
    case " $DEPTH_DONE " in
      *" $B "*) : ;;
      *)
        log "depth: 在 $B 编 depth_stream(SDK=$SDK)…"
        bssh "$B" "rm -f $SDK/bins/depth_stream" 2>/dev/null
        bscp "$B" "$DEPLOY/camera/depth_stream.cc" "$SDK/depth_stream.cc" 2>/dev/null
        bssh "$B" "cd $SDK && mkdir -p bins && OCV=\$(pkg-config --cflags --libs opencv4 2>/dev/null); [ -z \"\$OCV\" ] && OCV=\$(pkg-config --cflags --libs opencv 2>/dev/null); [ -z \"\$OCV\" ] && OCV=\"-I/usr/local/include/opencv4 -L/usr/local/lib -Wl,-rpath,/usr/local/lib \$(ls /usr/local/lib/libopencv_*.so 2>/dev/null | sed -E 's#.*/lib(opencv_[a-z0-9]+)\\.so#-l\\1#' | tr '\\n' ' ')\"; g++ -O2 -std=c++14 -pthread depth_stream.cc -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 -Wl,-rpath,$SDK/lib/arm64 -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \$OCV -o bins/depth_stream 2>&1 | tail -4" 2>&1 | tail -4
        DEPTH_DONE="$DEPTH_DONE $B"
        ;;
    esac
    if ! bssh "$B" "[ -x $SDK/bins/depth_stream ]" 2>/dev/null; then
      log "depth: $POS@$B 无有效二进制(编译失败?)→ 跳过服务"; continue
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
    bssh "$B" "echo $PW | sudo -S cp /tmp/$SVC.service /etc/systemd/system/$SVC.service; echo $PW | sudo -S systemctl daemon-reload; echo $PW | sudo -S systemctl enable $SVC >/dev/null 2>&1; echo $PW | sudo -S systemctl restart $SVC" 2>/dev/null
    log "depth: $POS@$B → 服务 $SVC 已装/重启(dev$DEVID, 端口 $PORT, SDK $SDK)。"
  done
else
  if [ "${DEPTH_ENABLE}" != "1" ]; then
    log "depth: DEPTH_ENABLE!=1 → 跳过(默认不与 camera RGB 抢设备;按需 DEPTH_ENABLE=1 启用)。"
  elif [ ! -f "$DEPLOY/camera/depth_stream.cc" ]; then
    log "✗ /deploy/camera/depth_stream.cc 不存在(Dockerfile 应 COPY camera/)→ depth 端跳过。"
  fi
fi

log "=== provision 完成 ==="
for row in "${CAM_ROWS[@]}"; do
  set -- $row; POS="$1"; B="$2"
  st=$(bssh "$B" "echo $PW|sudo -S systemctl is-active go1-rgb-$POS 2>/dev/null" 2>/dev/null | grep -vE 'password|sudo' | tr -d '\r')
  log "  rgb_$POS@$B=${st:-unknown}"
done
$SSH $R "echo -n '  beep_svc='; echo $PW|sudo -S systemctl is-active go1-beep-adapter 2>/dev/null; echo -n '  speaker_svc='; echo $PW|sudo -S systemctl is-active go1-speaker-adapter 2>/dev/null" 2>/dev/null | grep -vE 'password|sudo'
exit 0
