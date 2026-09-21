/**
 * pointcloud_stream.cc — 按需点云推流(camera_pointcloud 上游,Nano 端)。
 *
 * ★ 实现说明(2026-09 重写):
 *   旧版把 getRectStereoFrame 的第三个输出 feim 当视差图喂 cv::reprojectImageTo3D。
 *   实际 feim 是 PERSPECTIVE 投影的矫正左图(CV_8UC3,真机 journal 证实 type=16),
 *   reprojectImageTo3D 断言"单通道"失败 → SIGABRT core dump → systemd 无限重启。
 *
 *   新实现两级自动探测(首个出数的路径生效,并在日志标明):
 *     ① SDK 官方 getPointCloud(std::vector<cv::Vec3f>, t):startStereoCompute 后台线程
 *        已算好视差(Algorithm=1 → StereoBM(32,9)),SDK 用鱼眼球面模型直接反投影出
 *        相机系 XYZ(米),精度最好、零重复计算。
 *     ② 回退:getRectStereoFrame 的 left/right(LONGLAT 矫正对)转灰度 → 自建
 *        cv::StereoBM(32,9) 算视差 → getCalibParams() 内参构造 Q 矩阵 →
 *        reprojectImageTo3D 反投影(针孔近似,兜底可用)。
 *   两路都做:有限值过滤 + Z∈[0.2,5.0]m 范围过滤 + stride 抽稀。
 *
 * ★ 相机初始化:必须走 UnitreeCamera(config_file)(会加载立体标定,深度/点云才出得来)。
 * ★ 热切:相机"客户端连上才开、断开就释放"。
 * ★ 抢占:开相机前 fuser -k /dev/video<device_id> 释放占用者。
 * ★ SDK 析构 double-free:客户端断开后 _exit(0) 绕开析构,systemd Restart=always 重启回待命。
 *
 * 协议(每帧):[4字节大端 totalLen][totalLen 字节 payload]
 *            payload = [4字节大端 numPoints][numPoints × 3 × float32 (小端, x/y/z 米,相机系)]
 *
 * 用法:pointcloud_stream <port> <device_id> [stride]
 *   例:./bins/pointcloud_stream 9401 1 4      # front(dev1),端口 9401,抽稀 4
 */
#include <UnitreeCameraSDK.hpp>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>
#include <vector>

// 相机内参兜底值(仅回退路径 ② 用;运行时优先 getCalibParams())。
static const float FX_DEFAULT = 186.74f;
static const float CX_DEFAULT = 229.86f;
static const float CY_DEFAULT = 178.71f;
static const float TX_DEFAULT = 0.02443f;  // 基线(m)

// 有效深度范围(米):过滤视差无效点(背景/遮挡/噪声)。
static const float Z_MIN = 0.2f;
static const float Z_MAX = 5.0f;

static bool send_all(int fd, const uint8_t *p, size_t n) {
    size_t sent = 0;
    while (sent < n) {
        ssize_t k = send(fd, p + sent, n - sent, MSG_NOSIGNAL);
        if (k <= 0) return false;
        sent += (size_t)k;
    }
    return true;
}

// 按 device_id 生成一份最小 stereo config(标定从相机 flash 加载);返回文件路径,失败返回空。
// ★ 必须直接 fopen 写,不能"读 stock yaml 改 DeviceNode":stock 的 DeviceNode 常已等于目标
//   device_id,导致 done=false → 跳过写文件 → 返回不存在的路径 → 打开空 config 崩溃。
static std::string write_config(int device_id) {
    std::string out = "/tmp/pcl_dev" + std::to_string(device_id) + ".yaml";
    FILE *f = fopen(out.c_str(), "w");
    if (!f) return "";
    fprintf(f, "%%YAML:1.0\n---\n");
    auto m1 = [&](const char *k, double v) {
        fprintf(f, "%s: !!opencv-matrix\n   rows: 1\n   cols: 1\n   dt: d\n   data: [ %g ]\n", k, v);
    };
    m1("LogLevel", 1);
    m1("Threshold", 190);
    m1("Algorithm", 1);          // 1 = StereoBM(后台线程算视差,快;SGBM 在 Nano 上太慢)
    m1("IpLastSegment", 15);
    m1("DeviceNode", (double)device_id);
    m1("hFov", 90);
    fprintf(f, "FrameSize: !!opencv-matrix\n   rows: 1\n   cols: 2\n   dt: d\n   data: [ 928., 400. ]\n");
    fprintf(f, "RectifyFrameSize: !!opencv-matrix\n   rows: 1\n   cols: 2\n   dt: d\n   data: [ 464., 400. ]\n");
    m1("FrameRate", 30);
    m1("Transmode", -1);
    m1("Transrate", 30);
    m1("Depthmode", 1);
    fclose(f);
    return out;
}

static void free_device(int device_id) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "fuser -k /dev/video%d >/dev/null 2>&1", device_id);
    (void)system(cmd);
    usleep(500000);
}

// 标准 OpenCV Q 矩阵(仅回退路径 ② 用;针孔模型对 LONGLAT 矫正对是近似):
//   Q = [1  0   0    -cx  ]
//       [0  1   0    -cy  ]
//       [0  0   0     fx  ]
//       [0  0  -1/Tx   0  ]
static cv::Mat make_Q(float fx, float cx, float cy, float Tx) {
    cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
    Q.at<double>(0, 0) = 1.0;
    Q.at<double>(1, 1) = 1.0;
    Q.at<double>(0, 3) = -cx;
    Q.at<double>(1, 3) = -cy;
    Q.at<double>(2, 3) = fx;
    Q.at<double>(3, 2) = -1.0 / Tx;
    return Q;
}

// 路径 ②:自算视差(rectified left/right)→ Q 反投影 + 过滤 + stride 抽稀。
static void disp_to_xyz(const cv::Mat &disp, const cv::Mat &Q, int stride,
                        std::vector<float> &xyz_out) {
    xyz_out.clear();
    if (disp.empty() || Q.empty() || disp.channels() != 1) return;

    cv::Mat xyz_map;
    cv::reprojectImageTo3D(disp, xyz_map, Q, true);  // handleMissingValues=true

    int rows = xyz_map.rows, cols = xyz_map.cols;
    xyz_out.reserve((size_t)(rows * cols / (stride * stride) + 1) * 3);
    for (int v = 0; v < rows; v += stride) {
        for (int u = 0; u < cols; u += stride) {
            const cv::Vec3f &p = xyz_map.at<cv::Vec3f>(v, u);
            float x = p[0], y = p[1], z = p[2];
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (z < Z_MIN || z > Z_MAX) continue;
            xyz_out.push_back(x);
            xyz_out.push_back(y);
            xyz_out.push_back(z);
        }
    }
}

// 发送一帧 xyz(米,相机系)。
static bool send_frame(int cli, const std::vector<float> &xyz) {
    uint32_t numPoints = (uint32_t)(xyz.size() / 3);
    uint32_t payloadLen = 4 + numPoints * 12;
    uint32_t beTotal = htonl(payloadLen);
    uint32_t beCount = htonl(numPoints);
    static std::vector<uint8_t> frame;
    frame.clear();
    frame.resize(4 + payloadLen);
    std::memcpy(frame.data() + 0, &beTotal, 4);
    std::memcpy(frame.data() + 4, &beCount, 4);
    if (numPoints > 0)
        std::memcpy(frame.data() + 8, xyz.data(), (size_t)numPoints * 12);
    return send_all(cli, frame.data(), frame.size());
}

static void serve_client(int cli, int device_id, int stride) {
    std::string cfg = write_config(device_id);
    if (cfg.empty()) { fprintf(stderr, "[pointcloud_stream] 生成 config 失败\n"); return; }
    free_device(device_id);
    UnitreeCamera cam(cfg);
    for (int attempt = 0; attempt < 3 && !cam.isOpened(); ++attempt) {
        fprintf(stderr, "[pointcloud_stream] dev%d 未就绪,重试 %d...\n", device_id, attempt + 1);
        free_device(device_id);
        sleep(1);
    }
    if (!cam.isOpened()) {
        fprintf(stderr, "[pointcloud_stream] dev%d 打开失败(被占用/不可用),放弃本连接\n", device_id);
        return;
    }
    cam.startCapture();
    cam.startStereoCompute();   // SDK 后台线程算视差(路径 ① 的数据源)

    // 路径 ② 的 Q 矩阵:从 SDK 读内参和基线。
    // params[0]=LeftIntrinsicMatrix(fx/cx/cy), params[4]=Translation(Tx,mm)
    cv::Mat Q;
    {
        float fx = FX_DEFAULT, cx = CX_DEFAULT, cy = CY_DEFAULT, Tx = TX_DEFAULT;
        std::vector<cv::Mat> params;
        if (cam.getCalibParams(params) && params.size() >= 5
                && !params[0].empty() && !params[4].empty()) {
            const cv::Mat &K = params[0];
            const cv::Mat &T = params[4];
            if (K.rows >= 3 && K.cols >= 3) {
                fx = (float)K.at<double>(0, 0);
                cx = (float)K.at<double>(0, 2);
                cy = (float)K.at<double>(1, 2);
            }
            if (T.rows >= 1 && T.cols >= 1) {
                float tx_mm = (float)T.at<double>(0, 0);
                if (std::abs(tx_mm) > 1.0f)   // 单位 mm(典型 ~24mm)
                    Tx = tx_mm / 1000.0f;
                else
                    Tx = tx_mm;
            }
            fprintf(stderr, "[pointcloud_stream] dev%d 内参(getCalibParams): "
                    "fx=%.2f cx=%.2f cy=%.2f Tx=%.4fm\n", device_id, fx, cx, cy, Tx);
        } else {
            fprintf(stderr, "[pointcloud_stream] dev%d getCalibParams 失败,用兜底内参: "
                    "fx=%.2f cx=%.2f cy=%.2f Tx=%.4fm\n", device_id, fx, cx, cy, Tx);
        }
        Q = make_Q(fx, cx, cy, Tx);
    }

    // 回退路径 ② 的 StereoBM(参数镜像 SDK 内部 StereoBM::create(32,9))。
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(32, 9);

    fprintf(stderr, "[pointcloud_stream] dev%d 相机已开,开始推流(stride=%d;"
            "①SDK getPointCloud → ②回退 BM+Q)\n", device_id, stride);

    // mode: 0=探测中 1=SDK getPointCloud 2=自算 BM。
    // 探测规则:①连续 ~5s 无点 → 切 ②;一旦 ① 出过数就固定用 ①。
    int mode = 0;
    int pcl_fail_streak = 0;
    const int PCL_FAIL_MAX = 100;   // 100 × 50ms = 5s
    bool mode_logged = false;
    int empty_streak = 0;

    while (cam.isOpened()) {
        std::chrono::microseconds t;
        std::vector<float> xyz;
        bool got = false;

        // 路径 ①:SDK 官方 getPointCloud(鱼眼球面模型反投影,相机系米)。
        if (mode == 0 || mode == 1) {
            std::vector<cv::Vec3f> pcl;
            if (cam.getPointCloud(pcl, t) && !pcl.empty()) {
                mode = 1;
                pcl_fail_streak = 0;
                size_t step = (size_t)stride * (size_t)stride;   // 平铺向量按面积比抽稀
                if (step < 1) step = 1;
                xyz.reserve(pcl.size() / step * 3 + 3);
                for (size_t i = 0; i < pcl.size(); i += step) {
                    float x = pcl[i][0], y = pcl[i][1], z = pcl[i][2];
                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                    if (z < Z_MIN || z > Z_MAX) continue;
                    xyz.push_back(x);
                    xyz.push_back(y);
                    xyz.push_back(z);
                }
                got = true;
                if (!mode_logged) {
                    fprintf(stderr, "[pointcloud_stream] dev%d 路径① getPointCloud 生效"
                            "(raw=%zu pts,filtered=%zu)\n", device_id, pcl.size(), xyz.size() / 3);
                    mode_logged = true;
                }
            } else if (mode == 1) {
                pcl_fail_streak++;   // 已用①后偶发空帧:等下一帧
            } else {
                if (++pcl_fail_streak >= PCL_FAIL_MAX) {
                    fprintf(stderr, "[pointcloud_stream] dev%d getPointCloud 连续 %d 次无数据"
                            "→ 切路径②(自算 BM 视差)\n", device_id, pcl_fail_streak);
                    mode = 2;
                }
            }
        }

        // 路径 ②:rectified left/right → 自算 BM 视差 → Q 反投影。
        if (!got && (mode == 0 || mode == 2)) {
            cv::Mat left, right, feim;
            if (cam.getRectStereoFrame(left, right, feim, t)
                    && !left.empty() && !right.empty()) {
                cv::Mat gL, gR, d16, disp;
                if (left.channels() == 3) cv::cvtColor(left, gL, cv::COLOR_BGR2GRAY);
                else gL = left;
                if (right.channels() == 3) cv::cvtColor(right, gR, cv::COLOR_BGR2GRAY);
                else gR = right;
                bm->compute(gL, gR, d16);            // CV_16SC1,视差 = 值/16
                d16.convertTo(disp, CV_32FC1, 1.0 / 16.0);
                disp_to_xyz(disp, Q, stride, xyz);
                got = !xyz.empty();
                if (got) {
                    mode = 2;
                    if (!mode_logged) {
                        fprintf(stderr, "[pointcloud_stream] dev%d 路径② BM+Q 生效"
                            "(filtered=%zu pts)\n", device_id, xyz.size() / 3);
                        mode_logged = true;
                    }
                }
            }
        }

        if (!got) {
            empty_streak++;
            if (empty_streak % 200 == 1)
                fprintf(stderr, "[pointcloud_stream] dev%d 无点云(streak=%d,mode=%d),等待...\n",
                        device_id, empty_streak, mode);
            usleep(50000);
            continue;
        }
        empty_streak = 0;

        if (!send_frame(cli, xyz)) break;   // 对端断开
        usleep(100000);   // ~10Hz 上限
    }
    fprintf(stderr, "[pointcloud_stream] dev%d 客户端断开,_exit(0) 退出"
            "(systemd 重启回待命,规避 SDK 析构 double-free)\n", device_id);
    fflush(stderr);
    _exit(0);
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "用法: %s <port> <device_id> [stride]\n", argv[0]);
        _exit(1);
    }
    int port      = atoi(argv[1]);
    int device_id = atoi(argv[2]);
    int stride    = (argc > 3) ? atoi(argv[3]) : 4;
    if (stride < 1) stride = 1;
    signal(SIGPIPE, SIG_IGN);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(srv, (sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); _exit(4); }
    listen(srv, 1);
    fprintf(stderr, "[pointcloud_stream] 空闲待命(dev%d,相机未开),监听 0.0.0.0:%d ...\n",
            device_id, port);

    while (true) {
        int cli = accept(srv, nullptr, nullptr);
        if (cli < 0) continue;
        fprintf(stderr, "[pointcloud_stream] 客户端已连接 → 开 dev%d\n", device_id);
        serve_client(cli, device_id, stride);
        close(cli);
        fprintf(stderr, "[pointcloud_stream] 回到空闲待命(相机已释放),等待下一次连接...\n");
    }
    return 0;
}
