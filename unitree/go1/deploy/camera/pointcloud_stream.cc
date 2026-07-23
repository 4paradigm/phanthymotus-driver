/**
 * pointcloud_stream.cc — 按需点云推流(test_camera_pointcloud 上游,Nano 端)。
 *
 * ★ 实现说明:
 *   getPointCloud 在 Nano 上不出数据(无 GPU 支持)。
 *   getDepthFrame(color=false) 返回全零/无效深度(SDK stereo compute 在此 Nano 不可用)。
 *   getRectStereoFrame 的 feim 是矫正彩色图(CV_8UC3),不是视差图。
 *   最终方案:用 getRectStereoFrame(left, right) 取矫正左右图,自己用 StereoSGBM 算视差,
 *   再用 Q 矩阵 reprojectImageTo3D 反投影到 XYZ(米,相机系)。
 *
 * ★ 相机初始化:必须走 UnitreeCamera(config_file)(会加载立体标定)。
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

// 相机内参兜底值(来自 belly output_camCalibParams.yaml,RectifyFrameSize=232×200)。
// 运行时优先从 getCalibParams() 读取,只有读取失败时才用这组值。
static const float FX_DEFAULT = 186.74f;
static const float FY_DEFAULT = 191.80f;
static const float CX_DEFAULT = 229.86f;
static const float CY_DEFAULT = 178.71f;
static const float TX_DEFAULT = 0.02443f;  // 基线(m),来自 Translation[0]=24.43mm

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
// ★ 必须直接 fopen 写,不能"读 stock yaml 改 DeviceNode":stock 的 DeviceNode 常已等于目标 device_id,
//   导致 done=false → 跳过写文件 → 返回一个不存在的路径 → UnitreeCamera 打开空 config → 堆损坏崩溃。
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
    m1("Algorithm", 1);
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

// SDK 的 getDepthFrame 在此 Nano 上返回全零深度(stereo compute 不可用)。
// 改用 getRectStereoFrame(left, right) 取矫正左右图,自己用 StereoSGBM 算视差,
// 再用 Q 矩阵 reprojectImageTo3D 反投影到 XYZ(米,相机系)。
static const float Z_MIN = 0.1f;   // 最近有效距离(m)
static const float Z_MAX = 10.0f;  // 最远有效距离(m)

// 构造标准 OpenCV Q 矩阵(stereo rectify 输出用)
//   Q = [1  0   0    -cx  ]
//       [0  1   0    -cy  ]
//       [0  0   0     fx  ]
//       [0  0  1/Tx    0  ]
// Q[3][2] = 1/Tx(正): disparity 为正 → w = d/Tx > 0 → z = fx/w > 0(前方)。
static cv::Mat make_Q(float fx, float cx, float cy, float Tx) {
    cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
    Q.at<double>(0, 0) = 1.0;
    Q.at<double>(1, 1) = 1.0;
    Q.at<double>(0, 3) = -cx;
    Q.at<double>(1, 3) = -cy;
    Q.at<double>(2, 3) = fx;
    Q.at<double>(3, 2) = 1.0 / Tx;
    return Q;
}

static void disp_to_xyz(const cv::Mat &disp, const cv::Mat &Q, int stride,
                        std::vector<float> &xyz_out) {
    xyz_out.clear();
    if (disp.empty() || Q.empty()) return;

    cv::Mat xyz_map;
    cv::reprojectImageTo3D(disp, xyz_map, Q, true);  // true=处理无效视差

    int rows = xyz_map.rows, cols = xyz_map.cols;
    int cnt_valid = 0, cnt_inf = 0, cnt_zneg = 0, cnt_zfar = 0;
    for (int v = 0; v < rows; v += stride) {
        for (int u = 0; u < cols; u += stride) {
            const cv::Vec3f &p = xyz_map.at<cv::Vec3f>(v, u);
            float x = p[0], y = -p[1], z = p[2];  // 翻转 y: 图像 v 向下→相机系 y 向上
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                cnt_inf++;
                continue;
            }
            if (z < 0) { cnt_zneg++; continue; }
            if (z < Z_MIN || z > Z_MAX) { cnt_zfar++; continue; }
            cnt_valid++;
            xyz_out.push_back(x);
            xyz_out.push_back(y);
            xyz_out.push_back(z);
        }
    }
    fprintf(stderr, "[pointcloud_stream] disp→xyz: valid=%d inf=%d z<0=%d z_far=%d "
                    "(total_samples=%d)\n",
            cnt_valid, cnt_inf, cnt_zneg, cnt_zfar,
            (rows / stride) * (cols / stride));
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
    cam.startStereoCompute();

    // getCalibParams 需要 startCapture 后等参数初始化完成(同 rgb_stream)
    usleep(200000);

    // 从 SDK 读内参和基线,动态构造 Q 矩阵
    // params[0]=LeftIntrinsicMatrix(fx/fy/cx/cy), params[4]=Translation(Tx,单位 mm)
    float fx = FX_DEFAULT, fy = FY_DEFAULT, cx = CX_DEFAULT, cy = CY_DEFAULT, Tx = TX_DEFAULT;
    {
        std::vector<cv::Mat> params;
        if (cam.getCalibParams(params) && params.size() >= 5
                && !params[0].empty() && !params[4].empty()) {
            const cv::Mat &K = params[0];
            const cv::Mat &T = params[4];
            if (K.rows >= 3 && K.cols >= 3) {
                fx = (float)K.at<double>(0, 0);
                fy = (float)K.at<double>(1, 1);
                cx = (float)K.at<double>(0, 2);
                cy = (float)K.at<double>(1, 2);
            }
            if (T.rows >= 1 && T.cols >= 1) {
                float tx_mm = (float)T.at<double>(0, 0);
                if (std::abs(tx_mm) > 1.0f)   // 单位是 mm(典型值 ~24mm)
                    Tx = tx_mm / 1000.0f;
                else                           // 已是 m(不太可能,但防御)
                    Tx = tx_mm;
            }
            fprintf(stderr, "[pointcloud_stream] dev%d 内参(getCalibParams): "
                    "fx=%.2f fy=%.2f cx=%.2f cy=%.2f Tx=%.4fm\n", device_id, fx, fy, cx, cy, Tx);
        } else {
            fprintf(stderr, "[pointcloud_stream] dev%d getCalibParams 失败,用兜底内参: "
                    "fx=%.2f fy=%.2f cx=%.2f cy=%.2f Tx=%.4fm\n", device_id, fx, fy, cx, cy, Tx);
        }
    }
    cv::Mat Q = make_Q(fx, cx, cy, Tx);

    // SGBM 算法配置
    int numDisparities = 64;   // 视差搜索范围,必须是 16 的倍数
    int blockSize = 5;         // 匹配块大小
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, numDisparities, blockSize);
    sgbm->setPreFilterCap(63);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setP1(8 * blockSize * blockSize);
    sgbm->setP2(32 * blockSize * blockSize);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    fprintf(stderr, "[pointcloud_stream] dev%d 相机已开,开始推流(stride=%d,SGBM→reprojectImageTo3D)\n",
            device_id, stride);

    // 首帧打印 left 类型供调试(只打一次)
    bool type_logged = false;
    int empty_streak = 0;

    std::vector<uint8_t> frame;
    while (cam.isOpened()) {
        cv::Mat left, right, feim;
        std::chrono::microseconds t;
        if (!cam.getRectStereoFrame(left, right, feim, t) || left.empty() || right.empty()) {
            empty_streak++;
            if (empty_streak % 200 == 1)
                fprintf(stderr, "[pointcloud_stream] dev%d stereo frame 为空(streak=%d),等待...\n",
                        device_id, empty_streak);
            usleep(5000);
            continue;
        }
        empty_streak = 0;

        if (!type_logged) {
            fprintf(stderr, "[pointcloud_stream] dev%d left: type=%d rows=%d cols=%d ch=%d\n",
                    device_id, left.type(), left.rows, left.cols, left.channels());
            type_logged = true;
        }

        // SGBM 需要单通道图
        cv::Mat left_gray, right_gray, disp;
        if (left.channels() > 1)
            cv::cvtColor(left, left_gray, cv::COLOR_BGR2GRAY);
        else
            left_gray = left;
        if (right.channels() > 1)
            cv::cvtColor(right, right_gray, cv::COLOR_BGR2GRAY);
        else
            right_gray = right;

        sgbm->compute(left_gray, right_gray, disp);
        // disp 是 CV_16SC1,转 CV_32FC1 并 /16(SGBM 输出固定小数位)
        disp.convertTo(disp, CV_32FC1, 1.0f / 16.0f);

        std::vector<float> xyz;
        disp_to_xyz(disp, Q, stride, xyz);

        uint32_t numPoints = (uint32_t)(xyz.size() / 3);
        uint32_t payloadLen = 4 + numPoints * 12;
        uint32_t beTotal = htonl(payloadLen);
        uint32_t beCount = htonl(numPoints);
        frame.clear();
        frame.resize(4 + payloadLen);
        std::memcpy(frame.data() + 0, &beTotal, 4);
        std::memcpy(frame.data() + 4, &beCount, 4);
        if (numPoints > 0)
            std::memcpy(frame.data() + 8, xyz.data(), numPoints * 12);
        if (!send_all(cli, frame.data(), frame.size())) break;
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