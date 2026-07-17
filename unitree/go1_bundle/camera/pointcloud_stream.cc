/**
 * pointcloud_stream.cc — 按需点云推流(test_camera_pointcloud 上游,Nano 端)。
 *
 * ★ 实现说明:
 *   getPointCloud 在 Nano 上不出数据(无 GPU 支持);改用 getDepthFrame 取彩色深度可视化图
 *   (SDK 只暴露 JET colormap BGR 图,无原始深度值),通过 BGR→HSV 取 H 通道反推相对深度:
 *       H=0(红)=近(Z_NEAR),H=120(绿)=远(Z_FAR);超出丢弃(全黑无效像素、蓝色超远区域)
 *   再按相机内参逐像素反投影到相机坐标系 XYZ(米):
 *       X = (u - cx) * Z / fx
 *       Y = (v - cy) * Z / fy
 *   内参从 SDK getCalibParams() 运行时读取(params[5]=kfe,校正后投影矩阵),兜底用出厂典型值。
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

// 相机内参兜底值(来自 .13 output_camCalibParams.yaml,RectifyFrameSize=464×400)。
// 运行时优先从 getCalibParams() 读取,只有读取失败时才用这组值。
static const float FX_DEFAULT = 187.39f;
static const float FY_DEFAULT = 192.99f;
static const float CX_DEFAULT = 207.55f;
static const float CY_DEFAULT = 199.70f;

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

// SDK 在此 Nano 上只返回 JET 彩色可视化深度图(CV_8UC3 BGR),无原始深度值接口。
// 从 JET 颜色反推深度:BGR→HSV,取 H 通道(0~180 in OpenCV)。
// JET 映射: H=0(红)=近, H=60(黄)=中近, H=120(绿)=中, H=180/0(蓝/回红)=远。
// 实测场景 B=0(无蓝),说明都在近~中段(H=0..~90)。
// 线性映射: H∈[0,120] → Z∈[Z_NEAR, Z_FAR];超出范围丢弃(无效像素通常 B=G=R=0)。
static const float Z_NEAR = 0.3f;   // H=0(红) 对应最近距离(m),可按实际标定调整
static const float Z_FAR  = 5.0f;   // H=120(绿) 对应最远距离(m)

static void project_depth_to_xyz(const cv::Mat &depth_bgr, int stride,
                                  float fx, float fy, float cx, float cy,
                                  std::vector<float> &xyz_out) {
    xyz_out.clear();
    if (depth_bgr.empty()) return;

    // 若将来 SDK 升级返回真实深度图,兼容处理
    if (depth_bgr.type() == CV_16UC1) {
        int rows = depth_bgr.rows, cols = depth_bgr.cols;
        xyz_out.reserve((rows * cols / (stride * stride) + 1) * 3);
        for (int v = 0; v < rows; v += stride) {
            for (int u = 0; u < cols; u += stride) {
                uint16_t raw = depth_bgr.at<uint16_t>(v, u);
                if (raw == 0) continue;
                float z = raw / 1000.0f;
                if (z < 0.05f || z > 20.0f) continue;
                xyz_out.push_back((u - cx) * z / fx);
                xyz_out.push_back((v - cy) * z / fy);
                xyz_out.push_back(z);
            }
        }
        return;
    }
    if (depth_bgr.type() == CV_32FC1) {
        int rows = depth_bgr.rows, cols = depth_bgr.cols;
        xyz_out.reserve((rows * cols / (stride * stride) + 1) * 3);
        for (int v = 0; v < rows; v += stride) {
            for (int u = 0; u < cols; u += stride) {
                float z = depth_bgr.at<float>(v, u);
                if (z <= 0.0f || !std::isfinite(z) || z > 20.0f) continue;
                xyz_out.push_back((u - cx) * z / fx);
                xyz_out.push_back((v - cy) * z / fy);
                xyz_out.push_back(z);
            }
        }
        return;
    }

    // CV_8UC3:JET 彩色图 → HSV → H 通道反推深度
    if (depth_bgr.channels() != 3) return;
    cv::Mat hsv;
    cv::cvtColor(depth_bgr, hsv, cv::COLOR_BGR2HSV);

    int rows = hsv.rows, cols = hsv.cols;
    xyz_out.reserve((rows * cols / (stride * stride) + 1) * 3);
    for (int v = 0; v < rows; v += stride) {
        for (int u = 0; u < cols; u += stride) {
            const cv::Vec3b &px_bgr = depth_bgr.at<cv::Vec3b>(v, u);
            if (px_bgr[0] == 0 && px_bgr[1] == 0 && px_bgr[2] == 0) continue;
            const cv::Vec3b &px_hsv = hsv.at<cv::Vec3b>(v, u);
            float h = px_hsv[0];  // OpenCV HSV: H∈[0,180]
            if (h > 120.0f) continue;
            float z = Z_NEAR + (h / 120.0f) * (Z_FAR - Z_NEAR);
            float x = (u - cx) * z / fx;
            float y = (v - cy) * z / fy;
            xyz_out.push_back(x);
            xyz_out.push_back(y);
            xyz_out.push_back(z);
        }
    }
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

    // 从 SDK 读校正后内参(kfe = params[5]),失败则用兜底值
    float fx = FX_DEFAULT, fy = FY_DEFAULT, cx = CX_DEFAULT, cy = CY_DEFAULT;
    {
        std::vector<cv::Mat> params;
        if (cam.getCalibParams(params) && params.size() >= 6 && !params[5].empty()) {
            // kfe 是 3×3 投影矩阵: [fx 0 cx; 0 fy cy; 0 0 1]
            const cv::Mat &kfe = params[5];
            if (kfe.rows >= 3 && kfe.cols >= 3) {
                fx = (float)kfe.at<double>(0, 0);
                fy = (float)kfe.at<double>(1, 1);
                cx = (float)kfe.at<double>(0, 2);
                cy = (float)kfe.at<double>(1, 2);
                fprintf(stderr, "[pointcloud_stream] dev%d 内参(getCalibParams): "
                        "fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n", device_id, fx, fy, cx, cy);
            }
        } else {
            fprintf(stderr, "[pointcloud_stream] dev%d getCalibParams 失败,用兜底内参: "
                    "fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n", device_id, fx, fy, cx, cy);
        }
    }

    fprintf(stderr, "[pointcloud_stream] dev%d 相机已开,开始推流(stride=%d,depth→XYZ 模式)\n",
            device_id, stride);

    // 首帧打印 depth mat 类型供调试(只打一次)
    bool type_logged = false;
    int empty_streak = 0;

    std::vector<uint8_t> frame;
    while (cam.isOpened()) {
        // 用单输出重载(已知 .13 能出帧;color=false 期望灰度,但实测可能仍是 CV_8UC3 彩色可视化)
        cv::Mat depth_raw;
        std::chrono::microseconds t;
        if (!cam.getDepthFrame(depth_raw, false, t) || depth_raw.empty()) {
            empty_streak++;
            if (empty_streak % 200 == 1)
                fprintf(stderr, "[pointcloud_stream] dev%d depth 帧为空(streak=%d),等待...\n",
                        device_id, empty_streak);
            usleep(5000);
            continue;
        }
        empty_streak = 0;

        if (!type_logged) {
            // 打印类型供调试,判断是彩色可视化(CV_8UC3)还是原始深度(CV_16UC1/CV_32FC1)
            double mn = 0, mx = 0;
            if (depth_raw.channels() == 1) {
                cv::minMaxLoc(depth_raw, &mn, &mx);
            } else {
                cv::Mat gray;
                cv::cvtColor(depth_raw, gray, cv::COLOR_BGR2GRAY);
                cv::minMaxLoc(gray, &mn, &mx);
            }
            fprintf(stderr, "[pointcloud_stream] dev%d depth: type=%d rows=%d cols=%d ch=%d "
                    "min=%.1f max=%.1f (8UC3=%d 16UC1=%d 32FC1=%d)%s\n",
                    device_id, depth_raw.type(), depth_raw.rows, depth_raw.cols,
                    depth_raw.channels(), mn, mx, CV_8UC3, CV_16UC1, CV_32FC1,
                    depth_raw.channels() == 3 ? " [JET彩色→HSV反推Z]" : "");
            type_logged = true;
        }

        std::vector<float> xyz;
        project_depth_to_xyz(depth_raw, stride, fx, fy, cx, cy, xyz);

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