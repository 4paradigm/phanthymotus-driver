/**
 * depth_stream.cc — 按需深度推流(Nano 端),双路输出:
 *   1. 原JPEG端口(91xx):彩色深度JPEG,完全兼容现有画布显示功能
 *   2. 新增RAW端口(91xx+200=93xx):原始16UC1深度数据(毫米单位),用于避障算法
 *
 * 完全向下兼容:原有启动命令/服务/客户端无需任何修改,升级二进制即自动双路推流
 * 相机仅打开一次,不重复占用资源,任意端口有客户端连接即开相机,所有客户端断开才释放
 *
 * 协议:
 * · JPEG端口:每帧 = [4字节大端长度 N][N 字节 JPEG 数据](兼容旧版)
 * · RAW端口:每帧 = [4字节大端宽度][4字节大端高度][W*H*2字节 16UC1 原始深度,单位mm]
 */
#include <UnitreeCameraSDK.hpp>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

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
static std::string write_config(int device_id) {
    std::string path = "/tmp/depth_dev" + std::to_string(device_id) + ".yaml";
    FILE *f = fopen(path.c_str(), "w");
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
    return path;
}

// 释放该 device 节点的占用者
static void free_device(int device_id) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "fuser -k /dev/video%d >/dev/null 2>&1", device_id);
    (void)system(cmd);
    usleep(500000);
}

// 全局客户端管理
std::mutex cli_mtx;
std::vector<int> jpeg_clients;
std::vector<int> raw_clients;
std::atomic<bool> cam_running{false};
std::atomic<int> active_clients{0};

// 客户端接收线程:只负责accept新连接
void accept_thread(int srv, bool is_jpeg, int device_id) {
    while (true) {
        int cli = accept(srv, nullptr, nullptr);
        if (cli < 0) continue;

        {
            std::lock_guard<std::mutex> lock(cli_mtx);
            if (is_jpeg) jpeg_clients.push_back(cli);
            else raw_clients.push_back(cli);
        }
        active_clients++;
        fprintf(stderr, "[depth_stream] dev%d %s客户端已连接,总客户端数:%d\n",
                device_id, is_jpeg ? "JPEG" : "RAW", active_clients.load());
    }
}

// 深度转彩色JPEG(参考SDK默认配色)
void depth_to_color(const cv::Mat& raw, cv::Mat& color) {
    double min_val, max_val;
    cv::minMaxLoc(raw, &min_val, &max_val, nullptr, nullptr);
    if (max_val - min_val < 1) max_val = min_val + 1;
    cv::Mat normalized;
    raw.convertTo(normalized, CV_8UC1, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
    cv::applyColorMap(normalized, color, cv::COLORMAP_JET);
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "用法: %s <jpeg_port> <device_id>\n", argv[0]);
        fprintf(stderr, "  自动启动RAW端口: jpeg_port + 200 (如9105对应9305)\n");
        _exit(1);
    }
    int jpeg_port   = atoi(argv[1]);
    int raw_port    = jpeg_port + 200; // RAW端口自动偏移200,91xx→93xx
    int device_id   = atoi(argv[2]);
    signal(SIGPIPE, SIG_IGN);

    // 创建JPEG监听socket
    int srv_jpeg = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv_jpeg, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr_jpeg{};
    addr_jpeg.sin_family = AF_INET;
    addr_jpeg.sin_addr.s_addr = INADDR_ANY;
    addr_jpeg.sin_port = htons(jpeg_port);
    if (bind(srv_jpeg, (sockaddr *)&addr_jpeg, sizeof(addr_jpeg)) < 0) { perror("bind jpeg"); _exit(4); }
    listen(srv_jpeg, 5);

    // 创建RAW监听socket
    int srv_raw = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(srv_raw, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr_raw{};
    addr_raw.sin_family = AF_INET;
    addr_raw.sin_addr.s_addr = INADDR_ANY;
    addr_raw.sin_port = htons(raw_port);
    if (bind(srv_raw, (sockaddr *)&addr_raw, sizeof(addr_raw)) < 0) { perror("bind raw"); _exit(4); }
    listen(srv_raw, 5);

    // 启动accept线程
    std::thread(accept_thread, srv_jpeg, true, device_id).detach();
    std::thread(accept_thread, srv_raw, false, device_id).detach();

    fprintf(stderr, "[depth_stream] 空闲待命(dev%d),监听JPEG端口:%d, RAW端口:%d ...\n",
            device_id, jpeg_port, raw_port);

    std::vector<int> jpgparams = {cv::IMWRITE_JPEG_QUALITY, 85};
    UnitreeCamera *cam = nullptr;

    while (true) {
        // 等待客户端连接
        while (active_clients.load() == 0) {
            usleep(100000);
            // 如果相机已开,客户端都断开了,释放相机
            if (cam != nullptr) {
                fprintf(stderr, "[depth_stream] dev%d 所有客户端断开,释放相机回到待命\n", device_id);
                cam->stopCapture();
                cam->stopStereoCompute();
                delete cam;
                cam = nullptr;
                cam_running = false;
            }
        }

        // 有客户端连接,开相机
        if (cam == nullptr) {
            fprintf(stderr, "[depth_stream] dev%d 有客户端连接,打开相机\n", device_id);
            std::string cfg = write_config(device_id);
            free_device(device_id);
            cam = new UnitreeCamera(cfg);
            for (int attempt = 0; attempt < 3 && !cam->isOpened(); ++attempt) {
                fprintf(stderr, "[depth_stream] dev%d 未就绪,重试 %d...\n", device_id, attempt + 1);
                free_device(device_id);
                sleep(1);
            }
            if (!cam->isOpened()) {
                fprintf(stderr, "[depth_stream] dev%d 打开失败\n", device_id);
                delete cam;
                cam = nullptr;
                active_clients = 0;
                // 清理所有客户端
                std::lock_guard<std::mutex> lock(cli_mtx);
                for (int c : jpeg_clients) close(c);
                for (int c : raw_clients) close(c);
                jpeg_clients.clear();
                raw_clients.clear();
                continue;
            }
            cam->startCapture();
            cam->startStereoCompute();
            cam_running = true;
            fprintf(stderr, "[depth_stream] dev%d 相机已开,开始双路推流\n", device_id);
        }

        // 取原始16UC1深度帧
        cv::Mat raw_depth;
        std::chrono::microseconds t;
        if (!cam->getDepthFrame(raw_depth, false, t) || raw_depth.empty() || raw_depth.type() != CV_16UC1) {
            usleep(2000);
            continue;
        }

        // 生成彩色JPEG帧(给JPEG客户端)
        cv::Mat color_depth;
        depth_to_color(raw_depth, color_depth);
        std::vector<uchar> jpeg_buf;
        cv::imencode(".jpg", color_depth, jpeg_buf, jpgparams);
        uint32_t jpeg_len = htonl((uint32_t)jpeg_buf.size());

        // 生成RAW帧(给RAW客户端):[4B width][4B height][raw data]
        uint32_t w = htonl((uint32_t)raw_depth.cols);
        uint32_t h = htonl((uint32_t)raw_depth.rows);
        uint32_t raw_w = raw_depth.cols;
        uint32_t raw_h = raw_depth.rows;
        size_t raw_data_len = raw_w * raw_h * 2;

        // 发送给所有JPEG客户端
        std::vector<int> alive_jpeg, alive_raw;
        {
            std::lock_guard<std::mutex> lock(cli_mtx);
            // 发JPEG
            for (int fd : jpeg_clients) {
                bool ok = true;
                ok &= send_all(fd, reinterpret_cast<uint8_t *>(&jpeg_len), 4);
                ok &= send_all(fd, jpeg_buf.data(), jpeg_buf.size());
                if (ok) alive_jpeg.push_back(fd);
                else { close(fd); active_clients--; }
            }
            // 发RAW
            for (int fd : raw_clients) {
                bool ok = true;
                ok &= send_all(fd, reinterpret_cast<uint8_t *>(&w), 4);
                ok &= send_all(fd, reinterpret_cast<uint8_t *>(&h), 4);
                ok &= send_all(fd, raw_depth.data, raw_data_len);
                if (ok) alive_raw.push_back(fd);
                else { close(fd); active_clients--; }
            }
            jpeg_clients.swap(alive_jpeg);
            raw_clients.swap(alive_raw);
        }

        usleep(50000); // ~20Hz
    }
    return 0;
}