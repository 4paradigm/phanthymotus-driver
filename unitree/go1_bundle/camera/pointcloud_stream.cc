/**
 * pointcloud_stream.cc — 按需点云推流(test_camera_pointcloud 上游,Nano 端)。
 *
 * ★ 选相机:按 device_id 开(镜像 camera_adapter.cpp 的 UnitreeCamera(device_id) 构造),
 *   不依赖 per-device 的 config 文件 → 一份二进制可服务任意一路相机。
 * ★ 热切:相机"客户端连上才开、断开就释放"。UnitreeCamera 作用域限单次连接,断开即析构释放
 *   /dev/videoN。于是每路常驻挂一个本程序(空闲不占相机);画布切机位 → Pi 卡断旧连新 →
 *   对应 streamer 自动开/放相机。**无需重启即可在 5 路间热切**(一次一路)。
 * ★ 抢占:开相机前 fuser -k 释放该 device 节点的占用者(镜像 adapter 的 free_device_node),
 *   否则出厂 point_cloud_node / depth_stream 占着会打不开。
 *
 * 协议(每帧):[4字节大端 totalLen][totalLen 字节 payload]
 *            payload = [4字节大端 numPoints][numPoints × 3 × float32 (小端, x/y/z 米,相机系)]
 *
 * 约束:立体计算吃 Nano CPU + device 独占。5 路分布在 3 块板(.13=front dev1/chin dev0、
 *   .14=left dev0/right dev1、.15=belly dev0)。热切是"选 1 路"。头部/腹部与 depth_stream
 *   若指向同一 device 则互斥(fuser 会顶掉对方)。切换时相机 SDK 初始化约 3~4s,期间无帧属正常。
 *
 * 用法:pointcloud_stream <port> <device_id> [stride]
 *   例:./bins/pointcloud_stream 9401 1 4     # front(.13 dev1),端口 9401,抽稀 4
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

static bool send_all(int fd, const uint8_t *p, size_t n) {
    size_t sent = 0;
    while (sent < n) {
        ssize_t k = send(fd, p + sent, n - sent, MSG_NOSIGNAL);
        if (k <= 0) return false;
        sent += (size_t)k;
    }
    return true;
}

// 释放该 device 节点的占用者(出厂 point_cloud_node / depth_stream 等),否则 SDK 打不开。
static void free_device(int device_id) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "fuser -k /dev/video%d >/dev/null 2>&1", device_id);
    if (system(cmd) == 0) { /* 有占用者被杀 */ }
    usleep(500000);
}

// 单次连接:开相机(device_id)→ 推点云直到对端断开 → 返回(相机随 cam 析构释放)。
static void serve_client(int cli, int device_id, int stride) {
    free_device(device_id);
    UnitreeCamera cam(device_id);                 // [SDK-API] 按设备节点号构造(同 camera_adapter)
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
    fprintf(stderr, "[pointcloud_stream] dev%d 相机已开,开始推流(stride=%d)\n", device_id, stride);

    std::vector<uint8_t> frame;
    while (cam.isOpened()) {
        std::vector<cv::Vec3f> pcl;
        std::chrono::microseconds t;
        if (!cam.getPointCloud(pcl, t) || pcl.empty()) {
            usleep(2000);
            continue;
        }
        std::vector<float> xyz;
        xyz.reserve((pcl.size() / (size_t)stride + 1) * 3);
        for (size_t i = 0; i < pcl.size(); i += (size_t)stride) {
            const cv::Vec3f &p = pcl[i];
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) continue;
            if (p[0] == 0.0f && p[1] == 0.0f && p[2] == 0.0f) continue;
            xyz.push_back(p[0]); xyz.push_back(p[1]); xyz.push_back(p[2]);
        }
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
        if (!send_all(cli, frame.data(), frame.size())) break;   // 对端断开
        usleep(100000);   // ~10Hz 上限
    }
    cam.stopStereoCompute();
    cam.stopCapture();
    fprintf(stderr, "[pointcloud_stream] dev%d 客户端断开,已释放相机\n", device_id);
}

int main(int argc, char *argv[]) {
    int port      = (argc > 1) ? atoi(argv[1]) : 9401;
    int device_id = (argc > 2) ? atoi(argv[2]) : 0;
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
    fprintf(stderr, "[pointcloud_stream] 空闲待命(dev%d,相机未开),监听 0.0.0.0:%d ...\n", device_id, port);

    while (true) {
        int cli = accept(srv, nullptr, nullptr);   // 无连接时不占相机
        if (cli < 0) continue;
        fprintf(stderr, "[pointcloud_stream] 客户端已连接 → 开 dev%d\n", device_id);
        serve_client(cli, device_id, stride);
        close(cli);
        fprintf(stderr, "[pointcloud_stream] 回到空闲待命(相机已释放),等待下一次连接...\n");
    }
    return 0;
}
