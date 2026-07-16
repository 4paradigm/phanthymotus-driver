/**
 * pointcloud_stream.cc — 按需点云推流(test_camera_pointcloud 上游,Nano 端)。
 *
 * ★ 热切设计:相机**不在启动时打开**,而是"**有客户端连上才开、客户端断开就释放**"。
 *   UnitreeCamera 对象作用域限定在单次连接内,断开时析构 → 彻底释放 /dev/videoN。
 *   于是可在每块板为每路相机常驻挂一个本程序(空闲时不占相机);画布切到哪一路,
 *   Pi 侧 test_camera_pointcloud 断开旧连接、连上新端口 → 对应 streamer 自动开相机推流,
 *   旧的自动释放。**无需重启、无需手动干预,即可在 5 路间热切**(一次一路)。
 *
 * 协议(每帧):[4字节大端 totalLen][totalLen 字节 payload]
 *            payload = [4字节大端 numPoints][numPoints × 3 × float32 (小端, x/y/z 米,相机系)]
 *
 * 约束:立体计算吃 Nano CPU + 同一 device 独占。5 路分布在 3 块板(.13=front+chin、
 *   .14=left+right、.15=belly)。热切是"选 1 路",非"5 路同开";同板两路(不同 device)
 *   物理上是不同 /dev/videoN,但同时跑两个立体计算会压垮该板 CPU → 靠"一次只连一路"避免。
 *   头部(.13)与 depth_stream 若指向同一 device 则互斥(同一 device 只能被一个进程打开)。
 *   切换时相机 SDK 初始化约 3~4s,期间无帧属正常。
 *
 * 编译:放进 SDK examples/,CMakeLists 加 add_executable(pointcloud_stream ...)+link ${SDKLIBS}
 * 运行(每路一个,可常驻;config 指向该 device,端口按机位):
 *   ./bins/pointcloud_stream stereo_camera_config_front.yaml 9401 4   # .13 front(dev1)
 *   ./bins/pointcloud_stream stereo_camera_config_chin.yaml  9402 4   # .13 chin (dev0)
 */
#include <UnitreeCameraSDK.hpp>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <vector>
#include <string>

static bool send_all(int fd, const uint8_t *p, size_t n) {
    size_t sent = 0;
    while (sent < n) {
        ssize_t k = send(fd, p + sent, n - sent, MSG_NOSIGNAL);
        if (k <= 0) return false;
        sent += (size_t)k;
    }
    return true;
}

// 单次连接:开相机 → 推点云直到对端断开 → 返回(相机随 cam 析构释放)。
static void serve_client(int cli, const std::string &cfg, int stride) {
    UnitreeCamera cam(cfg);
    // 相机可能刚被上一路释放,给几次重试窗口
    for (int attempt = 0; attempt < 3 && !cam.isOpened(); ++attempt) {
        fprintf(stderr, "[pointcloud_stream] 相机未就绪,重试 %d...\n", attempt + 1);
        sleep(1);
    }
    if (!cam.isOpened()) {
        fprintf(stderr, "[pointcloud_stream] 相机打开失败(被占用/不可用),放弃本连接\n");
        return;
    }
    cam.startCapture();
    cam.startStereoCompute();
    fprintf(stderr, "[pointcloud_stream] 相机已开,开始推流(stride=%d)\n", stride);

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

        if (!send_all(cli, frame.data(), frame.size())) break;   // 对端断开 → 结束本连接
        usleep(100000);   // ~10Hz 上限
    }
    cam.stopStereoCompute();
    cam.stopCapture();
    // cam 析构 → 释放 /dev/videoN,让别的机位/进程可开
    fprintf(stderr, "[pointcloud_stream] 客户端断开,已释放相机\n");
}

int main(int argc, char *argv[]) {
    std::string cfg = (argc > 1) ? argv[1] : "stereo_camera_config.yaml";
    int port   = (argc > 2) ? atoi(argv[2]) : 9401;
    int stride = (argc > 3) ? atoi(argv[3]) : 4;
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
    fprintf(stderr, "[pointcloud_stream] 空闲待命(相机未开),监听 0.0.0.0:%d ...\n", port);

    while (true) {
        int cli = accept(srv, nullptr, nullptr);   // 无连接时不占相机
        if (cli < 0) continue;
        fprintf(stderr, "[pointcloud_stream] 客户端已连接 → 开相机\n");
        serve_client(cli, cfg, stride);
        close(cli);
        fprintf(stderr, "[pointcloud_stream] 回到空闲待命(相机已释放),等待下一次连接...\n");
    }
    return 0;
}
