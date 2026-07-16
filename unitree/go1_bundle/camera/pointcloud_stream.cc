/**
 * pointcloud_stream.cc — 按需点云推流(test_camera_pointcloud 上游,Nano 端)。
 *
 * ★ 相机初始化:必须走 UnitreeCamera(config_file)(会加载立体标定),点云才出得来。
 *   (SDK 自带 example_getPointCloud 也是 UnitreeCamera("stereo_camera_config.yaml");用设备号构造
 *    只能取原始帧,startStereoCompute/getPointCloud 无标定 → 不出点。)
 *   本程序按 device_id **自动生成一份最小 config**(镜像 camera_adapter 的做法:只填 DeviceNode+尺寸,
 *   标定从相机 flash 加载)→ 免外部 config 文件,一份二进制服务任意一路相机。
 * ★ 热切:相机"客户端连上才开、断开就释放"。UnitreeCamera 作用域限单次连接,断开即析构释放设备。
 * ★ 抢占:开相机前 fuser -k /dev/video<device_id> 释放占用者(出厂 point_cloud_node/depth_stream 等)。
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
    std::string path = "/tmp/pcl_dev" + std::to_string(device_id) + ".yaml";
    FILE *f = fopen(path.c_str(), "w");
    if (!f) return "";
    fprintf(f, "%%YAML:1.0\n---\n");
    auto m1 = [&](const char *k, double v) {
        fprintf(f, "%s: !!opencv-matrix\n   rows: 1\n   cols: 1\n   dt: d\n   data: [ %g ]\n", k, v);
    };
    m1("LogLevel", 1);
    m1("Threshold", 190);
    m1("Algorithm", 1);
    m1("IpLastSegment", 15);       // 不传图,值无关
    m1("DeviceNode", (double)device_id);
    m1("hFov", 90);
    fprintf(f, "FrameSize: !!opencv-matrix\n   rows: 1\n   cols: 2\n   dt: d\n   data: [ 928., 400. ]\n");
    fprintf(f, "RectifyFrameSize: !!opencv-matrix\n   rows: 1\n   cols: 2\n   dt: d\n   data: [ 464., 400. ]\n");
    m1("FrameRate", 30);
    m1("Transmode", -1);           // -1 = 不传图(只本地算点云)
    m1("Transrate", 30);
    m1("Depthmode", 1);
    fclose(f);
    return path;
}

// 释放该 device 节点的占用者(出厂 point_cloud_node / depth_stream 等),否则 SDK 打不开。
static void free_device(int device_id) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "fuser -k /dev/video%d >/dev/null 2>&1", device_id);
    (void)system(cmd);
    usleep(500000);
}

// 单次连接:开相机(生成的 config,含标定)→ 推点云直到对端断开 → 返回(相机随 cam 析构释放)。
static void serve_client(int cli, int device_id, int stride) {
    std::string cfg = write_config(device_id);
    if (cfg.empty()) { fprintf(stderr, "[pointcloud_stream] 生成 config 失败\n"); return; }
    free_device(device_id);
    UnitreeCamera cam(cfg);                       // [SDK-API] 配置文件构造 → 加载立体标定(点云必需)
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
