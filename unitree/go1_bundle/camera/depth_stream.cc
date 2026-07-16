/**
 * depth_stream.cc — 按需深度推流(test_camera_depth 上游,Nano 端)。
 *
 * ★ 相机初始化:必须走 UnitreeCamera(config_file)(会加载立体标定),深度才出得来。
 *   (SDK 自带 example_getDepthFrame 也是 UnitreeCamera("stereo_camera_config.yaml");用设备号构造
 *    只能取原始帧,startStereoCompute/getDepthFrame 无标定 → 连上无帧。见 pointcloud_stream 同款修复。)
 *   本程序按 device_id **自动生成一份最小 config**(镜像 pointcloud_stream:只填 DeviceNode+尺寸,
 *   标定从相机 flash 加载)→ 免外部 config 文件,一份二进制服务任意一路相机。
 * ★ 热切/不常占:相机"客户端连上才开、断开就释放"。UnitreeCamera 作用域限单次连接,断开即析构释放。
 *   于是本程序可常驻挂 systemd(空闲不占相机);画布拖入 test_camera_depth 卡 → Pi 卡连上 → 才开相机;
 *   卡 stop / 断开 → 相机立即释放。**无需常占、免重启热切。**
 * ★ 抢占:开相机前 fuser -k /dev/video<device_id> 释放占用者(出厂 point_cloud_node / pointcloud_stream 等)。
 *
 * 协议:每帧 = [4字节大端长度 N][N 字节 JPEG 数据](与 test_camera_depth.py 桥接约定一致)
 *   ★ 用 JPEG(不是 PNG):画布只渲染 image/jpeg 的 CompressedImage(对齐 camera_rgb);
 *     彩色深度图是 3 通道 BGR 可视化(给人看,非原始深度值),JPEG 有损无妨。
 *
 * 编译(nano_bootstrap 自动做):
 *   g++ -O2 -std=c++14 -pthread depth_stream.cc -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 \
 *     -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \
 *     $(pkg-config --cflags --libs opencv4) -o bins/depth_stream
 * 用法:depth_stream <port> <device_id>
 *   例:./bins/depth_stream 9105 0     # belly(.15 dev0),端口 9105
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
// 镜像 pointcloud_stream 的 write_config(点云已验证出帧);depth 走同一立体管线,同款 config。
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
    m1("IpLastSegment", 15);       // 不传图,值无关
    m1("DeviceNode", (double)device_id);
    m1("hFov", 90);
    fprintf(f, "FrameSize: !!opencv-matrix\n   rows: 1\n   cols: 2\n   dt: d\n   data: [ 928., 400. ]\n");
    fprintf(f, "RectifyFrameSize: !!opencv-matrix\n   rows: 1\n   cols: 2\n   dt: d\n   data: [ 464., 400. ]\n");
    m1("FrameRate", 30);
    m1("Transmode", -1);           // -1 = 不传图(只本地算深度)
    m1("Transrate", 30);
    m1("Depthmode", 1);
    fclose(f);
    return path;
}

// 释放该 device 节点的占用者(出厂 point_cloud_node / pointcloud_stream 等),否则 SDK 打不开。
static void free_device(int device_id) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "fuser -k /dev/video%d >/dev/null 2>&1", device_id);
    (void)system(cmd);
    usleep(500000);
}

// 单次连接:开相机(生成的 config,含标定)→ 推深度 JPEG 直到对端断开 → 返回(相机随 cam 析构释放)。
static void serve_client(int cli, int device_id) {
    std::string cfg = write_config(device_id);
    if (cfg.empty()) { fprintf(stderr, "[depth_stream] 生成 config 失败\n"); return; }
    free_device(device_id);
    UnitreeCamera cam(cfg);                       // [SDK-API] 配置文件构造 → 加载立体标定(深度必需)
    for (int attempt = 0; attempt < 3 && !cam.isOpened(); ++attempt) {
        fprintf(stderr, "[depth_stream] dev%d 未就绪,重试 %d...\n", device_id, attempt + 1);
        free_device(device_id);
        sleep(1);
    }
    if (!cam.isOpened()) {
        fprintf(stderr, "[depth_stream] dev%d 打开失败(被占用/不可用),放弃本连接\n", device_id);
        return;
    }
    cam.startCapture();
    cam.startStereoCompute();
    fprintf(stderr, "[depth_stream] dev%d 相机已开,开始推流\n", device_id);

    std::vector<int> jpgparams = {cv::IMWRITE_JPEG_QUALITY, 85};
    while (cam.isOpened()) {
        cv::Mat depth;
        std::chrono::microseconds t;
        if (!cam.getDepthFrame(depth, true, t) || depth.empty()) {
            usleep(2000);
            continue;
        }
        std::vector<uchar> buf;
        cv::imencode(".jpg", depth, buf, jpgparams);
        uint32_t n = htonl((uint32_t)buf.size());
        if (!send_all(cli, reinterpret_cast<uint8_t *>(&n), 4)) break;   // 对端断开
        if (!send_all(cli, buf.data(), buf.size())) break;
        usleep(50000);   // ~20Hz 上限(实际受深度计算限速)
    }
    cam.stopStereoCompute();
    cam.stopCapture();
    fprintf(stderr, "[depth_stream] dev%d 客户端断开,已释放相机\n", device_id);
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "用法: %s <port> <device_id>\n", argv[0]);
        _exit(1);
    }
    int port      = atoi(argv[1]);
    int device_id = atoi(argv[2]);
    signal(SIGPIPE, SIG_IGN);   // 客户端断开时不因写坏管道而崩

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(srv, (sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); _exit(4); }
    listen(srv, 1);
    fprintf(stderr, "[depth_stream] 空闲待命(dev%d,相机未开),监听 0.0.0.0:%d ...\n", device_id, port);

    while (true) {
        int cli = accept(srv, nullptr, nullptr);   // 无连接时不占相机
        if (cli < 0) continue;
        fprintf(stderr, "[depth_stream] 客户端已连接 → 开 dev%d\n", device_id);
        serve_client(cli, device_id);
        close(cli);
        fprintf(stderr, "[depth_stream] 回到空闲待命(相机已释放),等待下一次连接...\n");
    }
    return 0;
}
