/**
 * depth_stream.cc — 持续深度推流(camera_depth 推流版,NX 端)。
 * 相机常开,循环 getDepthFrame,把每帧 PNG 通过 TCP 发给连上来的客户端(树莓派上的 rclpy 桥)。
 * 与 depth_grab(单帧)不同:相机只开一次,持续出帧 → ~10Hz 真视频流。
 *
 * 协议:每帧 = [4字节大端长度 N][N 字节 PNG 数据]
 * 编译:放进 SDK examples/,CMakeLists 加 add_executable(depth_stream ...)+link ${SDKLIBS},cmake+make
 * 运行(需先释放相机):./bins/depth_stream stereo_camera_config.yaml 9101
 */
#include <UnitreeCameraSDK.hpp>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <vector>
#include <string>

int main(int argc, char *argv[]) {
    std::string cfg = (argc > 1) ? argv[1] : "stereo_camera_config.yaml";
    int port = (argc > 2) ? atoi(argv[2]) : 9101;
    signal(SIGPIPE, SIG_IGN);   // 客户端断开时不因写坏管道而崩

    UnitreeCamera cam(cfg);
    if (!cam.isOpened()) {
        fprintf(stderr, "[depth_stream] 相机打开失败(被占用?)\n");
        _exit(2);
    }
    cam.startCapture();
    cam.startStereoCompute();
    fprintf(stderr, "[depth_stream] 相机已开,监听 0.0.0.0:%d ...\n", port);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(srv, (sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); _exit(4); }
    listen(srv, 1);

    while (true) {
        fprintf(stderr, "[depth_stream] 等待客户端连接...\n");
        int cli = accept(srv, nullptr, nullptr);
        if (cli < 0) continue;
        fprintf(stderr, "[depth_stream] 客户端已连接,开始推流\n");
        std::vector<int> pngparams = {cv::IMWRITE_PNG_COMPRESSION, 1};
        bool alive = true;
        while (alive && cam.isOpened()) {
            cv::Mat depth;
            std::chrono::microseconds t;
            if (!cam.getDepthFrame(depth, true, t) || depth.empty()) {
                usleep(2000);
                continue;
            }
            std::vector<uchar> buf;
            cv::imencode(".png", depth, buf, pngparams);
            uint32_t n = htonl((uint32_t)buf.size());
            if (send(cli, &n, 4, MSG_NOSIGNAL) != 4) { alive = false; break; }
            size_t sent = 0;
            while (sent < buf.size()) {
                ssize_t k = send(cli, buf.data() + sent, buf.size() - sent, MSG_NOSIGNAL);
                if (k <= 0) { alive = false; break; }
                sent += k;
            }
            usleep(50000);   // ~20Hz 上限(实际受深度计算限速)
        }
        close(cli);
        fprintf(stderr, "[depth_stream] 客户端断开,回到等待\n");
    }
    return 0;
}
