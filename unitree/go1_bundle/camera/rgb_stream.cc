/**
 * rgb_stream.cc — 按需 RGB 推流(camera_rgb 上游,Nano 端)。
 *
 * 解决旧 camera_adapter 路线在 15678 上的三个问题:
 *   1) 只 front/chin 出图、left/right/belly 收不到 → 旧路线靠 UDP 图传口 + JSON 控制口双通道,
 *      .14/.15 上没装 adapter 服务就没流。本程序和 depth_stream 同套路:一份二进制 + 每路一个
 *      systemd 服务(9201~9205),Nano 现编,五路都能起。
 *   2) 鱼眼畸变 → 用 getRawFrame 取原始帧 + getCalibParams 从相机闪存读标定(K/D/xi) +
 *      按 CMei 投影方程手算 remap 映射表(零 opencv_contrib 依赖) → 完全透视平面图,无桶形畸变。
 *   3) 上下颠倒 → Go1 相机物理装反,remap 后 cv::flip(out, out, -1) 旋转 180° 翻正。
 *
 * ★ 去鱼眼管线(v2): getRawFrame(60fps) + CMei undistort remap,取代旧版 getRectStereoFrame。
 *   旧版用 startStereoCompute + getRectStereoFrame 做立体校正 → 残留桶形畸变+黑角+5-6s暖机。
 *   新版:
 *     · startCapture 后 getCalibParams() 从相机闪存读标定(K,D,xi) → 无需外部 YAML
 *     · build_undistort_maps() 按 CMei 投影方程手算映射表(搬自 camera_adapter.cpp,已验证逐像素0误差)
 *     · getRawFrame 取原始双目帧 → 裁左目 → cv::remap 去鱼眼 → cv::flip 翻正 → 自动裁黑边 → JPEG
 *     · 无 startStereoCompute → 零暖机、帧率从~14-30fps 提升到 30-60fps
 *     · focal_scale 控制输出 FOV,auto_crop 自动裁掉黑角 → 输出完整填充的无畸变平面图
 *   标定读取失败时 fallback 到旧版 getRectStereoFrame(向后兼容)。
 *
 * ★ 热切/不常占:相机"客户端连上才开、断开就释放"。客户端断开后本进程 _exit(0)(绕开 SDK 析构
 *   的 double-free 崩溃,同 depth_stream),进程退出即释放 /dev/videoN,由 systemd 重启回到空闲待命
 *   → 仍是"断开就放相机"。于是本程序可常驻挂 systemd(空闲不占相机);camera_rgb 卡 start → Pi 卡
 *   连上 → 才开相机;卡 stop / 断开 → 相机立即释放。**无需常占、免重启热切、五路各自独立服务。**
 * ★ 抢占:开相机前 fuser -k /dev/video<device_id> 释放占用者(出厂 point_cloud_node / pointcloud_stream
 *   / depth_stream 等)。与同机位的 pointcloud/depth 互斥(一相机一消费者,谁连上谁占,属预期)。
 *
 * 协议:每帧 = [4字节大端长度 N][N 字节 JPEG 数据](与 camera_rgb.py 桥接约定一致,镜像 depth_stream)
 *   用 JPEG:画布只渲染 image/jpeg 的 CompressedImage。
 *
 * 编译(nano_bootstrap 自动做):
 *   g++ -O2 -std=c++14 -pthread rgb_stream.cc -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 \
 *     -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group \
 *     $(pkg-config --cflags --libs opencv4) -o bins/rgb_stream
 * 用法:rgb_stream <port> <device_id>
 *   例:./bins/rgb_stream 9203 0     # left(.14 dev0),端口 9203
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
#include <cmath>

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
// 镜像 depth_stream / pointcloud_stream 的 write_config:只填 DeviceNode+尺寸,标定靠 flash。
static std::string write_config(int device_id) {
    std::string path = "/tmp/rgb_dev" + std::to_string(device_id) + ".yaml";
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
    m1("Transmode", -1);           // -1 = 不传图(只本地取帧)
    m1("Transrate", 30);
    m1("Depthmode", 1);            // 走立体管线才有 rectified;不实际算深度
    fclose(f);
    return path;
}

// 释放该 device 节点的占用者(出厂 point_cloud_node / pointcloud_stream / depth_stream 等),否则 SDK 打不开。
static void free_device(int device_id) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "fuser -k /dev/video%d >/dev/null 2>&1", device_id);
    (void)system(cmd);
    usleep(500000);
}

// ── CMei 去鱼眼:从 getCalibParams 读标定 → 手算 remap 映射表 → 自动裁黑边 ──────────────

// 去鱼眼状态(每次连接初始化一次,serve_client 内使用)
struct UndistortState {
    cv::Mat map1, map2;         // CV_16SC2 定点映射表
    cv::Rect crop;              // 自动裁切框(去黑角)
    bool ok = false;            // 映射表是否构建成功
};

// focal_scale 控制输出 FOV:值越大 → FOV 越窄 → 黑角越少。
// 0.45 对 Go1 xi≈0.5 时基本零黑角且保留足够视野;auto_crop 会二次精裁残余黑边。
static const double FOCAL_SCALE = 0.45;

// 按 CMei 投影方程手算 remap 映射表(搬自 camera_adapter.cpp build_undistort_maps,已验证逐像素0误差)。
// Go1 相机是 CMei 全向模型(标定带 xi),而板卡 opencv4(4.1.1) 无 ccalib/omnidir 头,
// cv::fisheye 又是等距模型(不同)套上会算错 → 手算零 contrib 依赖。
// 对每个输出像素:反投影射线→单位球→加 xi 投归一化平面→径向+切向畸变→内参(含 skew)得源鱼眼像素。
// 输出: map1_out/map2_out = CV_16SC2 定点映射表(remap 用); mapx_f/mapy_f = float 映射表(裁切计算用)。
static bool build_undistort_maps(const cv::Mat &K, const cv::Mat &D, double xi,
                                 int W, int H, double focal_scale,
                                 cv::Mat &map1_out, cv::Mat &map2_out,
                                 cv::Mat &mapx_f, cv::Mat &mapy_f) {
    if (K.empty() || D.total() < 4) return false;
    cv::Mat Kd, Dd;
    if (K.type() != CV_64F) K.convertTo(Kd, CV_64F); else Kd = K;
    if (D.type() != CV_64F) D.convertTo(Dd, CV_64F); else Dd = D;
    Dd = Dd.reshape(1, 1);

    const double fx = Kd.at<double>(0, 0), fy = Kd.at<double>(1, 1);
    const double cx = Kd.at<double>(0, 2), cy = Kd.at<double>(1, 2);
    const double skew = Kd.at<double>(0, 1);
    const double k1 = Dd.at<double>(0, 0), k2 = Dd.at<double>(0, 1);
    const double p1 = Dd.at<double>(0, 2), p2 = Dd.at<double>(0, 3);

    const double fxn = W * focal_scale, fyn = H * focal_scale;
    const double cxn = W * 0.5, cyn = H * 0.5;

    mapx_f.create(H, W, CV_32FC1);
    mapy_f.create(H, W, CV_32FC1);
    for (int v = 0; v < H; ++v) {
        float *mx = mapx_f.ptr<float>(v);
        float *my = mapy_f.ptr<float>(v);
        for (int u = 0; u < W; ++u) {
            double X = (u - cxn) / fxn, Y = (v - cyn) / fyn, Z = 1.0;
            double n = std::sqrt(X * X + Y * Y + Z * Z);
            double zs = Z / n;
            double denom = zs + xi;
            if (denom <= 1e-9) { mx[u] = -1.f; my[u] = -1.f; continue; }
            double xp = (X / n) / denom, yp = (Y / n) / denom;
            double r2 = xp * xp + yp * yp;
            double rad = 1.0 + k1 * r2 + k2 * r2 * r2;
            double xpp = xp * rad + 2.0 * p1 * xp * yp + p2 * (r2 + 2.0 * xp * xp);
            double ypp = yp * rad + p1 * (r2 + 2.0 * yp * yp) + 2.0 * p2 * xp * yp;
            mx[u] = (float)(fx * xpp + skew * ypp + cx);
            my[u] = (float)(fy * ypp + cy);
        }
    }
    cv::convertMaps(mapx_f, mapy_f, map1_out, map2_out, CV_16SC2);
    return !map1_out.empty();
}

// 自动计算最大无黑角裁切框:扫描 remap 映射表,找出所有行列的有效范围交集 → 输出完整填充的矩形。
// src_w/src_h = 源鱼眼图尺寸(用于判定映射目标是否在源图内)。
static cv::Rect compute_auto_crop(const cv::Mat &mapx, const cv::Mat &mapy,
                                  int src_w, int src_h, int margin = 2) {
    int H = mapx.rows, W = mapx.cols;
    int left = 0, right = W - 1, top = 0, bottom = H - 1;

    // 扫每行:找最左有效列和最右有效列
    for (int v = 0; v < H; ++v) {
        const float *mx = mapx.ptr<float>(v);
        const float *my = mapy.ptr<float>(v);
        int row_left = -1, row_right = -1;
        for (int u = 0; u < W; ++u) {
            float sx = mx[u], sy = my[u];
            if (sx >= 0 && sx < src_w - 1 && sy >= 0 && sy < src_h - 1) {
                if (row_left < 0) row_left = u;
                row_right = u;
            }
        }
        if (row_left < 0) { // 整行无效 → 裁掉
            if (v < H / 2) top = v + 1;
            else bottom = v - 1;
        } else {
            if (row_left > left) left = row_left;
            if (row_right < right) right = row_right;
        }
    }

    // 安全余量(避免插值时取到黑边像素)
    left   += margin;
    top    += margin;
    right  -= margin;
    bottom -= margin;

    if (right <= left || bottom <= top) {
        // fallback:不裁
        return cv::Rect(0, 0, W, H);
    }
    return cv::Rect(left, top, right - left + 1, bottom - top + 1);
}

// 初始化去鱼眼:用 getCalibParams 从闪存读标定 → 建映射表 → 算裁切框。
// 成功返回 true(用 raw+remap 管线),失败返回 false(fallback 到 getRectStereoFrame)。
static bool init_undistort(UnitreeCamera &cam, int enc_w, int enc_h, UndistortState &st) {
    st.ok = false;

    std::vector<cv::Mat> params;
    // getCalibParams 必须在 startCapture 之后调用;flag=false → 左目
    if (!cam.getCalibParams(params, false) || params.size() < 3) {
        fprintf(stderr, "[rgb_stream] getCalibParams 失败(标定未就绪),fallback 到 stereo rectify\n");
        return false;
    }

    cv::Mat K = params[0];   // 3×3 内参
    cv::Mat D = params[1];   // 1×4 畸变(k1,k2,p1,p2)
    cv::Mat Xi = params[2];  // 1×1 CMei xi

    if (K.empty() || D.total() < 4 || Xi.empty()) {
        fprintf(stderr, "[rgb_stream] 标定参数不完整(K=%dx%d, D=%zu, Xi=%zu),fallback\n",
                K.rows, K.cols, D.total(), Xi.total());
        return false;
    }

    double xi = 0.0;
    if (Xi.type() == CV_64F) xi = Xi.at<double>(0, 0);
    else { cv::Mat xid; Xi.convertTo(xid, CV_64F); xi = xid.at<double>(0, 0); }

    // 日志:需要 CV_64F 才能 at<double>,先转
    cv::Mat Klog;
    if (K.type() != CV_64F) K.convertTo(Klog, CV_64F); else Klog = K;
    fprintf(stderr, "[rgb_stream] 标定参数: xi=%.4f, fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f\n",
            xi, Klog.at<double>(0,0), Klog.at<double>(1,1), Klog.at<double>(0,2), Klog.at<double>(1,2));

    cv::Mat map1, map2, mapx_f, mapy_f;
    if (!build_undistort_maps(K, D, xi, enc_w, enc_h, FOCAL_SCALE, map1, map2, mapx_f, mapy_f)) {
        fprintf(stderr, "[rgb_stream] 构建映射表失败,fallback\n");
        return false;
    }

    st.crop = compute_auto_crop(mapx_f, mapy_f, enc_w, enc_h);
    st.map1 = map1;
    st.map2 = map2;
    st.ok = true;

    fprintf(stderr, "[rgb_stream] CMei 去鱼眼就绪: focal_scale=%.2f, crop=[%d,%d %dx%d] (原 %dx%d)\n",
            FOCAL_SCALE, st.crop.x, st.crop.y, st.crop.width, st.crop.height, enc_w, enc_h);
    return true;
}

// ── 单次连接:开相机 → 推 RGB JPEG 直到对端断开 → 返回 ──────────────────────────────────

static void serve_client(int cli, int device_id) {
    std::string cfg = write_config(device_id);
    if (cfg.empty()) { fprintf(stderr, "[rgb_stream] 生成 config 失败\n"); return; }
    free_device(device_id);
    UnitreeCamera cam(cfg);
    for (int attempt = 0; attempt < 3 && !cam.isOpened(); ++attempt) {
        fprintf(stderr, "[rgb_stream] dev%d 未就绪,重试 %d...\n", device_id, attempt + 1);
        free_device(device_id);
        sleep(1);
    }
    if (!cam.isOpened()) {
        fprintf(stderr, "[rgb_stream] dev%d 打开失败(被占用/不可用),放弃本连接\n", device_id);
        return;
    }
    cam.startCapture();

    // ── 尝试初始化 CMei 去鱼眼(新管线) ──
    const int enc_w = 464, enc_h = 400;
    UndistortState und;
    bool use_undistort = false;

    // getCalibParams 需要 startCapture 后等参数初始化完成
    usleep(200000);  // 200ms,与 example_getCalibParamsFile 的等待一致
    use_undistort = init_undistort(cam, enc_w, enc_h, und);

    if (use_undistort) {
        // 新管线:getRawFrame → 裁左目 → remap 去鱼眼 → flip → crop → JPEG
        fprintf(stderr, "[rgb_stream] dev%d 使用 CMei 去鱼眼管线(getRawFrame, ~30-60fps, 零暖机)\n", device_id);
    } else {
        // fallback:旧管线 getRectStereoFrame(需 startStereoCompute,有暖机)
        cam.startStereoCompute();
        fprintf(stderr, "[rgb_stream] dev%d fallback 到 stereo rectify 管线(getRectStereoFrame, ~5-6s 暖机)\n", device_id);
    }

    std::vector<int> jpgparams = {cv::IMWRITE_JPEG_QUALITY, 80};
    while (cam.isOpened()) {
        cv::Mat out;

        if (use_undistort) {
            // ── 新管线:raw + CMei remap ──
            cv::Mat raw;
            std::chrono::microseconds ts(0);
            bool got = cam.getRawFrame(raw, ts);
            if (!got || raw.empty()) { usleep(2000); continue; }
            // 原始帧是左右目拼接(928×400) → 裁左半(464×400)
            int hw = raw.cols / 2;
            cv::Mat left = raw(cv::Rect(0, 0, hw, raw.rows));
            // 尺寸不匹配(理论上不会,但防御)
            if (left.cols != enc_w || left.rows != enc_h)
                cv::resize(left, left, cv::Size(enc_w, enc_h));
            // CMei remap 去鱼眼
            cv::remap(left, out, und.map1, und.map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            // Go1 相机物理装反 → 旋转 180°
            cv::flip(out, out, -1);
            // 自动裁切黑边(crop 也要翻转:翻转后的裁切框位置镜像)
            // flip(-1) 把 (x,y) → (W-1-x, H-1-y),裁切框需要相应变换
            int cw = und.crop.width, ch = und.crop.height;
            int cx = out.cols - und.crop.x - cw;
            int cy = out.rows - und.crop.y - ch;
            cv::Rect flipped_crop(cx, cy, cw, ch);
            // 边界保护
            flipped_crop &= cv::Rect(0, 0, out.cols, out.rows);
            if (flipped_crop.width > 10 && flipped_crop.height > 10)
                out = out(flipped_crop).clone();
        } else {
            // ── fallback:stereo rectify 管线(旧行为) ──
            cv::Mat left, right, feim;
            std::chrono::microseconds t(0);
            bool got = cam.getRectStereoFrame(left, right, feim, t);
            if (!got || left.empty()) { usleep(2000); continue; }
            out = left;
            if (out.type() != CV_8UC3) cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
            cv::flip(out, out, -1);
        }

        // 确保 BGR 三通道
        if (out.type() != CV_8UC3) cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);

        std::vector<uchar> buf;
        if (!cv::imencode(".jpg", out, buf, jpgparams) || buf.empty()) continue;
        uint32_t n = htonl((uint32_t)buf.size());
        if (!send_all(cli, reinterpret_cast<uint8_t *>(&n), 4)) break;
        if (!send_all(cli, buf.data(), buf.size())) break;
    }

    fprintf(stderr, "[rgb_stream] dev%d 客户端断开,_exit(0) 退出(systemd 重启回到待命,规避 SDK 析构 double-free)\n", device_id);
    fflush(stderr);
    _exit(0);
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "用法: %s <port> <device_id>\n", argv[0]);
        _exit(1);
    }
    int port      = atoi(argv[1]);
    int device_id = atoi(argv[2]);
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
    fprintf(stderr, "[rgb_stream] 空闲待命(dev%d,相机未开),监听 0.0.0.0:%d ...\n", device_id, port);

    while (true) {
        int cli = accept(srv, nullptr, nullptr);
        if (cli < 0) continue;
        fprintf(stderr, "[rgb_stream] 客户端已连接 → 开 dev%d\n", device_id);
        serve_client(cli, device_id);
        close(cli);
        fprintf(stderr, "[rgb_stream] 回到空闲待命(相机已释放),等待下一次连接...\n");
    }
    return 0;
}
