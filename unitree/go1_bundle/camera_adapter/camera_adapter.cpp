// camera_adapter.cpp — Go1 板载相机 Adapter（能力卡片 §8 视觉扩展卡的板卡侧组件）。
//
// 运行位置：Go1 的 Nano 板卡（192.168.123.13/14/15），每个在用相机跑一个本进程实例。
// 角色（能力卡片 §8.6.1）：独占打开一台相机的采集会话（UnitreecameraSDK），对 Pi 侧驱动
//   （camera.py 的 camera_rgb 卡）提供：
//     · JSON-over-TCP 控制通道（probe / start / stop / snapshot，返回标定与实际配置）
//     · H.264/RTP 定向 UDP 图像流（§8.6.3，目标 IP/端口由 Pi 在 start 时下发，不硬编码）
//   depth / pointcloud 卡将来复用同一采集会话（本文件先只做 rgb 取帧 + 发流，留好扩展点）。
//
// 为什么自己用 gstreamer 重编码而不用 SDK 自带 UDP 图传：
//   UnitreecameraSDK 的 startCapture(true,...) 自带一套 UDP 传输，但其封装格式不保证是标准
//   RTP/H.264，Pi 侧 gstreamer 的 rtph264depay 未必能解。这里改为：用 SDK 只“取帧”（getRawFrame /
//   getRectStereoFrame 拿 cv::Mat），再喂给我们自己拉起的 gst-launch 编码管线
//   （fdsrc → videoconvert → x264enc → rtph264pay → udpsink）。编码端与 camera.py 的解码端
//   由同一套约定构造，天然对齐，避免依赖 SDK 不透明的私有协议。
//
// ⚠️ 与 robot_interface_v32.cpp 同样的现实：本文件在开发机上无法编译/联调（缺板载
//   UnitreecameraSDK 头文件与相机），须在板卡上用 build_adapter.sh 编译，按编译期报错据实修正
//   下面标注了 [SDK-API] 的调用签名（不同 SDK 版本的方法名/参数可能有出入，见 README）。
//
// 控制协议（逐行 JSON，一问一答；与 camera.py::_AdapterClient 对齐）：
//   → {"cmd":"probe","device_id":N}
//   ← {"ok":true,"device_id":N,"online":true,"busy":false,"serial":"...","width":W,"height":H,
//      "fps":F,"calibration":{...}}
//   → {"cmd":"start","device_id":N,"config":{"mode":..,"frame_size":"928x400","fps":30,
//      "rectified_size":..,"hfov_deg":..,"target_ip":"192.168.123.161","image_port":9201}}
//   ← {"ok":true,"applied":{...},"calibration":{...},"streams":[{"eye":"left","port":9201},...]}
//   → {"cmd":"stop","device_id":N}                    ← {"ok":true}
//   → {"cmd":"snapshot","device_id":N,"eye":"left"}   ← {"ok":true,"seq":S,"timestamp_us":T}
//   失败：{"ok":false,"code":"RESOURCE_BUSY"|"DEVICE_NOT_FOUND"|"INVALID_ARGUMENT"|...,"message":".."}

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

// [SDK-API] 板载 UnitreecameraSDK 的伞形头。真机路径通常为
//   /home/unitree/UnitreecameraSDK/include/UnitreeCameraSDK.hpp
// build_adapter.sh 通过 -I 指向其 include 目录。
#include <UnitreeCameraSDK.hpp>

// ── 极简 JSON：只针对本固定协议的扁平/单层嵌套消息，避免引第三方库（自足，随板编译）──
namespace mini {

static std::string get_string(const std::string& s, const std::string& key) {
    // 找 "key" : "value"
    std::string pat = "\"" + key + "\"";
    size_t k = s.find(pat);
    if (k == std::string::npos) return "";
    size_t colon = s.find(':', k + pat.size());
    if (colon == std::string::npos) return "";
    size_t q1 = s.find('"', colon + 1);
    if (q1 == std::string::npos) return "";
    size_t q2 = s.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return s.substr(q1 + 1, q2 - q1 - 1);
}

static long get_int(const std::string& s, const std::string& key, long dflt) {
    std::string pat = "\"" + key + "\"";
    size_t k = s.find(pat);
    if (k == std::string::npos) return dflt;
    size_t colon = s.find(':', k + pat.size());
    if (colon == std::string::npos) return dflt;
    // 跳过空白/引号
    size_t p = colon + 1;
    while (p < s.size() && (s[p] == ' ' || s[p] == '"')) p++;
    char* end = nullptr;
    long v = std::strtol(s.c_str() + p, &end, 10);
    if (end == s.c_str() + p) return dflt;
    return v;
}

static std::string esc(const std::string& in) {
    std::string o;
    for (char c : in) { if (c == '"' || c == '\\') o += '\\'; o += c; }
    return o;
}

}  // namespace mini

// ── 一台相机的采集/发流会话 ───────────────────────────────────────────────────

struct StreamCfg {
    std::string mode = "rectified_mono";   // raw_mono/raw_stereo/rectified_mono/rectified_stereo
    std::string frame_size = "928x400";
    int fps = 30;
    std::string target_ip = "192.168.123.161";
    int image_port = 9201;
};

static bool is_stereo(const std::string& mode) {
    return mode == "raw_stereo" || mode == "rectified_stereo";
}

class CameraSession {
public:
    CameraSession(int device_id, const std::string& device_node, const std::string& calib_file)
        : device_id_(device_id), device_node_(device_node), calib_file_(calib_file) {}

    ~CameraSession() { stop(); close_device(); }

    // probe：确保设备可打开（独占会话），回报基础信息；打不开 → online=false / busy。
    // 若正在推流用缓存信息回答；否则以 raw(int) 构造轻量开→读→关，探完即释放设备。
    std::string probe() {
        std::lock_guard<std::mutex> lk(mu_);
        if (!running_) {
            if (!open_raw_probe()) {
                return busy_ ? fail("RESOURCE_BUSY", "camera in use by another process")
                             : fail("DEVICE_NOT_FOUND", "cannot open camera device");
            }
        }
        std::ostringstream o;
        o << "{\"ok\":true,\"device_id\":" << device_id_
          << ",\"online\":true,\"busy\":false"
          << ",\"serial\":\"" << mini::esc(serial_) << "\""
          << ",\"width\":" << width_ << ",\"height\":" << height_ << ",\"fps\":" << native_fps_
          << ",\"calibration\":" << calibration_json_ << "}";
        if (!running_) close_device();   // 探测后释放；start() 再按 mode 正确重开
        return o.str();
    }

    std::string start(const std::string& req) {
        std::lock_guard<std::mutex> lk(mu_);
        StreamCfg c;
        c.mode = mini::get_string(req, "mode");
        if (c.mode.empty()) c.mode = "rectified_mono";
        std::string fs = mini::get_string(req, "frame_size");
        if (!fs.empty()) c.frame_size = fs;
        c.fps = (int)mini::get_int(req, "fps", 30);
        std::string tip = mini::get_string(req, "target_ip");
        if (!tip.empty()) c.target_ip = tip;
        c.image_port = (int)mini::get_int(req, "image_port", 9201);

        // depth 模式参数（带宽三旋钮：分片必上；限帧 depth_fps 为主；降分辨率 depth_scale_pct 默认 100=不缩）。
        depth_port_ = (int)mini::get_int(req, "depth_port", 9211);
        long dfps = mini::get_int(req, "depth_fps", 8);
        depth_fps_ = dfps > 0 ? (double)dfps : 8.0;
        long dscale = mini::get_int(req, "depth_scale_pct", 100);       // 10~100(%)，默认 100 不缩
        depth_scale_ = (dscale >= 10 && dscale <= 100) ? dscale / 100.0 : 1.0;
        depth_focal_px_  = (double)mini::get_int(req, "focal_px", 0);   // 有标定→出 mm；否则出放大视差
        depth_baseline_mm_ = (double)mini::get_int(req, "baseline_mm", 0);

        // 从 frame_size("WxH") 推导原始/编码尺寸：双目左右各半 → 单目 enc = W/2 x H。
        int rw = 928, rh = 400;
        if (std::sscanf(c.frame_size.c_str(), "%dx%d", &rw, &rh) != 2 || rw <= 1 || rh <= 0) {
            rw = 928; rh = 400;
        }
        raw_w_ = rw; raw_h_ = rh; enc_w_ = rw / 2; enc_h_ = rh;

        if (running_) stop_locked();   // 重复 start：先停旧流
        if (!open_stream(c.mode, c.fps)) {
            return busy_ ? fail("RESOURCE_BUSY", "camera in use by another process")
                         : fail("DEVICE_NOT_FOUND", "cannot open camera device");
        }

        cfg_ = c;
        streams_.clear();
        depth_on_ = (c.mode == "depth");
        // 一个 UDP socket 复用（JPEG 各路 + depth 分片共用）。
        udp_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd_ < 0) { stop_locked(); return fail("COMMUNICATION_ERROR", "cannot create udp socket"); }

        if (depth_on_) {
            // depth：不发 JPEG，改发**分片** 16-bit 深度到 target_ip:depth_port。
            std::memset(&depth_dest_, 0, sizeof(depth_dest_));
            depth_dest_.sin_family = AF_INET;
            depth_dest_.sin_port = htons((uint16_t)depth_port_);
            if (inet_pton(AF_INET, c.target_ip.c_str(), &depth_dest_.sin_addr) != 1) {
                stop_locked();
                return fail("INVALID_ARGUMENT", "invalid target_ip");
            }
            // SGBM：numDisparities 须为 16 的倍数；blockSize 奇数。CPU 算法 → 靠 depth_fps 限帧控负载。
            sgbm_ = cv::StereoSGBM::create(0 /*minDisp*/, 64 /*numDisp*/, 7 /*blockSize*/);
            depth_last_us_ = 0; depth_seq_ = 0;
        } else {
            // 组织输出流：stereo 两路（left=image_port, right=image_port+1），mono 一路。
            if (is_stereo(c.mode)) {
                streams_.push_back({"left",  c.image_port});
                streams_.push_back({"right", c.image_port + 1});
            } else {
                streams_.push_back({"mono", c.image_port});
            }
            for (auto& s : streams_) {
                std::memset(&s.dest, 0, sizeof(s.dest));
                s.dest.sin_family = AF_INET;
                s.dest.sin_port = htons((uint16_t)s.port);
                if (inet_pton(AF_INET, c.target_ip.c_str(), &s.dest.sin_addr) != 1) {
                    stop_locked();
                    return fail("INVALID_ARGUMENT", "invalid target_ip");
                }
            }
        }

        running_ = true;
        seq_ = 0;
        worker_ = std::thread(&CameraSession::loop, this);

        std::ostringstream o;
        o << "{\"ok\":true,\"applied\":{"
          << "\"mode\":\"" << mini::esc(c.mode) << "\",\"frame_size\":\"" << mini::esc(c.frame_size)
          << "\",\"fps\":" << c.fps << ",\"target_ip\":\"" << mini::esc(c.target_ip) << "\"";
        if (depth_on_) {
            o << ",\"depth_port\":" << depth_port_ << ",\"depth_fps\":" << (int)depth_fps_
              << ",\"encoding\":\"16UC1\"}" << ",\"calibration\":" << calibration_json_ << ",\"streams\":[]}";
        } else {
            o << ",\"image_port\":" << c.image_port << "}"
              << ",\"calibration\":" << calibration_json_ << ",\"streams\":[";
            for (size_t i = 0; i < streams_.size(); ++i) {
                o << "{\"eye\":\"" << streams_[i].eye << "\",\"port\":" << streams_[i].port << "}";
                if (i + 1 < streams_.size()) o << ",";
            }
            o << "]}";
        }
        return o.str();
    }

    std::string stop() {
        std::lock_guard<std::mutex> lk(mu_);
        stop_locked();
        return "{\"ok\":true}";
    }

    std::string snapshot(const std::string& req) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!running_) return fail("PRECONDITION_FAILED", "not streaming; start first");
        (void)req;  // eye 仅用于回执标注；帧本身走图像流
        long s = seq_.load();
        std::ostringstream o;
        o << "{\"ok\":true,\"seq\":" << s << ",\"timestamp_us\":" << last_ts_us_ << "}";
        return o.str();
    }

private:
    struct OutStream { std::string eye; int port; sockaddr_in dest{}; };

    static std::string fail(const std::string& code, const std::string& msg) {
        return "{\"ok\":false,\"code\":\"" + code + "\",\"message\":\"" + mini::esc(msg) + "\"}";
    }

    static bool is_rectified(const std::string& mode) {
        return mode == "rectified_mono" || mode == "rectified_stereo" || mode == "depth";
    }

    // 生成 UnitreecameraSDK 用的 OpenCV YAML 配置（rectified 构造需要它加载标定/校正）。
    // 只填 SDK 会读的字段；DeviceNode 用板卡实际节点号（=device_id_）。写到 /tmp 返回路径。
    std::string write_yaml(int fps) {
        std::string path = "/tmp/go1_cam_" + std::to_string(device_id_) + ".yaml";
        FILE* f = fopen(path.c_str(), "w");
        if (!f) return path;
        fprintf(f, "%%YAML:1.0\n---\n");
        auto m1 = [&](const char* k, double v){ fprintf(f,
            "%s: !!opencv-matrix\n   rows: 1\n   cols: 1\n   dt: d\n   data: [ %g ]\n", k, v); };
        m1("LogLevel", 1.0); m1("Threshold", 190.0); m1("Algorithm", 1.0);
        m1("IpLastSegment", 15.0);                    // SDK 自带 UDP 不启用，此值无关
        m1("DeviceNode", (double)device_id_);
        m1("hFov", 90.0);
        fprintf(f, "FrameSize: !!opencv-matrix\n   rows: 1\n   cols: 2\n   dt: d\n   data: [ %d., %d. ]\n", raw_w_, raw_h_);
        fprintf(f, "RectifyFrameSize: !!opencv-matrix\n   rows: 1\n   cols: 2\n   dt: d\n   data: [ %d., %d. ]\n", enc_w_, enc_h_);
        m1("FrameRate", (double)fps);
        fclose(f);
        return path;
    }

    // [SDK-API] 轻量探测：raw(int) 构造打开、读基础信息，不 startCapture。busy → isOpened=false。
    bool open_raw_probe() {
        if (cam_) return true;
        try {
            cam_ = new UnitreeCamera(device_id_);          // [SDK-API] 设备节点号构造
            if (!cam_->isOpened()) { busy_ = true; delete cam_; cam_ = nullptr; return false; }
            cv::Size fsz = cam_->getRawFrameSize();         // [SDK-API]
            if (fsz.width > 0) { width_ = fsz.width; height_ = fsz.height; }
            native_fps_ = (int)cam_->getRawFrameRate();     // [SDK-API] 返回 float
            serial_ = std::to_string(cam_->getSerialNumber());  // [SDK-API] 返回 int
            calibration_json_ = read_calibration();
            return true;
        } catch (...) {
            if (cam_) { delete cam_; cam_ = nullptr; }
            busy_ = false;
            return false;
        }
    }

    // 开相机前先腾设备：优雅 kill 掉占用 /dev/videoN 的进程再打开，实现"每次调用自愈"，
    // 免得用户手动杀 point_cloud_node/example_putImagetrans。
    // 🔴 只用 SIGTERM（fuser -k -TERM），**不用 -9**——-9 会把 V4L2 缓冲区留在卡死态
    // （SDK 之后 open 报 Internal data stream error）。占用者与本进程同为 unitree 用户，无需 sudo。
    // 调用点在 close_device() 之后，故本进程已不持有该设备，不会误杀自己。
    void free_device_node() {
        std::string check = "fuser " + device_node_ + " >/dev/null 2>&1";
        if (system(check.c_str()) != 0) return;                   // 无人占用，快速返回
        std::string term = "fuser -k -TERM " + device_node_ + " >/dev/null 2>&1";
        system(term.c_str());
        for (int i = 0; i < 20; ++i) {                            // 轮询等释放，最多 ~4s
            if (system(check.c_str()) != 0) return;               // 已空闲
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        fprintf(stderr, "[adapter] warn: %s 仍被占用（腾设备超时）\n", device_node_.c_str());
    }

    // [SDK-API] 按 mode 打开采集会话：rectified 用配置文件构造（加载标定），raw 用设备节点号。
    bool open_stream(const std::string& mode, int fps) {
        close_device();
        free_device_node();   // 先腾设备（自愈：杀掉抢 /dev/videoN 的进程）再开
        bool rect = is_rectified(mode);
        try {
            if (rect) cam_ = new UnitreeCamera(write_yaml(fps));   // [SDK-API] 配置文件构造
            else      cam_ = new UnitreeCamera(device_id_);        // [SDK-API] 设备节点号构造
            if (!cam_->isOpened()) { busy_ = true; delete cam_; cam_ = nullptr; return false; }
            cam_->setRawFrameSize(cv::Size(raw_w_, raw_h_));        // [SDK-API]
            cam_->setRawFrameRate(fps);                            // [SDK-API] 取 int
            if (rect) cam_->setRectFrameSize(cv::Size(enc_w_, enc_h_));  // [SDK-API]
            cv::Size fsz = cam_->getRawFrameSize();
            if (fsz.width > 0) { width_ = fsz.width; height_ = fsz.height; }
            native_fps_ = fps;
            serial_ = std::to_string(cam_->getSerialNumber());
            calibration_json_ = read_calibration();
            cam_->startCapture();                                 // [SDK-API] (false,false)：不启用 SDK 自带图传/共享内存
            return true;
        } catch (...) {
            if (cam_) { delete cam_; cam_ = nullptr; }
            busy_ = false;
            return false;
        }
    }

    void close_device() {
        if (cam_) {
            try { cam_->stopCapture(); } catch (...) {}   // [SDK-API]
            delete cam_;
            cam_ = nullptr;
        }
    }

    // 读标定为 JSON 字符串（内参/畸变/Xi/旋转/平移/校正内参）。取不到则回空对象 {}。
    std::string read_calibration() {
        // [SDK-API] 官方 SDK 通常从相机内 flash / 标定 yaml 读参数。这里给出占位结构，
        // 真机接通后按实际可取字段填充（能力卡片 §8.2 info 要求的完整标定项）。
        if (!calib_file_.empty()) {
            std::ostringstream o;
            o << "{\"source\":\"" << mini::esc(calib_file_) << "\",\"status\":\"file\"}";
            return o.str();
        }
        return "{\"status\":\"unverified\"}";
    }

    // 拉起 gst-launch 编码器：读 stdin 的原始 BGR 帧 → x264enc → RTP/H.264 → udpsink.
    // （已弃用：容器侧无 gstreamer，改为 loop() 内 cv::imencode + UDP 直发 JPEG。）

    // 采集循环：从 SDK 取帧 → 按 mode 切/校正 → resize 到编码尺寸 → 写各路编码器 stdin。
    void loop() {
        cv::Mat left, right, raw;
        std::chrono::microseconds ts(0);
        while (running_) {
            bool got = false;
            if (cfg_.mode == "rectified_stereo" || cfg_.mode == "rectified_mono" || cfg_.mode == "depth") {
                got = cam_->getRectStereoFrame(left, right);   // [SDK-API] 校正后的左右目（depth 也用）
            } else {
                got = cam_->getRawFrame(raw, ts);              // [SDK-API] 原始双目拼接帧 + 微秒戳
                if (got && !raw.empty()) {
                    int hw = raw.cols / 2;
                    left = raw(cv::Rect(0, 0, hw, raw.rows)).clone();
                    right = raw(cv::Rect(hw, 0, hw, raw.rows)).clone();
                }
            }
            if (!got) { std::this_thread::sleep_for(std::chrono::milliseconds(2)); continue; }

            last_ts_us_ = ts.count() ? (long long)ts.count()
                                     : (long long)std::chrono::duration_cast<std::chrono::microseconds>(
                                           std::chrono::system_clock::now().time_since_epoch()).count();

            if (depth_on_) {
                // ── 限帧（带宽主旋钮）：按 depth_fps_ 控制计算/发送频率，同时压住 SGBM 的 CPU 负载 ──
                long long now_us = (long long)std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                long long period_us = (long long)(1e6 / (depth_fps_ > 0 ? depth_fps_ : 8.0));
                if (depth_last_us_ != 0 && now_us - depth_last_us_ < period_us) continue;
                depth_last_us_ = now_us;
                if (left.empty() || right.empty()) continue;
                cv::Mat lg, rg;
                if (left.channels() == 3) cv::cvtColor(left, lg, cv::COLOR_BGR2GRAY); else lg = left;
                if (right.channels() == 3) cv::cvtColor(right, rg, cv::COLOR_BGR2GRAY); else rg = right;
                if (depth_scale_ > 0.0 && depth_scale_ < 0.999) {   // 降分辨率（最后手段，默认 1.0 不缩）
                    cv::resize(lg, lg, cv::Size(), depth_scale_, depth_scale_);
                    cv::resize(rg, rg, cv::Size(), depth_scale_, depth_scale_);
                }
                cv::Mat disp;
                sgbm_->compute(lg, rg, disp);       // CV_16S，视差×16
                cv::flip(disp, disp, -1);           // 相机装反 → 翻正（与 mono 一致）
                int w = disp.cols, h = disp.rows;
                cv::Mat out(h, w, CV_16U);
                bool metric = (depth_focal_px_ > 1.0 && depth_baseline_mm_ > 0.1);
                for (int y = 0; y < h; ++y) {
                    const short* dp = disp.ptr<short>(y);
                    uint16_t* op = out.ptr<uint16_t>(y);
                    for (int x = 0; x < w; ++x) {
                        double d = dp[x] / 16.0;    // 真实视差(px)
                        if (d <= 0.0) { op[x] = 0; continue; }   // 无效/遮挡 → 0
                        double v = metric ? (depth_focal_px_ * depth_baseline_mm_ / d)   // 深度 mm
                                          : (d * 256.0);                                 // 无标定：放大视差填 16-bit
                        op[x] = (v > 65535.0) ? (uint16_t)65535 : (uint16_t)v;
                    }
                }
                if (!out.isContinuous()) out = out.clone();
                send_depth_chunked((const uint8_t*)out.data, (size_t)w * (size_t)h * 2, w, h);
                seq_++;
                continue;   // depth 模式不发 JPEG
            }

            std::vector<int> jpg_params = {cv::IMWRITE_JPEG_QUALITY, 80};
            for (auto& s : streams_) {
                cv::Mat& src = (s.eye == "right") ? right : left;   // mono→left 目
                if (src.empty()) continue;
                cv::Mat out;
                if (src.cols != enc_w_ || src.rows != enc_h_)
                    cv::resize(src, out, cv::Size(enc_w_, enc_h_));
                else
                    out = src;
                if (out.type() != CV_8UC3) cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
                // Go1 前置相机物理装反 → 原始/校正帧都是上下颠倒的。编码前旋转 180°
                // （cv::flip flipCode=-1 = 同时翻 x/y 轴）翻正，免得下游订阅方全是倒像。
                cv::flip(out, out, -1);
                // 每帧编成 JPEG，一个 UDP 数据报发出（464x400@q80 约 20-40KB < 65507）。
                std::vector<uchar> jpg;
                if (!cv::imencode(".jpg", out, jpg, jpg_params) || jpg.empty()) continue;
                if (jpg.size() <= 65000 && udp_fd_ >= 0)
                    sendto(udp_fd_, jpg.data(), jpg.size(), 0,
                           (sockaddr*)&s.dest, sizeof(s.dest));
                // 超 UDP 上限的大帧直接丢（提示上层用较小 frame_size / rectified）。
            }
            seq_++;
        }
    }

    // 把一帧 16-bit 深度按 <=CHUNK 字节**分片**发到 depth_dest_（分片是带宽首选手段）。
    // 每片 14 字节头，与 depth.py 的重组头一致：struct "!2sIHHHH" =
    //   magic"DZ"(2) + seq(u32) + total(u16) + idx(u16) + w(u16) + h(u16)。
    // 头用网络序(htonl/htons)；深度像素本身按主机序(aarch64 小端)裸发，depth.py 侧 Image.is_bigendian=0。
    void send_depth_chunked(const uint8_t* data, size_t nbytes, int w, int h) {
        if (udp_fd_ < 0 || nbytes == 0) return;
        const size_t HDR = 14, CHUNK = 60000;   // CHUNK < 65507(UDP 上限)，留头空间
        uint16_t total = (uint16_t)((nbytes + CHUNK - 1) / CHUNK);
        uint32_t seq = (uint32_t)(depth_seq_++);
        std::vector<uint8_t> pkt(HDR + CHUNK);
        for (uint16_t idx = 0; idx < total; ++idx) {
            size_t off = (size_t)idx * CHUNK;
            size_t len = std::min(CHUNK, nbytes - off);
            pkt[0] = 'D'; pkt[1] = 'Z';
            uint32_t seqn = htonl(seq);       std::memcpy(&pkt[2], &seqn, 4);
            uint16_t totn = htons(total);     std::memcpy(&pkt[6], &totn, 2);
            uint16_t idxn = htons(idx);       std::memcpy(&pkt[8], &idxn, 2);
            uint16_t wn   = htons((uint16_t)w); std::memcpy(&pkt[10], &wn, 2);
            uint16_t hn   = htons((uint16_t)h); std::memcpy(&pkt[12], &hn, 2);
            std::memcpy(&pkt[HDR], data + off, len);
            sendto(udp_fd_, pkt.data(), HDR + len, 0, (sockaddr*)&depth_dest_, sizeof(depth_dest_));
        }
    }

    void stop_locked() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
        if (udp_fd_ >= 0) { close(udp_fd_); udp_fd_ = -1; }
        streams_.clear();
    }

    int device_id_;
    std::string device_node_, calib_file_;
    UnitreeCamera* cam_ = nullptr;
    bool busy_ = false;
    std::string serial_, calibration_json_ = "{}";
    int width_ = 0, height_ = 0, native_fps_ = 30;

    // 采集/编码尺寸：raw = 双目原始帧(WxH)；enc = 单目(W/2 x H)。start 时据 frame_size 更新。
    int raw_w_ = 928, raw_h_ = 400;
    int enc_w_ = 464, enc_h_ = 400;

    StreamCfg cfg_;
    std::vector<OutStream> streams_;
    int udp_fd_ = -1;                 // JPEG-over-UDP 发送 socket（一路复用，per-stream dest）
    std::atomic<long> seq_{0};
    long long last_ts_us_ = 0;
    std::atomic<bool> running_{false};
    std::thread worker_;
    std::mutex mu_;

    // ── depth 模式状态（mode=="depth"）──
    bool depth_on_ = false;
    int depth_port_ = 9211;
    double depth_fps_ = 8.0;              // 限帧（带宽主旋钮）
    double depth_scale_ = 1.0;            // 降分辨率（最后手段，1.0=不缩）
    double depth_focal_px_ = 0.0;         // 有标定→出深度 mm；否则出放大视差
    double depth_baseline_mm_ = 0.0;
    sockaddr_in depth_dest_{};
    cv::Ptr<cv::StereoSGBM> sgbm_;
    long long depth_last_us_ = 0;
    std::atomic<long> depth_seq_{0};
};

// ── JSON-TCP 控制服务 ─────────────────────────────────────────────────────────

static void handle_conn(int fd, CameraSession& sess, int device_id) {
    std::string buf;
    char tmp[4096];
    // 读一行（\n 结束）。
    while (buf.find('\n') == std::string::npos) {
        ssize_t n = recv(fd, tmp, sizeof(tmp), 0);
        if (n <= 0) { close(fd); return; }
        buf.append(tmp, n);
        if (buf.size() > (1u << 20)) break;
    }
    std::string line = buf.substr(0, buf.find('\n'));
    std::string cmd = mini::get_string(line, "cmd");
    long req_dev = mini::get_int(line, "device_id", device_id);

    std::string resp;
    if (req_dev != device_id) {
        resp = "{\"ok\":false,\"code\":\"DEVICE_NOT_FOUND\",\"message\":\"device_id mismatch\"}";
    } else if (cmd == "probe") {
        resp = sess.probe();
    } else if (cmd == "start") {
        resp = sess.start(line);
    } else if (cmd == "stop") {
        resp = sess.stop();
    } else if (cmd == "snapshot") {
        resp = sess.snapshot(line);
    } else {
        resp = "{\"ok\":false,\"code\":\"INVALID_ARGUMENT\",\"message\":\"unknown cmd\"}";
    }
    resp += "\n";
    (void)!send(fd, resp.c_str(), resp.size(), 0);
    close(fd);
}

int main(int argc, char** argv) {
    // 参数：--device-id N --device-node /dev/videoX --control-port P [--calib FILE]
    int device_id = 0, control_port = 9301;
    std::string device_node = "/dev/video0", calib_file;
    for (int i = 1; i < argc - 1; ++i) {
        std::string a = argv[i];
        if (a == "--device-id") device_id = atoi(argv[++i]);
        else if (a == "--device-node") device_node = argv[++i];
        else if (a == "--control-port") control_port = atoi(argv[++i]);
        else if (a == "--calib") calib_file = argv[++i];
    }
    fprintf(stderr, "[adapter] device_id=%d node=%s control_port=%d calib=%s\n",
            device_id, device_node.c_str(), control_port, calib_file.c_str());

    CameraSession sess(device_id, device_node, calib_file);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)control_port);
    if (bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[adapter] bind"); return 1;
    }
    listen(srv, 4);
    fprintf(stderr, "[adapter] control listening on :%d\n", control_port);

    while (true) {
        int fd = accept(srv, nullptr, nullptr);
        if (fd < 0) continue;
        // 串行处理（控制请求短、量小；采集在会话内部的 worker 线程跑）。
        handle_conn(fd, sess, device_id);
    }
    return 0;
}
