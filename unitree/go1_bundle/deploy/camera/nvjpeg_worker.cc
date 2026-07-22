/*
 * nvjpeg_worker.cc — 与 UnitreeCameraSDK 隔离的 Jetson NVJPG JPEG 编码助手。
 *
 * Unitree SDK/OpenCV 使用的 libjpeg ABI 与 JetPack R32 的 GStreamer nvjpeg 插件不兼容；
 * 若同进程加载会触发 "JPEG parameter struct mismatch" 并段错误。因此本程序不链接 SDK、
 * OpenCV 或 libjpeg，只在独立进程中运行 nvjpegenc。
 *
 * stdin: 反复 [u32 big-endian BGR byte count][BGR bytes]
 * stdout:反复 [u32 big-endian JPEG byte count][JPEG bytes]
 * argv: <width> <height>
 */
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static bool read_all(uint8_t *p, size_t n) {
    while (n) {
        ssize_t k = read(STDIN_FILENO, p, n);
        if (k <= 0) return false;
        p += k;
        n -= (size_t)k;
    }
    return true;
}
static bool write_all(const uint8_t *p, size_t n) {
    while (n) {
        ssize_t k = write(STDOUT_FILENO, p, n);
        if (k <= 0) return false;
        p += k;
        n -= (size_t)k;
    }
    return true;
}

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    const int width = atoi(argv[1]), height = atoi(argv[2]);
    if (width <= 0 || height <= 0) return 2;
    const size_t raw_bytes = (size_t)width * height * 3;
    gst_init(&argc, &argv);

    GError *err = nullptr;
    GstElement *pipeline = gst_parse_launch(
        "appsrc name=src is-live=true format=time block=true "
        "! video/x-raw,format=BGR "
        "! nvjpegenc quality=70 "
        "! appsink name=sink sync=false max-buffers=1 drop=true", &err);
    if (!pipeline) {
        fprintf(stderr, "[nvjpeg_worker] pipeline: %s\\n", err ? err->message : "unknown error");
        if (err) g_error_free(err);
        return 3;
    }
    GstElement *appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    GstCaps *caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR",
                                         "width", G_TYPE_INT, width, "height", G_TYPE_INT, height,
                                         "framerate", GST_TYPE_FRACTION, 30, 1, nullptr);
    g_object_set(appsrc, "caps", caps, "format", GST_FORMAT_TIME, nullptr);
    gst_caps_unref(caps);
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) return 4;

    std::vector<uint8_t> raw(raw_bytes);
    for (;;) {
        uint32_t wire_n = 0;
        if (!read_all(reinterpret_cast<uint8_t *>(&wire_n), sizeof(wire_n))) break;
        if (ntohl(wire_n) != raw_bytes || !read_all(raw.data(), raw.size())) break;
        GstBuffer *in = gst_buffer_new_allocate(nullptr, raw.size(), nullptr);
        GstMapInfo in_map;
        if (!in || !gst_buffer_map(in, &in_map, GST_MAP_WRITE)) { if (in) gst_buffer_unref(in); break; }
        memcpy(in_map.data, raw.data(), raw.size());
        gst_buffer_unmap(in, &in_map);
        if (gst_app_src_push_buffer(GST_APP_SRC(appsrc), in) != GST_FLOW_OK) break;
        GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), GST_SECOND);
        if (!sample) break;
        GstBuffer *out = gst_sample_get_buffer(sample);
        GstMapInfo out_map;
        const bool ok = out && gst_buffer_map(out, &out_map, GST_MAP_READ);
        uint32_t jpeg_n = ok ? htonl((uint32_t)out_map.size) : 0;
        bool sent = write_all(reinterpret_cast<uint8_t *>(&jpeg_n), sizeof(jpeg_n));
        if (sent && ok) sent = write_all(out_map.data, out_map.size);
        if (ok) gst_buffer_unmap(out, &out_map);
        gst_sample_unref(sample);
        if (!sent) break;
    }
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsrc); gst_object_unref(appsink); gst_object_unref(pipeline);
    return 0;
}
