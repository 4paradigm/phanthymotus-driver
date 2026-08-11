#include "liveview.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

/* This target's M300 Extension Port exposes only the aircraft FPV H.264
 * source. Payload-camera stream positions are intentionally not registered. */
#ifdef PSDK_ENABLED
#include "dji_liveview.h"
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <jpeglib.h>

#define OUT_WIDTH 720
#define OUT_HEIGHT 540
#define JPEG_QUALITY 60

typedef struct {
    const char *name;
    E_DjiLiveViewCameraPosition position;
    const char *frame_path;
    int running;
    AVCodecContext *codec;
    AVCodecParserContext *parser;
    struct SwsContext *sws;
    AVFrame *yuv;
    AVFrame *rgb;
    uint8_t *rgb_buffer;
    int src_width, src_height;
    unsigned int callback_count;
    pthread_mutex_t mutex;
} T_LiveviewStream;

static T_LiveviewStream s_streams[] = {
    {"fpv",      DJI_LIVEVIEW_CAMERA_POSITION_FPV,  "/dev/shm/dji_frame_fpv.jpg",      0, NULL, NULL, NULL, NULL, NULL, NULL, 0, 0, 0, PTHREAD_MUTEX_INITIALIZER},
};
#define STREAM_COUNT (sizeof(s_streams) / sizeof(s_streams[0]))

static int s_liveview_ready = 0;

static int s_liveview_init_attempted = 0;

static T_LiveviewStream *_find_stream(const char *camera) {
    if (!camera || !*camera || strcmp(camera, "default") == 0)
        return &s_streams[0];
    for (size_t i = 0; i < STREAM_COUNT; ++i)
        if (strcmp(camera, s_streams[i].name) == 0) return &s_streams[i];
    return NULL;
}

static int _encode_jpeg(const char *filename, uint8_t *rgb, int width, int height) {
    char tmp[160];
    snprintf(tmp, sizeof(tmp), "%s.tmp", filename);
    FILE *fp = fopen(tmp, "wb");
    if (!fp) return -1;
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, fp);
    cinfo.image_width = width; cinfo.image_height = height;
    cinfo.input_components = 3; cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo); jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    while (cinfo.next_scanline < (unsigned int)height) {
        JSAMPROW row = rgb + cinfo.next_scanline * width * 3;
        jpeg_write_scanlines(&cinfo, &row, 1);
    }
    jpeg_finish_compress(&cinfo); jpeg_destroy_compress(&cinfo); fclose(fp);
    return rename(tmp, filename);
}

static void _decode(T_LiveviewStream *stream, const uint8_t *data, uint32_t len) {
    pthread_mutex_lock(&stream->mutex);
    const uint8_t *buf = data;
    int remaining = (int)len;
    while (remaining > 0) {
        AVPacket packet;
        av_init_packet(&packet); packet.data = NULL; packet.size = 0;
        int parsed = av_parser_parse2(stream->parser, stream->codec, &packet.data, &packet.size,
            buf, remaining, AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        if (parsed <= 0) break;
        buf += parsed; remaining -= parsed;
        if (!packet.size) continue;
        int got_picture = 0;
        avcodec_decode_video2(stream->codec, stream->yuv, &got_picture, &packet);
        if (!got_picture) continue;
        if (stream->yuv->width != stream->src_width || stream->yuv->height != stream->src_height) {
            stream->src_width = stream->yuv->width; stream->src_height = stream->yuv->height;
            if (stream->sws) sws_freeContext(stream->sws);
            stream->sws = sws_getContext(stream->src_width, stream->src_height, stream->codec->pix_fmt,
                OUT_WIDTH, OUT_HEIGHT, AV_PIX_FMT_RGB24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
            free(stream->rgb_buffer); stream->rgb_buffer = malloc(OUT_WIDTH * OUT_HEIGHT * 3);
            if (stream->rgb) av_frame_free(&stream->rgb);
            stream->rgb = av_frame_alloc();
            av_image_fill_arrays(stream->rgb->data, stream->rgb->linesize, stream->rgb_buffer,
                AV_PIX_FMT_RGB24, OUT_WIDTH, OUT_HEIGHT, 1);
            printf("[liveview] %s %dx%d -> %dx%d\n", stream->name, stream->src_width, stream->src_height, OUT_WIDTH, OUT_HEIGHT);
        }
        if (stream->sws && stream->rgb)
            sws_scale(stream->sws, (const uint8_t *const *)stream->yuv->data, stream->yuv->linesize,
                0, stream->src_height, stream->rgb->data, stream->rgb->linesize);
        if (stream->rgb_buffer) _encode_jpeg(stream->frame_path, stream->rgb_buffer, OUT_WIDTH, OUT_HEIGHT);
    }
    pthread_mutex_unlock(&stream->mutex);
}

static void _h264_cb(E_DjiLiveViewCameraPosition position, const uint8_t *data, uint32_t len) {
    for (size_t i = 0; i < STREAM_COUNT; ++i) {
        T_LiveviewStream *stream = &s_streams[i];
        if (stream->position != position || !stream->running) continue;
        _decode(stream, data, len);
        if (++stream->callback_count % 90 == 0)
            DjiLiveview_RequestIntraframeFrameData(stream->position, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
        return;
    }
}

int liveview_init(void) {
    if (s_liveview_init_attempted)
        return s_liveview_ready ? 0 : -1;
    s_liveview_init_attempted = 1;
    s_liveview_ready = 0;
    T_DjiReturnCode rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    avcodec_register_all(); av_log_set_level(AV_LOG_FATAL);
    AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) return -1;
    for (size_t i = 0; i < STREAM_COUNT; ++i) {
        T_LiveviewStream *s = &s_streams[i];
        s->codec = avcodec_alloc_context3(codec);
        if (!s->codec) return -1;
        s->codec->thread_count = 2; s->codec->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
        if (avcodec_open2(s->codec, codec, NULL) < 0) return -1;
        s->parser = av_parser_init(AV_CODEC_ID_H264); s->yuv = av_frame_alloc();
        if (!s->parser || !s->yuv) return -1;
    }
    s_liveview_ready = 1;
    printf("[liveview] initialized; FPV is the verified source on this M300\n");
    return 0;
}

int liveview_start(const char *camera, liveview_frame_cb_t cb) {
    (void)cb;
    T_LiveviewStream *stream = _find_stream(camera);
    if (!stream) { printf("[liveview] unknown camera: %s\n", camera); return -1; }
    if (!s_liveview_ready && liveview_init() != 0)
        return -1;
    if (!s_liveview_ready || !stream->codec || !stream->parser || !stream->yuv) {
        printf("[liveview] start %s rejected: liveview initialization did not complete\n", stream->name);
        return -1;
    }
    if (stream->running) return 0;
    pthread_mutex_lock(&stream->mutex);
    avcodec_flush_buffers(stream->codec); stream->src_width = stream->src_height = 0; stream->callback_count = 0;
    pthread_mutex_unlock(&stream->mutex);
    T_DjiReturnCode rc = DjiLiveview_StartH264Stream(stream->position, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT, _h264_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { printf("[liveview] start %s failed: 0x%08llX\n", stream->name, (unsigned long long)rc); return -1; }
    stream->running = 1;
    DjiLiveview_RequestIntraframeFrameData(stream->position, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    printf("[liveview] started %s -> %s\n", stream->name, stream->frame_path);
    return 0;
}

void liveview_set_pipe_fd(int fd) { (void)fd; }

int liveview_stop(const char *camera) {
    if (!camera || !*camera || strcmp(camera, "all") == 0) {
        for (size_t i = 0; i < STREAM_COUNT; ++i) liveview_stop(s_streams[i].name);
        return 0;
    }
    T_LiveviewStream *stream = _find_stream(camera);
    if (!stream || !stream->running) return 0;
    T_DjiReturnCode rc = DjiLiveview_StopH264Stream(stream->position, DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    stream->running = 0;
    return rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? 0 : -1;
}

void liveview_cleanup(void) {
    if (s_liveview_ready) {
        liveview_stop("all");
        DjiLiveview_Deinit();
    }
    s_liveview_ready = 0;
    s_liveview_init_attempted = 0;
    for (size_t i = 0; i < STREAM_COUNT; ++i) {
        T_LiveviewStream *s = &s_streams[i];
        if (s->parser) av_parser_close(s->parser);
        if (s->codec) { avcodec_close(s->codec); avcodec_free_context(&s->codec); }
        if (s->yuv) av_frame_free(&s->yuv); if (s->rgb) av_frame_free(&s->rgb);
        if (s->sws) sws_freeContext(s->sws); free(s->rgb_buffer);
    }
}
#else
int liveview_init(void) { return 0; }
int liveview_start(const char *camera, liveview_frame_cb_t cb) { (void)camera; (void)cb; return 0; }
void liveview_set_pipe_fd(int fd) { (void)fd; }
int liveview_stop(const char *camera) { (void)camera; return 0; }
void liveview_cleanup(void) {}
#endif
