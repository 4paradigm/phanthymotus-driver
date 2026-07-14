#include "liveview.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

/*
 * PSDK Liveview for Mavic 3E — H.264 decode via FFmpeg → JPEG file output.
 *
 * H.264 NALUs arrive via DJI callback → FFmpeg parser → decoder → JPEG encode → /tmp/dji_frame.jpg
 * Python reads the JPEG file and publishes to ROS2 topic for dashboard.
 */

#ifdef PSDK_ENABLED
#include "dji_liveview.h"
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <jpeglib.h>

static liveview_frame_cb_t s_frame_cb = NULL;
static E_DjiLiveViewCameraSource s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;

/* FFmpeg state */
static AVCodecContext *s_codec_ctx = NULL;
static AVCodecParserContext *s_parser_ctx = NULL;
static struct SwsContext *s_sws_ctx = NULL;
static AVFrame *s_frame_yuv = NULL;
static AVFrame *s_frame_rgb = NULL;
static uint8_t *s_rgb_buffer = NULL;
static int s_width = 0, s_height = 0;
static int s_frame_count = 0;
static pthread_mutex_t s_decode_mutex = PTHREAD_MUTEX_INITIALIZER;

/* JPEG encode RGB frame to file */
static int _encode_jpeg(const char *filename, uint8_t *rgb_data, int width, int height) {
    /* Write to temp file then rename (atomic) */
    char tmp_path[128];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", filename);

    FILE *fp = fopen(tmp_path, "wb");
    if (!fp) return -1;

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, fp);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 70, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    int row_stride = width * 3;
    while (cinfo.next_scanline < (unsigned int)height) {
        uint8_t *row = rgb_data + cinfo.next_scanline * row_stride;
        jpeg_write_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(fp);

    rename(tmp_path, filename);
    return 0;
}

/* FFmpeg decode + JPEG encode */
static void _decode_h264(const uint8_t *data, uint32_t len) {
    pthread_mutex_lock(&s_decode_mutex);

    const uint8_t *buf = data;
    int remaining = (int)len;

    while (remaining > 0) {
        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = NULL;
        pkt.size = 0;

        int parsed = av_parser_parse2(s_parser_ctx, s_codec_ctx,
                                       &pkt.data, &pkt.size,
                                       buf, remaining,
                                       AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        if (parsed < 0) break;
        buf += parsed;
        remaining -= parsed;

        if (pkt.size > 0) {
            int got_picture = 0;
            avcodec_decode_video2(s_codec_ctx, s_frame_yuv, &got_picture, &pkt);

            if (got_picture) {
                /* Setup sws_ctx on first frame or resolution change */
                if (s_frame_yuv->width != s_width || s_frame_yuv->height != s_height) {
                    s_width = s_frame_yuv->width;
                    s_height = s_frame_yuv->height;

                    if (s_sws_ctx) sws_freeContext(s_sws_ctx);
                    s_sws_ctx = sws_getContext(s_width, s_height, s_codec_ctx->pix_fmt,
                                               s_width, s_height, AV_PIX_FMT_RGB24,
                                               SWS_BILINEAR, NULL, NULL, NULL);

                    if (s_rgb_buffer) free(s_rgb_buffer);
                    s_rgb_buffer = (uint8_t *)malloc(s_width * s_height * 3);

                    if (s_frame_rgb) av_frame_free(&s_frame_rgb);
                    s_frame_rgb = av_frame_alloc();
                    av_image_fill_arrays(s_frame_rgb->data, s_frame_rgb->linesize,
                                         s_rgb_buffer, AV_PIX_FMT_RGB24, s_width, s_height, 1);

                    printf("[liveview] resolution: %dx%d\n", s_width, s_height);
                }

                /* Convert YUV → RGB */
                sws_scale(s_sws_ctx,
                          (const uint8_t *const *)s_frame_yuv->data, s_frame_yuv->linesize,
                          0, s_height,
                          s_frame_rgb->data, s_frame_rgb->linesize);

                /* Encode JPEG every decoded frame */
                s_frame_count++;
                _encode_jpeg("/tmp/dji_frame.jpg", s_rgb_buffer, s_width, s_height);
                if (s_frame_count % 30 == 0) {
                    printf("[liveview] wrote frame #%d (%dx%d)\n", s_frame_count, s_width, s_height);
                }
            }
        }
    }

    pthread_mutex_unlock(&s_decode_mutex);
}

static void _h264_cb(E_DjiLiveViewCameraPosition pos,
                     const uint8_t *data, uint32_t len) {
    static int cb_count = 0;
    cb_count++;
    if (cb_count % 100 == 1) {
        printf("[liveview] h264_cb #%d len=%u decoded_frames=%d\n", cb_count, len, s_frame_count);
    }
    _decode_h264(data, len);
}

int liveview_init(void) {
    T_DjiReturnCode rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    /* Initialize FFmpeg decoder */
    avcodec_register_all();
    av_log_set_level(AV_LOG_FATAL);  /* Suppress all warnings/errors (SEI truncated noise) */
    AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        printf("[liveview] H264 codec not found\n");
        return -1;
    }

    s_codec_ctx = avcodec_alloc_context3(codec);
    s_codec_ctx->thread_count = 2;
    s_codec_ctx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;  /* show corrupted frames */

    if (avcodec_open2(s_codec_ctx, codec, NULL) < 0) {
        printf("[liveview] failed to open H264 codec\n");
        return -1;
    }

    s_parser_ctx = av_parser_init(AV_CODEC_ID_H264);
    s_frame_yuv = av_frame_alloc();

    printf("[liveview] initialized (FFmpeg H.264 → JPEG)\n");
    return 0;
}

int liveview_start(const char *camera, liveview_frame_cb_t cb) {
    s_frame_cb = cb;
    E_DjiLiveViewCameraPosition pos = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
    s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;

    T_DjiReturnCode rc = DjiLiveview_StartH264Stream(pos, s_camera_source, _h264_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] start failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    /* Request I-frame so decoder can initialize (without this, only P-frames arrive) */
    DjiLiveview_RequestIntraframeFrameData(pos, s_camera_source);

    printf("[liveview] stream started (camera=%s), I-frame requested\n", camera);
    return 0;
}

int liveview_stop(void) {
    DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, s_camera_source);
    s_frame_cb = NULL;
    return 0;
}

void liveview_cleanup(void) {
    liveview_stop();
    DjiLiveview_Deinit();

    pthread_mutex_lock(&s_decode_mutex);
    if (s_parser_ctx) { av_parser_close(s_parser_ctx); s_parser_ctx = NULL; }
    if (s_codec_ctx) { avcodec_close(s_codec_ctx); avcodec_free_context(&s_codec_ctx); }
    if (s_frame_yuv) { av_frame_free(&s_frame_yuv); }
    if (s_frame_rgb) { av_frame_free(&s_frame_rgb); }
    if (s_sws_ctx) { sws_freeContext(s_sws_ctx); s_sws_ctx = NULL; }
    if (s_rgb_buffer) { free(s_rgb_buffer); s_rgb_buffer = NULL; }
    pthread_mutex_unlock(&s_decode_mutex);
}

#else /* stub */

int liveview_init(void) { printf("[liveview] stub mode\n"); return 0; }
int liveview_start(const char *camera, liveview_frame_cb_t cb) { return 0; }
int liveview_stop(void) { return 0; }
void liveview_cleanup(void) {}

#endif
