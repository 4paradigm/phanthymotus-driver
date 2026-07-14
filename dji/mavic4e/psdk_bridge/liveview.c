#include "liveview.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

/*
 * PSDK Liveview for Mavic 4E — FFmpeg H.264 decode + scale to 720x540 + JPEG.
 * Output to /dev/shm/dji_frame.jpg (shared memory, no disk IO).
 */

#ifdef PSDK_ENABLED
#include "dji_liveview.h"
#include "dji_camera_manager.h"
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
static int s_src_width = 0, s_src_height = 0;
static int s_frame_count = 0;
static pthread_mutex_t s_decode_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Output resolution */
#define OUT_WIDTH 720
#define OUT_HEIGHT 540
#define JPEG_QUALITY 60
#define FRAME_PATH "/dev/shm/dji_frame.jpg"

/* JPEG encode RGB frame to file (atomic write via rename) */
static int _encode_jpeg(const char *filename, uint8_t *rgb_data, int width, int height) {
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
    jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE);
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

/* FFmpeg decode + scale + JPEG encode */
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
                if (s_frame_yuv->width != s_src_width || s_frame_yuv->height != s_src_height) {
                    s_src_width = s_frame_yuv->width;
                    s_src_height = s_frame_yuv->height;

                    if (s_sws_ctx) sws_freeContext(s_sws_ctx);
                    /* Scale from source resolution to 720x540 */
                    s_sws_ctx = sws_getContext(s_src_width, s_src_height, s_codec_ctx->pix_fmt,
                                               OUT_WIDTH, OUT_HEIGHT, AV_PIX_FMT_RGB24,
                                               SWS_FAST_BILINEAR, NULL, NULL, NULL);

                    if (s_rgb_buffer) free(s_rgb_buffer);
                    s_rgb_buffer = (uint8_t *)malloc(OUT_WIDTH * OUT_HEIGHT * 3);

                    if (s_frame_rgb) av_frame_free(&s_frame_rgb);
                    s_frame_rgb = av_frame_alloc();
                    av_image_fill_arrays(s_frame_rgb->data, s_frame_rgb->linesize,
                                         s_rgb_buffer, AV_PIX_FMT_RGB24, OUT_WIDTH, OUT_HEIGHT, 1);

                    printf("[liveview] %dx%d → %dx%d\n", s_src_width, s_src_height, OUT_WIDTH, OUT_HEIGHT);
                }

                /* Scale YUV → RGB 720x540 */
                sws_scale(s_sws_ctx,
                          (const uint8_t *const *)s_frame_yuv->data, s_frame_yuv->linesize,
                          0, s_src_height,
                          s_frame_rgb->data, s_frame_rgb->linesize);

                /* Encode every decoded frame */
                s_frame_count++;
                _encode_jpeg(FRAME_PATH, s_rgb_buffer, OUT_WIDTH, OUT_HEIGHT);
                if (s_frame_count % 300 == 0) {
                    printf("[liveview] frame #%d\n", s_frame_count);
                }
            }
        }
    }

    pthread_mutex_unlock(&s_decode_mutex);
}

static void _h264_cb(E_DjiLiveViewCameraPosition pos,
                     const uint8_t *data, uint32_t len) {
    _decode_h264(data, len);
    /* Periodically request I-frame */
    static int cb_count = 0;
    cb_count++;
    if (cb_count % 90 == 0) {
        DjiLiveview_RequestIntraframeFrameData(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, s_camera_source);
    }
}

int liveview_init(void) {
    T_DjiReturnCode rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    avcodec_register_all();
    av_log_set_level(AV_LOG_FATAL);

    AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) { printf("[liveview] H264 codec not found\n"); return -1; }

    s_codec_ctx = avcodec_alloc_context3(codec);
    s_codec_ctx->thread_count = 2;
    s_codec_ctx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;

    if (avcodec_open2(s_codec_ctx, codec, NULL) < 0) {
        printf("[liveview] failed to open codec\n"); return -1;
    }

    s_parser_ctx = av_parser_init(AV_CODEC_ID_H264);
    s_frame_yuv = av_frame_alloc();

    printf("[liveview] initialized (FFmpeg %dx%d, shm)\n", OUT_WIDTH, OUT_HEIGHT);
    return 0;
}

int liveview_start(const char *camera, liveview_frame_cb_t cb) {
    s_frame_cb = cb;
    E_DjiLiveViewCameraPosition pos = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;

    /* Stop any existing stream first */
    DjiLiveview_StopH264Stream(pos, s_camera_source);

    /* Determine liveview source and optionally switch stream source */
    if (strcmp(camera, "ir") == 0) {
        s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_M4T_IR;
    } else if (strcmp(camera, "4k") == 0) {
        s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_M4E_4K;
    } else {
        /* wide or zoom: both use VIS source, SetStreamSource selects lens */
        s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_M4E_VIS;
        E_DjiCameraManagerStreamSource stream_src = DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM;
        if (strcmp(camera, "zoom") == 0) {
            stream_src = DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM;
        }
        T_DjiReturnCode src_rc = DjiCameraManager_SetStreamSource(
            DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, stream_src);
        printf("[liveview] SetStreamSource(%d) → 0x%08llX\n",
               stream_src, (unsigned long long)src_rc);
        /* Give DJI time to switch lens internally */
        usleep(500000);
    }

    T_DjiReturnCode rc = DjiLiveview_StartH264Stream(pos, s_camera_source, _h264_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] start failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    DjiLiveview_RequestIntraframeFrameData(pos, s_camera_source);
    printf("[liveview] stream started (camera=%s, source=%d)\n", camera, s_camera_source);
    return 0;
}

void liveview_set_pipe_fd(int fd) { (void)fd; }

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
void liveview_set_pipe_fd(int fd) { (void)fd; }
int liveview_stop(void) { return 0; }
void liveview_cleanup(void) {}

#endif
