#include "liveview.h"
#include <stdio.h>
#include <string.h>

/*
 * PSDK Liveview (Camera Stream) for Mavic 3E.
 *
 * Uses DjiLiveview_StartH264Stream() to receive H.264 frames via callback.
 * Decoded to JPEG using ffmpeg/GStreamer pipeline (external).
 *
 * M3E stream specs: 1920x1080, 30fps, ~16Mbps H.264.
 */

#ifdef PSDK_ENABLED
#include "dji_liveview.h"

static liveview_frame_cb_t s_frame_cb = NULL;
static E_DjiLiveViewCameraSource s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;

static void _h264_cb(E_DjiLiveViewCameraPosition pos,
                     const uint8_t *data, uint32_t len) {
    /* TODO: decode H.264 to JPEG and call s_frame_cb */
    /* For now, raw H.264 NALUs arrive here. Need ffmpeg or hardware decoder. */
    (void)pos; (void)data; (void)len;
}

int liveview_init(void) {
    T_DjiReturnCode rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    printf("[liveview] initialized\n");
    return 0;
}

int liveview_start(const char *camera, liveview_frame_cb_t cb) {
    s_frame_cb = cb;
    E_DjiLiveViewCameraPosition pos = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
    /* camera: "wide" = M3E_VIS source, "zoom" = default */
    s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_M3E_VIS;
    T_DjiReturnCode rc = DjiLiveview_StartH264Stream(pos, s_camera_source, _h264_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] start failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    printf("[liveview] stream started (camera=%s)\n", camera);
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
}

#else /* stub */

int liveview_init(void) { printf("[liveview] stub mode\n"); return 0; }
int liveview_start(const char *camera, liveview_frame_cb_t cb) { return 0; }
int liveview_stop(void) { return 0; }
void liveview_cleanup(void) {}

#endif
