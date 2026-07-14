#include "liveview.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <pthread.h>

/*
 * PSDK Liveview for Mavic 3E — H.264 → pipe for GStreamer GPU decode.
 *
 * H.264 NALUs from DJI callback are written to a pipe fd.
 * Python side launches GStreamer subprocess reading from this pipe:
 *   fdsrc → h264parse → nvv4l2decoder → nvvidconv → jpegenc → fdsink
 *
 * This offloads decoding to Jetson's NVDEC hardware.
 */

#ifdef PSDK_ENABLED
#include "dji_liveview.h"

static liveview_frame_cb_t s_frame_cb = NULL;
static E_DjiLiveViewCameraSource s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;
static int s_h264_pipe_fd = -1;  /* Write end of FIFO to GStreamer */
static int s_frame_count = 0;
static pthread_t s_fifo_thread;
static int s_fifo_ready = 0;
#define H264_FIFO_PATH "/tmp/dji_h264_fifo"

/* Thread that opens FIFO write end (blocks until reader connects) */
static void *_fifo_open_thread(void *arg) {
    (void)arg;
    printf("[liveview] waiting for FIFO reader...\n");
    int fd = open(H264_FIFO_PATH, O_WRONLY);  /* Blocks until reader opens */
    if (fd >= 0) {
        s_h264_pipe_fd = fd;
        s_fifo_ready = 1;
        printf("[liveview] FIFO writer connected (fd=%d)\n", fd);
    } else {
        printf("[liveview] FIFO open failed\n");
    }
    return NULL;
}

static void _h264_cb(E_DjiLiveViewCameraPosition pos,
                     const uint8_t *data, uint32_t len) {
    s_frame_count++;
    if (s_h264_pipe_fd >= 0) {
        ssize_t n = write(s_h264_pipe_fd, data, len);
        if (n < 0) {
            close(s_h264_pipe_fd);
            s_h264_pipe_fd = -1;
            s_fifo_ready = 0;
        }
    }
    if (s_frame_count % 300 == 1) {
        printf("[liveview] h264_cb #%d len=%u pipe=%d\n", s_frame_count, len, s_h264_pipe_fd);
    }
}

int liveview_init(void) {
    T_DjiReturnCode rc = DjiLiveview_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    printf("[liveview] initialized (GStreamer GPU decode mode)\n");
    return 0;
}

int liveview_start(const char *camera, liveview_frame_cb_t cb) {
    s_frame_cb = cb;
    E_DjiLiveViewCameraPosition pos = DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
    s_camera_source = DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;

    /* Create named FIFO for H.264 data */
    unlink(H264_FIFO_PATH);
    if (mkfifo(H264_FIFO_PATH, 0666) < 0) {
        printf("[liveview] mkfifo failed\n");
    }

    /* Start thread to open FIFO write end (blocks until Python reader connects) */
    pthread_create(&s_fifo_thread, NULL, _fifo_open_thread, NULL);
    pthread_detach(s_fifo_thread);

    T_DjiReturnCode rc = DjiLiveview_StartH264Stream(pos, s_camera_source, _h264_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[liveview] start failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    DjiLiveview_RequestIntraframeFrameData(pos, s_camera_source);

    printf("[liveview] stream started (camera=%s), FIFO=%s\n", camera, H264_FIFO_PATH);
    return 0;
}

/* Called by Python (via IPC) to set the pipe fd for H.264 data */
void liveview_set_pipe_fd(int fd) {
    s_h264_pipe_fd = fd;
    printf("[liveview] pipe fd set to %d\n", fd);
}

int liveview_stop(void) {
    DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1, s_camera_source);
    s_frame_cb = NULL;
    s_h264_pipe_fd = -1;
    return 0;
}

void liveview_cleanup(void) {
    liveview_stop();
    DjiLiveview_Deinit();
}

#else /* stub */

int liveview_init(void) { printf("[liveview] stub mode\n"); return 0; }
int liveview_start(const char *camera, liveview_frame_cb_t cb) { return 0; }
void liveview_set_pipe_fd(int fd) { (void)fd; }
int liveview_stop(void) { return 0; }
void liveview_cleanup(void) {}

#endif
