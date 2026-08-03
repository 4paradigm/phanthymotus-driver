#ifndef LIVEVIEW_H
#define LIVEVIEW_H

#include <stdint.h>
#include <stddef.h>

/* Callback for decoded JPEG frames. */
typedef void (*liveview_frame_cb_t)(const uint8_t *jpeg_data, size_t jpeg_size);

int liveview_init(void);
int liveview_start(const char *camera, liveview_frame_cb_t cb);
void liveview_set_pipe_fd(int fd);
int liveview_stop(void);
void liveview_cleanup(void);

#endif
