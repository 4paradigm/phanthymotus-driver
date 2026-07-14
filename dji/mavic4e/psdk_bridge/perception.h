#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <stdint.h>
#include <stddef.h>

/* Callback for perception grayscale images. */
typedef void (*perception_image_cb_t)(const char *direction,
                                       const uint8_t *data, int width, int height);

int perception_init(void);
int perception_start(const char *direction, perception_image_cb_t cb);
int perception_stop(const char *direction);
void perception_cleanup(void);

#endif
