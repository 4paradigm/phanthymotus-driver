#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <stdint.h>
#include <stddef.h>

/* Callback for one physical perception camera's grayscale image. */
typedef void (*perception_image_cb_t)(const char *source,
                                       const uint8_t *data, int width, int height);

int perception_init(void);
/* source is one of: front_left/front_right, back_left/back_right,
 * left_left/left_right, right_left/right_right, up_left/up_right,
 * down_left/down_right. Legacy direction-only names select the left camera.
 */
int perception_start(const char *source, perception_image_cb_t cb);
int perception_stop(const char *source);
void perception_cleanup(void);

#endif
