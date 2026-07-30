#include "perception.h"
#include <stdio.h>
#include <string.h>

/*
 * PSDK Perception Image for Mavic 3E.
 *
 * 6 direction stereo cameras (grayscale):
 *   Up/Down: 640x480, 20fps
 *   Front/Back/Left/Right: 480x480, 20fps
 *
 * PSDK limitation: max 2 simultaneous streams.
 *
 * Key APIs:
 *   DjiPerception_Init()
 *   DjiPerception_SubscribePerceptionImage(direction, callback)
 *   DjiPerception_UnsubscribePerceptionImage(direction)
 */

#ifdef PSDK_ENABLED
#include "dji_perception.h"

static perception_image_cb_t s_image_cb = NULL;

static void _image_cb(T_DjiPerceptionImageInfo info,
                      uint8_t *data, uint32_t len) {
    if (s_image_cb) {
        const char *dir = "unknown";
        switch (info.rawInfo.direction) {
            case DJI_PERCEPTION_RECTIFY_FRONT: dir = "front"; break;
            case DJI_PERCEPTION_RECTIFY_REAR:  dir = "back";  break;
            case DJI_PERCEPTION_RECTIFY_LEFT:  dir = "left";  break;
            case DJI_PERCEPTION_RECTIFY_RIGHT: dir = "right"; break;
            case DJI_PERCEPTION_RECTIFY_UP:    dir = "up";    break;
            case DJI_PERCEPTION_RECTIFY_DOWN:  dir = "down";  break;
        }
        s_image_cb(dir, data, info.rawInfo.width, info.rawInfo.height);
    }
}

int perception_init(void) {
    T_DjiReturnCode rc = DjiPerception_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[perception] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    printf("[perception] initialized\n");
    return 0;
}

int perception_start(const char *direction, perception_image_cb_t cb) {
    s_image_cb = cb;
    E_DjiPerceptionDirection dir = DJI_PERCEPTION_RECTIFY_FRONT;
    if (strcmp(direction, "back") == 0)  dir = DJI_PERCEPTION_RECTIFY_REAR;
    else if (strcmp(direction, "left") == 0)  dir = DJI_PERCEPTION_RECTIFY_LEFT;
    else if (strcmp(direction, "right") == 0) dir = DJI_PERCEPTION_RECTIFY_RIGHT;
    else if (strcmp(direction, "up") == 0)    dir = DJI_PERCEPTION_RECTIFY_UP;
    else if (strcmp(direction, "down") == 0)  dir = DJI_PERCEPTION_RECTIFY_DOWN;

    T_DjiReturnCode rc = DjiPerception_SubscribePerceptionImage(dir, _image_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[perception] subscribe %s failed: 0x%08llX\n", direction, (unsigned long long)rc);
        return -1;
    }
    return 0;
}

int perception_stop(const char *direction) {
    E_DjiPerceptionDirection dir = DJI_PERCEPTION_RECTIFY_FRONT;
    if (strcmp(direction, "back") == 0)  dir = DJI_PERCEPTION_RECTIFY_REAR;
    else if (strcmp(direction, "left") == 0)  dir = DJI_PERCEPTION_RECTIFY_LEFT;
    else if (strcmp(direction, "right") == 0) dir = DJI_PERCEPTION_RECTIFY_RIGHT;
    else if (strcmp(direction, "up") == 0)    dir = DJI_PERCEPTION_RECTIFY_UP;
    else if (strcmp(direction, "down") == 0)  dir = DJI_PERCEPTION_RECTIFY_DOWN;

    DjiPerception_UnsubscribePerceptionImage(dir);
    return 0;
}

void perception_cleanup(void) {
    DjiPerception_Deinit();
}

#else /* stub */

int perception_init(void) { printf("[perception] stub mode\n"); return 0; }
int perception_start(const char *direction, perception_image_cb_t cb) { return 0; }
int perception_stop(const char *direction) { return 0; }
void perception_cleanup(void) {}

#endif
