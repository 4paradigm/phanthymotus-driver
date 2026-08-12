#include "perception.h"
#include <stdio.h>
#include <string.h>


#ifdef PSDK_ENABLED
#include "dji_perception.h"
#include <jpeglib.h>

static perception_image_cb_t s_image_cb = NULL;
static const char *s_direction_names[] = {"front", "back", "left", "right", "up", "down"};
static const E_DjiPerceptionDirection s_directions[] = {
    DJI_PERCEPTION_RECTIFY_FRONT, DJI_PERCEPTION_RECTIFY_REAR,
    DJI_PERCEPTION_RECTIFY_LEFT, DJI_PERCEPTION_RECTIFY_RIGHT,
    DJI_PERCEPTION_RECTIFY_UP, DJI_PERCEPTION_RECTIFY_DOWN,
};

/* One source identifies one physical camera in a stereo pair.  The numeric
 * values are E_DjiPerceptionCameraPosition values from dji_perception.h. */
static const char *s_source_names[] = {
    "front_left", "front_right",
    "back_left", "back_right",
    "left_left", "left_right",
    "right_left", "right_right",
    "up_left", "up_right",
    "down_left", "down_right",
};
static const int s_source_direction_index[] = {
    0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5,
};
static const uint32_t s_source_data_types[] = {
    RECTIFY_FRONT_LEFT, RECTIFY_FRONT_RIGHT,
    RECTIFY_REAR_LEFT, RECTIFY_REAR_RIGHT,
    RECTIFY_LEFT_LEFT, RECTIFY_LEFT_RIGHT,
    RECTIFY_RIGHT_LEFT, RECTIFY_RIGHT_RIGHT,
    RECTIFY_UP_LEFT, RECTIFY_UP_RIGHT,
    RECTIFY_DOWN_LEFT, RECTIFY_DOWN_RIGHT,
};

static int s_active_source[12] = {0};
static int s_subscribed_direction[6] = {0};

static int _source_index_from_name(const char *name) {
    for (int i = 0; i < 12; ++i) if (strcmp(name, s_source_names[i]) == 0) return i;
    /* Keep old callers working: a direction-only request means its left eye. */
    for (int i = 0; i < 6; ++i) {
        if (strcmp(name, s_direction_names[i]) == 0) return i * 2;
    }
    return -1;
}
static int _direction_index_from_enum(E_DjiPerceptionDirection direction) {
    for (int i = 0; i < 6; ++i) if (s_directions[i] == direction) return i;
    return -1;
}
static int _source_index_from_image(T_DjiPerceptionImageInfo info) {
    int direction_index = _direction_index_from_enum(info.rawInfo.direction);
    if (direction_index < 0) return -1;
    for (int i = 0; i < 12; ++i) {
        if (s_source_direction_index[i] == direction_index &&
            s_source_data_types[i] == info.dataType) return i;
    }
    return -1;
}
static int _direction_has_active_source(int direction_index) {
    for (int i = 0; i < 12; ++i) {
        if (s_source_direction_index[i] == direction_index && s_active_source[i]) return 1;
    }
    return 0;
}
static int _encode_gray_jpeg(const char *path, uint8_t *gray, int width, int height) {
    char tmp[160];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *fp = fopen(tmp, "wb"); if (!fp) return -1;
    struct jpeg_compress_struct cinfo; struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr); jpeg_create_compress(&cinfo); jpeg_stdio_dest(&cinfo, fp);
    cinfo.image_width = width; cinfo.image_height = height;
    cinfo.input_components = 1; cinfo.in_color_space = JCS_GRAYSCALE;
    jpeg_set_defaults(&cinfo); jpeg_set_quality(&cinfo, 75, TRUE); jpeg_start_compress(&cinfo, TRUE);
    while (cinfo.next_scanline < (unsigned int)height) {
        JSAMPROW row = gray + cinfo.next_scanline * width;
        jpeg_write_scanlines(&cinfo, &row, 1);
    }
    jpeg_finish_compress(&cinfo); jpeg_destroy_compress(&cinfo); fclose(fp);
    return rename(tmp, path);
}
static void _image_cb(T_DjiPerceptionImageInfo info, uint8_t *data, uint32_t len) {
    int source_index = _source_index_from_image(info);
    if (source_index < 0 || !s_active_source[source_index] || !data ||
        len < info.rawInfo.width * info.rawInfo.height) return;
    char path[128];
    snprintf(path, sizeof(path), "/dev/shm/dji_perception_%s.jpg", s_source_names[source_index]);
    _encode_gray_jpeg(path, data, info.rawInfo.width, info.rawInfo.height);
    if (s_image_cb) s_image_cb(s_source_names[source_index], data,
                                info.rawInfo.width, info.rawInfo.height);
}
int perception_init(void) {
    T_DjiReturnCode rc = DjiPerception_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { printf("[perception] init failed: 0x%08llX\n", (unsigned long long)rc); return -1; }
    printf("[perception] initialized\n"); return 0;
}
int perception_start(const char *source, perception_image_cb_t cb) {
    int source_index = _source_index_from_name(source); if (source_index < 0) return -1;
    int direction_index = s_source_direction_index[source_index];
    if (s_active_source[source_index]) return 0;
    if (!s_subscribed_direction[direction_index]) {
        T_DjiReturnCode rc = DjiPerception_SubscribePerceptionImage(
            s_directions[direction_index], _image_cb);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            printf("[perception] subscribe %s failed: 0x%08llX\n", source,
                   (unsigned long long)rc);
            return -1;
        }
        s_subscribed_direction[direction_index] = 1;
    }
    s_image_cb = cb;
    s_active_source[source_index] = 1;
    printf("[perception] subscribed %s (dataType=%u)\n", s_source_names[source_index],
           s_source_data_types[source_index]);
    return 0;
}
int perception_stop(const char *source) {
    int source_index = _source_index_from_name(source); if (source_index < 0) return -1;
    int direction_index = s_source_direction_index[source_index];
    if (!s_active_source[source_index]) return 0;
    s_active_source[source_index] = 0;
    if (!_direction_has_active_source(direction_index) && s_subscribed_direction[direction_index]) {
        DjiPerception_UnsubscribePerceptionImage(s_directions[direction_index]);
        s_subscribed_direction[direction_index] = 0;
    }
    return 0;
}
void perception_cleanup(void) {
    for (int i = 0; i < 12; ++i) if (s_active_source[i]) perception_stop(s_source_names[i]);
    DjiPerception_Deinit();
}
#else
int perception_init(void) { return 0; }
int perception_start(const char *direction, perception_image_cb_t cb) { (void)direction; (void)cb; return 0; }
int perception_stop(const char *direction) { (void)direction; return 0; }
void perception_cleanup(void) {}
#endif
