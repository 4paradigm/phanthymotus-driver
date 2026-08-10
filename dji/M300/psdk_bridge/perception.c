#include "perception.h"
#include <stdio.h>
#include <string.h>

/* M300 perception cameras deliver mono8 VGA frames.  PSDK permits at most two
 * concurrent subscriptions, so enforcement happens here as well as in the UI. */
#ifdef PSDK_ENABLED
#include "dji_perception.h"
#include <jpeglib.h>

static perception_image_cb_t s_image_cb = NULL;
static const char *s_names[] = {"front", "back", "left", "right", "up", "down"};
static int s_active[6] = {0};

static int _index_from_name(const char *name) {
    for (int i = 0; i < 6; ++i) if (strcmp(name, s_names[i]) == 0) return i;
    return -1;
}
static E_DjiPerceptionDirection _direction_from_index(int index) {
    static const E_DjiPerceptionDirection dirs[] = {
        DJI_PERCEPTION_RECTIFY_FRONT, DJI_PERCEPTION_RECTIFY_REAR,
        DJI_PERCEPTION_RECTIFY_LEFT, DJI_PERCEPTION_RECTIFY_RIGHT,
        DJI_PERCEPTION_RECTIFY_UP, DJI_PERCEPTION_RECTIFY_DOWN,
    };
    return dirs[index];
}
static int _index_from_direction(E_DjiPerceptionDirection direction) {
    for (int i = 0; i < 6; ++i) if (_direction_from_index(i) == direction) return i;
    return -1;
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
    int index = _index_from_direction(info.rawInfo.direction);
    if (index < 0 || !s_active[index] || !data || len < info.rawInfo.width * info.rawInfo.height) return;
    char path[128];
    snprintf(path, sizeof(path), "/dev/shm/dji_perception_%s.jpg", s_names[index]);
    _encode_gray_jpeg(path, data, info.rawInfo.width, info.rawInfo.height);
    if (s_image_cb) s_image_cb(s_names[index], data, info.rawInfo.width, info.rawInfo.height);
}
int perception_init(void) {
    T_DjiReturnCode rc = DjiPerception_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { printf("[perception] init failed: 0x%08llX\n", (unsigned long long)rc); return -1; }
    printf("[perception] initialized\n"); return 0;
}
int perception_start(const char *direction, perception_image_cb_t cb) {
    int index = _index_from_name(direction); if (index < 0) return -1;
    int active = 0; for (int i = 0; i < 6; ++i) active += s_active[i];
    if (!s_active[index] && active >= 2) { printf("[perception] at most two streams may be active\n"); return -1; }
    s_image_cb = cb;
    T_DjiReturnCode rc = DjiPerception_SubscribePerceptionImage(_direction_from_index(index), _image_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { printf("[perception] subscribe %s failed: 0x%08llX\n", direction, (unsigned long long)rc); return -1; }
    s_active[index] = 1; printf("[perception] subscribed %s\n", direction); return 0;
}
int perception_stop(const char *direction) {
    int index = _index_from_name(direction); if (index < 0) return -1;
    DjiPerception_UnsubscribePerceptionImage(_direction_from_index(index));
    s_active[index] = 0; return 0;
}
void perception_cleanup(void) { for (int i = 0; i < 6; ++i) if (s_active[i]) perception_stop(s_names[i]); DjiPerception_Deinit(); }
#else
int perception_init(void) { return 0; }
int perception_start(const char *direction, perception_image_cb_t cb) { (void)direction; (void)cb; return 0; }
int perception_stop(const char *direction) { (void)direction; return 0; }
void perception_cleanup(void) {}
#endif
