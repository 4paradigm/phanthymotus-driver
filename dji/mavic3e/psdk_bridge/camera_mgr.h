#ifndef CAMERA_MGR_H
#define CAMERA_MGR_H

#include <stddef.h>

int camera_mgr_init(void);
int camera_mgr_take_photo(const char *mode);
int camera_mgr_start_video(void);
int camera_mgr_stop_video(void);
int camera_mgr_set_mode(const char *mode);
int camera_mgr_set_zoom(float factor);
int camera_mgr_set_focus(float x, float y);
int camera_mgr_set_exposure(int iso, float aperture, float shutter, float ev);
int camera_mgr_get_storage(char *buf, size_t buflen);
int camera_mgr_ir_temp_point(float x, float y, char *buf, size_t buflen);
int camera_mgr_ir_temp_area(float ltx, float lty, float rbx, float rby, char *buf, size_t buflen);
void camera_mgr_cleanup(void);

#endif
