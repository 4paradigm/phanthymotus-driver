#ifndef CAMERA_MGR_H
#define CAMERA_MGR_H

#include <stddef.h>

int camera_mgr_init(void);
int camera_mgr_take_photo(const char *camera, const char *mode);
int camera_mgr_start_video(const char *camera);
int camera_mgr_stop_video(const char *camera);
int camera_mgr_set_mode(const char *camera, const char *mode);
int camera_mgr_set_zoom(const char *camera, float factor);
int camera_mgr_set_focus(const char *camera, float x, float y);
int camera_mgr_set_exposure(const char *camera, int iso, float aperture, float shutter, float ev);
int camera_mgr_get_storage(const char *camera, char *buf, size_t buflen);
int camera_mgr_ir_temp_point(const char *camera, float x, float y, char *buf, size_t buflen);
int camera_mgr_ir_temp_area(const char *camera, float ltx, float lty, float rbx, float rby, char *buf, size_t buflen);
void camera_mgr_cleanup(void);

#endif
