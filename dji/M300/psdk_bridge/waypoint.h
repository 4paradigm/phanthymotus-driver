#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <stddef.h>

int waypoint_init(void);
int waypoint_upload(const char *kmz_path);
int waypoint_start(void);
int waypoint_pause(void);
int waypoint_resume(void);
int waypoint_stop(void);
int waypoint_get_status(char *buf, size_t buflen);
void waypoint_cleanup(void);

#endif
