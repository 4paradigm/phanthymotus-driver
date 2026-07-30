#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <stdint.h>

#include <stddef.h>

int waypoint_init(void);
int waypoint_upload(const char *kmz_path);
/* Returns 0 on success.  When provided, raw_rc receives the PSDK result. */
int waypoint_start(uint64_t *raw_rc);
int waypoint_pause(void);
int waypoint_resume(void);
int waypoint_stop(void);
int waypoint_get_status(char *buf, size_t buflen);
void waypoint_cleanup(void);

#endif
