#include "waypoint.h"
#include <stdio.h>
#include <string.h>



int waypoint_init(void) { printf("[waypoint] disabled: M300 requires Waypoint V2\n"); return 0; }
int waypoint_upload(const char *kmz_path) { (void)kmz_path; return -1; }
int waypoint_start(void) { return -1; }
int waypoint_pause(void) { return -1; }
int waypoint_resume(void) { return -1; }
int waypoint_stop(void) { return -1; }
int waypoint_get_status(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"state\":\"unavailable\",\"reason\":\"M300 requires Waypoint V2\"}");
    return 0;
}
void waypoint_cleanup(void) {}
