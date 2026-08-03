#include "waypoint.h"
#include <stdio.h>
#include <string.h>

/*
 * M300 uses PSDK Waypoint V2, not the Mavic-only Waypoint V3/KMZ protocol.
 * The old V3 implementation is deliberately not compiled into this driver.
 * A V2 mission builder needs a different data model, so callers receive an
 * explicit unavailable result instead of sending an incompatible mission.
 */

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
