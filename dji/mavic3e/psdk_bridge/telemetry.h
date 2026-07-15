#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stddef.h>

/* Initialize FC subscription module.
 * Subscribes to attitude, position, velocity, battery, GPS, obstacles, etc. */
int telemetry_init(void);

/* Get latest telemetry as JSON string.
 * @param buf    Output buffer
 * @param buflen Buffer size
 * @return 0 on success */
int telemetry_get_json(char *buf, size_t buflen);

/* Get current GPS satellite count and fix state.
 * @return number of satellites used (0 if no fix) */
int telemetry_get_gps_satellite_count(void);

/* Get current flight display mode (E_DjiFcSubscriptionDisplayMode) */
int telemetry_get_display_mode(void);

/* Get current fused altitude above home point (meters) */
float telemetry_get_altitude(void);

/* Cleanup telemetry subscriptions. */
void telemetry_cleanup(void);

#endif
