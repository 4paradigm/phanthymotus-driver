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

/* Cleanup telemetry subscriptions. */
void telemetry_cleanup(void);

#endif
