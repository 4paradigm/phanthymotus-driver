#ifndef TIME_SYNC_H
#define TIME_SYNC_H

int time_sync_init(void);
int time_sync_get_aircraft_time(char *buf, size_t buflen);
void time_sync_cleanup(void);

#endif
