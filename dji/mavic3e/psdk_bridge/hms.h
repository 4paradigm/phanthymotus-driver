#ifndef HMS_H
#define HMS_H

#include <stddef.h>

int hms_init(void);
int hms_get_info(char *buf, size_t buflen);
void hms_cleanup(void);

#endif
