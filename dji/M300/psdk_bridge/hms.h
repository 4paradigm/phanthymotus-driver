#ifndef HMS_H
#define HMS_H

#include <stddef.h>
#include <stdint.h>

int hms_init(void);
int hms_get_info(char *buf, size_t buflen);
int hms_inject_error(uint32_t error_code, uint8_t error_level);
int hms_eliminate_error(uint32_t error_code);
void hms_cleanup(void);

#endif
