#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#include <stdint.h>
#include <stddef.h>

/* Format a PSDK error code into a JSON string with description and recovery hint.
 * Returns number of bytes written (excluding null terminator). */
int error_code_to_json(uint64_t code, char *buf, size_t buflen);

#endif
