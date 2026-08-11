#ifndef IPC_H
#define IPC_H

#include <stdint.h>
#include <stddef.h>




typedef int (*ipc_cmd_handler_t)(const char *cmd, const char *args,
                                 char *result, size_t result_size);

/* Initialize IPC server on the given Unix socket path.
 * Returns 0 on success, -1 on error. */
int ipc_init(const char *socket_path);

/* Set the command handler callback. */
void ipc_set_handler(ipc_cmd_handler_t handler);

/* Process pending IPC commands (non-blocking).
 * Call this in the main loop. */
void ipc_process(void);


int ipc_push(const char *type, const char *data);

/* Cleanup and close IPC server. */
void ipc_cleanup(void);

#endif /* IPC_H */
