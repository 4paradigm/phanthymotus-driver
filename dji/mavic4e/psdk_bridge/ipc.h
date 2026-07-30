#ifndef IPC_H
#define IPC_H

#include <stdint.h>
#include <stddef.h>

/**
 * IPC layer — Unix domain socket server for Python ↔ C bridge communication.
 *
 * Wire format: 4-byte big-endian length prefix + JSON payload (UTF-8).
 *
 * Request:  {"id": N, "cmd": "...", "args": {...}}
 * Response: {"id": N, "ok": true/false, "data": {...}}
 * Push:     {"push": "type", "data": {...}}
 */

/* Command handler callback type.
 * @param cmd    Command name (e.g., "takeoff", "get_telemetry")
 * @param args   JSON string of arguments
 * @param result Output buffer for JSON response (caller allocates)
 * @param result_size Size of result buffer
 * @return 0 on success, -1 on error
 */
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

/* Send a push message to connected client.
 * @param type  Push type (e.g., "telemetry", "frame", "hms")
 * @param data  JSON string of push data */
int ipc_push(const char *type, const char *data);

/* Cleanup and close IPC server. */
void ipc_cleanup(void);

#endif /* IPC_H */
