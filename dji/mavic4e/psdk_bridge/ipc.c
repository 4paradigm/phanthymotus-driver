#include "ipc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <fcntl.h>

#define MAX_MSG_SIZE (1024 * 1024)  /* 1MB max message */

static int s_server_fd = -1;
static int s_client_fd = -1;
static ipc_cmd_handler_t s_handler = NULL;
static char s_socket_path[256] = {0};

int ipc_init(const char *socket_path) {
    struct sockaddr_un addr;

    strncpy(s_socket_path, socket_path, sizeof(s_socket_path) - 1);

    /* Remove stale socket file */
    unlink(socket_path);

    s_server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s_server_fd < 0) {
        perror("[ipc] socket");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);

    if (bind(s_server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[ipc] bind");
        close(s_server_fd);
        return -1;
    }

    if (listen(s_server_fd, 1) < 0) {
        perror("[ipc] listen");
        close(s_server_fd);
        return -1;
    }

    /* Non-blocking accept */
    fcntl(s_server_fd, F_SETFL, O_NONBLOCK);

    printf("[ipc] listening on %s\n", socket_path);
    return 0;
}

void ipc_set_handler(ipc_cmd_handler_t handler) {
    s_handler = handler;
}

static int _recv_exact(int fd, void *buf, size_t len) {
    size_t received = 0;
    while (received < len) {
        ssize_t n = recv(fd, (char *)buf + received, len - received, 0);
        if (n <= 0) return -1;
        received += n;
    }
    return 0;
}

static int _send_exact(int fd, const void *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, (const char *)buf + sent, len - sent, 0);
        if (n <= 0) return -1;
        sent += n;
    }
    return 0;
}

static int _send_msg(int fd, const char *json_str) {
    uint32_t len = (uint32_t)strlen(json_str);
    uint32_t net_len = htonl(len);
    if (_send_exact(fd, &net_len, 4) < 0) return -1;
    if (_send_exact(fd, json_str, len) < 0) return -1;
    return 0;
}

void ipc_process(void) {
    /* Accept new connections */
    if (s_client_fd < 0) {
        s_client_fd = accept(s_server_fd, NULL, NULL);
        if (s_client_fd >= 0) {
            printf("[ipc] client connected\n");
        }
        return;  /* Process on next iteration */
    }

    /* Check if data available (non-blocking) */
    fd_set fds;
    struct timeval tv = {0, 10000};  /* 10ms timeout */
    FD_ZERO(&fds);
    FD_SET(s_client_fd, &fds);
    if (select(s_client_fd + 1, &fds, NULL, NULL, &tv) <= 0) {
        return;
    }

    /* Read length prefix */
    uint32_t net_len;
    if (_recv_exact(s_client_fd, &net_len, 4) < 0) {
        printf("[ipc] client disconnected\n");
        close(s_client_fd);
        s_client_fd = -1;
        return;
    }
    uint32_t msg_len = ntohl(net_len);
    if (msg_len > MAX_MSG_SIZE) {
        printf("[ipc] message too large: %u\n", msg_len);
        close(s_client_fd);
        s_client_fd = -1;
        return;
    }

    /* Read JSON payload */
    char *msg = (char *)malloc(msg_len + 1);
    if (!msg) return;
    if (_recv_exact(s_client_fd, msg, msg_len) < 0) {
        free(msg);
        close(s_client_fd);
        s_client_fd = -1;
        return;
    }
    msg[msg_len] = '\0';

    /* Simple JSON field extraction (avoid external JSON lib dependency).
     * For production, use cJSON or similar. */
    /* Extract "id", "cmd", "args" fields */
    /* TODO: integrate cJSON for robust parsing */

    char result_buf[MAX_MSG_SIZE];
    if (s_handler) {
        /* For now, pass the raw JSON to the handler */
        s_handler(msg, "", result_buf, sizeof(result_buf));
        _send_msg(s_client_fd, result_buf);
    }

    free(msg);
}

int ipc_push(const char *type, const char *data) {
    if (s_client_fd < 0) return -1;

    char buf[MAX_MSG_SIZE];
    snprintf(buf, sizeof(buf), "{\"push\":\"%s\",\"data\":%s}", type, data);
    return _send_msg(s_client_fd, buf);
}

void ipc_cleanup(void) {
    if (s_client_fd >= 0) {
        close(s_client_fd);
        s_client_fd = -1;
    }
    if (s_server_fd >= 0) {
        close(s_server_fd);
        s_server_fd = -1;
    }
    if (s_socket_path[0]) {
        unlink(s_socket_path);
    }
    printf("[ipc] cleaned up\n");
}
