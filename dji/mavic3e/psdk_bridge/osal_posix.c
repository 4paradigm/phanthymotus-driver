#include "osal_posix.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>

/* ── Task (Thread) ──────────────────────────────────────────────────── */

int Osal_TaskCreate(const char *name, OsalTaskFunc func, uint32_t stackSize,
                    void *arg, T_OsalTask *task) {
    if (!task || !func) return -1;

    pthread_t *tid = (pthread_t *)malloc(sizeof(pthread_t));
    if (!tid) return -1;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (stackSize > 0) {
        /* Minimum stack size check */
        size_t min_stack = 64 * 1024;
        if (stackSize < min_stack) stackSize = min_stack;
        pthread_attr_setstacksize(&attr, stackSize);
    }
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    int ret = pthread_create(tid, &attr, func, arg);
    pthread_attr_destroy(&attr);

    if (ret != 0) {
        printf("[osal] TaskCreate '%s' failed: %s\n", name ? name : "?", strerror(ret));
        free(tid);
        return -1;
    }

    /* Set thread name (best effort, max 15 chars on Linux) */
    if (name) {
        char short_name[16];
        strncpy(short_name, name, 15);
        short_name[15] = '\0';
        pthread_setname_np(*tid, short_name);
    }

    task->handle = tid;
    return 0;
}

int Osal_TaskDestroy(T_OsalTask *task) {
    if (!task || !task->handle) return -1;
    pthread_t *tid = (pthread_t *)task->handle;
    pthread_cancel(*tid);
    pthread_join(*tid, NULL);
    free(tid);
    task->handle = NULL;
    return 0;
}

int Osal_TaskSleepMs(uint32_t ms) {
    usleep((useconds_t)ms * 1000);
    return 0;
}

/* ── Mutex ──────────────────────────────────────────────────────────── */

int Osal_MutexCreate(T_OsalMutex *mutex) {
    if (!mutex) return -1;
    pthread_mutex_t *m = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
    if (!m) return -1;

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    int ret = pthread_mutex_init(m, &attr);
    pthread_mutexattr_destroy(&attr);

    if (ret != 0) {
        free(m);
        return -1;
    }
    mutex->handle = m;
    return 0;
}

int Osal_MutexLock(T_OsalMutex *mutex) {
    if (!mutex || !mutex->handle) return -1;
    return pthread_mutex_lock((pthread_mutex_t *)mutex->handle) == 0 ? 0 : -1;
}

int Osal_MutexUnlock(T_OsalMutex *mutex) {
    if (!mutex || !mutex->handle) return -1;
    return pthread_mutex_unlock((pthread_mutex_t *)mutex->handle) == 0 ? 0 : -1;
}

int Osal_MutexDestroy(T_OsalMutex *mutex) {
    if (!mutex || !mutex->handle) return -1;
    pthread_mutex_destroy((pthread_mutex_t *)mutex->handle);
    free(mutex->handle);
    mutex->handle = NULL;
    return 0;
}

/* ── Semaphore ──────────────────────────────────────────────────────── */

int Osal_SemaphoreCreate(uint32_t initValue, T_OsalSemaphore *sem) {
    if (!sem) return -1;
    sem_t *s = (sem_t *)malloc(sizeof(sem_t));
    if (!s) return -1;
    if (sem_init(s, 0, initValue) != 0) {
        free(s);
        return -1;
    }
    sem->handle = s;
    return 0;
}

int Osal_SemaphoreWait(T_OsalSemaphore *sem) {
    if (!sem || !sem->handle) return -1;
    while (sem_wait((sem_t *)sem->handle) != 0) {
        if (errno != EINTR) return -1;
    }
    return 0;
}

int Osal_SemaphoreTimedWait(T_OsalSemaphore *sem, uint32_t timeout_ms) {
    if (!sem || !sem->handle) return -1;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000L;
    }

    int ret = sem_timedwait((sem_t *)sem->handle, &ts);
    if (ret != 0) {
        return (errno == ETIMEDOUT) ? 1 : -1;  /* 1 = timeout, -1 = error */
    }
    return 0;
}

int Osal_SemaphorePost(T_OsalSemaphore *sem) {
    if (!sem || !sem->handle) return -1;
    return sem_post((sem_t *)sem->handle) == 0 ? 0 : -1;
}

int Osal_SemaphoreDestroy(T_OsalSemaphore *sem) {
    if (!sem || !sem->handle) return -1;
    sem_destroy((sem_t *)sem->handle);
    free(sem->handle);
    sem->handle = NULL;
    return 0;
}

/* ── Time ──────────────────────────────────────────────────────────── */

uint64_t Osal_GetTimeMs(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

uint64_t Osal_GetTimeUs(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ── Memory ────────────────────────────────────────────────────────── */

void *Osal_Malloc(uint32_t size) {
    return malloc(size);
}

void Osal_Free(void *ptr) {
    free(ptr);
}
