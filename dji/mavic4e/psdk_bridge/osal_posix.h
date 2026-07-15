#ifndef OSAL_POSIX_H
#define OSAL_POSIX_H

#include <stdint.h>
#include <stddef.h>

/**
 * OSAL (Operating System Abstraction Layer) — POSIX implementation for PSDK.
 *
 * Provides thread, mutex, semaphore, time, and memory operations
 * required by the PSDK core library.
 */

/* Task (Thread) */
typedef void *(*OsalTaskFunc)(void *arg);

typedef struct {
    void *handle;  /* pthread_t internal */
} T_OsalTask;

int Osal_TaskCreate(const char *name, OsalTaskFunc func, uint32_t stackSize,
                    void *arg, T_OsalTask *task);
int Osal_TaskDestroy(T_OsalTask *task);
int Osal_TaskSleepMs(uint32_t ms);

/* Mutex */
typedef struct {
    void *handle;  /* pthread_mutex_t internal */
} T_OsalMutex;

int Osal_MutexCreate(T_OsalMutex *mutex);
int Osal_MutexLock(T_OsalMutex *mutex);
int Osal_MutexUnlock(T_OsalMutex *mutex);
int Osal_MutexDestroy(T_OsalMutex *mutex);

/* Semaphore */
typedef struct {
    void *handle;  /* sem_t internal */
} T_OsalSemaphore;

int Osal_SemaphoreCreate(uint32_t initValue, T_OsalSemaphore *sem);
int Osal_SemaphoreWait(T_OsalSemaphore *sem);
int Osal_SemaphoreTimedWait(T_OsalSemaphore *sem, uint32_t timeout_ms);
int Osal_SemaphorePost(T_OsalSemaphore *sem);
int Osal_SemaphoreDestroy(T_OsalSemaphore *sem);

/* Time */
uint64_t Osal_GetTimeMs(void);
uint64_t Osal_GetTimeUs(void);

/* Memory */
void *Osal_Malloc(uint32_t size);
void Osal_Free(void *ptr);

#endif /* OSAL_POSIX_H */
