#ifdef PSDK_ENABLED

#include "osal_posix.h"

#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

static uint32_t s_local_time_ms_offset;
static uint64_t s_local_time_us_offset;

T_DjiReturnCode Osal_TaskCreate(const char *name, void *(*taskFunc)(void *),
                                uint32_t stackSize, void *arg, T_DjiTaskHandle *task) {
    (void)stackSize; /* hzhy uses the pthread default stack for PSDK tasks. */
    if (!task || !taskFunc)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;

    pthread_t *thread = malloc(sizeof(*thread));
    if (!thread)
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    if (pthread_create(thread, NULL, taskFunc, arg) != 0) {
        free(thread);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (name) {
        char thread_name[16] = {0};
        strncpy(thread_name, name, sizeof(thread_name) - 1);
        (void)pthread_setname_np(*thread, thread_name);
    }
    *task = (T_DjiTaskHandle)thread;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_TaskDestroy(T_DjiTaskHandle task) {
    if (!task)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    (void)pthread_cancel(*(pthread_t *)task);
    free(task); /* hzhy intentionally does not join PSDK worker tasks. */
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_TaskSleepMs(uint32_t timeMs) {
    usleep((useconds_t)timeMs * 1000U);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_MutexCreate(T_DjiMutexHandle *mutex) {
    if (!mutex)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    pthread_mutex_t *value = malloc(sizeof(*value));
    if (!value)
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    if (pthread_mutex_init(value, NULL) != 0) {
        free(value);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    *mutex = (T_DjiMutexHandle)value;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_MutexDestroy(T_DjiMutexHandle mutex) {
    if (!mutex || pthread_mutex_destroy((pthread_mutex_t *)mutex) != 0)
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    free(mutex);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_MutexLock(T_DjiMutexHandle mutex) {
    return mutex && pthread_mutex_lock((pthread_mutex_t *)mutex) == 0 ?
           DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

T_DjiReturnCode Osal_MutexUnlock(T_DjiMutexHandle mutex) {
    return mutex && pthread_mutex_unlock((pthread_mutex_t *)mutex) == 0 ?
           DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

T_DjiReturnCode Osal_SemaphoreCreate(uint32_t initValue, T_DjiSemaHandle *semaphore) {
    if (!semaphore)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    sem_t *value = malloc(sizeof(*value));
    if (!value)
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    if (sem_init(value, 0, initValue) != 0) {
        free(value);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    *semaphore = (T_DjiSemaHandle)value;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_SemaphoreDestroy(T_DjiSemaHandle semaphore) {
    if (!semaphore || sem_destroy((sem_t *)semaphore) != 0)
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    free(semaphore);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_SemaphoreWait(T_DjiSemaHandle semaphore) {
    return semaphore && sem_wait((sem_t *)semaphore) == 0 ?
           DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

T_DjiReturnCode Osal_SemaphoreTimedWait(T_DjiSemaHandle semaphore, uint32_t waitTime) {
    struct timeval now;
    struct timespec deadline;
    if (!semaphore)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    gettimeofday(&now, NULL);
    deadline.tv_sec = now.tv_sec + waitTime / 1000U;
    deadline.tv_nsec = (long)(now.tv_usec + (waitTime % 1000U) * 1000U) * 1000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec += deadline.tv_nsec / 1000000000L;
        deadline.tv_nsec %= 1000000000L;
    }
    return sem_timedwait((sem_t *)semaphore, &deadline) == 0 ?
           DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

T_DjiReturnCode Osal_SemaphorePost(T_DjiSemaHandle semaphore) {
    return semaphore && sem_post((sem_t *)semaphore) == 0 ?
           DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

T_DjiReturnCode Osal_GetTimeMs(uint32_t *ms) {
    struct timeval now;
    if (!ms)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    gettimeofday(&now, NULL);
    *ms = (uint32_t)(now.tv_sec * 1000ULL + now.tv_usec / 1000U);
    if (s_local_time_ms_offset == 0)
        s_local_time_ms_offset = *ms;
    else
        *ms -= s_local_time_ms_offset;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_GetTimeUs(uint64_t *us) {
    struct timeval now;
    if (!us)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    gettimeofday(&now, NULL);
    *us = (uint64_t)now.tv_sec * 1000000ULL + (uint64_t)now.tv_usec;
    if (s_local_time_us_offset == 0)
        s_local_time_us_offset = *us;
    else
        *us -= s_local_time_us_offset;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode Osal_GetRandomNum(uint16_t *randomNum) {
    if (!randomNum)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    srand((unsigned)time(NULL));
    *randomNum = (uint16_t)(random() % 65535U);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void *Osal_Malloc(uint32_t size) { return malloc(size); }
void Osal_Free(void *ptr) { free(ptr); }

#endif /* PSDK_ENABLED */
