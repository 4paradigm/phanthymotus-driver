#ifndef OSAL_POSIX_H
#define OSAL_POSIX_H


#ifdef PSDK_ENABLED

#include "dji_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

T_DjiReturnCode Osal_TaskCreate(const char *name, void *(*taskFunc)(void *),
                                uint32_t stackSize, void *arg, T_DjiTaskHandle *task);
T_DjiReturnCode Osal_TaskDestroy(T_DjiTaskHandle task);
T_DjiReturnCode Osal_TaskSleepMs(uint32_t timeMs);

T_DjiReturnCode Osal_MutexCreate(T_DjiMutexHandle *mutex);
T_DjiReturnCode Osal_MutexDestroy(T_DjiMutexHandle mutex);
T_DjiReturnCode Osal_MutexLock(T_DjiMutexHandle mutex);
T_DjiReturnCode Osal_MutexUnlock(T_DjiMutexHandle mutex);

T_DjiReturnCode Osal_SemaphoreCreate(uint32_t initValue, T_DjiSemaHandle *semaphore);
T_DjiReturnCode Osal_SemaphoreDestroy(T_DjiSemaHandle semaphore);
T_DjiReturnCode Osal_SemaphoreWait(T_DjiSemaHandle semaphore);
T_DjiReturnCode Osal_SemaphoreTimedWait(T_DjiSemaHandle semaphore, uint32_t waitTime);
T_DjiReturnCode Osal_SemaphorePost(T_DjiSemaHandle semaphore);

T_DjiReturnCode Osal_GetTimeMs(uint32_t *ms);
T_DjiReturnCode Osal_GetTimeUs(uint64_t *us);
T_DjiReturnCode Osal_GetRandomNum(uint16_t *randomNum);
void *Osal_Malloc(uint32_t size);
void Osal_Free(void *ptr);

#ifdef __cplusplus
}
#endif

#endif /* PSDK_ENABLED */
#endif /* OSAL_POSIX_H */
