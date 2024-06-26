/*! *********************************************************************************
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017, 2019 NXP
* All rights reserved.
*
* \file
*
* This is the source file for the OS Abstraction layer for freertos. 
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "fsl_os_abstraction_free_rtos.h"
#include <string.h>
#include "GenericList.h"
#include "fsl_common.h"

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define millisecToTicks(millisec) (((millisec) * configTICK_RATE_HZ + 999U)/1000U)

#ifdef DEBUG_ASSERT
#define OS_ASSERT(condition) if(!(condition))while(TRUE);
#else
#define OS_ASSERT(condition) (void)(condition);
#endif

#if defined(osNumberOfEvents) && (osNumberOfEvents > 0)
#define osObjectAlloc_c 1
#else
#define osObjectAlloc_c 0
#endif

/*! @brief Converts milliseconds to ticks*/
#define MSEC_TO_TICK(msec)  (((uint32_t)(msec)+500uL/(uint32_t)configTICK_RATE_HZ) \
                             *(uint32_t)configTICK_RATE_HZ/1000uL)
#define TICKS_TO_MSEC(tick) ((uint32_t)((uint64_t)(tick)*1000uLL/(uint64_t)configTICK_RATE_HZ))
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

typedef struct osEventStruct_tag
{
    uint32_t inUse;
    event_t event;
}osEventStruct_t;

typedef struct osObjStruct_tag
{
    uint32_t inUse;
    uint32_t osObj;
}osObjStruct_t;

typedef struct osObjectInfo_tag
{
    void* pHeap;
    uint32_t objectStructSize;
    uint32_t objNo;
} osObjectInfo_t;


/*! *********************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
********************************************************************************** */
#if osObjectAlloc_c
static void* osObjectAlloc(const osObjectInfo_t* pOsObjectInfo);
static bool_t osObjectIsAllocated(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct);
static osaStatus_t osObjectFree(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct);
#endif
extern void main_task(void const *argument);
extern void hardware_init(void);
void startup_task(void* argument);

/*! *********************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
const uint8_t gUseRtos_c = USE_RTOS;  // USE_RTOS = 0 for BareMetal and 1 for OS
static uint32_t g_base_priority_array[OSA_MAX_ISR_CRITICAL_SECTION_DEPTH];
static int32_t  g_base_priority_top = 0;

/*! *********************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
********************************************************************************** */

#if defined(osNumberOfEvents) && (osNumberOfEvents > 0)
static osEventStruct_t osEventHeap[osNumberOfEvents];
static const osObjectInfo_t osEventInfo = {osEventHeap, sizeof(osEventStruct_t),osNumberOfEvents};
#endif


/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */

/*FUNCTION**********************************************************************
 *
 * Function Name : startup_task
 * Description   : Wrapper over main_task..
 *
 *END**************************************************************************/
void startup_task(void* argument)
{
    main_task((osaTaskParam_t) argument);
    while(TRUE)
    {
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskGetId
 * Description   : This function is used to get current active task's handler.
 *
 *END**************************************************************************/
osaTaskId_t OSA_TaskGetId(void)
{
    return (osaTaskId_t)xTaskGetCurrentTaskHandle();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskYield
 * Description   : When a task calls this function, it will give up CPU and put
 * itself to the tail of ready list.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskYield(void)
{
    taskYIELD();
    return osaStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskGetPriority
 * Description   : This function returns task's priority by task handler.
 *
 *END**************************************************************************/
osaTaskPriority_t OSA_TaskGetPriority(osaTaskId_t taskId)
{
  return (osaTaskPriority_t)(PRIORITY_RTOS_TO_OSA(uxTaskPriorityGet((task_handler_t)taskId)));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskSetPriority
 * Description   : This function sets task's priority by task handler.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskSetPriority(osaTaskId_t taskId, osaTaskPriority_t taskPriority)
{
    vTaskPrioritySet((task_handler_t)taskId, (UBaseType_t)PRIORITY_OSA_TO_RTOS(taskPriority));
    return osaStatus_Success;
}

/*FUNCTION**********************************************************************
*
* Function Name : OSA_TaskCreate
* Description   : This function is used to create a task and make it ready.
* Param[in]     :  threadDef  - Definition of the thread.
*                  task_param - Parameter to pass to the new thread.
* Return Thread handle of the new thread, or NULL if failed.
*
*END**************************************************************************/
osaTaskId_t OSA_TaskCreate(osaThreadDef_t *thread_def,osaTaskParam_t task_param)
{
  osaTaskId_t taskId = NULL;
  task_handler_t task_handler;
  
  if (xTaskCreate(
                  (task_t)thread_def->pthread,  /* pointer to the task */
                  (char const*)thread_def->tname, /* task name for kernel awareness debugging */
                  (configSTACK_DEPTH_TYPE)thread_def->stacksize/sizeof(portSTACK_TYPE), /* task stack size */
                  (task_param_t)task_param, /* optional task startup argument */
                  PRIORITY_OSA_TO_RTOS(thread_def->tpriority),  /* initial priority */
                  &task_handler /* optional task handle to create */
                    ) == pdPASS)
  {
    taskId = (osaTaskId_t)task_handler;
  }
  return taskId;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TaskDestroy
 * Description   : This function destroy a task. 
 * Param[in]     :taskId - Thread handle.
 * Return osaStatus_Success if the task is destroied, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_TaskDestroy(osaTaskId_t taskId)
{
  osaStatus_t status;
  uint16_t oldPriority;
  /*Change priority to avoid context switches*/
  oldPriority = OSA_TaskGetPriority(OSA_TaskGetId());
  (void)OSA_TaskSetPriority(OSA_TaskGetId(), OSA_PRIORITY_REAL_TIME);
#if INCLUDE_vTaskDelete /* vTaskDelete() enabled */
  vTaskDelete((task_handler_t)taskId);
  status = osaStatus_Success;
#else
  status = osaStatus_Error; /* vTaskDelete() not available */
#endif
  (void)OSA_TaskSetPriority(OSA_TaskGetId(), oldPriority);
  
  return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TimeDelay
 * Description   : This function is used to suspend the active thread for the given number of milliseconds.
 *
 *END**************************************************************************/
void OSA_TimeDelay(uint32_t millisec)
{
    vTaskDelay(millisecToTicks(millisec));
}
/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_TimeGetMsec
 * Description   : This function gets current time in milliseconds.
 *
 *END**************************************************************************/
uint32_t OSA_TimeGetMsec(void)
{
    portTickType ticks;

    if (__get_IPSR() != 0U)
    {
        ticks = xTaskGetTickCountFromISR();
    }
    else
    {
        ticks = xTaskGetTickCount();
    }

    return TICKS_TO_MSEC(ticks);
}
/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreCreate
 * Description   : This function is used to create a semaphore. 
 * Return         : Semaphore handle of the new semaphore, or NULL if failed. 
  *
 *END**************************************************************************/
osaSemaphoreId_t OSA_SemaphoreCreate(uint32_t initValue)
{
#if defined(osNumberOfSemaphores) && (osNumberOfSemaphores > 0)
    semaphore_t sem;
    sem = (semaphore_t)xSemaphoreCreateCounting(0xFF, initValue);
    return (osaSemaphoreId_t)sem;
#else 
    (void)initValue;
    return NULL;
#endif
    
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreDestroy
 * Description   : This function is used to destroy a semaphore.
 * Return        : osaStatus_Success if the semaphore is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphoreDestroy(osaSemaphoreId_t semId)
{
#if defined(osNumberOfSemaphores) && (osNumberOfSemaphores > 0)
  semaphore_t sem = (semaphore_t)semId; 
  if(sem == NULL)
  {
    return osaStatus_Error;
  }   
  vSemaphoreDelete(sem);
  return osaStatus_Success;
#else
  (void)semId;
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphoreWait
 * Description   : This function checks the semaphore's counting value, if it is
 * positive, decreases it and returns osaStatus_Success, otherwise, timeout
 * will be used for wait. The parameter timeout indicates how long should wait
 * in milliseconds. Pass osaWaitForever_c to wait indefinitely, pass 0 will
 * return osaStatus_Timeout immediately if semaphore is not positive.
 * This function returns osaStatus_Success if the semaphore is received, returns
 * osaStatus_Timeout if the semaphore is not received within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphoreWait(osaSemaphoreId_t semId, uint32_t millisec)
{
#if defined(osNumberOfSemaphores) && (osNumberOfSemaphores > 0)
  uint32_t timeoutTicks;
  if(semId == NULL)
  {
    return osaStatus_Error;
  }
  semaphore_t sem = (semaphore_t)semId;
  
  /* Convert timeout from millisecond to tick. */
  if (millisec == osaWaitForever_c)
  {
    timeoutTicks = portMAX_DELAY;
  }
  else
  {
    timeoutTicks = MSEC_TO_TICK(millisec);
  }
  
  if (xSemaphoreTake(sem, timeoutTicks)==pdFALSE)
  {
    return osaStatus_Timeout; /* timeout */
  }
  else
  {
    return osaStatus_Success; /* semaphore taken */
  }
#else
  (void)semId; 
  (void)millisec;
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_SemaphorePost
 * Description   : This function is used to wake up one task that wating on the
 * semaphore. If no task is waiting, increase the semaphore. The function returns
 * osaStatus_Success if the semaphre is post successfully, otherwise returns
 * osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_SemaphorePost(osaSemaphoreId_t semId)
{
#if defined(osNumberOfSemaphores) && (osNumberOfSemaphores > 0)
  osaStatus_t status = osaStatus_Error;
  if(semId != NULL)
  {
    semaphore_t sem = (semaphore_t)semId;
    
    if (__get_IPSR() != 0U)
    {
      portBASE_TYPE taskToWake = pdFALSE;
      
      if (pdTRUE==xSemaphoreGiveFromISR(sem, &taskToWake))
      {
        portYIELD_FROM_ISR(taskToWake);
        status = osaStatus_Success;
      }
      else
      {
        status = osaStatus_Error;
      }
    }
    else
    {
      if (pdTRUE == xSemaphoreGive(sem))
      {
        status = osaStatus_Success; /* sync object given */
      }
      else
      {
        status = osaStatus_Error;
      }
    }    
  }
  else
  {
    ;
  }
  return status;
#else
  (void)semId;
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexCreate
 * Description   : This function is used to create a mutex.
 * Return        : Mutex handle of the new mutex, or NULL if failed. 
 *
 *END**************************************************************************/
osaMutexId_t OSA_MutexCreate(void)
{
#if defined(osNumberOfMutexes) && (osNumberOfMutexes > 0)
    mutex_t mutex;
    mutex = xSemaphoreCreateMutex();
    return (osaMutexId_t)mutex;
#else
    return NULL;
#endif      
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexLock
 * Description   : This function checks the mutex's status, if it is unlocked,
 * lock it and returns osaStatus_Success, otherwise, wait for the mutex.
 * This function returns osaStatus_Success if the mutex is obtained, returns
 * osaStatus_Error if any errors occur during waiting. If the mutex has been
 * locked, pass 0 as timeout will return osaStatus_Timeout immediately.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexLock(osaMutexId_t mutexId, uint32_t millisec)
{
#if defined(osNumberOfMutexes) && (osNumberOfMutexes > 0)
    uint32_t timeoutTicks;
    void * mutexHolder;
    void * taskHandle;
    mutex_t mutex = (mutex_t)mutexId;
    if(mutexId == NULL)
    {
     return osaStatus_Error;
    }

    mutexHolder = xSemaphoreGetMutexHolder(mutex);
    taskHandle = xTaskGetCurrentTaskHandle();
    /* If pMutex has been locked by current task, return error. */
    if (mutexHolder == taskHandle)
    {
        return osaStatus_Error;
    }

    /* Convert timeout from millisecond to tick. */
    if (millisec == osaWaitForever_c)
    {
        timeoutTicks = portMAX_DELAY;
    }
    else
    {
        timeoutTicks = MSEC_TO_TICK(millisec);
    }

    if (xSemaphoreTake(mutex, timeoutTicks)==pdFALSE)
    {
        return osaStatus_Timeout; /* timeout */
    }
    else
    {
        return osaStatus_Success; /* semaphore taken */
    }
#else
    (void)mutexId;
    (void)millisec;  
    return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexUnlock
 * Description   : This function is used to unlock a mutex.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexUnlock(osaMutexId_t mutexId)
{
#if defined(osNumberOfMutexes) && (osNumberOfMutexes > 0)
  BaseType_t result;
  void * mutexHolder;
  void * taskHandle;
  mutex_t mutex = (mutex_t)mutexId;
  if(mutexId == NULL)
  {
    return osaStatus_Error;
  }

  mutexHolder = xSemaphoreGetMutexHolder(mutex);
  taskHandle = xTaskGetCurrentTaskHandle();
  /* If pMutex has been locked by current task, return error. */
  if (mutexHolder != taskHandle)
  {
    return osaStatus_Error;
  }
  result = xSemaphoreGive(mutex);
  assert(pdPASS == result);
  NOT_USED(result);
  return osaStatus_Success;
#else
  (void)mutexId;
  return osaStatus_Error;  
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MutexDestroy
 * Description   : This function is used to destroy a mutex.
 * Return        : osaStatus_Success if the lock object is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MutexDestroy(osaMutexId_t mutexId)
{
#if defined(osNumberOfMutexes) && (osNumberOfMutexes > 0)
  mutex_t mutex = (mutex_t)mutexId;
  if(mutexId == NULL)
  {
    return osaStatus_Error;    
  }
  vSemaphoreDelete(mutex);
  return osaStatus_Success;
#else
  (void)mutexId;
  return osaStatus_Error;    
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventCreate
 * Description   : This function is used to create a event object.
 * Return        : Event handle of the new event, or NULL if failed. 
 *
 *END**************************************************************************/
osaEventId_t OSA_EventCreate(bool_t autoClear)
{
#if defined(osNumberOfEvents) && (osNumberOfEvents > 0)
  osaEventId_t eventId;
  osEventStruct_t* pEventStruct; 
  OSA_InterruptDisable();
  pEventStruct = osObjectAlloc(&osEventInfo);
  eventId = pEventStruct;
  OSA_InterruptEnable();
  if(eventId == NULL)
  {
    return NULL;
  }
  
  pEventStruct->event.eventHandler = xEventGroupCreate();
  if (pEventStruct->event.eventHandler != NULL)
  {
    pEventStruct->event.autoClear = autoClear;
  }
  else
  {
    OSA_InterruptDisable();
    (void)osObjectFree(&osEventInfo, eventId);
    OSA_InterruptEnable();
    eventId = NULL;
  }
  return eventId;
#else
  (void)autoClear;
  return NULL;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventSet
 * Description   : Set one or more event flags of an event object.
 * Return        : osaStatus_Success if set successfully, osaStatus_Error if failed.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventSet(osaEventId_t eventId, osaEventFlags_t flagsToSet)
{
#if defined(osNumberOfEvents) && (osNumberOfEvents > 0)
  osEventStruct_t* pEventStruct; 
  BaseType_t result;
  portBASE_TYPE taskToWake = pdFALSE;
  if(osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
  {
    return osaStatus_Error;
  }
  pEventStruct = (osEventStruct_t*)eventId;  
  if(pEventStruct->event.eventHandler == NULL)
  {
    return osaStatus_Error;
  }
  if (__get_IPSR() != 0U)
  {
    result = xEventGroupSetBitsFromISR(pEventStruct->event.eventHandler, (event_flags_t)flagsToSet, &taskToWake);
    assert(pdPASS == result);
    NOT_USED(result);
    portYIELD_FROM_ISR(taskToWake);
  }
  else
  {
    (void)xEventGroupSetBits(pEventStruct->event.eventHandler, (event_flags_t)flagsToSet);
  }
  return osaStatus_Success;
#else
  (void)eventId;
  (void)flagsToSet;  
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventClear
 * Description   : Clear one or more event flags of an event object.
 * Return        :osaStatus_Success if clear successfully, osaStatus_Error if failed.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventClear(osaEventId_t eventId, osaEventFlags_t flagsToClear)
{
#if defined(osNumberOfEvents) && (osNumberOfEvents > 0)
  
  osEventStruct_t* pEventStruct; 
  if(osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
  {
    return osaStatus_Error;
  }
  pEventStruct = (osEventStruct_t*)eventId;  
  if(pEventStruct->event.eventHandler == NULL)
  {
    return osaStatus_Error;
  } 
  
  if (__get_IPSR() != 0U)
  {
    (void)xEventGroupClearBitsFromISR(pEventStruct->event.eventHandler, (event_flags_t)flagsToClear);
  }
  else
  {
    (void)xEventGroupClearBits(pEventStruct->event.eventHandler, (event_flags_t)flagsToClear);
  }
  
  return osaStatus_Success;
#else
  (void)eventId;
  (void)flagsToClear;  
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventWait
 * Description   : This function checks the event's status, if it meets the wait
 * condition, return osaStatus_Success, otherwise, timeout will be used for
 * wait. The parameter timeout indicates how long should wait in milliseconds.
 * Pass osaWaitForever_c to wait indefinitely, pass 0 will return the value
 * osaStatus_Timeout immediately if wait condition is not met. The event flags
 * will be cleared if the event is auto clear mode. Flags that wakeup waiting
 * task could be obtained from the parameter setFlags.
 * This function returns osaStatus_Success if wait condition is met, returns
 * osaStatus_Timeout if wait condition is not met within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventWait(osaEventId_t eventId, osaEventFlags_t flagsToWait, bool_t waitAll, uint32_t millisec, osaEventFlags_t *pSetFlags)
{
#if defined(osNumberOfEvents) && (osNumberOfEvents > 0)
  osEventStruct_t* pEventStruct; 
  BaseType_t clearMode;
  uint32_t timeoutTicks;
  event_flags_t flagsSave;
  if(osObjectIsAllocated(&osEventInfo, eventId) == FALSE)
  {
    return osaStatus_Error;
  }
  
  /* Clean FreeRTOS cotrol flags */
  flagsToWait = flagsToWait & 0x00FFFFFFU;
  
  pEventStruct = (osEventStruct_t*)eventId;  
  if(pEventStruct->event.eventHandler == NULL)
  {
    return osaStatus_Error;
  }
  
  /* Convert timeout from millisecond to tick. */
  if (millisec == osaWaitForever_c)
  {
    timeoutTicks = portMAX_DELAY;
  }
  else
  {
    timeoutTicks = millisec/portTICK_PERIOD_MS;
  }
  
  clearMode = (pEventStruct->event.autoClear) ? pdTRUE: pdFALSE;
  
  flagsSave = xEventGroupWaitBits(pEventStruct->event.eventHandler,(event_flags_t)flagsToWait,clearMode,(BaseType_t)waitAll,timeoutTicks);
  
  flagsSave &= (event_flags_t)flagsToWait;
  if(pSetFlags != NULL)
  {
    *pSetFlags = (osaEventFlags_t)flagsSave;
  }
  
  if (flagsSave != 0U)
  {
    return osaStatus_Success;
  }
  else
  {
    return osaStatus_Timeout;
  }
#else
  (void)eventId;
  (void)flagsToWait;  
  (void)waitAll;  
  (void)millisec;  
  (void)pSetFlags;  
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EventDestroy
 * Description   : This function is used to destroy a event object. Return
 * osaStatus_Success if the event object is destroyed successfully, otherwise
 * return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_EventDestroy(osaEventId_t eventId)
{
#if defined(osNumberOfEvents) && (osNumberOfEvents > 0)
  osEventStruct_t* pEventStruct = (osEventStruct_t*)eventId;
  EventGroupHandle_t eventHandle = pEventStruct->event.eventHandler;
  OSA_InterruptDisable();
  if(osaStatus_Error == osObjectFree(&osEventInfo, eventId))
  {
    OSA_InterruptEnable();
    return osaStatus_Error;
  }
  OSA_InterruptEnable();

  if(NULL == eventHandle)
  {
    return osaStatus_Error;
  }
  vEventGroupDelete(eventHandle);
  return osaStatus_Success;
#else
  (void)eventId;
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQCreate
 * Description   : This function is used to create a message queue.
 * Return        : the handle to the message queue if create successfully, otherwise
 * return NULL.
 *
 *END**************************************************************************/
osaMsgQId_t OSA_MsgQCreate( uint32_t  msgNo )
{
#if defined(osNumberOfMessageQs) && (osNumberOfMessageQs > 0)
    msg_queue_handler_t msg_queue_handler; 

    /* Create the message queue where each element is a pointer to the message item. */
    msg_queue_handler = xQueueCreate(msgNo,sizeof(osaMsg_t));
    return (osaMsgQId_t)msg_queue_handler;
#else
    (void)msgNo;
    return NULL;
#endif  
}


/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQPut
 * Description   : This function is used to put a message to a message queue.
* Return         : osaStatus_Success if the message is put successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQPut(osaMsgQId_t msgQId, osaMsg_t pMessage)
{
#if defined(osNumberOfMessageQs) && (osNumberOfMessageQs > 0)
  msg_queue_handler_t handler;
  osaStatus_t osaStatus;
  if(msgQId == NULL)
  {
    return osaStatus_Error;
  }
  handler = (msg_queue_handler_t)msgQId;
  {
    if (__get_IPSR() != 0U)
    {
      portBASE_TYPE taskToWake = pdFALSE;
      
      if (pdTRUE == xQueueSendToBackFromISR(handler, pMessage, &taskToWake))
      {
        portYIELD_FROM_ISR(taskToWake);
        osaStatus = osaStatus_Success;
      }
      else
      {
        osaStatus =  osaStatus_Error;
      }
      
    }
    else
    {
      osaStatus = (xQueueSendToBack(handler, pMessage, 0)== pdPASS)?(osaStatus_Success):(osaStatus_Error);
    }
  }
  return osaStatus;
#else
  (void)msgQId;
  (void)pMessage;
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQGet
 * Description   : This function checks the queue's status, if it is not empty,
 * get message from it and return osaStatus_Success, otherwise, timeout will
 * be used for wait. The parameter timeout indicates how long should wait in
 * milliseconds. Pass osaWaitForever_c to wait indefinitely, pass 0 will return
 * osaStatus_Timeout immediately if queue is empty.
 * This function returns osaStatus_Success if message is got successfully,
 * returns osaStatus_Timeout if message queue is empty within the specified
 * 'timeout', returns osaStatus_Error if any errors occur during waiting.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQGet(osaMsgQId_t msgQId, osaMsg_t pMessage, uint32_t millisec)
{
#if defined(osNumberOfMessageQs) && (osNumberOfMessageQs > 0)
  osaStatus_t osaStatus;
  msg_queue_handler_t handler;
  uint32_t timeoutTicks;
  if( msgQId == NULL )
  {
    return osaStatus_Error;
  }
  handler = (msg_queue_handler_t)msgQId;
  if (millisec == osaWaitForever_c)
  {
    timeoutTicks = portMAX_DELAY;
  }
  else
  {
    timeoutTicks = MSEC_TO_TICK(millisec);
  }
  if (xQueueReceive(handler, pMessage, timeoutTicks)!=pdPASS)
  {
    osaStatus =  osaStatus_Timeout; /* not able to send it to the queue? */
  }
  else
  {
    osaStatus = osaStatus_Success;
  }
  return osaStatus;
#else
  (void)msgQId;
  (void)pMessage;
  (void)millisec;
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_MsgQDestroy
 * Description   : This function is used to destroy the message queue.
 * Return        : osaStatus_Success if the message queue is destroyed successfully, otherwise return osaStatus_Error.
 *
 *END**************************************************************************/
osaStatus_t OSA_MsgQDestroy(osaMsgQId_t msgQId)
{
#if defined(osNumberOfMessageQs) && (osNumberOfMessageQs > 0)
  msg_queue_handler_t handler;
  if(msgQId == NULL )
  {
    return osaStatus_Error;
  }
  handler = (msg_queue_handler_t)msgQId;
  vQueueDelete(handler);
  return osaStatus_Success;
#else
  (void)msgQId;
  return osaStatus_Error;
#endif  
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InterruptEnable
 * Description   : self explanatory.
 *
 *END**************************************************************************/
void OSA_InterruptEnable(void)
{
  if (__get_IPSR() != 0U)
  {
    if(g_base_priority_top != 0)
    {
    g_base_priority_top--;
    portCLEAR_INTERRUPT_MASK_FROM_ISR(g_base_priority_array[g_base_priority_top]);
    }

  }
  else
  {
    portEXIT_CRITICAL();
  }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InterruptDisable
 * Description   : self explanatory.
 *
 *END**************************************************************************/
void OSA_InterruptDisable(void)
{
  if (__get_IPSR() != 0U)
  {
    if((uint32_t)g_base_priority_top < OSA_MAX_ISR_CRITICAL_SECTION_DEPTH)
    {
      g_base_priority_array[g_base_priority_top] = portSET_INTERRUPT_MASK_FROM_ISR();
      g_base_priority_top++;
    }
    
  }
  else
  {
    portENTER_CRITICAL();
  }
  
}

uint32_t gInterruptDisableCount = 0;
/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_EnableIRQGlobal
 * Description   : enable interrupts using PRIMASK register.
 *
 *END**************************************************************************/
void OSA_EnableIRQGlobal(void)
{
    if (gInterruptDisableCount > 0U)
    {
        gInterruptDisableCount--;
        
        if (gInterruptDisableCount == 0U)
        {
            __enable_irq();
        }
        /* call core API to enable the global interrupt*/
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_DisableIRQGlobal
 * Description   : disable interrupts using PRIMASK register.
 *
 *END**************************************************************************/
void OSA_DisableIRQGlobal(void)
{
    /* call core API to disable the global interrupt*/
    __disable_irq();
    
    /* update counter*/
    gInterruptDisableCount++;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : OSA_InstallIntHandler
 * Description   : This function is used to install interrupt handler.
 *
 *END**************************************************************************/
void OSA_InstallIntHandler(uint32_t IRQNumber, void (*handler)(void))
{

#if defined ( __IAR_SYSTEMS_ICC__ )
    _Pragma ("diag_suppress = Pm138")
#endif
     union 
     {
        handlerFucPtr_t funHandlePtr;
        uint32_t irqHandler;
     }osaIrqHandler;

    osaIrqHandler.funHandlePtr = handler;
    (void)InstallIRQHandler((IRQn_Type)IRQNumber, osaIrqHandler.irqHandler);
#if defined ( __IAR_SYSTEMS_ICC__ )
    _Pragma ("diag_remark = PM138")
#endif
}

/*! *********************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
********************************************************************************** */
static OSA_TASK_DEFINE(startup_task, gMainThreadPriority_c, 1, gMainThreadStackSize_c, 0)  ;
void main (void)
{
    /* Initialize MCU clock */
    hardware_init();
    (void)OSA_TaskCreate(OSA_TASK(startup_task), NULL);
    vTaskStartScheduler();
}

/*! *********************************************************************************
* \brief     Allocates a osObjectStruct_t block in the osObjectHeap array.
* \param[in] pointer to the object info struct.
* Object can be semaphore, mutex, message Queue, event
* \return Pointer to the allocated osObjectStruct_t, NULL if failed.
*
* \pre 
*
* \post
*
* \remarks Function is unprotected from interrupts. 
*
********************************************************************************** */
#if osObjectAlloc_c
static void* osObjectAlloc(const osObjectInfo_t* pOsObjectInfo)
{
    uint32_t i;
    uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
    for( i=0 ; i < pOsObjectInfo->objNo ; i++)
    {
        if(((osObjStruct_t*)(uint32_t)pObj)->inUse == 0U)
        {
            ((osObjStruct_t*)(uint32_t)pObj)->inUse = 1U;
            return (void*)pObj;
        }
        pObj += pOsObjectInfo->objectStructSize;
    }
    return NULL;
}
#endif

/*! *********************************************************************************
* \brief     Verifies the object is valid and allocated in the osObjectHeap array.
* \param[in] the pointer to the object info struct.
* \param[in] the pointer to the object struct.
* Object can be semaphore, mutex,  message Queue, event
* \return TRUE if the object is valid and allocated, FALSE otherwise
*
* \pre 
*
* \post
*
* \remarks Function is unprotected from interrupts. 
*
********************************************************************************** */
#if osObjectAlloc_c
static bool_t osObjectIsAllocated(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct)
{
    uint32_t i;
    uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
    for( i=0 ; i < pOsObjectInfo->objNo ; i++ )
    {
        if(pObj == pObjectStruct)
        {
            if(((osObjStruct_t*)(uint32_t)pObj)->inUse != 0U)
            {
                return TRUE;
            }
            break;
        }
        pObj += pOsObjectInfo->objectStructSize;
    }
    return FALSE;
}
#endif

/*! *********************************************************************************
* \brief     Frees an osObjectStruct_t block from the osObjectHeap array.
* \param[in] pointer to the object info struct.
* \param[in] Pointer to the allocated osObjectStruct_t to free.
* Object can be semaphore, mutex, message Queue, event
* \return none.
*
* \pre 
*
* \post
*
* \remarks Function is unprotected from interrupts. 
*
********************************************************************************** */
#if osObjectAlloc_c
static osaStatus_t osObjectFree(const osObjectInfo_t* pOsObjectInfo, void* pObjectStruct)
{
    uint32_t i;
    uint8_t* pObj = (uint8_t*)pOsObjectInfo->pHeap;
    for( i=0; i < pOsObjectInfo->objNo; i++)
    {
        if(pObj == pObjectStruct)
        {
            if (((osObjStruct_t*)(uint32_t)pObj)->inUse != 0U)
            {
                ((osObjStruct_t*)(uint32_t)pObj)->inUse = 0U;
                return osaStatus_Success;
            }
            break;
        }
        pObj += pOsObjectInfo->objectStructSize;
    }
    return osaStatus_Error;
}
#endif

/*! *********************************************************************************
* \brief    FreeRTOS application malloc failed hook
*
*
* \remarks  Function is called by FreeRTOS if there is not enough space in the
*           heap for task stack allocation or for OS object allocation
*
********************************************************************************** */
#if (configUSE_MALLOC_FAILED_HOOK==1)
void vApplicationMallocFailedHook (void)
{
    panic(0,(uint32_t)vApplicationMallocFailedHook,0,0);
}
#endif

