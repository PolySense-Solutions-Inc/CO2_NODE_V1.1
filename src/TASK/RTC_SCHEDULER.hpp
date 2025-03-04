#ifndef RTC_SCHEDULER_H
#define RTC_SCHEDULER_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CALLBACK_TASK,
    RTOS_TASK
} TaskType;


//Requests Placed in queue for adding to task scheduler
typedef struct {
    TaskType taskType;                 // Type of task (Callback or RTOS)
    union {
        void (*f_cb_ptr)(void);        // Callback function pointer
        TaskHandle_t rtosTaskHandle;   // FreeRTOS Task Handle
    } taskFunc;

    int64_t delay;                     // Delay before the first execution
    uint32_t iterations;               // Number of times to execute
    char tskname[32];                  // Task name
} TaskAddRequest;

//Scheduler task item, part of doubly linked list.
typedef struct DeferredTask {
    TaskType taskType;                 // Type of task (Callback or RTOS)
    union {
        void (*f_cb_ptr)(void);        // Callback function pointer
        TaskHandle_t rtosTaskHandle;   // FreeRTOS Task Handle
    } taskFunc;

    int64_t nextTrigger;               // Next trigger time
    uint32_t iterations;               // Number of times to call the function
    uint32_t executionCount;           // Number of times the function has been called
    struct DeferredTask *nextTask_ptr; // Pointer to the next task
    struct DeferredTask *prevTask_ptr; // Pointer to the previous task
    char tskname[32];                  // Task name
} DeferredTask;

/**
 * @brief The scheduler task that manages and executes deferred tasks.
 *
 * @param param Pointer to the head of the linked list of DeferredTask structures.
 */
void schedulerTask(void *param);

/**
 * @brief Adds a new deferred task to the scheduler.
 *
 * @param callback Function pointer to the task's callback function.
 * @param delay The initial delay before the task's first execution (in microseconds).
 * @param iterations The number of times the task should be executed.
 */
void addCallbackTask(void (*callback)(void), int64_t delay, uint32_t iterations, const char * taskNameShort);

/**
 * @brief Adds a new deferred RTOS task resumption to the scheduler.
 *
 * @param taskHandle FreeRTOS TaskHandle_t of task to resume
 * @param delay The initial delay before the task's first execution (in microseconds).
 * @param iterations The number of times the task should be executed.
 */
void addRTOSTask(TaskHandle_t taskHandle, int64_t delay, uint32_t iterations, const char * taskNameShort);

/**
 * @brief Create the RTC based SchedulerTask
 */
void createRTCSchedulerTask();


//Utility Functions
/**
 * @brief Puts the ESP32 into light sleep for the specified duration, managing power pins.
 *
 * @param sleepTime_us The duration to sleep in microseconds.
 */
void holdSleep(uint64_t sleepTime_us);

/**
 * @brief Enables hold functionality for specific power pins before light sleep.
 */
void enSlpPwrPinHolds();

/**
 * @brief Disables hold functionality for specific power pins after light sleep.
 */
void disSlpPwrPinHolds();


void runRTCScheduler();
void stopRTCScheduler();

#ifdef __cplusplus
}
#endif

#endif // RTC_SCHEDULER_H
