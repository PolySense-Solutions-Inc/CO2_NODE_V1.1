#include "RTC_SCHEDULER.hpp"
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "../CONFIG/NODE_CONFIG.h"
#if USE_USB_SERIAL_CONN_TASK
#include "../UTILS/USB_CONN.hpp"
#endif

#define DBG_LOGGING 0

#define TASK_QUEUE_SIZE 10
QueueHandle_t taskAddQueue;



extern SemaphoreHandle_t mtx_radioInUseLockout;
extern TaskHandle_t taskHandle_RTCScheduler;
SemaphoreHandle_t mtx_RTCTasks;

// File-level global for task list head
static DeferredTask *head = NULL;

volatile bool isRunning = false;
volatile bool taskListChanged = false;

void enSlpPwrPinHolds() {

    //pinMode(STATUS_LED, OUTPUT);
    //digitalWrite(STATUS_LED, LOW);
    //gpio_hold_en((gpio_num_t)STATUS_LED);

    gpio_hold_en((gpio_num_t)I2C_PULLUPS);
    gpio_hold_en((gpio_num_t)K33_PWR);
    gpio_hold_en((gpio_num_t)EN_3V3_ALT);
    gpio_hold_en((gpio_num_t)EN_5V_BOOST);
}

void disSlpPwrPinHolds() {
    gpio_hold_dis((gpio_num_t)I2C_PULLUPS);
    gpio_hold_dis((gpio_num_t)K33_PWR);
    gpio_hold_dis((gpio_num_t)EN_3V3_ALT);
    gpio_hold_dis((gpio_num_t)EN_5V_BOOST);

    //gpio_hold_dis((gpio_num_t)STATUS_LED);
    //pinMode(STATUS_LED, OUTPUT);
    //digitalWrite(STATUS_LED, HIGH);
    
}

void holdSleep(uint64_t sleepTime_us) {
    uint64_t startTime = esp_timer_get_time();
    if (!USB_CDC_Connected()) {
        ESP_LOGI("HoldSleep", "Light Sleeping between actions for %llu uS", sleepTime_us);
        enSlpPwrPinHolds();
        esp_sleep_enable_timer_wakeup(sleepTime_us);
        esp_light_sleep_start();
        disSlpPwrPinHolds();
    } else {
        ESP_LOGI("HoldSleep", "Skipping light sleep since USB CDC is connected!");
        delayMicroseconds(sleepTime_us);
    }
    uint64_t stopTime = esp_timer_get_time();
    
    //Not available unless tickless idle is supported, and even then, since its privileged, might need the MPU.
    //vTaskStepTick((TickType_t)((stopTime - startTime)/1000));
}
void addCallbackTask(void (*callback)(void), int64_t delay, uint32_t iterations, const char *taskNameShort) {
    
    if (taskAddQueue == NULL) {
        ESP_LOGE("RTCScheduler", "Task add queue is not initialized!");
        return;
    }
    TaskAddRequest request;
    request.taskType = CALLBACK_TASK;
    request.taskFunc.f_cb_ptr = callback;
    request.delay = esp_timer_get_time() +delay;
    request.iterations = iterations;
    strncpy(request.tskname, taskNameShort, sizeof(request.tskname) - 1);
    request.tskname[sizeof(request.tskname) - 1] = '\0'; // Ensure null-termination

    if (xQueueSend(taskAddQueue, &request, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE("RTCScheduler", "Failed to enqueue callback task '%s'", taskNameShort);
    } else {
        ESP_LOGI("RTCScheduler", "Enqueued callback task '%s'", taskNameShort);
    }
}

void addRTOSTask(TaskHandle_t taskHandle, int64_t delay, uint32_t iterations, const char *taskNameShort) {
    
    if (taskAddQueue == NULL) {
        ESP_LOGE("RTCScheduler", "Task add queue is not initialized!");
        return;
    }
    TaskAddRequest request;
    request.taskType = RTOS_TASK;
    request.taskFunc.rtosTaskHandle = taskHandle;
     if (DBG_LOGGING) ESP_LOGE("DBG","Add Taskhandle pointer = %p",taskHandle);
    request.delay = esp_timer_get_time() + delay;
    request.iterations = iterations;
    strncpy(request.tskname, taskNameShort, sizeof(request.tskname) - 1);
    request.tskname[sizeof(request.tskname) - 1] = '\0'; // Ensure null-termination

    if (xQueueSend(taskAddQueue, &request, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGE("RTCScheduler", "Failed to enqueue RTOS task '%s'", taskNameShort);
    } else {
        ESP_LOGI("RTCScheduler", "Enqueued RTOS task '%s'", taskNameShort);
    }
}

void processTaskAddQueue() {
    
    if (taskAddQueue == NULL) {
        ESP_LOGE("RTCScheduler", "PROCESS Task add queue is not initialized!");
        return;
    }

    TaskAddRequest request;
    while (xQueueReceive(taskAddQueue, &request, 0) == pdTRUE) {
        DeferredTask *newTask = (DeferredTask *)malloc(sizeof(DeferredTask));
        if (!newTask) {
            ESP_LOGE("RTCScheduler", "Failed to allocate memory for new task '%s'!", request.tskname);
            continue;
        }

        // Set common fields
        newTask->taskType = request.taskType;
        newTask->nextTrigger = request.delay;
        newTask->iterations = request.iterations;
        newTask->executionCount = 0;
        newTask->nextTask_ptr = NULL;
        newTask->prevTask_ptr = NULL;
        strncpy(newTask->tskname, request.tskname, sizeof(newTask->tskname) - 1);

        // Assign the correct union member
        if (request.taskType == CALLBACK_TASK) {
            newTask->taskFunc.f_cb_ptr = request.taskFunc.f_cb_ptr;
        } else if (request.taskType == RTOS_TASK) {
            newTask->taskFunc.rtosTaskHandle = request.taskFunc.rtosTaskHandle;
        }

        // Add the new task to the task list
        if (!head) {
            head = newTask;
        } else {
            DeferredTask *current = head;
            while (current->nextTask_ptr) {
                current = current->nextTask_ptr;
            }
            current->nextTask_ptr = newTask;
            newTask->prevTask_ptr = current;
        }

        ESP_LOGI("RTCScheduler", "Added task '%s' to scheduler, (PTR: %p)", request.tskname, request.taskFunc);
    }
}






void executeTask(DeferredTask *task) {
    
    if (task->taskType == CALLBACK_TASK) {
        if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Executing callback task '%s'", task->tskname);
        task->taskFunc.f_cb_ptr();
    } else if (task->taskType == RTOS_TASK) {
        if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Resuming RTOS task '%s' (@%p)", task->tskname, task->taskFunc.rtosTaskHandle);
        vTaskResume(task->taskFunc.rtosTaskHandle);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    task->executionCount++;
}

void updateOrRemoveTask(DeferredTask *task, int64_t currentTime) {
    if (task->executionCount >= task->iterations) {
        if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Task '%s' completed: removing from list.", task->tskname);
        // Remove task from the list
        if (task->prevTask_ptr) {
            task->prevTask_ptr->nextTask_ptr = task->nextTask_ptr;
        }
        if (task->nextTask_ptr) {
            task->nextTask_ptr->prevTask_ptr = task->prevTask_ptr;
        }
        if (task == head) {
            head = task->nextTask_ptr;
        }
        free(task);
    } else {
        // Reschedule the task
        task->nextTrigger += (task->nextTrigger - currentTime);
        if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Rescheduling task '%s' to nextTrigger=%lld", task->tskname, task->nextTrigger);
    }
}

bool noRunningRTOSTasks() {
    DeferredTask *current = head;

    // Traverse the task list
    while (current) {
        if (current->taskType == RTOS_TASK) {
            // Check the state of the FreeRTOS task
            eTaskState state = eTaskGetState(current->taskFunc.rtosTaskHandle);
            if (state != eSuspended) {
                // If any task is not suspended, return false
                return false;
            }
        }
        current = current->nextTask_ptr; // Move to the next task
    }

    // If all RTOS tasks are suspended or no RTOS tasks exist, return true
    return true;
}

void schedulerTask(void *param) {
    // Wait for the scheduler to start
    if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Scheduler is not running, waiting...");
    while (!isRunning) {
        vTaskDelay(pdMS_TO_TICKS(33)); // Short delay (33ms)
    }

    if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Scheduler is now running.");
    uint32_t sleepOrWait = 0;

    while (isRunning) {
        // Process task add requests
        processTaskAddQueue();

        DeferredTask *nextTask = NULL;
        int64_t nextTriggerTime = INT64_MAX;

        // Traverse the task list
        int64_t currentTime = esp_timer_get_time();
        if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Current time: %lld us", currentTime);

        DeferredTask *current = head;
        nextTask = NULL;
        nextTriggerTime = INT64_MAX;

        if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Traversing task list...");
        while (current) {
            if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Task: Name = %s, nextTrigger=%lld, executionCount=%u/%u",
                                       current->tskname, current->nextTrigger, current->executionCount, current->iterations);

            if (current->executionCount < current->iterations) {
                if (current->nextTrigger < nextTriggerTime) {
                    nextTriggerTime = current->nextTrigger;
                    nextTask = current;
                }
            }
            current = current->nextTask_ptr; // Move to the next task
        }

        if (nextTask) {
            int64_t remainingTime = nextTriggerTime - esp_timer_get_time();
            if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Next task to execute: Name = %s, nextTrigger=%lld, remainingTime=%lld us",
                                       nextTask->tskname, nextTask->nextTrigger, remainingTime);

            if (remainingTime <= 80000 && remainingTime > 0) {
                // Busy-wait until task execution time
                if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Tight looping for task due in %lld us", remainingTime);
                while(esp_timer_get_time()< nextTask->nextTrigger){

                }

                executeTask(nextTask);
                updateOrRemoveTask(nextTask, esp_timer_get_time());
            } else if (remainingTime <= 0) {
                // Log warning and trigger overdue task immediately
                if (1) ESP_LOGW("schedulerTask", "Task '%s' is overdue by %lld us, executing immediately.",
                                           nextTask->tskname, -remainingTime);
                executeTask(nextTask);
                updateOrRemoveTask(nextTask, esp_timer_get_time());
            } else {
                // Hold sleep for remainingTime - 80ms if the queue is empty
                int64_t sleepTime = remainingTime - 80000;
                if (sleepTime > 0 && uxQueueMessagesWaiting(taskAddQueue) == 0) {
                    if (xSemaphoreTake(mtx_radioInUseLockout, pdMS_TO_TICKS(5)) == pdTRUE) {
                        if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Holding sleep for %lld us", sleepTime);
                        if (noRunningRTOSTasks()) holdSleep(sleepTime);
                        xSemaphoreGive(mtx_radioInUseLockout);
                    } else {
                        if (DBG_LOGGING) ESP_LOGW("schedulerTask", "Radio lockout active, skipping hold sleep.");
                    }
                }
            }
        } else {
            // Check if the task add queue is empty before idle logic
            if (uxQueueMessagesWaiting(taskAddQueue) == 0) { 
                vTaskDelay(pdMS_TO_TICKS(5));
                if (DBG_LOGGING) ESP_LOGI("schedulerTask", "No tasks to execute, SLEEPING for 25 ms.");
                if (xSemaphoreTake(mtx_radioInUseLockout, pdMS_TO_TICKS(0)) == pdTRUE) {
                    if (noRunningRTOSTasks()) holdSleep(25 * 1000);
                    xSemaphoreGive(mtx_radioInUseLockout);
                } else {
                    if (DBG_LOGGING) ESP_LOGW("schedulerTask", "Radio lockout active, skipping hold sleep during idle.");
                }
            } else {
                if (DBG_LOGGING) ESP_LOGI("schedulerTask", "Skipping idle logic, messages pending in task add queue.");
            }
        }
    }

    ESP_LOGI("schedulerTask", "Stopping...");
    vTaskDelete(NULL);
}


void createRTCSchedulerTask() {

    // Create the queue with TASK_QUEUE_SIZE items, each of size TaskAddRequest
    taskAddQueue = xQueueCreate(TASK_QUEUE_SIZE, sizeof(TaskAddRequest));
    if (taskAddQueue == NULL) {
        ESP_LOGE("RTCScheduler", "Failed to create task add queue!");
        // Handle error: you may want to halt or retry here
        while (1);
    } else {
        ESP_LOGI("RTCScheduler", "Task add queue created successfully");
    }

    ESP_LOGI("RTC_SCHEDULER", "Creating Scheduler Task...");
    xTaskCreate(schedulerTask, "SchedulerTask", 8192 *2, NULL, 5, &taskHandle_RTCScheduler);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void runRTCScheduler(){
    isRunning = true;
}
void stopRTCScheduler(){
    isRunning = false;
}
