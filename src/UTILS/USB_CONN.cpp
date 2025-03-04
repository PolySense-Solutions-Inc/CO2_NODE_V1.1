/** @file USB_CONN.cpp
 * 
 * @brief Utility functions to verify USB CDC connectivity of the ESP32-C3
 *          ***Intended as a development utility, not for production use!***
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed by David Stewart.  All rights reserved.
 */ 

#include "USB_CONN.hpp"
#include "esp_log.h" // Include the ESP log library

static bool usbConnectedStatus = false;
static SemaphoreHandle_t usbStatusSemaphore = NULL;
static bool previousUSBStatus = false;
static const char* TAG = "USB CDC"; // Define a tag for logging

static volatile bool usbTaskRunning = false;

// Function to check USB connection status
bool CheckUSBCDCConnectedViaJTAGFRAME(void) {
    uint32_t first = *(uint32_t*)USB_SERIAL_JTAG_FRAM_NUM_REG;
    vTaskDelay(pdMS_TO_TICKS(2));
    //delayMicroseconds(5000);
    bool isDifferent = (*(uint32_t*)USB_SERIAL_JTAG_FRAM_NUM_REG != first);
    return isDifferent;
}

// Task to periodically check USB connection status
void USB_Conn_Task(void *pvParameters) {
    usbTaskRunning = true;
    while (true) {
        bool currentStatus = CheckUSBCDCConnectedViaJTAGFRAME();
        if (xSemaphoreTake(usbStatusSemaphore, portMAX_DELAY)) {
            usbConnectedStatus = currentStatus;
            if (currentStatus != previousUSBStatus) {
                if (currentStatus) {
                    Serial.setTxTimeoutMs(100);
                    ESP_LOGI(TAG, "Connected");
                    
                } else {
                    Serial.setTxTimeoutMs(0);
                    ESP_LOGI(TAG, "Disconnected");
                }
                previousUSBStatus = currentStatus;
            }
            xSemaphoreGive(usbStatusSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Check every 500 ms
    }
}

// Initializer to start the USB CDC Monitoring task
bool USB_Conn_Task_Init(void) {
    
    bool success = true;

    if (usbStatusSemaphore == NULL) {
        usbStatusSemaphore = xSemaphoreCreateMutex();
        if (usbStatusSemaphore == NULL) {
            ESP_LOGE(TAG, "Failed to create semaphore");
            success = false;
        }
    }

    if (success) {
        if (xTaskCreate(USB_Conn_Task, "USB_Conn_Task", 2048, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create task");
            success = false;
        }
    }

    while (!usbTaskRunning){
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return success;
}

// Getter function to get the current USB connection status
bool USB_CDC_Connected(void) {
    //ESP_LOGI("USB CON QUERY", "Connected? %s", usbConnectedStatus? "YES":"NO");
    bool status = false;
    if (usbStatusSemaphore == NULL){
        return false;
    }

    if (xSemaphoreTake(usbStatusSemaphore, pdMS_TO_TICKS(10))) {
        status = usbConnectedStatus;
        xSemaphoreGive(usbStatusSemaphore);
    }else{
        status = false;
    }
    return status;
}

/*** end of file ***/