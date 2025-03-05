/** @file main.cpp
 * 
 * @brief An integrated CO2, Temperature, Humidity wireless sensor node platform based on the ESP32-C3.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for Polysense Solutions Inc., by David Stewart.  All rights reserved.
 */

#include "main.hpp"



/*
EOD Notes: Dec 12

    Need to synchronize SHT40 data captured & K33 Data captured before triggering radio task for transmission.
    Idea: bring back dataReady mutexes.  If LoRa task resumes and both aren't ready, add a resume to the queue for 1 second later.

*/
void LoraTXTask_CB();

void startI2C(){
    //ESP_LOGI("STARTI2C", "MUTEX TAKE");
    xSemaphoreTake(mtx_I2C, portMAX_DELAY);
    digitalWrite(I2C_PULLUPS, HIGH);
    K33_BUS.begin(I2C_SDA, I2C_SCL,I2C_CLOCK_HZ);
}

void stopI2C(){
    K33_BUS.end();
    digitalWrite(I2C_PULLUPS, LOW);
    //ESP_LOGI("STARTI2C", "MUTEX GIVE");
    xSemaphoreGive(mtx_I2C);
}

//Re-usable Exponential Moving Average Filter function
float ewma(float previous, float current, float alpha) {
    return alpha * current + (1 - alpha) * previous;
}


//Development function to help characterize thermal mass of SHT41 PCB & response to heating
void characterizeMediumHeater() {
    const float alpha = 0.3f; // EWMA smoothing factor
    float tempOriginal = 0, humidOriginal = 0;
    float tempMeasured = 0, humidMeasured = 0;
    uint16_t anyError = 0;

    // Initialize I2C and Sensor
    startI2C();
    SHT40_Sensor.begin(K33_BUS, SHT40_I2C_ADDR_44);
    SHT40_Sensor.softReset();

    // Measure and set original values
    anyError = SHT40_Sensor.measureHighPrecision(tempOriginal, humidOriginal);
    if (anyError) {
        ESP_LOGE("Characterize Heater", "Initial measurement failed, error code: %u", anyError);
        stopI2C();
        return;
    }

    tempMeasured = tempOriginal;
    humidMeasured = humidOriginal;

    ESP_LOGI("Characterize Heater", "Original Temperature: %.2f C, Original Humidity: %.2f %%", tempOriginal, humidOriginal);

    // Apply Medium Heater Power Short
    ESP_LOGI("Characterize Heater Step 1", "Activating Medium Heater Power Short...");
    anyError = SHT40_Sensor.activateMediumHeaterPowerShort(tempMeasured, humidMeasured);
    if (anyError) {
        ESP_LOGE("Characterize Heater", "Medium heater activation failed, error code: %u", anyError);
        stopI2C();
        return;
    }

    // Time to return to original temperature
    uint64_t startTime = esp_timer_get_time();
    int iteration = 0;
    while (true) {
        anyError = SHT40_Sensor.measureHighPrecision(tempMeasured, humidMeasured);
        if (anyError) {
            ESP_LOGE("Characterize Heater", "Measurement error, code: %u", anyError);
            break;
        }

        tempMeasured = ewma(tempMeasured, tempOriginal, alpha);
        if (iteration % 5 == 0) {
            ESP_LOGI("Characterize Heater", "Cooling down: Current Temperature = %.2f C", tempMeasured);
        }

        if (fabs(tempMeasured - tempOriginal) < 0.1f) { // Close enough to original
            break;
        }
        iteration++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    float timeToCoolTemp = (esp_timer_get_time() - startTime) / 1e6f; // Convert to seconds
    ESP_LOGI("Characterize Heater", "Time to return to original temperature: %.2f seconds", timeToCoolTemp);

    // Time to return to original humidity
    ESP_LOGI("Characterize Heater Step 2", "Activating Medium Heater Power Short...");
    anyError = SHT40_Sensor.activateMediumHeaterPowerShort(tempMeasured, humidMeasured);
    startTime = esp_timer_get_time();
    iteration = 0;
    while (true) {
        anyError = SHT40_Sensor.measureHighPrecision(tempMeasured, humidMeasured);
        if (anyError) {
            ESP_LOGE("Characterize Heater", "Measurement error, code: %u", anyError);
            break;
        }

        humidMeasured = ewma(humidMeasured, humidOriginal, alpha);
        if (iteration % 5 == 0) {
            ESP_LOGI("Characterize Heater", "Cooling down: Current Humidity = %.2f %%", humidMeasured);
        }

        if (fabs(humidMeasured - humidOriginal) < 0.1f) { // Close enough to original
            break;
        }
        iteration++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    float timeToCoolHumid = (esp_timer_get_time() - startTime) / 1e6f; // Convert to seconds
    ESP_LOGI("Characterize Heater", "Time to return to original humidity: %.2f seconds", timeToCoolHumid);

    stopI2C();
}
//Development function to help characterize thermal mass of SHT41 PCB & response to heating
void characterizeHighHeater() {
    const float alpha = 0.3f; // EWMA smoothing factor
    float tempOriginal = 0, humidOriginal = 0;
    float tempMeasured = 0, humidMeasured = 0;
    uint16_t anyError = 0;

    // Initialize I2C and Sensor
    startI2C();
    SHT40_Sensor.begin(K33_BUS, SHT40_I2C_ADDR_44);
    SHT40_Sensor.softReset();

    // Measure and set original values
    anyError = SHT40_Sensor.measureHighPrecision(tempOriginal, humidOriginal);
    if (anyError) {
        ESP_LOGE("Characterize Heater", "Initial measurement failed, error code: %u", anyError);
        stopI2C();
        return;
    }

    tempMeasured = tempOriginal;
    humidMeasured = humidOriginal;

    ESP_LOGI("Characterize Heater", "Original Temperature: %.2f C, Original Humidity: %.2f %%", tempOriginal, humidOriginal);

    // Apply Highest Heater Power Long
    ESP_LOGI("Characterize Heater Step 1", "Activating Highest Heater Power Long...");
    anyError = SHT40_Sensor.activateHighestHeaterPowerLong(tempMeasured, humidMeasured);
    if (anyError) {
        ESP_LOGE("Characterize Heater", "High heater activation failed, error code: %u", anyError);
        stopI2C();
        return;
    }

    // Time to return to original temperature
    uint64_t startTime = esp_timer_get_time();
    int iteration = 0;
    while (true) {
        anyError = SHT40_Sensor.measureHighPrecision(tempMeasured, humidMeasured);
        if (anyError) {
            ESP_LOGE("Characterize Heater", "Measurement error, code: %u", anyError);
            break;
        }

        tempMeasured = ewma(tempMeasured, tempOriginal, alpha);
        if (iteration % 5 == 0) {
            ESP_LOGI("Characterize Heater", "Cooling down: Current Temperature = %.2f C", tempMeasured);
        }

        if (fabs(tempMeasured - tempOriginal) < 0.1f) { // Close enough to original
            break;
        }
        iteration++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    float timeToCoolTemp = (esp_timer_get_time() - startTime) / 1e6f; // Convert to seconds
    ESP_LOGI("Characterize Heater", "Time to return to original temperature: %.2f seconds", timeToCoolTemp);

    // Time to return to original humidity
    ESP_LOGI("Characterize Heater Step 2", "Activating Highest Heater Power Long...");
    anyError = SHT40_Sensor.activateHighestHeaterPowerLong(tempMeasured, humidMeasured);
    startTime = esp_timer_get_time();
    iteration = 0;
    while (true) {
        anyError = SHT40_Sensor.measureHighPrecision(tempMeasured, humidMeasured);
        if (anyError) {
            ESP_LOGE("Characterize Heater", "Measurement error, code: %u", anyError);
            break;
        }

        humidMeasured = ewma(humidMeasured, humidOriginal, alpha);
        if (iteration % 5 == 0) {
            ESP_LOGI("Characterize Heater", "Cooling down: Current Humidity = %.2f %%", humidMeasured);
        }

        if (fabs(humidMeasured - humidOriginal) < 0.1f) { // Close enough to original
            break;
        }
        iteration++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    float timeToCoolHumid = (esp_timer_get_time() - startTime) / 1e6f; // Convert to seconds
    ESP_LOGI("Characterize Heater", "Time to return to original humidity: %.2f seconds", timeToCoolHumid);

    stopI2C();
}


void SampleK33_CB(){
    
    static uint32_t currentStep = 0;
    int sampleCount = 0;

    // Variables for sensor reading accumulation or additional processing can be added here.
    uint32_t CO2_Raw_Accumulator = 0;
    K33SensorError_t readError;
    uint32_t co2SampleAttempts = 0;
    
    uint16_t CO2_Raw_Val = 0;
    readError = K33_TIMEOUT_ERROR; // Dummy error type to start the while loop
    int retryCount = 0;
    ESP_LOGW("CO2 Task", "LAST CO2 Reading was: %.2f ppm", averageCO2);
    if (currentStep == 0){
        xSemaphoreTake(mtx_dataReady_K33, portMAX_DELAY);
        startI2C();
        CO2_Sensor.disableABC_Temporary();
        stopI2C();
        ESP_LOGI("CO2 Task", "Disabling K33 ABC (Temporary)");
        
        currentStep = 1;
    }
    else if (currentStep == 1){     
        ESP_LOGI("CO2 Task", "Sampling CO2 Value now.");
        cereal.flush();

        startI2C();
        readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
        stopI2C();
        while((readError != K33_NO_ERROR && readError != K33_VALUE_RANGE_WARNING) && retryCount < 5) {
            startI2C();
            readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
            stopI2C();

            if (readError == K33_NO_ERROR) {
                ESP_LOGI("CO2 Task", "K33 CO2 Read @ %.2f ppm [No Error]", (float)(CO2_Raw_Val * 10));
            }
            else if (readError == K33_VALUE_RANGE_WARNING) {
                ESP_LOGI("CO2 Task", "K33 CO2 Read @ %.2f ppm [Value Range Warning]", (float)(CO2_Raw_Val * 10));
            }
            else { 
                ESP_LOGI("CO2 Task", "K33 CO2 Read @ %.2f ppm [Error encountered]", (float)(CO2_Raw_Val * 10));
                retryCount++;
                updateRTC_K33ErrorCounts(readError);
                vTaskDelay(pdTICKS_TO_MS(10));
            }

            // Exit loop if we obtain a valid reading
            if (readError == K33_NO_ERROR || readError == K33_VALUE_RANGE_WARNING) {
                break;
            }
        }

        if (retryCount >= 5 && readError != K33_NO_ERROR) {
            ESP_LOGE("CO2 Task", "K33 retry count exceeded; failed to sample CO2. [Guard value of 32768.0 sent!]");
            averageCO2 = 32768.0; // Guard value indicating failure
        }
        else {
            float currentReading = (float)(CO2_Raw_Val * 10);
            ESP_LOGI("CO2 Task", "Initial CO2 reading: %.2f ppm", currentReading);

            // Outlier check: only if we have a previous valid reading.
            if (averageCO2 != 0.0) {
                // Example threshold: if new reading is less than half or greater than 1.3 times the previous value.
                if (currentReading < (averageCO2 / 2.0) || currentReading > (averageCO2 * 1.3)) {
                    ESP_LOGW("CO2 Task", "Outlier detected: current reading %.2f ppm vs previous average %.2f ppm", currentReading, averageCO2);
                    ESP_LOGI("CO2 Task", "Attempting recheck...");
                    uint16_t recheckCO2 = 0;
                    startI2C();
                    K33SensorError_t recheckError = CO2_Sensor.readCO2Data(recheckCO2);
                    stopI2C();
                    if (recheckError == K33_NO_ERROR || recheckError == K33_VALUE_RANGE_WARNING) {
                        float recheckReading = (float)(recheckCO2 * 10);
                        ESP_LOGI("CO2 Task", "Recheck reading: %.2f ppm", recheckReading);
                        // Choose the reading closer to the previous average.
                        if (fabs(recheckReading - averageCO2) < fabs(currentReading - averageCO2)) {
                            currentReading = recheckReading;
                            ESP_LOGI("CO2 Task", "Recheck reading accepted: %.2f ppm", currentReading);
                        } else {
                            ESP_LOGI("CO2 Task", "Retaining original reading: %.2f ppm", currentReading);
                        }
                    }
                    else {
                        ESP_LOGW("CO2 Task", "Recheck failed (error %u). Retaining original reading: %.2f ppm", recheckError, currentReading);
                    }
                }
                else {
                    ESP_LOGI("CO2 Task", "No outlier detected. Previous average: %.2f ppm, current reading: %.2f ppm", averageCO2, currentReading);
                }
            }
            else {
                ESP_LOGI("CO2 Task", "No previous reading available for outlier check.");
            }
            ESP_LOGI("CO2 Task", "Final CO2 Report: %.2f ppm", currentReading);
            averageCO2 = currentReading;
        }
        CO2_Sensor.powerOff();
        xSemaphoreGive(mtx_dataReady_K33);
        // Additional scheduling/RTOS logic can be inserted here if needed.
    }
}




// void SampleK33_CB(){
    
//     static uint32_t currentStep = 0;
//     int sampleCount = 0;

//     //startCO2Sensor();
//     uint32_t CO2_Raw_Accumulator = 0;
//     K33SensorError_t readError;
//     uint32_t co2SampleAttempts = 0;
    
//     uint16_t CO2_Raw_Val = 0;
//     readError = K33_TIMEOUT_ERROR; //Dummy error type to start the while loop
//     int retryCount = 0;

//     if (currentStep ==0){
//         xSemaphoreTake(mtx_dataReady_K33, portMAX_DELAY);
//         startI2C();
        
//         CO2_Sensor.disableABC_Temporary();
//         stopI2C();
//         ESP_LOGI("CO2 Task", "Disabling K33 ABC (Temporary)");
        
//         currentStep = 1;
//     }else if (currentStep==1){     
//         ESP_LOGI("CO2 Task", "Sampling CO2 Value now.");
//         cereal.flush();

//         startI2C();
//         readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
//         stopI2C();
//         while((readError != K33_NO_ERROR && readError != K33_VALUE_RANGE_WARNING) && retryCount < 5) {
//             startI2C();
//             readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
//             stopI2C();

//             if (readError == K33_NO_ERROR) {
//                 ESP_LOGI("CO2 Task", "K33 CO2 Read @ %d ppm [ No Error ]", CO2_Raw_Val * 10);
//             }else if (readError == K33_VALUE_RANGE_WARNING) {
//                 ESP_LOGI("CO2 Task", "K33 CO2 Read @ %d ppm [ Value Range Warning ]", CO2_Raw_Val * 10);
//             }else { 
//                 ESP_LOGI("CO2 Task", "K33 CO2 Read @ %d ppm [An Error!]", CO2_Raw_Val * 10);
//                 retryCount++;
//                 updateRTC_K33ErrorCounts(readError);
//                 vTaskDelay(pdTICKS_TO_MS(10));
//             }

//             // Exit the loop if no error or warning
//             if (readError == K33_NO_ERROR || readError == K33_VALUE_RANGE_WARNING) {
//                 break;
//             }
//         }

//         if (retryCount >= 5 && readError != K33_NO_ERROR) {
//             ESP_LOGE("CO2 Task", "K33 retry count exceeded, failed to sample CO2. [Guard value of 32768 sent!]");
//             averageCO2 = 32768; // Guard value indicating failure
//         }else {
//             ESP_LOGI("CO2 Task", "CO2 Reports %d ppm", CO2_Raw_Val * 10);
//             averageCO2 = CO2_Raw_Val * 10;
//         }
//         CO2_Sensor.powerOff();
//         xSemaphoreGive(mtx_dataReady_K33);
//         //addDeferredTask(LoraTXTask_CB, pdMS_TO_TICKS(10)*1000, 1);
//         //uint32_t bootCount = 0;
//         //rtcSysStat.getDeepWakes(bootCount);
//         //if (bootCount % 5 == 0)
//         //vTaskResume(taskHandle_Tx);
//         //addRTOSTask(taskHandle_Tx, 15 * 1000, 1, "LoRa");
//     }
// }


// void SampleSHT40_Task(void *param) {

//     static uint32_t sampleCount = 0;
//     static float tempAccumulator = 0;
//     static float humidAccumulator = 0; 
//     static float originalAmbientTemp = 0.0f;
//     static float originalHumidity = 0.0f;
//     static uint32_t currentState = 0;
//     xSemaphoreTake(mtx_dataReady_SHT40, portMAX_DELAY);
//     //vTaskSuspend(NULL);
//     while (1) {
//         const uint64_t SHT40_MAXHEAT_COOLDOWN = 20 * 1000 * 1000; //20 Seconds
//         const uint64_t SHT40_LOWHEAT_COOLDOWN = 7 * 1000 * 1000; //7 Seconds
//         float tempVal;
//         float humidVal;
//         uint16_t anyError = 0;

//         startI2C();
//         SHT40_Sensor.begin(K33_BUS, SHT40_I2C_ADDR_44);    
//         anyError = SHT40_Sensor.softReset();

//         if (currentState == 0) {
//             uint32_t bootCount = 0;
//             anyError = SHT40_Sensor.measureHighPrecision(originalAmbientTemp, originalHumidity);
//             rtcSysStat.getDeepWakes(bootCount);

//             if (bootCount % 10 == 0) {
//                 ESP_LOGI("SHT40 Task", "Intensive Humidity De-Creep Heating Started  (Ambient temperature is: %.2f C)", originalAmbientTemp);            
//                 anyError = SHT40_Sensor.activateHighestHeaterPowerLong(tempVal, humidVal);
//                 //addRTOSTask(taskHandle_SHT40, SHT40_MAXHEAT_COOLDOWN, 1, "SHT40");
//                 //SHT40_HEAT_COOL_DONE = esp_timer_get_time() + SHT40_MAXHEAT_COOLDOWN;
//             } else {
//                 ESP_LOGI("SHT40 Task", "Prophylactic De-Creep Humidity Heating, (Ambient temperature is: %.2f C)", originalAmbientTemp);            
//                 anyError = SHT40_Sensor.activateMediumHeaterPowerShort(tempVal, humidVal);
//                 //addRTOSTask(taskHandle_SHT40, SHT40_LOWHEAT_COOLDOWN, 1, "SHT40");
//                 //SHT40_HEAT_COOL_DONE = esp_timer_get_time() + SHT40_LOWHEAT_COOLDOWN;
//             }
//             stopI2C();
//             averageTemperature = originalAmbientTemp;
//             averageHumidity = originalHumidity;
//             ESP_LOGI("SHT40 Task", "Sampling completed. Temp=%.2f, Humidity=%.2f", originalAmbientTemp, originalHumidity);

//             // Indicate data is ready (if needed)
//             xSemaphoreGive(mtx_dataReady_SHT40);
//             currentState = 1;

//         }
//         // Suspend the task until explicitly resumed
//         //ESP_LOGI("SHT40 Task", "currentState = %d", currentState);
//         vTaskSuspend(NULL);
//     }
// }
//
//--added resample of temp/humid in event we get a zero. also added sudden drop detection to run heater
void SampleSHT40_Task(void *param) {

    static uint32_t sampleCount = 0;
    static float tempAccumulator = 0;
    static float humidAccumulator = 0; 
    static float originalAmbientTemp = 0.0f;
    static float originalHumidity = 0.0f;
    static uint32_t currentState = 0;
    xSemaphoreTake(mtx_dataReady_SHT40, portMAX_DELAY);
    //vTaskSuspend(NULL);
    while (1) {
        const uint64_t SHT40_MAXHEAT_COOLDOWN = 20 * 1000 * 1000; // 20 Seconds
        const uint64_t SHT40_LOWHEAT_COOLDOWN = 7 * 1000 * 1000;   // 7 Seconds
        float tempVal;
        float humidVal;
        uint16_t anyError = 0;

        startI2C();
        SHT40_Sensor.begin(K33_BUS, SHT40_I2C_ADDR_44);    
        anyError = SHT40_Sensor.softReset();

        if (currentState == 0) {
            uint32_t bootCount = 0;
            anyError = SHT40_Sensor.measureHighPrecision(originalAmbientTemp, originalHumidity);
            rtcSysStat.getDeepWakes(bootCount);

            if (bootCount % 10 == 0) {
                ESP_LOGI("SHT40 Task", "Intensive Humidity De-Creep Heating Started (Ambient temperature is: %.2f C)", originalAmbientTemp);            
                anyError = SHT40_Sensor.activateHighestHeaterPowerLong(tempVal, humidVal);
            } else {
                ESP_LOGI("SHT40 Task", "Prophylactic De-Creep Humidity Heating (Ambient temperature is: %.2f C)", originalAmbientTemp);            
                anyError = SHT40_Sensor.activateMediumHeaterPowerShort(tempVal, humidVal);
            }
            stopI2C();

            // --- Added Logging and Extra Heater Activation Section ---
            // Save the previous global humidity value for comparison.
            float previousHumidity = averageHumidity;
            // Update global averages with the new measurements.
            averageTemperature = originalAmbientTemp;
            averageHumidity = originalHumidity;
            ESP_LOGI("SHT40 Task", "Sampling completed. Temp = %.2f, Humidity = %.2f", originalAmbientTemp, originalHumidity);

            // Check for a sudden drop in humidity.
            if (previousHumidity != 0 && (previousHumidity - originalHumidity) >= 30) {
                ESP_LOGW("SHT40 Task", "Sudden humidity drop detected: previous = %.2f, current = %.2f. Running highest heater twice.", previousHumidity, originalHumidity);
                startI2C();
                anyError = SHT40_Sensor.activateHighestHeaterPowerLong(tempVal, humidVal);
                vTaskDelay(pdMS_TO_TICKS(100));  // Brief delay between activations
                anyError = SHT40_Sensor.activateHighestHeaterPowerLong(tempVal, humidVal);
                stopI2C();
            } else {
                ESP_LOGI("SHT40 Task", "No sudden humidity drop detected (previous = %.2f, current = %.2f). No additional heater activation required.", previousHumidity, originalHumidity);
            }
            // --- End of Added Section ---

            // Indicate data is ready
            xSemaphoreGive(mtx_dataReady_SHT40);
            currentState = 1;
        }
        // Suspend the task until explicitly resumed
        vTaskSuspend(NULL);
    }
}



//FreeRTOS timer that reboots the system after 10 minutes when USB still connected.
TimerHandle_t fakeSleep_THandle;
void rebootTimer_CB(TimerHandle_t xTimer){
    esp_restart();
}

//Trigger a reboot in SLEEP_TIME_NORMAL - total time awake this interval
void setupFakePMU_Reboot() {
    
    //To avoid sample time drift, offset the sleep time by the duration of this activation cycle
    int64_t totalActiveTime = ((esp_timer_get_time() - LAST_BOOT_RTC_TIME) / 1000);
    if (totalActiveTime>0 && (SLEEP_TIME_NORMAL * 60 * 1000)>totalActiveTime){
        ESP_LOGI("FAKEPMU Reboot","xTimerCreate reboot timer for %ld", pdMS_TO_TICKS(SLEEP_TIME_NORMAL * 60 * 1000 - (uint32_t)totalActiveTime));
        fakeSleep_THandle = xTimerCreate("RebootTimer", pdMS_TO_TICKS(SLEEP_TIME_NORMAL * 60 * 1000 - (uint32_t)totalActiveTime), pdFALSE, NULL, rebootTimer_CB);
    }else{
        ESP_LOGI("FAKEPMU Reboot","xTimerCreate reboot timer for %ld", pdMS_TO_TICKS(SLEEP_TIME_NORMAL * 60 * 1000));
        fakeSleep_THandle = xTimerCreate("RebootTimer", pdMS_TO_TICKS(SLEEP_TIME_NORMAL * 60 * 1000), pdFALSE, NULL, rebootTimer_CB);
    }

    // Check if the timer was created successfully
    if (fakeSleep_THandle != NULL) {
        // Start the timer
        ESP_LOGE("FAKEPMU", "Reboot timer set for 10 minutes!");
        xTimerStart(fakeSleep_THandle, 0);
    } else {
        // Handle error if timer creation failed
        ESP_LOGE("FAKEPMU", "Failed to create reboot timer for 10 minutes!");
    }
}


// void sleepDeep10(){
    
//     //To avoid sample time drift, offset the sleep time by the duration of this activation cycle
//     int64_t totalActiveTime = esp_timer_get_time() - LAST_BOOT_RTC_TIME;
    
//     updateLastBootInfo();
//     //if (!USB_CDC_Connected()){        
//         ESP_LOGI("FAKEPMU", "Deep sleep ~10M");
//         if (totalActiveTime>=0 && (SLEEP_TIME_NORMAL * 60 * 1000 * 1000)>totalActiveTime){
//             esp_sleep_enable_timer_wakeup(SLEEP_TIME_NORMAL * 60 *1000 * 1000 - totalActiveTime);
//         }else{
//             esp_sleep_enable_timer_wakeup(SLEEP_TIME_NORMAL * 60 *1000 * 1000);
//         }
//         esp_deep_sleep_start();
//     // }else{
//     //     setupFakePMU_Reboot();
//     // }
// }

Preferences nvPrefs;

// Function to save variables to NVS before sleep
void saveVariablesToNVS() {
    // Open preferences in read/write mode
    nvPrefs.begin("co2sensor", false);
    
    // Save our sensor values
    int co2result = nvPrefs.putFloat("co2", averageCO2);
    int tempResults = nvPrefs.putFloat("temp", averageTemperature);
    int humidResults = nvPrefs.putFloat("humid", averageHumidity);
    
    ESP_LOGI("NVS_SAVE RESULTS", "results CO2=%d, Temp=%d, Humid=%d", 
        co2result, tempResults, humidResults);
    // Save USB connection state
    nvPrefs.putBool("usb", USB_CDC_Connected());
    
    // Close the preferences
    nvPrefs.end();
    
    ESP_LOGI("NVS_SAVE", "Saved to NVS: CO2=%.2f, Temp=%.2f, Humid=%.2f", 
             averageCO2, averageTemperature, averageHumidity);
}

// Function to load variables from NVS after wake
void loadVariablesFromNVS() {
    ESP_LOGI("NVS_DEBUG", "Before loading: CO2=%.2f, Temp=%.2f, Humid=%.2f", 
             averageCO2, averageTemperature, averageHumidity);
    
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI("NVS_DEBUG", "Wake cause: %d", wakeup_cause);
    
    // Only load values if waking from deep sleep
    if (wakeup_cause == ESP_SLEEP_WAKEUP_TIMER) {
        // Open preferences in read-only mode
        if (!nvPrefs.begin("co2sensor", true)) {
            ESP_LOGE("NVS_DEBUG", "Failed to open NVS namespace");
            return;
        }
        
        // Check if the keys exist
        bool co2Exists = nvPrefs.isKey("co2");
        bool tempExists = nvPrefs.isKey("temp");
        bool humidExists = nvPrefs.isKey("humid");
        
        ESP_LOGI("NVS_DEBUG", "NVS keys exist? CO2: %d, Temp: %d, Humid: %d", 
                co2Exists, tempExists, humidExists);
        
        // Load sensor values with defaults if not found
        float oldCO2 = averageCO2;
        float oldTemp = averageTemperature;
        float oldHumid = averageHumidity;
        
        averageCO2 = nvPrefs.getFloat("co2", 0.0f);
        averageTemperature = nvPrefs.getFloat("temp", 0.0f);
        averageHumidity = nvPrefs.getFloat("humid", 0.0f);
        
        // Close the preferences
        nvPrefs.end();
        
        ESP_LOGI("NVS_DEBUG", "Values changed: CO2: %.2f → %.2f, Temp: %.2f → %.2f, Humid: %.2f → %.2f", 
                oldCO2, averageCO2, oldTemp, averageTemperature, oldHumid, averageHumidity);
    } else {
        ESP_LOGI("NVS_DEBUG", "Not waking from deep sleep (cause: %d), using default values", wakeup_cause);
    }
}


void sleepDeep10() {
    // Save variables to NVS
    saveVariablesToNVS();
    
    // To avoid sample time drift, offset the sleep time by the duration of this activation cycle
    int64_t totalActiveTime = esp_timer_get_time() - LAST_BOOT_RTC_TIME;
    
    updateLastBootInfo();
    
    if (USB_CDC_Connected()) {
        ESP_LOGI("SLEEP", "USB connected before sleep - will restore on wake");
    }
    
    ESP_LOGI("SLEEP", "Entering deep sleep ~10M");
    
    if (totalActiveTime >= 0 && (SLEEP_TIME_NORMAL * 60 * 1000 * 1000) > totalActiveTime) {
        esp_sleep_enable_timer_wakeup(SLEEP_TIME_NORMAL * 60 * 1000 * 1000 - totalActiveTime);
    } else {
        esp_sleep_enable_timer_wakeup(SLEEP_TIME_NORMAL * 60 * 1000 * 1000);
    }
    
    // Properly flush all output before sleep
    Serial.flush();
    cereal.flush();
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

void reconnectUSB() {
    ESP_LOGI("USB", "Reconnecting USB after deep sleep");
    
    // Re-initialize USB
    Serial.end();
    delay(100);
    Serial.begin(115200);
    
    // Give USB time to reconnect
    delay(300);
    
    if (USB_CDC_Connected()) {
        ESP_LOGI("USB", "USB reconnected successfully");
    } else {
        ESP_LOGW("USB", "USB reconnection attempt failed");
    }
}


// void restoreUSBConnection() {
//     if (resumingFromDeepSleep && USB_WasConnected) {
//         ESP_LOGI("USB", "Restoring USB connection after deep sleep");
        
//         // Re-initialize USB CDC
//         Serial.end();
//         delay(100);
//         Serial.begin(115200);
        
//         // Give USB time to reconnect to host
//         int timeout = 1000; // 1 second timeout
//         int elapsed = 0;
//         int checkInterval = 50; // Check every 50ms
        
//         while (!USB_CDC_Connected() && elapsed < timeout) {
//             delay(checkInterval);
//             elapsed += checkInterval;
//         }
        
//         if (USB_CDC_Connected()) {
//             ESP_LOGI("USB", "USB connection restored successfully");
//         } else {
//             ESP_LOGW("USB", "Failed to restore USB connection within timeout");
//         }
//     }
    
//     // Reset the flag for next sleep cycle
//     resumingFromDeepSleep = false;
// }

TickType_t xSensorStartTime = 0;

void setupdeferredRTCTasks(){
    
    startCO2Sensor(); //Power on the CO2 Sensor
    uint32_t currTime = esp_timer_get_time()/1000;

    createRTCSchedulerTask(); //Create the RTC Scheduler task

    //SHT40 task runs at startup to start the heating pulse ASAP then resumes itself at the right time.
    //Since the heater pulse depends on the wakeup cycle count, the call to sample data is variable.
    //thus the task itself puts itself into the RTC scheduler for the actual sampling time.


    //First K33 call disables ABC
    addCallbackTask(SampleK33_CB, (K33_WARMUP_WAIT + 150) * 1000, 1, "K33");
    

    #if 0
    //Comment: Was part of K33 Sampling repeatability testing
    uint32_t wakeCount = 0;
    rtcSysStat.getDeepWakes(wakeCount);
    ESP_LOGI("RTC WAKE COUNT", "is at %ld", wakeCount);
    addCallbackTask(SampleK33_CB, pdMS_TO_TICKS(K33_WARMUP_WAIT + (AVERAGING_FILTER_LENGTH_K33 - wakeCount/2) * K33_POLLING_INTERVAL -750 )*1000,1,"K33");
    #endif

    K33_SAMPLE_READY_BY =currTime + (K33_WARMUP_WAIT + (AVERAGING_FILTER_LENGTH_K33) * K33_POLLING_INTERVAL -750 +2250);
    
    
    SHT40_SampleReadyBy = K33_SAMPLE_READY_BY; //Testing change:  Sample, heat, data ready --> Let cooldown while sleeping
    ESP_LOGW("RTCSTARTER", "Samples ready for K33: %llu, SHT40: %llu", K33_SAMPLE_READY_BY, SHT40_SampleReadyBy);
    //Second call samples CO2, also schedules TX Task resumption

    addCallbackTask(SampleK33_CB, pdMS_TO_TICKS(K33_SAMPLE_READY_BY)*1000, 1, "K33");

    runRTCScheduler(); //Start the RTC scheduler task
}

void setup() {
    //Record bootup time per RTC, (microsecond resolution)
    //delay(5000);
    LAST_BOOT_RTC_TIME = esp_timer_get_time();
    //esp_disable_brownout_detection();
    //Load variables from NVS
    loadVariablesFromNVS();
    EarlyPinInit();
    bool GPIO8_LOW_AT_BOOT = !digitalRead(GPIO8_TP);
    
    bool KEY0_PRESSED_AT_BOOT = !digitalRead(KEY0_IN);
    
    serialInitPhase1(); 
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI("WAKEUP", "Wakeup cause: %d", wakeup_cause);

    ESP_LOGI("SENSOR_AVERAGES", "Startup: averageCO2 = %.2f ppm, averageTemperature = %.2f C, averageHumidity = %.2f %%", 
        averageCO2, averageTemperature, averageHumidity);

    // Verify RTC System Operation Stat data, re-init if neccessary
    rtcSysStat.VerifySystemStatusCRC();

    //Print the bootup message
    cereal.println(BOOT_MESSAGE);
    
    bool wasFirstBoot = isVeryFirstBoot();

    mtx_I2C = xSemaphoreCreateMutex();
    mtx_radioInUseLockout = xSemaphoreCreateMutex();
    mtx_dataReady_SHT40 = xSemaphoreCreateMutex();
    mtx_dataReady_K33 = xSemaphoreCreateMutex();

    bool CO2_BG_Calibration_Armed = false;
    bool CO2_ZERO_Calibration_Armed = false;

    uint32_t KEY0_time = 0;
    //KEY0_time = MAX_KEY0_Z_CAL-100;
    //KEY0_PRESSED_AT_BOOT=true;
    if (!wasFirstBoot && KEY0_PRESSED_AT_BOOT){
        uint32_t KEY0_start =millis();
        while(!digitalRead(KEY0_IN)){
            if ((millis() - KEY0_start)%1000 > 500){
                digitalWrite(STATUS_LED, LOW);
            }else{
                digitalWrite(STATUS_LED, HIGH);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        KEY0_time = millis()- KEY0_start;
        
        if (KEY0_time>=MIN_KEY0_BG_CAL && KEY0_time<MAX_KEY0_BG_CAL){
            CO2_BG_Calibration_Armed = true;

        }else if (KEY0_time>=MIN_KEY0_Z_CAL && KEY0_time<MAX_KEY0_Z_CAL){
            //CO2_ZERO_Calibration_Armed = true;
        }
        cereal.printf("\r\n KEY0 held for %u mSec\r\n", KEY0_time);
    }
  

    taskInitPhase1(); //Start PMU & USB CDC Tasks
    //requestPMUChange(PMU_SET_CPU_10MHZ);

    //Record the battery voltage
    batt_mVolts_AtBoot = readBatteryVoltageMillivolts();

    //USB Connection to Host -> We probably want to capture serial data, so wait a bit longer to get going. 
    #if EXTENDED_USB_STARTUP_DELAY  
    if (batt_mVolts_AtBoot>= 4500){
        ESP_LOGI("INFO", "USB Connected -> Extended Startup Delay by 2 Seconds\n");
        vTaskDelay(pdMS_TO_TICKS(2000));   
    }
    #endif
    //dumpSystemStatsToSerial();

    //If GPIO8 held low at boot, initiate factory reset.
    if (GPIO8_LOW_AT_BOOT){
        cereal.println("    GPIO8 held low @ boot, Factory Reset Initiated");
        factoryReset();
    }
       
    
    if (wasFirstBoot){
        getManufacturingInfo();

    }else if (!GPIO8_LOW_AT_BOOT  && !CO2_BG_Calibration_Armed && !CO2_ZERO_Calibration_Armed){
        taskInitPhase2();

    }else if (CO2_BG_Calibration_Armed){
        //cereal.println("BG Calibration not tested just yet!");
        cereal.printf("**WARNING**  CO2 Background calibration armed, starts in %u Minutes!\r\n", MANU_BG_CAL_DELAY);
        K33_WaitAndCalibrate(false);

    }else if (CO2_ZERO_Calibration_Armed){
        cereal.println("Zero Calibration not tested just yet!");
        //cereal.printf("**WARNING**  CO2 Zero calibration armed, starts in %u Minutes!\r\n", MANU_ZERO_CAL_DELAY);
        //K33_WaitAndCalibrate(true);
    }
    
    cereal.flush();
}

void dumpSystemStatsToSerial(){
    char statusBuffer[500] = {0};
    rtcSysStat.printStatusToBuffer(statusBuffer, sizeof(statusBuffer));
    //ESP_LOGD("RTC VARS", "\n%s", statusBuffer);
    cereal.println(statusBuffer);
    //cereal.flush();
}

void loop() {
    // Empty since Specific Tasks handle all the work.

    //Since we don't use it, delete it to free up resources.
    vTaskDelete(NULL);
}


void factoryReset(){
    
    nvStorage.begin("CO2_NODE", false);
    nvStorage.clear();
    nvStorage.end();
    
    rtcSysStat.initializeRTCData();
    //pReqShim("Factory Reset", PMU_ALT_3V3_ON);
    ESP_LOGI("Factory Reset", "Re-initializing settings of LoRaWAN module");
    LoRa_Radio.begin();
    LoRa_Radio.initializeModule();
    //If desirable, send a message indicating the unit has been factory reset
    #if SEND_FACT_RESET_LORA_MSG
    Tx_Aux_Packet(MSG_FACTORY_RESET);
    #endif
    
    cereal.println("\r\n    NV Storage reset & RTC Memory Cleared, restart in 10 S.");
    vTaskDelay(pdMS_TO_TICKS(10000));
    esp_restart();
}

void EarlyPinInit(void){
    earlyPowerPinInit();

    pinMode(VBAT_ADC_IN, INPUT);
    pinMode(KEY0_IN, INPUT_PULLUP);

    pinMode(GPIO8_TP, INPUT);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);
    
    setupADC1();
}

uint16_t readBatteryVoltageMillivolts() {
    // Read the ADC value
    //vTaskDelay(pdMS_TO_TICKS(10)); //Make sure PMU Task is running
    #if BOARD_IS_HW_2_PROTO_MONSTER
    requestPMUChange(PMU_BOOST_5V_ON);
    vTaskDelay(pdMS_TO_TICKS(300));
    #endif
    uint32_t totalVoltage = 0;

    // Oversample 32x
    for (int i = 0; i < 32; i++) {
        // Read the ADC value and get the voltage in millivolts
        uint32_t voltage = 0;
        esp_adc_cal_get_voltage(ADC_CHANNEL_3, adc_chars, &voltage);
        totalVoltage += voltage;
        //vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to allow for ADC stabilization
    }
    
    // Calculate the average voltage in millivolts
    uint32_t avgVoltage = totalVoltage >>5;

    // Calculate the battery voltage in millivolts using the voltage divider formula
    float batteryVoltage = avgVoltage * (VBAT_DIV_R1 + VBAT_DIV_R2) / VBAT_DIV_R2;
    
    #if BOARD_IS_HW_2_PROTO_MONSTER
    requestPMUChange(PMU_BOOST_5V_OFF);
    #endif
    // Return the battery voltage as uint16_t
    return (uint16_t)(batteryVoltage + VBAT_OFFSET);
}

void setupADC1() {
    // Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(ADC1_CHANNEL_3, atten);

    // Get ADC Characteristics / Factory calibration information
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, ADC_REF_VOLTAGE, adc_chars);
}

void startCO2Sensor(){
    CO2_Sensor.setup(K33_PWR); //Initialize state variables & counters
    CO2_Sensor.powerOn();
}

//Update RTC memory with I2C and Value errors from K33 Driver
void updateRTC_K33ErrorCounts(K33SensorError_t errK33){
    if (errK33 !=K33_NO_ERROR || errK33 != K33_VALUE_RANGE_WARNING){
        RTC_SystemStatus currRTCCopy;
        rtcSysStat.readData(currRTCCopy);
        K33DriverStats K33Errors = CO2_Sensor.getDriverErrorStats();
        rtcSysStat.setI2cErr(currRTCCopy.I2C_ERR + K33Errors.K33_I2CERRS);
        rtcSysStat.setK33Val(currRTCCopy.K33_VAL + K33Errors.K33_VALERRS);
    }
}

#if 1
void getRadioLockout(){    
    xSemaphoreTake(mtx_radioInUseLockout, portMAX_DELAY);
}
void releaseRadioLockout(){
    xSemaphoreGive(mtx_radioInUseLockout);
}

void Tx_Task(void *pvParameters) {

    /*Recommendation:  Modify MKL62BA driver to return more information if an attempt to send returns device busy.
        This would permit better sleep behaviour while waiting for a transmit to complete.
    
    */

    uint32_t currentState = 0;
    
    Node_Data_pkt dataToSend;
    initToZero(&dataToSend, sizeof(dataToSend));
    dataToSend.PKT_Type = PKT_DAT;
    RTC_SystemStatus rtcData;
    uint32_t connectAttempts = 0;
    uint32_t radioResets = 0;
    bool haveSHT40Data = false;
    bool haveK33Data = false;
    bool connectSuccess = false;
    while(1){

        //Initial wake up state
        if (currentState==0){
            getRadioLockout();
            LoRa_Radio.begin();
            rtcSysStat.readData(rtcData);
            dataToSend.Last_BootReason =  rtcData.Last_BootReason & 0b00001111;
            dataToSend.CRC_MISMATCH_FLAG = rtcData.CRCFailed & 0b00000001;
            dataToSend.Wake_Count = rtcData.RST_DEEPSLEEP &0b0000111111111111;
            dataToSend.Curr_Batt_mVolts = readBatteryVoltageMillivolts();
            dataToSend.Prev_Batt_mVolts = rtcData.Last_BattVolts;
            currentState = 1;
            addRTOSTask(taskHandle_Tx, 4000 * 1000, 1, "LoRa");
            bool connectSuccess = LoRa_Radio.connect();
            releaseRadioLockout();

        //Connection in progress
        }else if (currentState ==1){
            getRadioLockout();

            if (connectAttempts >= 8){
                LoRa_Radio.resetChip();
                radioResets++;
                connectAttempts = 0;
                addRTOSTask(taskHandle_Tx, 3000 * 1000, 1,"LoRa RST-->Con");
            }

            bool connectSuccess = LoRa_Radio.connect();
            connectAttempts++;

            if (connectSuccess) {
                ESP_LOGI("TX_TASK", "Radio connection succeeded.");
                currentState = 2;
                uint64_t currTime = esp_timer_get_time();

                // Convert SHT40_SampleReadyBy and K33_SAMPLE_READY_BY to microseconds
                uint64_t SHT40_SampleReadyBy_us = (SHT40_SampleReadyBy + 500) * 1000;
                uint64_t K33_SAMPLE_READY_BY_us = (K33_SAMPLE_READY_BY +750)* 1000;
                ESP_LOGW("TX_TASK", "CTIME: %llu, K33: %llu, SHT40: %llu", currTime, K33_SAMPLE_READY_BY_us, SHT40_SampleReadyBy_us );
                

                // Determine the next task execution time based on the differences
                uint64_t nextExecutionTime;
                if (currTime < SHT40_SampleReadyBy_us && currTime < K33_SAMPLE_READY_BY_us) {
                    // Both sample ready times are in the future, use the LATEST one
                    nextExecutionTime = (SHT40_SampleReadyBy_us < K33_SAMPLE_READY_BY_us) ?
                                        K33_SAMPLE_READY_BY_us - currTime :
                                        SHT40_SampleReadyBy_us - currTime;
                } else if (currTime < SHT40_SampleReadyBy_us) {
                    // Only SHT40 sample ready time is in the future
                    nextExecutionTime = SHT40_SampleReadyBy_us - currTime;
                } else if (currTime < K33_SAMPLE_READY_BY_us) {
                    // Only K33 sample ready time is in the future
                    nextExecutionTime = K33_SAMPLE_READY_BY_us - currTime;
                } else {
                    // Both sample ready times are in the past, use a default delay
                    nextExecutionTime = 250 * 1000; // 250 ms
                }

                // Queue up the next execution of the task
                addRTOSTask(taskHandle_Tx, nextExecutionTime, 1, "LoRa Con-->TX");

            }else{
                connectAttempts++;
                addRTOSTask(taskHandle_Tx, 5000 * 1000, 1,"LoRa Con??");
            }

            releaseRadioLockout();

            if (radioResets >= 4){
                ESP_LOGE("TX_TASK", "Lora module unable to connect, despite resets! Deep sleep for 10 and try again.");
                //TODO handle K33 keep active when USB Connected.
                stopRTCScheduler();
                sleepDeep10();
            }

        //Wait for Sensor tasks to signal data collection is complete
        }else if (currentState == 2){
            if (!haveSHT40Data){
                if (xSemaphoreTake(mtx_dataReady_SHT40, 0)==pdTRUE){
                    haveSHT40Data = true;
                    xSemaphoreGive(mtx_dataReady_SHT40);
                    ESP_LOGW("TX_TASK", "GOT SHT40 Data Mutex");
                }
            }

            if (!haveK33Data){
                if (xSemaphoreTake(mtx_dataReady_K33, 0)==pdTRUE){
                    haveK33Data = true;
                    xSemaphoreGive(mtx_dataReady_K33);
                    ESP_LOGW("TX_TASK", "GOT K33 Data Mutex");
                }
            }
            
            if (haveK33Data && !haveSHT40Data){
                uint64_t currTime = esp_timer_get_time();
                ESP_LOGI("TX_TASK","Ext Timing info: SHT40 Sample Ready By = %lld     currTime = %lld", SHT40_SampleReadyBy, currTime);
                int64_t nextCheckTime = SHT40_SampleReadyBy - currTime;
                if (nextCheckTime>0){
                    addRTOSTask(taskHandle_Tx, nextCheckTime + 2000 *1000, 1,"LoRa 1/2 MTX");
                }else{
                    addRTOSTask(taskHandle_Tx, 500 * 1000, 1,"LoRa 1/2 MTX");
                }
            }else if (!haveK33Data && haveSHT40Data){
                uint64_t currTime = esp_timer_get_time();
                ESP_LOGI("TX_TASK","Ext Timing info: K33_SAMPLE_READY_BY = %lld     currTime = %lld", K33_SAMPLE_READY_BY*1000, currTime);
                int64_t nextCheckTime = K33_SAMPLE_READY_BY*1000 - currTime;
                if (nextCheckTime>0){
                    addRTOSTask(taskHandle_Tx, nextCheckTime + 2000 *1000, 1,"LoRa 1/2 MTX");
                }else{
                    addRTOSTask(taskHandle_Tx, 500 * 1000, 1,"LoRa 1/2 MTX");
                }
            } else if (haveK33Data && haveSHT40Data){
                stopRTCScheduler();
                currentState = 3;
            }
        }else if (currentState ==3){ //Assemble data packet and transmit
            
            //From this point on, we just let the TX Task handle the system state to reduce complexity and pathways for bugs.
            getRadioLockout();
            stopRTCScheduler();
            
            K33DriverStats K33DRVStats = CO2_Sensor.getDriverErrorStats();
            //Bit masks below to truncate results to fit normal data packets
            dataToSend.K33_VAL = K33DRVStats.K33_VALERRS & 0b00001111;
            dataToSend.I2C_ERR = K33DRVStats.K33_I2CERRS & 0b00001111;
            dataToSend.K33_CHK = K33DRVStats.K33_CHKERRS & 0b00000111;

            //Comment, was part of K33 Sample Repeatability testing
            //uint32_t wakeCount = 0;
            //rtcSysStat.getDeepWakes(wakeCount);
            //dataToSend.K33_CHK= AVERAGING_FILTER_LENGTH_K33 - wakeCount/2;

            
            dataToSend.Curr_BootReason = rtcSysStat.getBootReason();

            dataToSend.filteredCO2 = averageCO2;
            dataToSend.filteredTemp = averageTemperature;
            dataToSend.filteredHumidity = averageHumidity;

            txBuffLen = dataToHexString(&dataToSend, sizeof(dataToSend), TxBuffer, sizeof(TxBuffer));

            //Means it didn't fit. By design this should never occur, so it might be memory corruption, in which case we should just go to sleep and try again later.
            if (txBuffLen ==0){
                ESP_LOGE("TX_TASK", "Attempt to transmit with zero sized buffer!  Critical Failure!");
                sleepDeep10();
            }

            uint32_t txAttempts = 1;
            ESP_LOGI("TX_TASK", "Sending Data: CO2 = %f, Temp = %f, Humid = %f",averageCO2, averageTemperature, averageHumidity);

            bool transmitSuccess = LoRa_Radio.sendDataPacket(TxBuffer, txBuffLen);
            while (!transmitSuccess && txAttempts < 2){
                rtcSysStat.incTxFailures();
                bool isConnected = LoRa_Radio.connect();
                transmitSuccess = LoRa_Radio.sendDataPacket(TxBuffer, txBuffLen);                
                txAttempts++;
            }
            releaseRadioLockout();


            if (transmitSuccess){
                ESP_LOGI("TX Task", "Completed Data Packet Transmission in %d attempts.", txAttempts);
            }else{
                ESP_LOGI("TX Task", "Failed Data Packet Transmission in %d attempts.", txAttempts);
            }
            sleepDeep10();
            vTaskSuspend(NULL);
        }
        
        if (currentState<3){
            ESP_LOGI("TX Task", "Current state = %d, suspending", currentState);
            vTaskSuspend(NULL);
        } 
    }
}
#endif

void updateLastBootInfo(){
    rtcSysStat.setLastBattVolts(batt_mVolts_AtBoot);
    rtcSysStat.setLastBootReason(rtcSysStat.getBootReason());
}

#if 0
void Sensor_WD_Task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(mtx_I2C, pdMS_TO_TICKS(10000)) == pdFALSE) { // 10 second timeout
            // Semaphore contention detected
            ESP_LOGE("Sensor WD Task", "I2C Bus contention detected. Restarting Node!");
            
            if (taskHandle_K33CO2 != NULL) {
                vTaskDelete(taskHandle_K33CO2);
            }

            if (taskHandle_SHT40 != NULL) {
                vTaskDelete(taskHandle_SHT40);
            }
            
            if (taskHandle_Tx != NULL){
                vTaskDelete(taskHandle_Tx);
            }
            digitalWrite(EN_5V_BOOST, LOW);
            digitalWrite(EN_3V3_ALT, LOW);
            digitalWrite(I2C_PULLUPS, LOW);

            rtcSysStat.incI2cReinits();
            vTaskDelay(pdMS_TO_TICKS(10));
            esp_restart();

        }else{
            xSemaphoreGive(mtx_I2C);
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
}
#endif



void earlyPowerPinInit(){
    
    pinMode(EN_5V_BOOST, OUTPUT);
    pinMode(EN_3V3_ALT, OUTPUT);
    pinMode(K33_PWR, OUTPUT);
    pinMode(I2C_PULLUPS, OUTPUT);
    
    //Turn on everything ASAP (except K33)
    digitalWrite(EN_5V_BOOST, HIGH);
    digitalWrite(EN_3V3_ALT, HIGH);
    digitalWrite(I2C_PULLUPS, HIGH);
    
}

//Check for presence of "VeryFirstBoot" in nvStorage.
bool isVeryFirstBoot(){
    nvStorage.begin("CO2_NODE", false);
    bool veryFirstBoot = nvStorage.getBool("VeryFirstBoot", true);
    if (veryFirstBoot){
        nvStorage.begin("CO2_NODE", false);
        ESP_LOGD("MANUFACTURING", "Very First Power On!");
        nvStorage.putBool("VeryFirstBoot", false);
    }
    nvStorage.end();

    return veryFirstBoot;
}

void getManufacturingInfo(){
    
    cereal.println("(FIRST BOOT!)Gathering Manufacturing Information, please wait.\r\n");
    cereal.flush();
    //esp_disable_brownout_detection();
    K33VersionInfo K33SensorInfo;
    setCpuFrequencyMhz(10);
    digitalWrite(EN_5V_BOOST, HIGH);
    digitalWrite(EN_3V3_ALT, HIGH);
    digitalWrite(I2C_PULLUPS, HIGH);
    
    K33_BUS.begin(I2C_SDA, I2C_SCL,I2C_CLOCK_HZ); 
    vTaskDelay(pdMS_TO_TICKS(20));

    LoRa_Radio.begin();
    startCO2Sensor();
    uint8_t sensorInfoBuffer[300];
    uint16_t currentCO2[5] = {0};
    
    cereal.print("    [CO2 Sensor]");
    cereal.flush();
    vTaskDelay(pdMS_TO_TICKS(2900));
    CO2_Sensor.getSensorInfo(K33SensorInfo);
    CO2_Sensor.dumpSensorInfoToString(K33SensorInfo, sensorInfoBuffer, sizeof(sensorInfoBuffer));
    
    #if MANU_TEST_K33
    for (int i = 0; i<5; i++){
        CO2_Sensor.readCO2Data(currentCO2[i]);
        vTaskDelay(pdMS_TO_TICKS(K33_POLLING_INTERVAL));
    }
    #endif
    CO2_Sensor.powerOff();


    cereal.print("    [LORAWan Radio]");
    cereal.flush();
    #define MKLINFO_BUFFSIZE 80

    char MKL_devEUI[MKLINFO_BUFFSIZE];
    char MKL_appEUI[MKLINFO_BUFFSIZE];
    char MKL_appKEY[MKLINFO_BUFFSIZE];
    char MKL_region[MKLINFO_BUFFSIZE];
    char MKL_version[MKLINFO_BUFFSIZE];
    

    
    vTaskDelay(pdMS_TO_TICKS(100));
    LoRa_Radio.getModuleInformation(MKL_devEUI, MKL_appEUI, MKL_appKEY, MKL_region, MKL_version, MKLINFO_BUFFSIZE);
    
    #if MANU_TEST_RADIO
    //Placeholder for LoRaWAN manufacturing test
    #endif

    //requestPMUChange(PMU_ALT_3V3_OFF);
    //requestPMUChange(PMU_BOOST_5V_OFF);

    cereal.print("    [Temp/Humid]");
    cereal.flush();
    uint32_t SHT40_SNum =0;
    SHT40_Sensor.begin(K33_BUS, SHT40_I2C_ADDR_44);
    uint16_t anyError = SHT40_Sensor.softReset();
    SHT40_Sensor.serialNumber(SHT40_SNum);
    #if MANU_TEST_SHT40
    float SHT40Temp[5] = {0.0};
    float SHT40Humid[5] = {0.0};
    for (int i = 0; i<5; i++){
        uint16_t anyError = SHT40_Sensor.measureHighPrecision(SHT40Temp[i], SHT40Humid[i]);
        if (anyError){
            ESP_LOGE("SHT40", "Read failed!");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    #endif

    //requestPMUChange(PMU_ALT_3V3_OFF);
    //requestPMUChange(PMU_BOOST_5V_OFF);
    //requestPMUChange(PMU_I2C_PWR_OFF);
    cereal.print("    [ESP32 Sys]    Done!\r\n");
    cereal.flush();
    uint32_t currentBatteryMilliVolts = readBatteryVoltageMillivolts();
    vTaskDelay(pdMS_TO_TICKS(250));
    cereal.flush();
    cereal.println("\nManufacturing Information:\r\n");
    
    
    #if MANU_ESP_CHIP_INFO
    get_chip_version(ESP_INFO_STRING, sizeof(ESP_INFO_STRING));
    cereal.printf("  %s\r\n", ESP_INFO_STRING);
    #endif

    #if MANU_WIFI_MAC_INFO
    get_wifi_mac(MAC_STRING, sizeof(MAC_STRING));
    cereal.printf("  ESP32 C3 WiFi MAC: %s\r\n\n", MAC_STRING);
    #endif


    //MK62BLA Information
    cereal.printf("  MKL62BA - LoRaWAN Radio Information:\r\n");
    cereal.printf("     Version: %s\r\n", MKL_version);
    cereal.printf("     Region: %s\r\n", MKL_region);
    cereal.printf("     DEVEUI: %s\r\n",MKL_devEUI);
    cereal.printf("     APPEUI: %s\r\n\n", MKL_appEUI);
    cereal.flush();
    //K33 Information
    //ESP_LOGD("MANUFACTURING INFO", "\n%s", (char*)sensorInfoBuffer);
    cereal.print((char *) sensorInfoBuffer);
    
    #if MANU_TEST_K33
    cereal.printf("    CO2 Test Readings (ppm): [ ");
    for (int i = 0; i<5; i++){
        cereal.printf("%u ",currentCO2[i]*10);
    }
    cereal.printf("]\r\n");
    #endif

    //SHT40 Information
    cereal.printf("\n  SHT40 Serial Numer: %u\r\n", SHT40_SNum);
    #if MANU_TEST_SHT40
    cereal.printf("    5x Values (T,H):[ ");
    for (int i = 0; i<5; i++){
        cereal.printf("%2.2f,%2.2f  ",SHT40Temp[i], SHT40Humid[i]);
    }
    cereal.printf("]\r\n");
    #endif
    
    cereal.printf("\n  Current Battery Voltage: %u mV\r\n",currentBatteryMilliVolts);
    cereal.flush();
     
    cereal.println("\nManufacturing Info Dump complete, entering deep sleep in 5 seconds.\r\n\n");
    cereal.flush();
    vTaskDelay(pdMS_TO_TICKS(300));
    //esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    esp_deep_sleep_start();
    

}


void serialInitPhase1(){
    
    Serial.begin(115200);//USB Serial
    Serial.setTimeout(0);
    Serial.setTxTimeoutMs(0);
    
    //Wait for GPIO8 to not be low, no point in stressing serial output drivers
    while(!digitalRead(GPIO8_TP)){
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /*  Serial port information:
    *       *EARLY* Device boot messages are only available on pin 21!
    *       ESP Log messages and other Debug output available always on PIN 8
    *       If dualPrintShim is used, ESP Log output available over USB CDC and Pin 8
    */
    
    #if USE_DEBUG_SERIAL_SHIM
    //Redirect logging to the serial output shim, auto-duplicates serial output to USB CDC if connected
    Serial0.begin(115200, SERIAL_8N1, -1, GPIO8_TP);
    cereal.redirectESPLogging();
    #endif
    #if !USE_DEBUG_SERIAL_SHIM
    cereal.begin(115200, SERIAL_8N1, -1, GPIO8_TP);
    #endif

    esp_log_level_set("*", ESP_LOG_VERBOSE);

    LoRaSerial.begin(static_cast<uint32_t>(LORA_BAUD), SERIAL_8N1, LORA_RX, LORA_TX);
}

void taskInitPhase1(){
    
    #if USE_USB_SERIAL_CONN_TASK
    bool USBTASKOK = USB_Conn_Task_Init();
    #endif
    #if !USE_USB_SERIAL_CONN_TASK
    bool USBTASKOK = true;
    #endif
    
    if (!USBTASKOK){
        ESP_LOGE("CRITICAL", "Phase 1 Task init failure, Try again after Deep Sleep Interval");
        cereal.flush();
        sleepDeep10();
    }

    #if USE_USB_DEBUG_INTERFACE
    xTaskCreate(DBGSerialTask,"SerialTask", 4096, NULL, 1, &serialTaskHandle);
    #endif

}

void taskInitPhase2(){
    
    bool tasksOK = true;
    
    
    
    if (!initTxTask()){
        ESP_LOGE("PH2 Startup", "Failed to create Tx Task.");
        tasksOK = false;
    }
    #if 0
    if (!initCO2Task()){
        ESP_LOGE("PH2 Startup", "Failed to create CO2 Task.");
        tasksOK = false;
    }
    #endif
    if (!initSHT40Task()){
        ESP_LOGE("PH2 Startup", "Failed to create SHT40 Task.");
        tasksOK = false;
    }
    

    if (!tasksOK){
        ESP_LOGE("CRITICAL", "Phase 2 Task init failure(s), HALTING");
        vTaskDelay(100);
        cereal.flush();
        esp_deep_sleep_start();
    }


    setupdeferredRTCTasks();
    //Race to start, Then reduce speed while sampling / waiting for tasks to finish before sleeping
    //requestPMUChange(PMU_SET_CPU_10MHZ);
    setCpuFrequencyMhz(10);
}

#if 0
bool initCO2Task(){

    
    if (xTaskCreate(CO2_Task, "CO2 Sensor Task", 4096, NULL, 5, &taskHandle_K33CO2)!=pdPASS){
        return false;
    }

    return true;
}
#endif

bool initSHT40Task(){
    if (xTaskCreate(SampleSHT40_Task, "SHT40 Sensor Task", 4096, NULL, 5, &taskHandle_SHT40)!=pdPASS){
        return false;
    }
    return true;
}


bool initTxTask(){
    if (xTaskCreate(Tx_Task, "LoRaWan Tx Task", 8192, NULL,10, &taskHandle_Tx)!=pdPASS){
        return false;
    }
    return true;
}

void K33_WaitAndCalibrate(bool isZero){
    
    digitalWrite(EN_5V_BOOST, LOW);
    digitalWrite(EN_3V3_ALT, LOW);
    digitalWrite(I2C_PULLUPS, LOW);
    setCpuFrequencyMhz(10);
    
    uint32_t ledOnTime = isZero ? 200 : 300;
    uint32_t ledOffTime = isZero ? 200 : 300;
    uint32_t numBlinks = isZero ? 5 : 2;

    uint32_t wakend = (isZero ? MANU_ZERO_CAL_DELAY : MANU_BG_CAL_DELAY) * 60 * 1000;
    uint32_t waitStart = millis();

    const uint32_t totalSleepPeriod = 20000;

    while (millis() - waitStart < wakend) {
        uint32_t elapsed = millis() - waitStart;
        uint32_t blinkTime = numBlinks * (ledOnTime + ledOffTime);
        uint32_t adjustedSleepTime = totalSleepPeriod - blinkTime;
        uint32_t timeLeft = wakend - elapsed;

        if (timeLeft < adjustedSleepTime) {
            adjustedSleepTime = timeLeft - blinkTime;
        }

        if (adjustedSleepTime > 0) {
            for (int i = 0; i < numBlinks; i++) {
                digitalWrite(STATUS_LED, LOW);
                delay(ledOnTime);
                digitalWrite(STATUS_LED, HIGH);
                delay(ledOffTime);
            }

            esp_sleep_enable_timer_wakeup(adjustedSleepTime * 1000);
            esp_light_sleep_start();
        } else {
            break;
        }
    }

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);


    digitalWrite(EN_5V_BOOST, HIGH);
    digitalWrite(EN_3V3_ALT, HIGH);
    digitalWrite(I2C_PULLUPS, HIGH);

    vTaskDelay(pdMS_TO_TICKS(100));
    
    //Send info that a calibration is about to occur
    Tx_Aux_Packet(isZero? MSG_PRE_ZERO_CAL:MSG_PRE_BACK_CAL);

    startI2C();
    startCO2Sensor();
    ESP_LOGI("K33_Calibration", "Waiting for K33 To take a number of readings");
    vTaskDelay(pdMS_TO_TICKS(K33_WARMUP_WAIT + K33_POLLING_INTERVAL*AVERAGING_FILTER_LENGTH_K33));
    ESP_LOGI("K33_Calibration", "Sending Cal Command over I2C");
    
    #if !DONT_ACTUALLY_CALIBRATE
    K33SensorError_t anyError = CO2_Sensor.start_FACT_CAL(isZero);
    #endif
    #if DONT_ACTUALLY_CALIBRATE
    //Don't actually trigger the calibration, but pretend it was all ok anyways.
    K33SensorError_t anyError = K33_NO_ERROR;
    
    #endif
    //Wait a bit to be sure EEPROM is updated on K33 before powering it off.
    ESP_LOGI("K33_Calibration", "Command Issued, Waiting before power cycling");
    
    vTaskDelay(pdMS_TO_TICKS(K33_POLLING_INTERVAL));
    CO2_Sensor.powerOff();
    ESP_LOGI("K33_Calibration", "Power Cycling now...");
    stopI2C();


    vTaskDelay(pdMS_TO_TICKS(1000));
   
    //Send a followup packet with the results
    if (anyError == K33_NO_ERROR){
        Tx_Aux_Packet(isZero? MSG_POST_ZERO_CAL:MSG_POST_BACK_CAL);
    }else{
        Tx_Aux_Packet(isZero? MSG_ZERO_FAILED:MSG_BACK_FAILED);
    }
    
    digitalWrite(EN_5V_BOOST, LOW);
    digitalWrite(EN_3V3_ALT, LOW);
    digitalWrite(I2C_PULLUPS, LOW);
    
    deepSleepForever();
}



bool Tx_Aux_Packet(TX_PKT_TYPE mType){
    /*
        Notes: Assumes power to radio and sensors is on already.
    */
    static bool connectedAlready = false;
    Node_Cal_pkt auxPkt;
    initToZero(&auxPkt, sizeof(auxPkt));
    auxPkt.PKT_Type = PKT_CAL;
    
    if (mType == MSG_NODE_DATA || mType == MSG_DEV_INFO) return false; //Not the right function for this
    
    auxPkt.Curr_Batt_mVolts = readBatteryVoltageMillivolts();
    ESP_LOGI("CALPKT","Batt Voltage = %d", auxPkt.Curr_Batt_mVolts);
    if (!connectedAlready){
        LoRa_Radio.begin();
        //Get battery information

        bool radioConnected = false;
        while (!radioConnected){
            ESP_LOGI("CALPKT","LoRa Not yet connected...");
            radioConnected= LoRa_Radio.connect();
            vTaskDelay(pdMS_TO_TICKS(2500));
        }
        connectedAlready = radioConnected;
    }
    //If its a calibration related packet, need to get more data from K33 as well as Temp/Humid 
    if (mType == MSG_PRE_BACK_CAL || mType == MSG_POST_BACK_CAL || mType == MSG_PRE_ZERO_CAL ||
        mType == MSG_POST_ZERO_CAL || mType == MSG_BACK_FAILED || mType == MSG_ZERO_FAILED){
        ESP_LOGI("CALPKT","Gathering Test environment data...");
        startI2C();
        //Get temperature & humidity
        SHT40_Sensor.begin(K33_BUS, SHT40_I2C_ADDR_44);
        uint16_t anyError = SHT40_Sensor.softReset();
        float SHT40Temp = 0.0;
        float SHT40Humid = 0.0;
    
        for (int i = 0; i<5; i++){
            uint16_t anyError = SHT40_Sensor.measureHighPrecision(SHT40Temp, SHT40Humid);
            auxPkt.filteredTemp += SHT40Temp;
            auxPkt.filteredHumidity += SHT40Humid;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        auxPkt.filteredTemp /= 5.0;
        auxPkt.filteredHumidity /= 5.0;
        ESP_LOGI("CALPKT","Temp Avg = %f, Humid Avg = %f", auxPkt.filteredTemp, auxPkt.filteredHumidity);
        startCO2Sensor();
        vTaskDelay(pdMS_TO_TICKS(K33_WARMUP_WAIT + K33_POLLING_INTERVAL * AVERAGING_FILTER_LENGTH_K33));
        
        //Get K33 Calibration parameters
        CO2_Sensor.get_CAL_BCC(auxPkt.CAL_BCC);
        CO2_Sensor.get_CAL_ZeroTrim(auxPkt.CAL_Trim);
        CO2_Sensor.get_CAL_Zero(auxPkt.CAL_Zero);
        
        //Get Some CO2 Readings for validation
        uint16_t co2Data = 0;
        for (int i = 0; i<5; i++){
            CO2_Sensor.readCO2Data(co2Data);
            auxPkt.CO2_Readings[i] = co2Data*10;
            ESP_LOGI("CALPKT K33", "Got CO2 Sample #%i @ %d ppm", i, co2Data*10);
            vTaskDelay(pdMS_TO_TICKS(K33_POLLING_INTERVAL));
        }

        ESP_LOGI("CALPKT","K33 BCC = %d, K33 Trim = %d, K33 Zero = %d", auxPkt.CAL_BCC, auxPkt.CAL_Trim, auxPkt.CAL_Zero);
        ESP_LOGI("CALPKT K33 Data","%i, %i, %i, %i, %i", auxPkt.CO2_Readings[0], auxPkt.CO2_Readings[1],auxPkt.CO2_Readings[2],auxPkt.CO2_Readings[3],auxPkt.CO2_Readings[4]);
        CO2_Sensor.powerOff();
        stopI2C();
    }

    switch(mType){
        case(MSG_FACTORY_RESET):
            auxPkt.FACT_RESET = 1;
            break;
        case(MSG_PRE_ZERO_CAL):
            auxPkt.CAL_ZERO_ARMED = 1;
            break;
        
        case(MSG_POST_ZERO_CAL):
            auxPkt.CAL_ZERO_DONE = 1;
            auxPkt.CAL_ZERO_OK = 1;
            break;
        
        case(MSG_ZERO_FAILED):
            auxPkt.CAL_ZERO_DONE = 1;
            auxPkt.CAL_ZERO_OK = 0;
            break;

        case(MSG_PRE_BACK_CAL):
            auxPkt.CAL_BACK_ARMED = 1;
            break;
        
        case(MSG_POST_BACK_CAL):
            auxPkt.CAL_BACK_DONE = 1;
            auxPkt.CAL_BACK_OK = 1;
            break;
        
        case(MSG_BACK_FAILED):
            auxPkt.CAL_BACK_DONE = 1;
            auxPkt.CAL_BACK_OK = 0;
            break;
        default:
            break;
    }


    txBuffLen = dataToHexString(&auxPkt, sizeof(auxPkt), TxBuffer, sizeof(TxBuffer));
    
    if (txBuffLen ==0){
        digitalWrite(EN_5V_BOOST, LOW);
        digitalWrite(EN_3V3_ALT, LOW);
        digitalWrite(I2C_PULLUPS, LOW);
        deepSleepForever();
    }

    //bool radioConnected = LoRa_Radio.connect();
    uint32_t txAttempts = 1;
    bool transmitSuccess = LoRa_Radio.sendDataPacket(TxBuffer, txBuffLen);
    while (!transmitSuccess && txAttempts < 4){
        rtcSysStat.incTxFailures();
        vTaskDelay(pdMS_TO_TICKS(7000));
        LoRa_Radio.connect();
        transmitSuccess = LoRa_Radio.sendDataPacket(TxBuffer, txBuffLen);
        txAttempts++;
    }

    return transmitSuccess;
}

#if REPORT_DEVICE_STATS_EVERY
void sendDeviceStats(){
    
    Node_Info_pkt infoPkt;
    initToZero(&infoPkt, sizeof(infoPkt));
    
    requestPMUChange(PMU_BOOST_5V_ON);
    requestPMUChange(PMU_ALT_3V3_ON);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    //LoRa_Radio.begin already called in initPhase2

    while (!LoRa_Radio.isInitialized()){
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    RTC_SystemStatus currRTCData;
    rtcSysStat.readData(currRTCData);
    
    infoPkt.PKT_Type = PKT_INF;
    infoPkt.Curr_BootReason = rtcSysStat.getBootReason() & 0x0F;
    infoPkt.Last_BootReason = currRTCData.Last_BootReason & 0x0F;
    infoPkt.CRC_MISMATCH_FLAG = currRTCData.CRCFailed & 0x01;
    
    infoPkt.K33_VAL = currRTCData.K33_VAL;
    infoPkt.I2C_ERR = currRTCData.I2C_ERR;
    infoPkt.I2C_REINITS = currRTCData.I2C_REINITS;
        
    infoPkt.RST_SW = currRTCData.RST_SW;
    infoPkt.RST_PANIC = currRTCData.RST_PANIC;
    infoPkt.RST_ALL_WDT = currRTCData.RST_INT_WDT + currRTCData.RST_TASK_WDT + currRTCData.RST_WDT;
    infoPkt.RST_DEEPSLEEP = currRTCData.RST_DEEPSLEEP;
    infoPkt.RST_BROWNOUT = currRTCData.RST_BROWNOUT;

    txBuffLen = dataToHexString(&infoPkt, sizeof(infoPkt), TxBuffer, sizeof(TxBuffer));
    
    if (txBuffLen ==0){
        deepSleepForever();
    }

    bool radioConnected = LoRa_Radio.connect();
    uint32_t txAttempts = 1;
    bool transmitSuccess = LoRa_Radio.sendDataPacket(TxBuffer, txBuffLen);

    while (!transmitSuccess && txAttempts < 4){
        rtcSysStat.incTxFailures();
        vTaskDelay(pdMS_TO_TICKS(100));
        LoRa_Radio.connect();
        transmitSuccess = LoRa_Radio.sendDataPacket(TxBuffer, txBuffLen);
        txAttempts++;
    }
    requestPMUChange(PMU_ALT_3V3_OFF);
    requestPMUChange(PMU_BOOST_5V_OFF);
}
#endif

size_t dataToHexString(void *data, size_t dataSize, char *hexString, size_t hexStringSize) {
    // Check if buffer is large enough
    if (hexStringSize < (2 * dataSize + 1)) {
        ESP_LOGE("B2HString", "Failed, target buffer too small!");
        return 0;
    }

    uint16_t bIndex = 0;
    for (size_t i = 0; i < dataSize; i++) {
        bIndex += sprintf(hexString + bIndex, "%02X", *((uint8_t *)data + i));
    }

    hexString[bIndex] = '\0'; // Null-terminate the string
    return bIndex; // Return the length of the hex string
}

void deepSleepForever(){
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    cereal.flush();
    vTaskDelay(pdMS_TO_TICKS(1));
    esp_deep_sleep_start();
}

// Utility function to set all bits in a given struct to 0
void initToZero(void *struct_ptr, size_t struct_size) {
    memset(struct_ptr, 0, struct_size);
}



void DBGSerialTask(void *parameter) {
  const int bufferSize = 200;   // Define the size of the buffer
  char inputCharBuffer[bufferSize]; // Character buffer to hold the input
  int bufferIndex = 0;          // Index to track buffer position
  const TickType_t checkInterval = pdMS_TO_TICKS(100); // 100ms interval
  bool usbConnected = false;
  uint32_t lastLEDToggle = millis();
  bool ledState = 1;

  // Initialize and clear buffer before entering the loop
  memset(inputCharBuffer, 0, bufferSize);  // Clear the buffer
  bufferIndex = 0;  // Reset buffer index

  while (true) {
    usbConnected = USB_CDC_Connected();  // Check if USB serial is connected
    if (DBG_MANUAL_MODE_ACTIVE) {
        if (millis() - lastLEDToggle > 333) {
            digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
            ledState = !ledState;
            lastLEDToggle = millis();
        }
    }

    if (usbConnected) {
      // USB is connected, check for incoming serial data
      uint32_t getCMDTimeout = millis();
      bool cmdRX = false;

      while (!cmdRX && millis() - getCMDTimeout < 500) {
        if (Serial.available() > 0) {
          char incomingChar = Serial.read();

          // If Enter key (newline) is pressed, process the full command
          if (incomingChar == '\n') {
            inputCharBuffer[bufferIndex] = '\0';  // Null-terminate the string
            String inputBuffer = String(inputCharBuffer);  // Convert to String
            inputBuffer.trim();  // Remove any trailing/leading whitespace
            Serial.println("INPUT BUFFER: " + inputBuffer);
            processCommand(inputBuffer);  // Call the command processor

            // After processing, reset the buffer and index
            memset(inputCharBuffer, 0, bufferSize);  // Clear the buffer
            bufferIndex = 0;  // Reset buffer index
            cmdRX = true;  // Exit the loop after processing
          } else {
            // Append the character to the buffer if there's space
            if (bufferIndex < bufferSize - 1) {
              inputCharBuffer[bufferIndex++] = incomingChar;
            }
          }
        }
        vTaskDelay(pdMS_TO_TICKS(5));  // Short delay to prevent busy-waiting
      }
    } else {
      // USB not connected, skip processing for 500ms total
      vTaskDelay(pdMS_TO_TICKS(400));
    }

    // Delay for 100ms before checking again
    vTaskDelay(checkInterval);
  }
}


// Function to process commands based on the input string
// Only outputs to USB CDC Serial since its only intended to be used via USB Debugging
void processCommand(String command) {
  if (command.equalsIgnoreCase(DBG_CMD_SYS_REBOOT)) {
    Serial.println(">       Debug Command: Reboot!");
    delay(100);  // Short delay to allow message to be printed
    ESP.restart();  // Reboot the ESP32
  }else if (command.equalsIgnoreCase(DBG_CMD_SYS_FIRST_BOOT)){
    Serial.println(">       Debug Command: Factory Reset!");
    factoryReset();
  
  } else if (command.equalsIgnoreCase(DBG_CMD_SYS_GET_BATT)) {
    Serial.println(">       Debug Command: Get VBatt!");
    uint16_t milliVolts = readBatteryVoltageMillivolts();
    Serial.printf( ">                     VBatt = %d\n", milliVolts);

  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_5V_ON)) {
    Serial.println(">       Debug Command: PMU, Enable 5V Boost");
    //pReqShim("DBG Task", PMU_BOOST_5V_ON);
    digitalWrite(EN_5V_BOOST, HIGH);
  
  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_5V_OFF)) {
    Serial.println(">       Debug Command: PMU, Disable 5V Boost");
    digitalWrite(EN_5V_BOOST, LOW);
    //pReqShim("DBG Task", PMU_BOOST_5V_OFF);

  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_3V3_ON)) {
    Serial.println(">       Debug Command: PMU, Enable Alt 3V3");
    digitalWrite(EN_3V3_ALT, HIGH);

  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_3V3_OFF)) {
    Serial.println(">       Debug Command: PMU, Disable Alt 3V3");
    digitalWrite(EN_3V3_ALT, LOW);
    
  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_GET_STATUS)) {
    Serial.println(">       Debug Command: PMU, Get status");
    bool PWR_3V3_ISON = digitalRead(EN_3V3_ALT);
    bool PWR_5V_ISON = digitalRead(EN_5V_BOOST);
    bool PWR_I2C_ISON = digitalRead(I2C_PULLUPS);
    bool PWR_K33_ISON = digitalRead(K33_PWR);

    Serial.println("Power Status:");
    Serial.printf("    5V Boost: %s\n", PWR_5V_ISON?"ON":"OFF");
    Serial.printf("    Alt 3V3: %s\n", PWR_3V3_ISON?"ON":"OFF");
    Serial.printf("    I2C Pullups: %s\n", PWR_I2C_ISON?"ON":"OFF");
    Serial.printf("    K33 Power: %s\n", PWR_K33_ISON?"ON":"OFF");
    

  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_SET_NOUSERS)) {
    Serial.println(">       Debug Command: PMU, Force No Users");
    digitalWrite(EN_3V3_ALT, LOW);
    digitalWrite(EN_5V_BOOST, LOW);
    digitalWrite(I2C_PULLUPS, LOW);
    digitalWrite(K33_PWR, LOW);

  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_I2C_ON)) {
    Serial.println(">       Debug Command: PMU, Enable I2C Pullups");
    digitalWrite(I2C_PULLUPS, HIGH);
    K33_BUS.begin(I2C_SDA, I2C_SCL,I2C_CLOCK_HZ);

  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_I2C_OFF)) {
    Serial.println(">       Debug Command: PMU, Disable I2C Pullups");
    K33_BUS.end();
    digitalWrite(I2C_PULLUPS, LOW);

  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_LIGHT_SLEEP)){
    Serial.println(">       Debug Command: PMU, LIGHT SLEEP");
    //pReqShim("DBG Task", PMU_LIGHT_SLEEP_1M);
    Serial.println("Not implemented yet.");
  
  } else if (command.equalsIgnoreCase(DBG_CMD_PMU_DEEP_SLEEP)){
    Serial.println(">       Debug Command: PMU, DEEP SLEEP");
    Serial.println("Not implemented yet.");
    //pReqShim("DBG Task", PMU_DEEP_SLEEP_1M);
  
  }else if (command.equalsIgnoreCase(DBG_CMD_SHT40_GET_ALL)){
    Serial.println(">       Debug Command: SHT40, Get sensor data");
    float temperature = -100;
    float humidity =-100;
    startI2C();
    int16_t anyError = SHT40_Sensor.measureHighPrecision(temperature, humidity);
    Serial.printf( ">                   : Temperature = %f,  Humidity = %f\n",temperature, humidity);
    stopI2C();

  }else if (command.equalsIgnoreCase(DBG_CMD_SHT40_HEAT_M_SHORT)){
    Serial.println(">       Debug Command: SHT40, Determine 'cooldown' period for Medium Heat, Short Pulse");
    characterizeMediumHeater();

  }else if (command.equalsIgnoreCase(DBG_CMD_SHT40_HEAT_H_LONG)){
    Serial.println(">       Debug Command: SHT40, Determine 'cooldown' period for Highest Heat, Long Pulse");
    characterizeHighHeater();
    #if 0
    float ogTemperature = -100;
    float ogHumidity =-100;
    uint32_t StartTime = millis();
    int16_t anyError = SHT40_Sensor.measureHighPrecision(ogTemperature, ogHumidity);
    ESP_LOGI("SHT40 Task", "Activating Heater to De-Creep Humidity reading, original Temperature | Humidity Was:   %.2f C | %.2f %%", ogTemperature, ogHumidity);
    float tempVal = 0;
    float humidVal = 0;
    anyError = SHT40_Sensor.activateHighestHeaterPowerLong(tempVal,humidVal);
    //anyError = SHT40_Sensor.activateMediumHeaterPowerShort(tempVal,humidVal);
    ESP_LOGI("SHT40 Task", "Post-Heat Measured  %.2f C | %.2f %%", tempVal, humidVal);
    float currentTemperature = 0;
    float currentHumidity = 0;
    bool isSettled = false;

    while (!isSettled) {
        anyError = SHT40_Sensor.measureHighPrecision(currentTemperature, currentHumidity);
        if (anyError != 0) {
            ESP_LOGE("SHT40 Task", "Error measuring temperature and humidity: %d", anyError);
            break;
        }

        Serial.printf(">>       @%ld ms: %.2f C, %.2f %%\n", millis() - StartTime,currentTemperature, currentHumidity);

        if (fabs(currentTemperature - ogTemperature) <= 0.1) {
            isSettled = true;
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    uint32_t stopTime = millis();
    if (isSettled) {
        uint32_t elapsedTime = stopTime - StartTime;
        Serial.printf(">       Settled Temperature: %.2f C | Time Elapsed: %d ms\n", currentTemperature, elapsedTime);
    } else {
        Serial.println(">       Failed to settle temperature within acceptable range.");
    }
    #endif
  } else if (command.equalsIgnoreCase(DBG_CMD_LORA_JOIN)) {
    Serial.println("Executing LORA JOIN...");
    LoRa_Radio.begin();
    LoRa_Radio.connect();

  } else if (command.equalsIgnoreCase(DBG_CMD_LORA_REINIT)) {
    Serial.println("Executing LORA Reinit...");
    LoRa_Radio.initializeModule();
  
  } else if (command.equalsIgnoreCase(DBG_CMD_K33_START)) {
    Serial.printf(">       Debug Command: K33, Start Sensor\n");
    CO2_Sensor.powerOn();

  }else if (command.startsWith(DBG_CMD_LORA_ISSUE_CMD)){
    ParseCMDInputToCMDBuffer(command);
    char respBuffer[500];
    LoRa_Radio.sendAndReceive(DBG_cmdInputBuffer,respBuffer,"",500,AT_CMD,0,10000);
  
  } else if (command.equalsIgnoreCase(DBG_CMD_K33_STOP)) {
    Serial.printf(">       Debug Command: K33, Stop Sensor\n");
    startI2C();
    CO2_Sensor.powerOff();
    CO2_Sensor.setup(K33_PWR);
    stopI2C();
    
  } else if (command.equalsIgnoreCase(DBG_CMD_K33_GET_CO2)) {
    Serial.printf(">       Debug Command: K33, Get CO2 Value\n");
    uint16_t CO2_Raw_Val = 45000; //Value somewhere outside of any conceivable range.
    startI2C();
    K33SensorError_t readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
    stopI2C();
    Serial.printf(">                      CO2 = %d ppm\n", CO2_Raw_Val*10);

  }else if (command.equalsIgnoreCase(DBG_CMD_K33_GET_INFO)){
    Serial.printf(">       Debug Command: K33, Get Sensor Info\n");
    K33VersionInfo K33SensorInfo;
    uint8_t sensorInfoBuffer[300];
    startI2C();
    CO2_Sensor.getSensorInfo(K33SensorInfo);
    CO2_Sensor.dumpSensorInfoToString(K33SensorInfo, sensorInfoBuffer, sizeof(sensorInfoBuffer));
    stopI2C();
    Serial.println((char*)sensorInfoBuffer);
  
  /*} else if (command.equalsIgnoreCase(DBG_CMD_K33_SET_TRIM)) {
    Serial.printf(">       Debug Command: K33, Set TRIM Value\n");
    int16_t newTrim = -41;
    //CO2_Sensor.powerOff();
    //vTaskDelay(250);
    //CO2_Sensor.powerOn();
    //vTaskDelay(K33_WARMUP_WAIT + 4000);
    CO2_Sensor.set_CAL_ZeroTrim(newTrim);
    vTaskDelay(500);
    CO2_Sensor.powerOff();
    CO2_Sensor.setup(K33_PWR);
    vTaskDelay(500);
    CO2_Sensor.powerOn();

    processCommand(DBG_CMD_K33_GET_INFO);
  */
  }else if (command.equalsIgnoreCase(DBG_CMD_K33_FCAL_ZERO)){
    Serial.printf(">       Debug Command: K33, Start Factory Zero Calibration Routine\n");
    uint16_t CO2_Raw_Val = 45000; //Value somewhere outside of any conceivable range.
    startI2C();
    K33SensorError_t readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
    Serial.printf(">                      Prior CO2 = %d ppm\n", CO2_Raw_Val*10);
    
    CO2_Sensor.start_FACT_CAL(true);
    vTaskDelay(2000);
    CO2_Sensor.powerOff();
    vTaskDelay(500);
    CO2_Sensor.setup(K33_PWR);
    CO2_Sensor.powerOn();
    CO2_Sensor.warmup();
    
    readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
    stopI2C();
    Serial.printf(">                      Post CO2 = %d ppm\n", CO2_Raw_Val*10);

  }else if (command.equalsIgnoreCase(DBG_CMD_K33_FCAL_BG)){
    Serial.printf(">       Debug Command: K33, Start Factory Background Calibration Routine\n");
    uint16_t CO2_Raw_Val = 45000; //Value somewhere outside of any conceivable range.
    startI2C();
    K33SensorError_t readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
    Serial.printf(">                      Prior CO2 = %d ppm\n", CO2_Raw_Val*10);
      
    CO2_Sensor.start_FACT_CAL(false);
     vTaskDelay(2000);
    CO2_Sensor.powerOff();
    vTaskDelay(500);
    CO2_Sensor.setup(K33_PWR);
    CO2_Sensor.powerOn();
    CO2_Sensor.warmup();

    readError = CO2_Sensor.readCO2Data(CO2_Raw_Val);
    stopI2C();
    Serial.printf(">                      Post CO2 = %d ppm\n", CO2_Raw_Val*10);
  
  } else if (command.equalsIgnoreCase(DBG_CMD_SYS_MEM_STATS)) {
    Serial.println(">       Debug Command: Get Memory Info");
    printMemoryInfo();

  } else if (command.equalsIgnoreCase(DBG_CMD_SYS_DUMP_RTC)) {
    char infoBuffer[500];
    rtcSysStat.printStatusToBuffer(infoBuffer,500);
    Serial.println(">       Debug Command: Dump RTC");
    Serial.println(infoBuffer);


  } else if (command.equalsIgnoreCase(DBG_CMD_SYS_RESET_RTC)) {
    char infoBuffer[500];
    rtcSysStat.printStatusToBuffer(infoBuffer,500);
    Serial.println(">       Debug Command: Clear RTC\n>      <<DATA WAS>>");
    Serial.println(infoBuffer);
    Serial.println(">                                        <<CLEARED!>>");
    rtcSysStat.initializeRTCData();
  
  } else if (command.equalsIgnoreCase(DBG_CMD_SYS_EN_MANUAL_MODE)) {
    DBG_MANUAL_MODE_ACTIVE = true; 
    Serial.println(">       Debug Command: MANUAL Mode (REBOOT to clear!)");
    //if (taskHandle_K33CO2 != NULL) vTaskSuspend(taskHandle_K33CO2);
    //if (taskHandle_SHT40 != NULL) vTaskSuspend(taskHandle_SHT40);
    //if (taskHandle_Tx != NULL) vTaskSuspend(taskHandle_Tx);
    //if (taskHandle_RTCScheduler != NULL) vTaskSuspend(taskHandle_RTCScheduler);
    stopRTCScheduler();
    digitalWrite(STATUS_LED, LOW);
    Serial.println(">       All Sampling/Transmission tasks suspended!");

    } else if (command.equalsIgnoreCase(DBG_CMD_HELP)) {
    Serial.printf("%s",debugCommandsWithExplanations);

  } else {
    Serial.println("Unknown command: " + command);
  }
}


void ParseCMDInputToCMDBuffer(String input){
    Serial.printf("         Parsing: %s\r\n", input);
    int index = input.indexOf(">>");
    if (index != -1) {
        // Get the substring after the ">>"
        String remainingPart = input.substring(index + 2);
        
        // Remove any newlines or carriage returns
        remainingPart.replace("\n", "");
        remainingPart.replace("\r", "");
        
        // Copy the result to the global cmdInputBuffer
        strncpy(DBG_cmdInputBuffer, remainingPart.c_str(), sizeof(DBG_cmdInputBuffer) - 1);
        DBG_cmdInputBuffer[sizeof(DBG_cmdInputBuffer) - 1] = '\0'; // Ensure null-termination
    } else {
        DBG_cmdInputBuffer[0] = '\0'; // No ">>" found, clear the buffer
    }
}

void printMemoryInfo() {
  // Get task statistics (prints task name, state, stack usage, and priority) (NOT SUPPORTED IN Arduino Framework)
  //char taskListBuffer[1024];
  //vTaskList(taskListBuffer);
  //Serial.println("Task Name\tState\tPrio\tStack\tNum");
  //Serial.println(taskListBuffer);

  // Get total free heap memory
  size_t freeHeap = esp_get_free_heap_size();
  Serial.printf("Total free heap: %u bytes\n", freeHeap);

  // Get largest free block of heap memory
  size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  Serial.printf("Largest free heap block: %u bytes\n", largestBlock);

  // Get minimum free heap size since boot
  size_t minFreeHeap = esp_get_minimum_free_heap_size();
  Serial.printf("Minimum free heap size since boot: %u bytes\n", minFreeHeap);

  // Free heap specifically in different memory types (8-bit)
  size_t freeInternalRAM = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  Serial.printf("Free internal RAM (8-bit accessible): %u bytes\n", freeInternalRAM);

  // Stack high water mark (remaining stack space) for this task
  UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("DBG task stack high watermark: %u bytes\n", highWaterMark);


}


/*** end of file ***/