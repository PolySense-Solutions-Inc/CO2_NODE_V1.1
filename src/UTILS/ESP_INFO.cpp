/** @file NODE_WIFI.cpp
 * 
 * @brief Utility function to get ESP32-C3 Chip information.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for Polysense Solutions Inc., by David Stewart.  All rights reserved.
 */

#include "ESP_INFO.hpp"
// Function to get ESP32 chip version and format it into a provided buffer
void get_chip_version(char *buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size < 32) { // Ensure buffer has enough space
        return;
    }

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    snprintf(buffer, buffer_size, "ESP32-C3 Info: %d CPU core(s), WiFi%s%s, silicon revision %d",
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             chip_info.revision);
}

/*** end of file ***/