/** @file NODE_WIFI.cpp
 * 
 * @brief Utility function to get ESP32 WiFi MAC address
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for Polysense Solutions Inc., by David Stewart.  All rights reserved.
 */ 

#include "NODE_WIFI.hpp"
// Function to get Wi-Fi MAC address and format it into a provided buffer
esp_err_t get_wifi_mac(char *buffer, size_t buffer_size) {
    if (buffer == NULL || buffer_size < 18) { // MAC address requires 18 characters including null terminator
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t mac[6];
    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err == ESP_OK) {
        snprintf(buffer, buffer_size, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    return err;
}

/*** end of file ***/