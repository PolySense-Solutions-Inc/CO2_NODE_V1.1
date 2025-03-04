/** @file NODE_WIFI.hpp
 * 
 * @brief Utility function to get ESP32 WiFi MAC address
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for Polysense Solutions Inc., by David Stewart.  All rights reserved.
 */ 
#ifndef __PLATFORM_WIFI_HPP
#define __PLATFORM_WIFI_HPP
#include "esp_wifi.h"

esp_err_t get_wifi_mac(char *buffer, size_t buffer_size);

#endif

/*** end of file ***/