/** @file NODE_WIFI.hpp
 * 
 * @brief Utility function to get ESP32-C3 Chip information.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for Polysense Solutions Inc., by David Stewart.  All rights reserved.
 */

#ifndef __ESP_INFO_HPP
#define __ESP_INFO_HPP
#include <stddef.h>
#include <stdio.h>
#include "esp_system.h"

void get_chip_version(char *buffer, size_t buffer_size);

#endif

/*** end of file ***/