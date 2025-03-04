/** @file USB_CONN.hpp
 * 
 * @brief Utility functions to verify USB CDC connectivity of the ESP32-C3 
 *          ***Intended as a development utility, not for production use!***
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed by David Stewart.  All rights reserved.
 */ 

#ifndef __USB_CONN_HPP
#define __USB_CONN_HPP

#include "soc/usb_serial_jtag_reg.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <HardwareSerial.h> // Include the Serial library

bool CheckUSBCDCConnectedViaJTAGFRAME(void);
bool USB_Conn_Task_Init(void); // Updated to return bool
bool USB_CDC_Connected(void);

#endif

/*** end of file ***/