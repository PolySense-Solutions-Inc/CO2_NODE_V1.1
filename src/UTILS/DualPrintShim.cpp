/** @file DualPrintShim.cpp
 * 
 * @brief A utility class to help debug ESP32-C3 USB connected device development by routing print output to multiple ports.
 *          ***Intended as a development utility, not for production use!***
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed by David Stewart.  All rights reserved.
 */ 

#include "DualPrintShim.hpp"
#include "USB_CONN.hpp"
#include "../CONFIG/NODE_CONFIG.h"

DualPrintShim* DualPrintShim::instance = nullptr;  // Initialize the instance pointer

DualPrintShim::DualPrintShim(HardwareSerial& uart, HWCDC& usbSerial)
    : _uart(uart), _usbSerial(usbSerial) {
    instance = this;  // Set the instance pointer
}

size_t DualPrintShim::write(uint8_t c) {
    size_t n = _uart.write(c);
    #if USE_USB_SERIAL_CONN_TASK
    if (USB_CDC_Connected()) {
        n += _usbSerial.write(c);
    }
    #endif
    return n;
}

size_t DualPrintShim::write(const uint8_t* buffer, size_t size) {
    size_t n = _uart.write(buffer, size);
    #if USE_USB_SERIAL_CONN_TASK
    if (USB_CDC_Connected()) {
        n += _usbSerial.write(buffer, size);
    }
    #endif
    return n;
}

void DualPrintShim::flush() {
    _uart.flush();
    #if USE_USB_SERIAL_CONN_TASK
    if (USB_CDC_Connected()) {
        _usbSerial.flush();
    }
    #endif
}

void DualPrintShim::redirectESPLogging() {
    esp_log_set_vprintf(custom_vprintf);

    // Example log
    ESP_LOGI("LOGGER", "Logging now routed through DualPrintShim.");
}

// Custom vprintf function
int DualPrintShim::custom_vprintf(const char *fmt, va_list args) {
    char buffer[256];
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    if (len > 0) {
        instance->write((const uint8_t *)buffer, len);
    }
    return len;
}

/*** end of file ***/