/** @file DualPrintShim.hpp
 * 
 * @brief A utility class to help debug ESP32-C3 USB connected device development by routing print output to multiple ports.
 *          ***Intended as a development utility, not for production use!***
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed by David Stewart.  All rights reserved.
 */ 

#ifndef DUAL_PRINT_SHIM_HPP
#define DUAL_PRINT_SHIM_HPP

#include <HardwareSerial.h>
#include "esp_log.h"

class DualPrintShim : public Print {
public:
    DualPrintShim(HardwareSerial& uart, HWCDC& usbSerial);

    void redirectESPLogging();
    static int custom_vprintf(const char *fmt, va_list args);

    virtual size_t write(uint8_t c) override;
    virtual size_t write(const uint8_t* buffer, size_t size) override;
    virtual void flush();

    using Print::write; // Pull in write(str) and write(buf, size) from Print

private:
    HardwareSerial& _uart;
    HWCDC& _usbSerial;
    static DualPrintShim* instance;
};

#endif // PRINT_SHIM_HPP

/*** end of file ***/