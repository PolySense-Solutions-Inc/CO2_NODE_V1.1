/** @file RTC_SysStat.hpp
 * 
 * @brief System status counters stored in semi-volatile RTC memory for the Polysense Solutions ESP32 based sensor platform.
 *          Key Points:
 *              - All data is marked RTC_NOINIT_ATTR to avoid initializing memory during boot.
 *              - Data integrity is always verified at boot via CRC comparison.
 *              - CRC is updated whenever contents of systemStatus is updated via incrementer / setter functions. 
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for Polysense Solutions Inc., by David Stewart.  All rights reserved.
 */ 

#ifndef RTC_SYSTEMSTATUS_H
#define RTC_SYSTEMSTATUS_H

#include <esp_system.h>
#include <rom/rtc.h>


// Rob Tiliart MIT Licensed CRC Library
#include <CRC32.h>

#include "../CONFIG/NODE_CONFIG.h"

// System Status Variables Structure
typedef struct {
    
    //Run time metadata
    uint32_t K33_VAL;               // K33 Invalid value errors
    uint32_t I2C_ERR;               // I2C Bus Errors
    uint32_t I2C_REINITS;           // I2C Bus ReInits + Sensor Task Restarts (Contention Issues)
    uint16_t Last_BattVolts;        // Previous Bootup Battery (mVolts)
    uint8_t Last_BootReason;        // Previous Bootup Reason

    uint32_t failedTxCount;         // Number of times a LoRaWAN transmission has failed
    
    // ESP32 Reset Reason Counters
    uint32_t RST_UNKNOWN;           // Unknown
    uint32_t RST_POWERON;           // Power On (Power Applied)
    uint32_t RST_EXT;               // External Reset
    uint32_t RST_SW;                // Software Reset
    uint32_t RST_PANIC;             // Panic Reset
    uint32_t RST_INT_WDT;           // Interrupt WDT Reset
    uint32_t RST_TASK_WDT;          // Task WDT Reset
    uint32_t RST_WDT;               // Other WDT Reset
    uint32_t RST_DEEPSLEEP;         // Deep Sleep Resumptions
    uint32_t RST_BROWNOUT;          // Brownout Resets
    uint32_t RST_SDIO;              // SDIO Triggered Resets
    uint8_t CRCFailed;              // CRC Corrupted Flag, Indicates RTC System Status data has corrupted at least once since first poweron
} RTC_SystemStatus;

// Declare (Externally) SystemStatus Struct with RTC Attributes
extern RTC_NOINIT_ATTR RTC_SystemStatus systemStatus;
extern RTC_NOINIT_ATTR uint32_t systemStatusCRC;

class RTC_SysStat {
public:
    RTC_SysStat();

    // Functions to update run time meta data
    void setK33Val(uint32_t value);
    void setI2cErr(uint32_t value);
    void setI2cReinits(uint32_t value);
    void incI2cReinits();
    void setLastBattVolts(uint32_t value);
    void setLastBootReason(uint32_t value);
    void incTxFailures();
    
    
    
    // Get all the data in RTC
    void readData(RTC_SystemStatus &data) const;

    void getDeepWakes(uint32_t &wakeCount);
    esp_reset_reason_t getBootReason();
    
    bool VerifySystemStatusCRC();  // Verify the CRC of the data inside RTC Memory, re-init if mismatched
    void initializeRTCData();      // Initialize the data inside RTC Memory.
    
    // Print stored information to a buffer
    void printStatusToBuffer(char *buffer, size_t bufferSize) const;

private:
    CRC32 crc;

    esp_reset_reason_t bootReason;

    void updateSystemStatusCRC();
    bool isPowerOnReset() const;
    void incrementBootReasonCounter();  // Update boot reason counters
    void setCRCFailedFlag();
};

#endif // RTC_SYSTEMSTATUS_H
