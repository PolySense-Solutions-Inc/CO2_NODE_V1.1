/** @file RTC_SysStat.h
 * 
 * @brief System status counters stored in semi-volatile RTC memory for the Polysense Solutions Inc. ESP32 based sensor platform.
 *          Key Points:
 *              - All data is marked RTC_NOINIT_ATTR to avoid initializing memory during boot.
 *              - Data integrity is always verified at boot via CRC comparison.
 *              - CRC is updated whenever contents of systemStatus is updated via incrementer / setter functions. 
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for Polysense Solutions Inc., by David Stewart.  All rights reserved.
 */

#include "RTC_SysStat.hpp"

static const char* LOGGING_TAG = "RTC VARS";

// Define the system status structure with RTC memory attributes
RTC_NOINIT_ATTR RTC_SystemStatus systemStatus;
RTC_NOINIT_ATTR uint32_t systemStatusCRC;

RTC_SysStat::RTC_SysStat() {
    bool POR = isPowerOnReset();
    bool CRC_OK = VerifySystemStatusCRC();
    
    //This is broken in platformio / ESP Arduino...
    //if (!LOG_RTC_UPDATES) esp_log_level_set(LOGGING_TAG, ESP_LOG_NONE);

    if ((!POR) && CRC_OK) {
        incrementBootReasonCounter();
    } else if (!POR) {
        
        ESP_LOGW(LOGGING_TAG, "CRC mismatch, Re-initializing RTC data");
        initializeRTCData();
        setCRCFailedFlag();
    }else {
        ESP_LOGW(LOGGING_TAG, "Power-On-Reset, initializing RTC data");
        initializeRTCData();
    }
}

// Functions to update run time meta data

//Update count of # of times the K33 has given an 'Out of Range' value
void RTC_SysStat::setK33Val(uint32_t value) {
    systemStatus.K33_VAL = value;
    updateSystemStatusCRC();
}

//Set flag indicating a CRC mismatch on RTC memory has occured
void RTC_SysStat::setCRCFailedFlag() {
    systemStatus.CRCFailed = 1;
    updateSystemStatusCRC();
}

// Update count of # of times an I2C transaction error has occured
void RTC_SysStat::setI2cErr(uint32_t value) {
    systemStatus.I2C_ERR = value;
    updateSystemStatusCRC();
}

//Update count of # of times I2C bus has been re-initialized due to bus contention
void RTC_SysStat::setI2cReinits(uint32_t value) {
    systemStatus.I2C_REINITS = value;
    updateSystemStatusCRC();
}


//Increment count of # of times I2C bus has been re-initialized due to bus contention
void RTC_SysStat::incI2cReinits() {
    systemStatus.I2C_REINITS++;
    updateSystemStatusCRC();
}

//Increment count of # of times LoRa Tx Failed.
void RTC_SysStat::incTxFailures() {
    systemStatus.failedTxCount++;
    updateSystemStatusCRC();
}

//Update the stored voltage of the battery
void RTC_SysStat::setLastBattVolts(uint32_t value) {
    systemStatus.Last_BattVolts = value;
    updateSystemStatusCRC();
}

//Update the stored reason for the last boot
void RTC_SysStat::setLastBootReason(uint32_t value) {
    systemStatus.Last_BootReason = value;
    updateSystemStatusCRC();
}

//Return the complete RTC_SystemStatus data structure for more thorough examination
void RTC_SysStat::readData(RTC_SystemStatus &data) const {
    data = systemStatus;
}

void RTC_SysStat::getDeepWakes(uint32_t &wakeCount){
    wakeCount = systemStatus.RST_DEEPSLEEP;
}

//Verify the CRC of the RTC System Status data structure, and re-initialize if there is a mismatch
bool RTC_SysStat::VerifySystemStatusCRC() {
    crc.restart();
    crc.add((uint8_t*)&systemStatus, sizeof(systemStatus));
    uint32_t currSystemStatusCRC = crc.calc();
    if (currSystemStatusCRC != systemStatusCRC) {
        ESP_LOGE(LOGGING_TAG, "CRC MISMATCH! Current: 0x%x Saved: 0x%x", currSystemStatusCRC, systemStatusCRC);
        return false;
    } else {
        return true;
    }
}

//Helper function to recalculate & update stored CRC of RTC System Status structure
void RTC_SysStat::updateSystemStatusCRC() {
    crc.restart();
    crc.add((uint8_t*)&systemStatus, sizeof(systemStatus));
    systemStatusCRC = crc.calc();
    #if LOG_RTC_CRC_UPDATES
    ESP_LOGI(LOGGING_TAG, "Updated, new CRC = 0x%x", systemStatusCRC);
    #endif
}

//Struct data Initializer
void RTC_SysStat::initializeRTCData() {
    systemStatus = {0};
    updateSystemStatusCRC();
}

//Dump all the stored information to a char buffer
void RTC_SysStat::printStatusToBuffer(char *buffer, size_t bufferSize) const {
    int offset = 0;
    offset += snprintf(buffer + offset, bufferSize - offset, "<System Operation Stats>\r\n");
    offset += snprintf(buffer + offset, bufferSize - offset, "  System Reset Counters:\r\n");
    offset += snprintf(buffer + offset, bufferSize - offset, "  Unknown: %u\r\n", systemStatus.RST_UNKNOWN);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Power-on: %u\r\n", systemStatus.RST_POWERON);
    offset += snprintf(buffer + offset, bufferSize - offset, "  External: %u\r\n", systemStatus.RST_EXT);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Software: %u\r\n", systemStatus.RST_SW);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Panic: %u\r\n", systemStatus.RST_PANIC);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Int. WDT: %u\r\n", systemStatus.RST_INT_WDT);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Task WDT: %u\r\n", systemStatus.RST_TASK_WDT);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Other WDT: %u\r\n", systemStatus.RST_WDT);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Deep Sleep: %u\r\n", systemStatus.RST_DEEPSLEEP);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Brownout: %u\r\n", systemStatus.RST_BROWNOUT);
    offset += snprintf(buffer + offset, bufferSize - offset, "  SDIO: %u\r\n", systemStatus.RST_SDIO);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Corrupt CRC Det.: %u\r\n", systemStatus.CRCFailed);
    
    offset += snprintf(buffer + offset, bufferSize - offset, "\nPlatform Specific Info: \r\n");
    offset += snprintf(buffer + offset, bufferSize - offset, "  K33 Value Errors: %u\r\n", systemStatus.K33_VAL);
    offset += snprintf(buffer + offset, bufferSize - offset, "  I2C Errors: %u\r\n", systemStatus.I2C_ERR);
    offset += snprintf(buffer + offset, bufferSize - offset, "  I2C Bus Recovered: %u\r\n", systemStatus.I2C_REINITS);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Failed LoRa Tx: %u\r\n", systemStatus.failedTxCount);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Last Boot Battery: %u mV\r\n", systemStatus.Last_BattVolts);
    offset += snprintf(buffer + offset, bufferSize - offset, "  Last Boot Reason: %u\r\n", systemStatus.Last_BootReason);
        
    offset += snprintf(buffer + offset, bufferSize - offset, "\n  RTC Status CRC: 0x%X\r\n", systemStatusCRC);
    offset += snprintf(buffer + offset, bufferSize - offset, " </System Operation Stats>\r\n");
}


bool RTC_SysStat::isPowerOnReset() const {
    esp_reset_reason_t reason = esp_reset_reason();
    return (reason == ESP_RST_POWERON);
}

esp_reset_reason_t RTC_SysStat::getBootReason(){
    return bootReason;
};

void RTC_SysStat::incrementBootReasonCounter() {
    bootReason = esp_reset_reason();
    switch (bootReason) {
        case ESP_RST_UNKNOWN:
            systemStatus.RST_UNKNOWN++;
            break;
        case ESP_RST_POWERON:
            systemStatus.RST_POWERON++;
            break;
        case ESP_RST_EXT:
            systemStatus.RST_EXT++;
            break;
        case ESP_RST_SW:         
            systemStatus.RST_SW++;
            break;
        case ESP_RST_PANIC:
            systemStatus.RST_PANIC++;
            break;
        case ESP_RST_INT_WDT:
            systemStatus.RST_INT_WDT++;
            break;
        case ESP_RST_TASK_WDT:
            systemStatus.RST_TASK_WDT++;
            break;
        case ESP_RST_WDT:
            systemStatus.RST_WDT++;
            break;
        case ESP_RST_DEEPSLEEP:
            systemStatus.RST_DEEPSLEEP++;
            break;
        case ESP_RST_BROWNOUT:
            systemStatus.RST_BROWNOUT++;
            break;
        case ESP_RST_SDIO:
            systemStatus.RST_SDIO++;
            break;
        default:
            break;
    }
    updateSystemStatusCRC();
}

/*** end of file ***/