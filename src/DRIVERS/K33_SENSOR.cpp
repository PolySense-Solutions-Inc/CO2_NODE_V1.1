/** @file K33_SENSOR.cpp
 * 
 * @brief A comprehensive I2C driver for the Sensair K33 CO2 Sensor 
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */

/*  Recommendations & Todo:
 *      * Make device time stamps dependent on RTC timer vs 'millis' since light sleep does not increment millis.
 *      * Implement/Fix explicit calibration write functions
 *
*/


/*  DATA Endianess WARNING!
  
    ESP32 stores data such as a uint16_t in Little Endian format.
    Example, suppose we store a uint16_t of value 0x1234 at memory location 0x32323200

        Memory location             Content             Note
        0x32323200                  0x34                LSB
        0x32323201                  0X12                MSB

    When we write to I2C where a uint16_t has been reinterpret_cast<uint8_t*>, we would send the LSB first!

    To help deal with this problem, a private swapEndianess(uint8_t *buffer, size_t buffLen) function has been provided.
        (See also, LOG_K33_DEBUG_ENDIANESS macro definition in the header file)
*/

#include "K33_SENSOR.hpp"
#include <Arduino.h>

static const char* LOGGING_TAG = "K33 Driver";



K33Sensor::K33Sensor(uint8_t i2cAddress, TwoWire *wireObject)
    : i2cAddress(i2cAddress), wire(wireObject), state(K33_OFF), sensorLibInit(false) {}

void K33Sensor::setup(uint8_t pwrPin){
   pinMode(this->powerPin, OUTPUT);

   this->sensorStats.K33_CHKERRS = 0;
   this->sensorStats.K33_CMDERRS = 0;
   this->sensorStats.K33_TIMERRS = 0;
   this->sensorStats.K33_I2CERRS = 0;
   this->sensorStats.K33_VALERRS = 0;
   this->sensorStats.K33_OKCOUNT = 0;
   this->powerPin = pwrPin;
   this->PowerOnTime = 0;
   this->sensorLibInit = true;
}

void K33Sensor::wake() {
    // Wake sensor function, only needed if sensor uses sleep modes.
    // (Not required when power cycling)
    
    wire->beginTransmission(0x00);
    wire->endTransmission();
    this->PowerOnTime = millis();
    this->state = K33_WARMUP;
}

void K33Sensor::powerOff() {
    
    #if !KEEP_K33_ON_IF_USB_CONNECTED
    // Power off the K33
    this->state = K33_OFF;
    this->PowerOnTime = 0;
    this->sensorStats.K33_CHKERRS = 0;
    this->sensorStats.K33_CMDERRS = 0;
    this->sensorStats.K33_TIMERRS = 0;
    this->sensorStats.K33_I2CERRS = 0;
    this->sensorStats.K33_VALERRS = 0;
    this->sensorStats.K33_OKCOUNT = 0;
    digitalWrite(this->powerPin, LOW);
    #else
    ESP_LOGI(LOGGING_TAG, "Not powering K33 Off, Configured to keep K33 active when USB is connected.");
    #endif
}

void K33Sensor::powerOn() {
    // Power on the K33
    digitalWrite(this->powerPin, HIGH);
    this->PowerOnTime = millis();
    this->state = K33_WARMUP;
    #if LOG_K33_TIMING_INFO
        ESP_LOGI(LOGGING_TAG, "Sensor Powered On @ %u", this->PowerOnTime);
    #endif
    //warmup();
}

void K33Sensor::warmup(){
    if (this->state != K33_ACTIVE){
        #if LOG_K33_TIMING_INFO
        ESP_LOGI(LOGGING_TAG, "Sensor Warmup Started");
        #endif
        int32_t waitTime = K33_WARMUP_WAIT - (millis() - (this->PowerOnTime));
        if (waitTime>0) vTaskDelay(pdMS_TO_TICKS(waitTime));
        #if LOG_K33_TIMING_INFO
        ESP_LOGI(LOGGING_TAG, "Sensor Warmup DONE!");
        #endif
        this->state = K33_ACTIVE;
    }
}

#define WINDOW_OFFSET_BEFORE 1250     // Equivalent to 750 ms before interval start, but positive
#define WINDOW_WIDTH 750              // Total window width of 1500 ms

// Delay until within the next K33 Communication Window
void K33Sensor::delay_until_next_window() {

    uint32_t elapsed_time = millis() - this->PowerOnTime;

    //ESP_LOGI(LOGGING_TAG, ">       elapsed Time: %lu", elapsed_time);

    // Calculate time since the initial warmup in whole periods
    uint32_t periods_elapsed = (elapsed_time - K33_WARMUP_WAIT) / K33_POLLING_INTERVAL;
    //ESP_LOGI(LOGGING_TAG, ">       Periods elapsed: %lu", periods_elapsed);
    uint32_t next_interval_start = K33_WARMUP_WAIT + (periods_elapsed + 1) * K33_POLLING_INTERVAL;

    //ESP_LOGI(LOGGING_TAG, ">       Next Start: %lu", next_interval_start);
    // Calculate the open boundary for the window
    uint32_t window_open = next_interval_start - WINDOW_OFFSET_BEFORE;

    //ESP_LOGI(LOGGING_TAG, ">       Window Opens: %lu", window_open);
    uint32_t window_close = window_open + WINDOW_WIDTH;
    
    //ESP_LOGI(LOGGING_TAG, ">       Window Closes: %lu", window_close);

    // Check if we're already within the window
    if (elapsed_time >= window_open && elapsed_time <= window_close) {
        return;  // No delay needed, already in the window
    }
    
    // If we're past the window, adjust to the next one
    if (elapsed_time > window_close) {
        window_open += K33_POLLING_INTERVAL;
    }

    // Calculate delay time until the adjusted window opens
    uint32_t delay_time = window_open - elapsed_time;
    //ESP_LOGI(LOGGING_TAG, "K33 Comms window delay: %lu", delay_time);
    vTaskDelay(pdMS_TO_TICKS(delay_time));  // Delay until the next window opens
}


void K33Sensor::issueCmd(uint8_t tarAddr, uint8_t* cmdBytes, uint8_t cmdSize) {

    //If we are still in the WARMUP state then wait till that finishes
    //if (this->state == K33_WARMUP){
    //    warmup();
    //}
    if (this->state != K33_ACTIVE){
        if ((millis()- (this->PowerOnTime)) >K33_WARMUP_WAIT) this->state = K33_ACTIVE;
    }
    wire->beginTransmission(tarAddr);
    wire->write(cmdBytes, cmdSize);
    
    uint8_t I2C_Result = wire->endTransmission();
    
    //0: success.
    //1: data too long to fit in transmit buffer.
    //2: received NACK on transmit of address.
    //3: received NACK on transmit of data.
    //4: other error.
    //5: timeout

    #if LOG_K33_TRANSACTION_DATA
    // Create a buffer to hold the formatted log message
    char logBuffer[128];  // Adjust size if necessary depending on the cmdSize
    int len = 0;          // Length of formatted string
    
    // Start with the target address
    len += sprintf(logBuffer + len, "I2C TX: Target @: 0x%02X, Command Bytes: ", tarAddr);
    
    // Append the command bytes in hex format
    for (int i = 0; i < cmdSize; i++) {
        len += sprintf(logBuffer + len, "0x%02X ", cmdBytes[i]);
    }

    // Log the entire buffer
    ESP_LOGI(LOGGING_TAG, "%s", logBuffer);
    #endif

    if (I2C_Result != 0){
        ESP_LOGE(LOGGING_TAG, "I2C Transaction Error, Type: %u", I2C_Result);
        K33DriverEventTracker(K33_I2C_ERROR);
    }
}

void K33Sensor::requestData(uint8_t tarAddr, uint8_t* rxBuff, uint8_t reqSize) {

    // Request data from the target address
    wire->requestFrom(tarAddr, reqSize, (uint8_t)0); // Cast to avoid ambiguity
    
    for (uint8_t i = 0; i < reqSize; i++) {
        if (wire->available()) {
            rxBuff[i] = wire->read();
        }
    }

    // Create a buffer to hold the formatted log message
    char logBuffer[128];  // Adjust size if necessary based on reqSize
    int len = 0;          // Length of formatted string
    
    // Start with the target address
    len += sprintf(logBuffer + len, "I2C Data Request: Target Address: 0x%02X, Received Bytes: ", tarAddr);
    
    // Append the received bytes in hex format
    for (uint8_t i = 0; i < reqSize; i++) {
        len += sprintf(logBuffer + len, "0x%02X ", rxBuff[i]);
    }

    // Log the entire buffer
    ESP_LOGI("K33Sensor", "%s", logBuffer);
}

K33SensorError_t K33Sensor::readBytesFromK33(uint8_t *data, uint8_t numBytes, uint16_t memAddress, bool fromEEPROM, bool swapByteOrder) {
    if (!sensorLibInit) {
        K33DriverEventTracker(K33_DRIVER_NOINIT_ERROR);
        return K33_DRIVER_NOINIT_ERROR;
    }

    if ((numBytes == 0) || (numBytes > 16)) {
        K33DriverEventTracker(K33_DRIVER_INVALID_PARAMETERS);
        return K33_DRIVER_INVALID_PARAMETERS;
    }

    if (this->state != K33_ACTIVE){
        if ((millis()- (this->PowerOnTime)) >K33_WARMUP_WAIT) this->state = K33_ACTIVE;
        ESP_LOGI(LOGGING_TAG, "K33 Warmup finished before readBytes");
    }
    //if (this->state == K33_WARMUP){
    //    ESP_LOGW(LOGGING_TAG, "Read Request but not warmed up!");
    //    warmup();
    //}
    //Ensure communications lines up with a valid window
    //delay_until_next_window();
    //taskENTER_CRITICAL(K33_Comms_Mux);
    uint32_t start_time = millis(); // Track when the transaction starts
    uint8_t commandSequence[4] = {0};
    if (!fromEEPROM) {
        commandSequence[0] = 0x20 | (numBytes & 0b00001111); // 0x20 Read from RAM
    } else {
        commandSequence[0] = 0x40 | (numBytes & 0b00001111); // 0x40 Read from EEPROM
    }

    // Handle memory address endianess
    commandSequence[1] = (memAddress) >> 8;
    commandSequence[2] = memAddress & 0xFF;
    commandSequence[3] = commandSequence[0] + commandSequence[1] + commandSequence[2];

    // Issue the I2C command
    issueCmd(i2cAddress, commandSequence, sizeof(commandSequence));
    //taskEXIT_CRITICAL(K33_Comms_Mux);
    vTaskDelay(pdMS_TO_TICKS(K33_CMD_ISSUE_DELAY));
    //taskENTER_CRITICAL(K33_Comms_Mux);
    // Receive data from the sensor
    uint8_t rxBuffer[18] = {0}; // Buffer for received data + checksum
    requestData(i2cAddress, rxBuffer, numBytes + 2);

    uint32_t stop_time = millis(); // Track when the transaction ends
    //taskEXIT_CRITICAL(K33_Comms_Mux);
    // Check for timeout
    if ((stop_time - start_time) > K33_TRANSACTION_TIMEOUT) {
        K33DriverEventTracker(K33_TIMEOUT_ERROR);
        return K33_TIMEOUT_ERROR;
    }

    // Calculate checksum
    byte calc_checksum = 0;
    for (int i = 0; i < numBytes + 1; i++) {
        calc_checksum += rxBuffer[i];
    }
    bool checksumSuccess = (calc_checksum == rxBuffer[numBytes + 1]);

    // Verify command success
    bool lastCommandSuccess = ((rxBuffer[0] & 0x01) == 1);
    if (!lastCommandSuccess) {
        K33DriverEventTracker(K33_COMMAND_ERROR);
        return K33_COMMAND_ERROR;
    }

    if (!checksumSuccess) {
        K33DriverEventTracker(K33_CHECKSUM_ERROR);
        return K33_CHECKSUM_ERROR;
    }

    // Optionally swap byte order if requested
    if (swapByteOrder && numBytes > 1) {
        swapEndianess(rxBuffer + 1, numBytes);
    }

    // Copy data to output buffer
    for (int i = 0; i < numBytes; i++) {
        data[i] = rxBuffer[i + 1];
    }

    #if LOG_K33_TIMING_INFO
    ESP_LOGI(LOGGING_TAG, "Data Request Started %u ms after boot, finished %u ms later.", (start_time - this->PowerOnTime), (stop_time - start_time));
    #endif

    K33DriverEventTracker(K33_NO_ERROR);
    return K33_NO_ERROR;
}



K33SensorError_t K33Sensor::writeBytesToK33(uint8_t *data, uint8_t numBytes, uint16_t memAddress, bool toEEPROM, bool swapByteOrder) {
    
    /*
        Major TODO:     Verify byte ordering of sent commands and use swapEndianess if required!
    */
    
    if (!sensorLibInit) {
        K33DriverEventTracker(K33_DRIVER_NOINIT_ERROR);
        return K33_DRIVER_NOINIT_ERROR;
    }

    if ((numBytes == 0) || (numBytes > 16)) {
        K33DriverEventTracker(K33_DRIVER_INVALID_PARAMETERS);
        return K33_DRIVER_INVALID_PARAMETERS;
    }

    if (this->state != K33_ACTIVE){
        if ((millis()- (this->PowerOnTime)) >K33_WARMUP_WAIT) this->state = K33_ACTIVE;
    }
    //if (this->state == K33_WARMUP){
    //    ESP_LOGW(LOGGING_TAG, "Write Request but not warmed up!");
    //    warmup();
    //}

    uint8_t commandSequence[20] = {0}; // Allocate buffer to accommodate 16 bytes of data + 4 bytes for command and address

    if (!toEEPROM) {
        commandSequence[0] = 0x10 | (numBytes & 0b00001111); // 0x10 Write to RAM, up to 16 bytes only
    } else {
        commandSequence[0] = 0x30 | (numBytes & 0b00001111); // 0x30 Write to EEPROM, up to 16 bytes only
    }
    
    //This corrects memory address endianess vs ESP32's little-endianess
    commandSequence[1] = (memAddress) >> 8;
    commandSequence[2] = memAddress & 0xFF;

    // Copy the data to be written into the command sequence
    for (int i = 0; i < numBytes; i++) {
        commandSequence[3 + i] = data[i];
    }

    // Calculate the checksum
    byte checksum = 0;
    for (int i = 0; i < 3 + numBytes; i++) {
        checksum += commandSequence[i];
    }

    commandSequence[3 + numBytes] = checksum; // Append checksum at the end
    
    //Ensure communications lines up with a valid window
    //delay_until_next_window();
    
    uint32_t start_time = millis();
    
    issueCmd(i2cAddress, commandSequence, 4 + numBytes); // Send the command sequence

    vTaskDelay(pdMS_TO_TICKS(K33_CMD_ISSUE_DELAY));

    uint8_t rxBuffer[2] = {0};
    requestData(i2cAddress, rxBuffer, sizeof(rxBuffer));

    uint32_t stop_time = millis();
    if ((stop_time - start_time) > K33_TRANSACTION_TIMEOUT){
        K33DriverEventTracker(K33_TIMEOUT_ERROR);
        return K33_TIMEOUT_ERROR;
    }
 

    bool lastCommandSuccess = ((rxBuffer[0] & 0x01) == 1);
    if (!lastCommandSuccess) {
        K33DriverEventTracker(K33_COMMAND_ERROR);
        return K33_COMMAND_ERROR;
    }

    byte calc_checksum = rxBuffer[0];// + rxBuffer[1];
    bool checksumSuccess = (calc_checksum == rxBuffer[1]);

    if (!checksumSuccess) {
        K33DriverEventTracker(K33_CHECKSUM_ERROR);
        return K33_CHECKSUM_ERROR;
    }

    #if LOG_K33_TIMING_INFO
    ESP_LOGI(LOGGING_TAG, "Data Write Started %u ms after boot, finished %u ms later.", (start_time - this->PowerOnTime), (stop_time - start_time) );
    #endif
    K33DriverEventTracker(K33_NO_ERROR);
    return K33_NO_ERROR;
}

void K33Sensor::swapEndianess(uint8_t *buffer, size_t buffLen) {

    #if LOG_K33_DEBUG_ENDIANESS
    ESP_LOGD(LOGGING_TAG,"Endianness Swap on %p of size %u", buffer, buffLen);
    char pBuff[100] = {0};
    uint16_t offset = 0;
    for (int i = 0; i<buffLen; i++){
        offset += sprintf(pBuff+offset, "%X", buffer[i]);
    }
    pBuff[offset] = '\0';

    ESP_LOGD(LOGGING_TAG, "Initial Data: %s", pBuff);
    #endif
    if (buffer == NULL || buffLen == 0) {
        return; // Handle invalid buffer or length
    }
    
    for (size_t i = 0; i < buffLen / 2; i++) {
        uint8_t temp = buffer[i];
        buffer[i] = buffer[buffLen - 1 - i];
        buffer[buffLen - 1 - i] = temp;
    }
    
    #if LOG_K33_DEBUG_ENDIANESS
    offset = 0;
    for (int i = 0; i<buffLen; i++){
        offset += sprintf(pBuff+offset, "%X", buffer[i]);
    }
    pBuff[offset] = '\0';
    ESP_LOGD(LOGGING_TAG, "Endianess Swapped Data: %s", pBuff);
    #endif
}
K33SensorError_t K33Sensor::readCO2Data(uint16_t &data){
    //Per the communications guide 2_15, data is MSB first, so swapEndianess required.
    uint16_t tempCO2Data = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempCO2Data), 2, K33_RAM_ADDR_CO2_DATA, false, true);
    if (readResultError == K33_NO_ERROR){
        
        if (tempCO2Data>K33_MAX_VAL){
            K33DriverEventTracker(K33_VALUE_ERROR);
            ESP_LOGW(LOGGING_TAG, "K33 Reports CO2 Raw Value =  %u", tempCO2Data);
            uint8_t K33StatusInfo = 0;
            getErrorStatus(K33StatusInfo);
            #if LOG_K33_EXTENDED_ERROR_INFO
            if (K33StatusInfo){
                uint8_t errDecoded[200];
                decodeK33ErrorStatusToString(K33StatusInfo, errDecoded, sizeof(errDecoded));
                ESP_LOGE(LOGGING_TAG, "Error Status\n  %s",errDecoded);
            }
            #endif
            return K33_VALUE_RANGE_WARNING;
        }else{
            data = tempCO2Data;
            return K33_NO_ERROR;
        }
    }

    return readResultError;
}


K33SensorError_t K33Sensor::getErrorStatus(uint8_t &StatusErrors){
    uint8_t tempStatus = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempStatus), 1, K33_RAM_ADDR_ERR_DATA, false, false);
    if (readResultError == K33_NO_ERROR){
        StatusErrors = tempStatus;
        return K33_NO_ERROR;
    }
    
    return readResultError;
}

K33DriverStats K33Sensor::getDriverErrorStats (){
    return sensorStats;
}

K33SensorError_t K33Sensor::getSensorInfo(K33VersionInfo &SensorInfo) {
    uint8_t fwBuffer[3] = {0};

    auto checkError = [](K33SensorError_t error) -> K33SensorError_t {
        return (error != K33_NO_ERROR) ? error : K33_NO_ERROR;
    };

    K33SensorError_t anyError;
    
    if ((anyError = checkError(getSerialNumber(SensorInfo.SNUMBER))) != K33_NO_ERROR) return anyError;
    if ((anyError = checkError(getFWInfo(fwBuffer))) != K33_NO_ERROR) return anyError;
    if ((anyError = checkError(getMemMapID(SensorInfo.MMAPID))) != K33_NO_ERROR) return anyError;
    if ((anyError = checkError(getSensorTypeID(SensorInfo.TYPEID))) != K33_NO_ERROR) return anyError;
    if ((anyError = checkError(get_CAL_BCC(SensorInfo.CAL_BCC))) != K33_NO_ERROR) return anyError;
    if ((anyError = checkError(get_CAL_Zero(SensorInfo.CAL_ZERO))) != K33_NO_ERROR) return anyError;
    if ((anyError = checkError(get_CAL_ZeroTrim(SensorInfo.CAL_TRIM))) != K33_NO_ERROR) return anyError;

    SensorInfo.FMVER_T = fwBuffer[0];
    SensorInfo.FWVER_M = fwBuffer[1];
    SensorInfo.FWVER_S = fwBuffer[2];
    
    return K33_NO_ERROR;
}

K33SensorError_t K33Sensor::getSensorTypeID(uint32_t &SensorTypeId){
    //Per the communications guide 2_15, data is MSB first, so swapEndianess required.
    uint32_t tempID = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempID), 3, K33_RAM_ADDR_ID_TYPE, false, true);
    if (readResultError == K33_NO_ERROR){
            SensorTypeId = tempID;
            return K33_NO_ERROR;
    }

    return readResultError;
}

K33SensorError_t K33Sensor::getSerialNumber(uint32_t &SensorSerialNum){
    uint32_t tempSN = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempSN), 4, K33_RAM_ADDR_ID_SNUM, false, true);
    if (readResultError == K33_NO_ERROR){
            SensorSerialNum = tempSN;
            return K33_NO_ERROR;
    }
    
    return readResultError;
}

K33SensorError_t K33Sensor::getMemMapID(uint8_t &MemMapID){
    uint8_t tempMapID = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempMapID), 1, K33_RAM_ADDR_ID_MMAP, false, false);
    if (readResultError == K33_NO_ERROR){
            MemMapID = tempMapID;
            return K33_NO_ERROR;
    }
    
    return readResultError;
}

K33SensorError_t K33Sensor::getMeterCtrl(uint8_t &MeterCtrl){
    uint8_t tempMeterCtrl = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempMeterCtrl), 1, K33_EEPROM_ADDR_METERCTRL, true, false);
    if (readResultError == K33_NO_ERROR){
            MeterCtrl = tempMeterCtrl;
            return K33_NO_ERROR;
    }
    
    return readResultError;
}

K33SensorError_t K33Sensor::setMeterCtrl(uint8_t &MeterCtrl){
    K33SensorError_t writeResultError = writeBytesToK33(reinterpret_cast<uint8_t*>(&MeterCtrl), 1, K33_EEPROM_ADDR_METERCTRL, true, false);
    if (writeResultError == K33_NO_ERROR){
        return K33_NO_ERROR;
    }

    return writeResultError;
}

K33SensorError_t K33Sensor::disableABC_Temporary(){
    //In this case the period is LSB first as per the communications guide 2_15, so no endianess swap is required.
    uint16_t ABCPeriod = 0;
    K33SensorError_t writeResultError = writeBytesToK33(reinterpret_cast<uint8_t*>(&ABCPeriod), 2, K33_RAM_ADDR_ABC_PERIOD, false, false);
    if (writeResultError == K33_NO_ERROR){
        return K33_NO_ERROR;
    }

    return writeResultError;
}

K33SensorError_t K33Sensor::setABC_Period(uint16_t &ABCPeriod){
    //In this case the period is LSB first as per the communications guide 2_15, so no endianess swap is required.
    K33SensorError_t writeResultError = writeBytesToK33(reinterpret_cast<uint8_t*>(&ABCPeriod), 2, K33_RAM_ADDR_ABC_PERIOD, false, false);
    if (writeResultError == K33_NO_ERROR){
        return K33_NO_ERROR;
    }

    return writeResultError;
}

K33SensorError_t K33Sensor::enableABC_Persistent(){
    uint8_t meterCtrlReg =0;
    K33SensorError_t anyError = getMeterCtrl(meterCtrlReg);
    if (anyError == K33_NO_ERROR){
        meterCtrlReg &= 0b11111101;
        anyError = setMeterCtrl(meterCtrlReg);
    }
    
    return anyError;
}

K33SensorError_t K33Sensor::disableABC_Persistent(){
    uint8_t meterCtrlReg =0;
    K33SensorError_t anyError = getMeterCtrl(meterCtrlReg);
    if (anyError == K33_NO_ERROR){
        meterCtrlReg |= 0b00000010;
        anyError = setMeterCtrl(meterCtrlReg);
    }

    return anyError;
}

K33SensorError_t K33Sensor::getFWInfo(uint8_t *fwInfo){
    //Per the communications guide 2_15, three unsigned 8bit integers, so no endianess swapping
    uint8_t tempFWInfo[3] = {0};
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempFWInfo), 3, K33_RAM_ADDR_FW_TYPE, false, false);
    if (readResultError == K33_NO_ERROR){
        fwInfo[0] = tempFWInfo[0];
        fwInfo[1] = tempFWInfo[1];
        fwInfo[2] = tempFWInfo[2];
        
        return K33_NO_ERROR;
    }
    
    return readResultError;
}

K33SensorError_t K33Sensor::update_ZeroTrim_ZERO(){
    return K33_DRIVER_FEATURE_NOT_IMPLEMENTED;
    /*
        Algorithm from I2C Comms Guide
            
            Verify no trend in "OLD" Register, noise is fine, trend is not
            Read "OLD", "Zero"
            Calculate "ZeroTrim" as
                ZeroTrim = ((2048 * 61440) / "OLD") - "Zero"
                Write to EEPROM
                Wait 100mSec
                Power Cycle Sensor
    */

}

K33SensorError_t K33Sensor::update_ZeroTrim_BG(){
    return K33_DRIVER_FEATURE_NOT_IMPLEMENTED;
    /*
        Algorithm from I2C Comms Guide

            Provide Fresh Air (400PPM CO2)
            Verify no trend in "OLD" Register, noise is fine, trend is not
            Read "OLD", "Zero", "BCC"
            Calculate "ZeroTrim" as
                ZeroTrim = ((2048 * "BCC") / "OLD") - "Zero"
                Write to EEPROM
                Wait 100mSec
                Power Cycle Sensor
    */

}

K33SensorError_t K33Sensor::start_FACT_CAL(bool isZeroCal) {

    uint8_t K33_SPC_START_BG[2] = K33_SPECIAL_CMD_CAL_BG;
    uint8_t K33_SPC_START_ZERO[2] = K33_SPECIAL_CMD_CAL_ZERO;

    //Per the communications guide 2_15, data is MSB first, so swapEndianess required.
    const uint8_t* SPC = isZeroCal ? K33_SPC_START_ZERO : K33_SPC_START_BG;

    uint16_t CommandRegLocation = K33_RAM_ADDR_SPECIAL_CMD_1;
    uint8_t sensorMemMapID = 0;
    K33SensorError_t getMapError = getMemMapID(sensorMemMapID);
    ESP_LOGI(LOGGING_TAG, "K33 Memory map ID: %d", sensorMemMapID);
    if (sensorMemMapID>8){
        CommandRegLocation = K33_RAM_ADDR_SPECIAL_CMD_2;
    }
    ESP_LOGI(LOGGING_TAG, "Issueing command to: 0x%X", CommandRegLocation);
    ESP_LOGI(LOGGING_TAG, "Calibration Command bytes to issue: 0x%X%X", SPC[0], SPC[1]);
    K33SensorError_t readResultError = writeBytesToK33(const_cast<uint8_t*>(SPC), 2, CommandRegLocation, false, true);
    if (readResultError == K33_NO_ERROR) {
        return K33_NO_ERROR;
    }

    return readResultError;
}

K33SensorError_t K33Sensor::get_CAL_OLD(uint16_t &CAL_OLD) {
    //Per the communications guide 2_15, data is MSB first, so swapEndianess required.
    uint16_t tempCALOld = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempCALOld), 2, K33_RAM_ADDR_CAL_OLD, false, true);
    if (readResultError == K33_NO_ERROR){
        CAL_OLD = tempCALOld;
        return K33_NO_ERROR;
    }

    return readResultError;
}

K33SensorError_t K33Sensor::get_CAL_Zero(uint16_t &ZeroVal){
    //Per the communications guide 2_15, data is MSB first, so swapEndianess required.
    uint16_t tempZeroVal = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempZeroVal), 2, K33_EEPROM_ADDR_CAL_ZERO, true, true);
    if (readResultError == K33_NO_ERROR){
        ZeroVal = tempZeroVal;
        return K33_NO_ERROR;
    }

    return readResultError;
}

K33SensorError_t K33Sensor::get_CAL_ZeroTrim(int16_t &ZeroTrimVal){
    //Per the communications guide 2_15, data is MSB first, so swapEndianess required.
    int16_t tempZeroTrimVal = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempZeroTrimVal), 2, K33_EEPROM_ADDR_CAL_ZTRIM, true, true);
    if (readResultError == K33_NO_ERROR){
        ZeroTrimVal = tempZeroTrimVal;
        return K33_NO_ERROR;
    }

    return readResultError;
}

K33SensorError_t K33Sensor::set_CAL_ZeroTrim(int16_t &ZeroTrimVal){
    //Per the communications guide 2_15, data is MSB first, so swapEndianess required.
    K33SensorError_t writeResultError = writeBytesToK33(reinterpret_cast<uint8_t*>(&ZeroTrimVal), 2, K33_EEPROM_ADDR_CAL_ZTRIM, true, true);
    if (writeResultError == K33_NO_ERROR){
        return K33_NO_ERROR;
    }

    return writeResultError;
}

K33SensorError_t K33Sensor::get_CAL_BCC(uint16_t &BCCVal){
    //Per the communications guide 2_15, data is MSB first, so swapEndianess required.
    uint16_t tempBCCVal = 0;
    K33SensorError_t readResultError = readBytesFromK33(reinterpret_cast<uint8_t*>(&tempBCCVal), 2, K33_EEPROM_ADDR_CAL_BCC, true, true);
    if (readResultError == K33_NO_ERROR){
        BCCVal = tempBCCVal;
        return K33_NO_ERROR;
    }

    return readResultError;
}

void K33Sensor::K33DriverEventTracker(K33SensorError_t error_type){
    switch(error_type){
        case(K33_I2C_ERROR):
            #if LOG_K33_ERRORS
            ESP_LOGE(LOGGING_TAG, "I2C Error");
            #endif
            this->sensorStats.K33_I2CERRS++;
        break;
        case(K33_TIMEOUT_ERROR):
            #if LOG_K33_ERRORS
            ESP_LOGE(LOGGING_TAG, "Timeout Error");
            #endif
            this->sensorStats.K33_TIMERRS++;
        break;
        case(K33_CHECKSUM_ERROR):
            #if LOG_K33_ERRORS
            ESP_LOGE(LOGGING_TAG, "Checksum Error");
            #endif
            this->sensorStats.K33_CHKERRS++;
        break;
        case(K33_COMMAND_ERROR):
            #if LOG_K33_ERRORS
            ESP_LOGE(LOGGING_TAG, "Command Error");
            #endif
            this->sensorStats.K33_CMDERRS++;
        break;
        case(K33_VALUE_ERROR):
            #if LOG_K33_ERRORS
            ESP_LOGE(LOGGING_TAG, "Value Error");
            #endif        
            this->sensorStats.K33_VALERRS++;
        break;
        case(K33_DRIVER_NOINIT_ERROR):
            #if LOG_K33_ERRORS
            ESP_LOGE(LOGGING_TAG, "Driver Not Initialized");
            #endif
        break;
        case(K33_DRIVER_INVALID_PARAMETERS):
            #if LOG_K33_ERRORS
            ESP_LOGE(LOGGING_TAG, "Invalid Parameters Specified");
            #endif
        break;
        case(K33_NO_ERROR):
            this->sensorStats.K33_OKCOUNT++;
        break;
        default:
            #if LOG_K33_ERRORS
            ESP_LOGE(LOGGING_TAG, "?Unspecified Error?");
            #endif
        break;
    }
}

void K33Sensor::dumpSensorInfoToString(K33VersionInfo &SensorInfo, uint8_t *strBuffer, uint16_t strBufferSize){
    
    if (strBufferSize > 200){
        uint16_t offset = 0;

        offset += sprintf((char*)strBuffer + offset, "  SenseAir K33 CO2 Sensor Information:\r\n");

        offset += sprintf((char*)strBuffer + offset, "    SensorType: %u    Firmware Info: Type = %u, Version = %u.%u\r\n", SensorInfo.TYPEID,
                            SensorInfo.FMVER_T, SensorInfo.FWVER_M, SensorInfo.FWVER_S);
        
        offset += sprintf((char*)strBuffer + offset, "    Memory Map ID: 0x%x    Serial #: 0x%X\r\n", SensorInfo.MMAPID, SensorInfo.SNUMBER);

        offset += sprintf((char*)strBuffer + offset, "    Calibration info: ZERO: %u, BCC: %u, TRIM: %d\r\n", SensorInfo.CAL_ZERO, SensorInfo.CAL_BCC, SensorInfo.CAL_TRIM);

    }
}

void K33Sensor::decodeK33ErrorStatusToString(uint8_t &statusErrors, uint8_t *strBuffer, uint16_t strBufferSize) {
    if (strBufferSize < 200) { // Ensure there's enough space in the buffer
        return;
    }
    uint16_t offset = 0;

    offset += sprintf((char*)strBuffer + offset, "  <START K33 ERROR STATUS INFORMATION>\n");

    if (statusErrors & (1 << 7)) {
        offset += sprintf((char*)strBuffer + offset, "    Bit 7: Space Temp out of range (N/A)\n");
    }
    if (statusErrors & (1 << 6)) {
        offset += sprintf((char*)strBuffer + offset, "    Bit 6: Memory Error\n");
    }
    if (statusErrors & (1 << 5)) {
        offset += sprintf((char*)strBuffer + offset, "    Bit 5: CO2 Out of Range\n");
    }
    if (statusErrors & (1 << 4)) {
        offset += sprintf((char*)strBuffer + offset, "    Bit 4: Detector Temp Out of range\n");
    }
    // Bit 3 is blank
    if (statusErrors & (1 << 2)) {
        offset += sprintf((char*)strBuffer + offset, "    Bit 2: Temp/Humid Sensor Comm Error\n");
    }
    if (statusErrors & (1 << 1)) {
        offset += sprintf((char*)strBuffer + offset, "    Bit 1: CO2 Measurement Error (Offset Regulation)\n");
    }
    if (statusErrors & (1 << 0)) {
        offset += sprintf((char*)strBuffer + offset, "    Bit 0: Fatal Error\n");
    }

    offset += sprintf((char*)strBuffer + offset, "  <END K33 ERROR STATUS INFORMATION>\n");
}

/*** end of file ***/