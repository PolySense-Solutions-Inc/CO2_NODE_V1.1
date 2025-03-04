/** @file MKL62BA_DRIVER.hpp
 * 
 * @brief A driver module for the MKL62BA LoRaWAN radio module.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */ 

#include "MKL62BA_DRIVER.hpp"
#include "../UTILS/USB_CONN.hpp"
#include "driver/uart.h"

#define MKL62BA_TX_SLEEP_TIMEOUT 50 * 1000   //Timeout of sleep before attempting retransmission or giveup (milliseconds)

extern SemaphoreHandle_t mtx_radioInUseLockout;
static const char* LOGGING_TAG = "MKL62BA";

//Error Response codes
const char* errorResps[] = {
        "ERROR(-1)", // AT Command Error
        "ERROR(-2)", // AT Parameter Error
        "ERROR(-3)", // Device Busy
        "ERROR(-5)", // No Network Joined
        "ERROR(-7)"  // Timeout
    };

//Error Response meanings
const char* errorMessages[] = {
        "AT Command Error",
        "AT Parameter Error",
        "Device Busy",
        "No Network Joined",
        "Timeout"
    };

// Initialize static member NetworkIsConnected variable to false
bool MKL62BA_DRIVER::NetworkIsConnected = false;

//These are used by the experimental sleep mode in sendhex
void enableSleepPowerPinHolds(){
    //pinMode(STATUS_LED, OUTPUT);
    //digitalWrite(STATUS_LED, LOW);
    //gpio_hold_en((gpio_num_t)STATUS_LED);

    gpio_hold_en((gpio_num_t)I2C_PULLUPS);
    gpio_hold_en((gpio_num_t)K33_PWR);
    gpio_hold_en((gpio_num_t)EN_3V3_ALT);
    gpio_hold_en((gpio_num_t)EN_5V_BOOST);
}
void disableSleepPowerPinHolds(){
    gpio_hold_dis((gpio_num_t)I2C_PULLUPS);
    gpio_hold_dis((gpio_num_t)K33_PWR);
    gpio_hold_dis((gpio_num_t)EN_3V3_ALT);
    gpio_hold_dis((gpio_num_t)EN_5V_BOOST);
    
    //gpio_hold_dis((gpio_num_t)STATUS_LED);
    //pinMode(STATUS_LED, OUTPUT);
    //digitalWrite(STATUS_LED, HIGH);
    
}

// Helper function to trim leading and trailing whitespace
void strTrimWhiteSpaces(const char *src, char *dest, size_t dest_size) {
    // Skip leading whitespace
    while (isspace((unsigned char)*src)) {
        src++;
    }

    // Copy the trimmed part to the destination string, limited by dest_size
    size_t len = strlen(src);
    if (len >= dest_size) {
        len = dest_size - 1;
    }
    strncpy(dest, src, len);
    dest[len] = '\0';

    // Trim trailing whitespace
    char *end = dest + strlen(dest) - 1;
    while (end > dest && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
}

int saferAtoi(const char *inputString) {
    if (inputString == NULL || *inputString == '\0') {
        //ESP_LOGI("SAFERATOI", "NULL address or string");
        return 0; // Return 0 for null or empty string
    }

    //ESP_LOGI("SAFERATOI", "Input: %s", inputString);

    // Ensure the input is within the buffer size limit
    if (strlen(inputString) >= 30) {
        return 0; // Return 0 if the input string is too long
    }

    // Trim the whitespace from the input string
    char trimmedString[30];
    strTrimWhiteSpaces(inputString, trimmedString, sizeof(trimmedString));
    //ESP_LOGI("SAFERATOI", "After whitespace trim: %s", trimmedString);

    // Check if the trimmed string is empty
    if (*trimmedString == '\0') {
        //ESP_LOGI("SAFERATOI", "Empty string after trimming spaces");
        return 0; // Return 0 if the string was only whitespace
    }

    // Check each character to ensure the string is a valid integer representation
    const char *ptr = trimmedString;
    if (*ptr == '-' || *ptr == '+') { // Allow an optional leading sign
        ptr++;
    }

    while (*ptr != '\0') { // Loop until the null terminator
        if (!isdigit((unsigned char)*ptr)) { // If any character is not a digit
            //ESP_LOGI("SAFERATOI", "Non-digit encountered!");
            return 0; // Return 0 on encountering non-digit characters
        }
        ptr++;
    }

    // If the string is valid, use atoi to convert and return the result
    return atoi(trimmedString);
}


MKL62BA_DRIVER::MKL62BA_DRIVER(HardwareSerial &serial, int txPin, int rxPin, MKL62_BaudRate baudRate)
    : loRaSerial(serial), txPin(txPin), rxPin(rxPin), baudRate(baudRate), isReady(false), sz_mtu(0),
    cmdBufferSize(0), respBufferSize(0){
}

void MKL62BA_DRIVER::begin() {
    
    //Broken in PlatformIO ESP32 Arduino...
    //if (!LOG_LORA_INFO) esp_log_level_set(LOGGING_TAG, ESP_LOG_NONE);

    //This init is called in main.c to speed things up as much as possible
    //loRaSerial.begin(static_cast<uint32_t>(baudRate), SERIAL_8N1, rxPin, txPin);

    isInitialized();

    clearBuffer(cmdBuffer, LORA_HEX_BUFFER_SIZE);
    clearBuffer(respBuffer, LORA_HEX_BUFFER_SIZE);
    cmdBufferSize= 0;
    respBufferSize = 0;
    
}

void MKL62BA_DRIVER::sendCommand(const char *command, SerialCommandType cmdType, const char * setValue) {
    clearRxBuffer();
    char cmdBuffer[LORA_HEX_BUFFER_SIZE + 24] = {0};
    uint16_t bufPos = 0;
    bufPos += snprintf(cmdBuffer, sizeof(cmdBuffer), "\r\n%s", command);
    
    if (cmdType == AT_GET) {
        bufPos += snprintf(cmdBuffer + bufPos, sizeof(cmdBuffer) - bufPos, "=?");
    } else if (cmdType == AT_SET) {
        bufPos += snprintf(cmdBuffer + bufPos, sizeof(cmdBuffer) - bufPos, "=");
        bufPos += snprintf(cmdBuffer + bufPos, sizeof(cmdBuffer) - bufPos, "%s", setValue);
    }

    loRaSerial.println(cmdBuffer);
    loRaSerial.flush();
    
    #if LOG_LORA_INFO
    if (bufPos < sizeof(cmdBuffer)) {
        ESP_LOGI(LOGGING_TAG, "Sent to MKL62BA: %s", cmdBuffer);
    } else {
        ESP_LOGW(LOGGING_TAG, "Command buffer was truncated: %s", cmdBuffer);
    }
    #endif
}

bool MKL62BA_DRIVER::sendDataHex(char *dataBuffer, uint16_t dataLen) {
//Experimental sleep mode puts the esp32 into light sleep with UART wakeup while waiting for the response.

    clearRxBuffer(); //Clear serial rx buffer
    if (dataLen >sz_mtu){
        ESP_LOGE(LOGGING_TAG, "Data to send exceeds maximum packet size allowed! (%u> %u)", dataLen, sz_mtu);
        return false;
    }

    
    clearBuffer(cmdBuffer, LORA_HEX_BUFFER_SIZE);
    cmdBufferSize = 0;
    respBufferSize = 0;
    clearBuffer(respBuffer, LORA_HEX_BUFFER_SIZE);

    uint16_t bufPos = 0;
    bufPos += sprintf(cmdBuffer, AT_CMD_SEND_HEX);
    bufPos += sprintf(cmdBuffer + bufPos, "=%u:%u:%s",LORA_TX_RETRIES, LORA_TX_PORT, dataBuffer);
    cmdBufferSize = bufPos;

    #if LORA_FAKE_CONNECTION
    ESP_LOGD("MKL62BA", "Would have sent: %s", cmdBuffer);
    return true;
    #endif

    #if !LORA_FAKE_CONNECTION
        #if LOG_LORA_INFO
        ESP_LOGI(LOGGING_TAG, "Sent to MKL62BA: %s", cmdBuffer);
        #endif    
        loRaSerial.println(cmdBuffer);
        loRaSerial.flush();
        
        #if EXPERIMENTAL_NAP_AFTER_TX
        //xSemaphoreTake(mtx_radioInUseLockout, portMAX_DELAY);
        uint8_t wake_thresh = 3;//In general, 3 pulses is enough to trash the RECVB: part of the response
        uart_set_wakeup_threshold(UART_NUM_1, wake_thresh);
        gpio_sleep_set_direction((gpio_num_t)LORA_RX, GPIO_MODE_INPUT);
        esp_sleep_enable_uart_wakeup(1);  // UART1 corresponds to the serial object `LoRaSerial`
        #endif
        readResponse(respBuffer, LORA_HEX_BUFFER_SIZE,dataBuffer, MKL62_MAX_TIME_OUT,true);
        readResponse(respBuffer, LORA_HEX_BUFFER_SIZE,"OK", MKL62_MAX_TIME_OUT,true);
        
        #if EXPERIMENTAL_NAP_AFTER_TX
     
        if (!USB_CDC_Connected()){   
            
            
            enableSleepPowerPinHolds();                 
            ESP_LOGI(LOGGING_TAG, "Napping while we wait...");
            //esp_sleep_enable_timer_wakeup(7000 *1000);
            esp_sleep_enable_timer_wakeup(MKL62BA_TX_SLEEP_TIMEOUT *1000);
            
            esp_light_sleep_start();
            readResponse(respBuffer, LORA_HEX_BUFFER_SIZE,":ACK", MKL62_MAX_TIME_OUT, false);

            disableSleepPowerPinHolds();
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_UART);
            //xSemaphoreGive(mtx_radioInUseLockout);
        }else{
            ESP_LOGI(LOGGING_TAG, "USB Connected, NOT Napping while we wait...");
            delayMicroseconds(4500000);
            readResponse(respBuffer, LORA_HEX_BUFFER_SIZE,RESPID_RECV_HEX, MKL62_MAX_TIME_OUT, true);
        }
        ESP_LOGI(LOGGING_TAG, "Woke UP!");
        #endif
        #if !EXPERIMENTAL_NAP_AFTER_TX
        vTaskDelay(pdMS_TO_TICKS(25));
        readResponse(respBuffer, LORA_HEX_BUFFER_SIZE,RESPID_RECV_HEX, MKL62_MAX_TIME_OUT);
        #endif

        #if LOG_LORA_INFO
        ESP_LOGI(LOGGING_TAG, "Recieved from MKL62BA: %s", respBuffer);
        #endif 
   
    //WARNING Since we wake with uart toggling, that first character is corrupted and might be a null!
    //respBuffer[0] = 0x32; //Space character
    if ((respBuffer[0] != 0x00) && responseACKorOK(respBuffer)){
        ESP_LOGI(LOGGING_TAG, "DBG  Transmit ok!");
        return true;
    }else{
        ESP_LOGI(LOGGING_TAG, "DBG  Transmit failed!");
        return false;
    }   
    #endif

}

//Search the response buffer for an ACK or OK
bool MKL62BA_DRIVER::responseACKorOK(const char *respBuffer) {
    // Check if "OK" or "ACK" is found anywhere in the respBuffer

    if (strstr(respBuffer, "OK") != NULL || strstr(respBuffer, "ACK") != NULL) {
        return true;  // Success
    } else {
        return false; // Failure
    }
}


void MKL62BA_DRIVER::readResponse(char *response, size_t maxLength, const char *respid, uint32_t timeout = MKL62_MAX_TIME_OUT, bool isolateResponse = true) {
    
#define MAX_NEW_LINES 18

uint32_t responseTimeout = 0;
if (timeout <= MKL62_MAX_TIME_OUT){
     responseTimeout = timeout;
}else responseTimeout = MKL62_MAX_TIME_OUT;

    size_t index = 0;
    size_t newlineCount = 0;
    size_t respIdLength = strlen(respid);
    char buffer[maxLength];
    bool idFound = false;
    
    const int numErrors = sizeof(errorResps) / sizeof(errorResps[0]);

    #if LOG_LORA_INFO
    char logBuffer[LORA_HEX_BUFFER_SIZE] = {0};
    uint16_t logBufferIndex = 0;
    #endif

    uint32_t startTime = millis();
    // Read available serial data until we accumulate MAX_NEW_LINES, find the desired response id, or until the timeout is reached.
    while (newlineCount < MAX_NEW_LINES && millis() - startTime <= responseTimeout) {
        while (loRaSerial.available()) {
            char c = loRaSerial.read();

            if (index < maxLength - 1) {
                buffer[index++] = c;
            }

            #if LOG_LORA_INFO
            if (logBufferIndex < sizeof(logBuffer) - 1) {
                logBuffer[logBufferIndex++] = c;
            } else {
                ESP_LOGW(LOGGING_TAG, "Log buffer overflow. Truncating response log.");
            }
            #endif

            if (c == '\n') {
                newlineCount++;
                
                // Null-terminate the buffer temporarily for strstr() function
                buffer[index] = '\0'; 

                // Check if the response ID is in the buffer up to this point
                if (strstr(buffer, respid) != nullptr) {
                    idFound = true;
                    #if LOG_LORA_INFO
                    ESP_LOGI(LOGGING_TAG, "Response ID found!");
                    #endif
                    
                    //If desired, Extract the response after the ID.
                    //Not isolating the response is useful if you want to check for an ACK or OK separately.
                    if (isolateResponse){
                        size_t matchedIndex = 0;
                        for (size_t i = 0; i < index; i++) {
                            if (buffer[i] == respid[matchedIndex]) {
                                matchedIndex++;
                                if (matchedIndex == respIdLength) {
                                    size_t responseIndex = 0;
                                    for (size_t j = i + 1; j < index && responseIndex < maxLength - 1; j++) {
                                        if (buffer[j] == '\n') {
                                            break;
                                        }
                                        response[responseIndex++] = buffer[j];
                                    }
                                    response[responseIndex] = '\0';
                                    respBufferSize = responseIndex;
                                    break;
                                }
                            } else {
                                matchedIndex = 0;
                            }
                        }
                    }else{
                        //Not isolating response so just copy whole response to buffer.
                        strncpy(response, buffer , index);
                    }
                    break;
                }
            }

            // Check timeout condition
            if (millis() - startTime >= responseTimeout) {
                #if LOG_LORA_INFO
                ESP_LOGI(LOGGING_TAG, "Response timeout!");
                #endif
                break;
            }
        }

        if (idFound) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    buffer[index] = '\0'; // Null-terminate the buffer

    #if LOG_LORA_INFO
    logBuffer[logBufferIndex] = '\0';
    ESP_LOGI(LOGGING_TAG, "MKL62BA Full Response: \r\n%s", logBuffer);
    #endif

    // If no ID found, check for error responses
    if (!idFound) {
        for (int k = 0; k < numErrors; k++) {
            if (strstr(buffer, errorResps[k]) != nullptr) {
                strncpy(response, errorMessages[k], maxLength);
                respBufferSize = strlen(response);
                ESP_LOGE(LOGGING_TAG, "Error detected: %s", response);
                idFound = true;
                break;
            }
        }
    }

    // If no match is found at all
    if (!idFound) {
        ESP_LOGE(LOGGING_TAG, "No match found for '%s'. Response: %s", respid, response);
        strncpy(response, "ERROR READING RESPONSE", maxLength);
        respBufferSize = strlen(response);
    }
}




void MKL62BA_DRIVER::clearRxBuffer(){
    uint32_t startTime = millis();
    //Assumes 5 ms of no activity is enough time to say its cleared.
    while (loRaSerial.available() && millis()-startTime<5) {
        if (loRaSerial.available()){
            loRaSerial.read();
            startTime = millis();
        }
    }
}
bool MKL62BA_DRIVER::sendAndReceive(const char *command, char *response, const char * respid, size_t maxLength, SerialCommandType cmdType, const char * setValue, uint32_t timeout = MKL62_MAX_TIME_OUT ) {
    clearRxBuffer(); //Clear the serial input buffer
    clearBuffer(response, maxLength); //Clear the buffer where responses are assembled from incoming serial data
    
    sendCommand(command, cmdType, setValue);
    readResponse(response, maxLength,respid, timeout, true);
    
    // If string is not empty, sending a command was a success
    // WARNING: ***Actual success depends on response contents!***
    return response[0] != 0x00;
}

//Used by wrappedSendAndRecieve to indicate a critical error occured!
void flashErrorLED() {
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    for (int i = 0; i < 40; ++i) {
        gpio_set_level(GPIO_NUM_9, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO_NUM_9, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

bool MKL62BA_DRIVER::wrappedSendAndReceive(const char* cmd, char* respBuffer, const char* respId, size_t maxLength, SerialCommandType atCmdType, const char* setValue, uint32_t timeout, const char* errorMsg) {
    int attempts = 0;
    bool success = false;
    
    while (!success && attempts < 3) {
        sendAndReceive(cmd, respBuffer, respId, maxLength, atCmdType, setValue, timeout);
        success = (responseACKorOK(respBuffer) || (strstr(respBuffer, respId)!=NULL));
        if (setValue !=NULL) success |= (strstr(respBuffer, setValue)!=NULL);
        if (!success) {
            
            ESP_LOGE("LoRa Module", "%s (Attempt %d)", errorMsg, attempts + 1);
            attempts++;
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
    
    if (!success) {
        ESP_LOGE("LoRa Module", "Critical failure after 3 attempts to set parameter!");
        flashErrorLED();
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        ESP_LOGE("LoRa Module", "Deep sleeping now!");
        esp_deep_sleep_start();
    }
    
    return success;
}


void MKL62BA_DRIVER::initializeModule() {
    
    /*
        TODOS:
            -Verify responses and handle error cases.
    */

    clearBuffer(respBuffer, LORA_HEX_BUFFER_SIZE);
    respBufferSize = 0;

    clearRxBuffer();
    // Check communication
    loRaSerial.end();
    loRaSerial.begin(static_cast<uint32_t>(BAUD_9600), SERIAL_8N1, rxPin, txPin);
    bool Comm_9600 = wrappedSendAndReceive(AT_CMD_TEST, respBuffer, RESPID_TEST, sizeof(respBuffer), AT_CMD, NULL, 200, "Failed to get attention at 9600 Baud");
    bool Comm_57600  = false;
    if (!Comm_9600){
        ESP_LOGI("MKL Init", "Unable to communicate @ 9600 Baud, Trying 57600");
        loRaSerial.end();
        loRaSerial.begin(static_cast<uint32_t>(BAUD_57600), SERIAL_8N1, rxPin, txPin);
        Comm_57600 = wrappedSendAndReceive(AT_CMD_TEST, respBuffer, RESPID_TEST, sizeof(respBuffer), AT_CMD, NULL, 200,"Failed to get attention at 57600 Baud");
        if (!Comm_57600){
            ESP_LOGE("MKL Init", "Unable to communicate @ 57600 Baud either!  CRITICAL!");
            isReady = false;
            return;
        }else{
            ESP_LOGI("MKL Init", "Continuing @ 57600 Baud");    
        }
    }else{
        ESP_LOGI("MKL Init", "Continuing @ 9600 Baud");
    }

    //clearRxBuffer();

    
    // Factory reset
    wrappedSendAndReceive(AT_CMD_FACTORY_RESET, respBuffer, RESPID_FACTORY_RESET, sizeof(respBuffer), AT_CMD, NULL, 1000, "Failed to factory reset module.");
    vTaskDelay(pdMS_TO_TICKS(1000));
    clearRxBuffer();
    wrappedSendAndReceive(AT_CMD_TEST, respBuffer, RESPID_TEST, sizeof(respBuffer), AT_CMD, NULL, 300, "Failed second stage attention test.");
    
    //loRaSerial.begin(static_cast<uint32_t>(57600), SERIAL_8N1, rxPin, txPin);

    
    vTaskDelay(pdMS_TO_TICKS(100));
    wrappedSendAndReceive(AT_CMD_ECHO, respBuffer, RESPID_ECHO ,sizeof(respBuffer), AT_SET, "OFF", 2000, "Failed to set echo off.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #ifdef NODE_LORA_AUTOJOIN_ACTIVE
    wrappedSendAndReceive(AT_CMD_AUTO_JOIN, respBuffer, RESPID_AUTO_JOIN ,sizeof(respBuffer), AT_SET, "ON", 2000, "Failed to set AutoJoin on.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif

    #ifndef NODE_LORA_AUTOJOIN_ACTIVE
    wrappedSendAndReceive(AT_CMD_AUTO_JOIN, respBuffer, RESPID_AUTO_JOIN ,sizeof(respBuffer), AT_SET, "OFF", 2000, "Failed to set AutoJoin off.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif

    wrappedSendAndReceive(AT_CMD_CONFIRM, respBuffer, RESPID_CONFIRM, sizeof(respBuffer), AT_SET, "ON", 2000, "Failed to set confirm on.");
    vTaskDelay(pdMS_TO_TICKS(100));

    //Set DevEUI
    #ifdef NODE_LORA_DEVEUI_OVERRIDE
    wrappedSendAndReceive(AT_CMD_DEVEUI, respBuffer, RESPID_DEVEUI, sizeof(respBuffer), AT_SET, NODE_LORA_DEVEUI_OVERRIDE, 2000, "Failed to set DEVEUI override.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif
    
    //Set TX Power
    #ifdef NODE_LORA_TX_POWER_OVERRIDE
    wrappedSendAndReceive(AT_CMD_TX_POWER, respBuffer, RESPID_TX_POWER, sizeof(respBuffer), AT_SET, NODE_LORA_TX_POWER_OVERRIDE, 2000, "Failed to set TX Power override.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif
    
    //Set Region
    #ifdef NODE_LORA_REGION_OVERRIDE
    wrappedSendAndReceive(AT_CMD_REGION, respBuffer, RESPID_REGION, sizeof(respBuffer), AT_SET, NODE_LORA_REGION_OVERRIDE, 2000, "Failed to set region override.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif
    
    //Set Lora Device Class
    #ifdef NODE_LORA_CLASS_OVERRIDE
    wrappedSendAndReceive(AT_CMD_CLASS, respBuffer, RESPID_CLASS, sizeof(respBuffer), AT_SET, NODE_LORA_CLASS_OVERRIDE, 2000, "Failed to set Lora Class Override.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif
    //Set Channels
    wrappedSendAndReceive(AT_CMD_CHANNEL, respBuffer, RESPID_CHANNEL, sizeof(respBuffer), AT_SET, NODE_LORA_CH_RANGE, 2000, "Failed to set channels.");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    //Set AppEUI
    wrappedSendAndReceive(AT_CMD_APPEUI, respBuffer, RESPID_APPEUI, sizeof(respBuffer), AT_SET, NODE_LORA_APPEUI, 10000, "Failed to set AppEUI.");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    
    //Set AppKey for security!              (Not set in V4.0 Code provided, uses module defaults)
    #ifdef NODE_LORA_APPKEY_OVERRIDE
    wrappedSendAndReceive(AT_CMD_APPKEY, respBuffer, RESPID_APPKEY, sizeof(respBuffer), AT_SET, NODE_LORA_APPKEY_OVERRIDE, 2000, "Failed to set AppKEY.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif

    //Turn off BLE advertising
    wrappedSendAndReceive(AT_CMD_ADVI, respBuffer, RESPID_ADVI, sizeof(respBuffer), AT_SET, "0", 2000, "Failed to disable BLE advertising.");  
    vTaskDelay(pdMS_TO_TICKS(100));
    //TODO:  Add ADR disable setting
    #if NODE_LORA_AUTODR_ACTIVE
    wrappedSendAndReceive(AT_CMD_ADR, respBuffer, RESPID_ADR, sizeof(respBuffer), AT_SET, "ON", 2000, "Failed to enable Auto Data Rate.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif
    #if !NODE_LORA_AUTODR_ACTIVE
    wrappedSendAndReceive(AT_CMD_ADR, respBuffer, RESPID_ADR, sizeof(respBuffer), AT_SET, "OFF", 2000, "Failed to disable Auto Data Rate");
    vTaskDelay(pdMS_TO_TICKS(100));
    wrappedSendAndReceive(AT_CMD_DATA_RATE, respBuffer, RESPID_DATA_RATE, sizeof(respBuffer), AT_SET, NODE_LORA_DATARATE_OVERRIDE, 2000, "Failed to force data rate = 3.");
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif

    clearRxBuffer();
}

void MKL62BA_DRIVER::getModuleInformation(char *devEUI, char *appEUI, char *appKEY, char *region, char *version, uint16_t buffSize) {
    sendAndReceive(AT_CMD_DEVEUI, devEUI, RESPID_DEVEUI, buffSize, AT_GET, NULL,200);
    
    sendAndReceive(AT_CMD_APPEUI, appEUI, RESPID_APPEUI, buffSize, AT_GET, NULL,200);
    
    sendAndReceive(AT_CMD_APPKEY, appKEY, RESPID_APPKEY, buffSize, AT_GET, NULL,200);
    
    sendAndReceive(AT_CMD_REGION, region, RESPID_REGION, buffSize, AT_GET, NULL,200);
    
    sendAndReceive(AT_CMD_VERSION, version, RESPID_VERSION, buffSize, AT_GET, NULL,200);
    
}

bool MKL62BA_DRIVER::sendDataPacket(char* dataPacket, uint16_t dataLen){
    clearRxBuffer();
    bool connected = networkConnected();
    bool transmissible = (dataLen <(sz_mtu-LORA_PKT_OVERHEAD_BUFFER));
    bool result = false;
    if (connected && transmissible){
        result = sendDataHex(dataPacket, dataLen);
    }
    if(!connected){
        ESP_LOGW(LOGGING_TAG, "Warning - Can't send data when not connected!");
    }else if(!transmissible){
        ESP_LOGW(LOGGING_TAG, "Warning - Can't send data exceeding current MTU of %u", sz_mtu);
    }

    return result;
    
}




bool MKL62BA_DRIVER::updateMTU(){
    
    clearBuffer(respBuffer, LORA_HEX_BUFFER_SIZE);
    respBufferSize = 0;
    bool cmdSuccess = sendAndReceive(AT_CMD_TX_LEN, respBuffer,RESPID_TX_LEN, sizeof(respBuffer), AT_GET, NULL );
    

    if (cmdSuccess){
        uint8_t resp_sz_mtu = saferAtoi(respBuffer);
        if (resp_sz_mtu>0){
            #if LOG_LORA_INFO
            ESP_LOGI(LOGGING_TAG,"Radio reports %s as maximum transmissible data stream.", respBuffer);
            #endif
            sz_mtu = resp_sz_mtu;
        } else {
            //Must be an error or unconnected!
            ESP_LOGE(LOGGING_TAG,"[INVALID] Radio reports '%s' as maximum transmissible data stream.", respBuffer);
            sz_mtu = 0;
            NetworkIsConnected=false;
        }
    }
    return cmdSuccess;
}

bool MKL62BA_DRIVER::joinNetwork(){
 bool joinCMDResult = sendAndReceive(AT_CMD_JOIN_NETWORK, respBuffer,  RESPID_JOIN_NETWORK, sizeof(respBuffer), AT_GET, NULL, 10000);
    if(joinCMDResult){
        if (strstr(respBuffer, "JOINED")!=NULL){
            ESP_LOGI(LOGGING_TAG, "Direct network join suceeded.");
            NetworkIsConnected=true;
            updateMTU();
            
        }else{
            ESP_LOGI(LOGGING_TAG, "Direct network join failed.");
            NetworkIsConnected=false;
            sz_mtu = 0;
        }
    }
    return NetworkIsConnected;
}

bool MKL62BA_DRIVER::connect(){
    
    #if LORA_FAKE_CONNECTION
    return true;
    #endif

    #if !LORA_FAKE_CONNECTION
    clearBuffer(respBuffer, LORA_HEX_BUFFER_SIZE);
    
    //If autojoin is active, then we should just check the module's status, but if the status is "JOIN FAIL" we should try again
    #ifdef NODE_LORA_AUTOJOIN_ACTIVE
    bool joinCMDResult = sendAndReceive(AT_CMD_JOIN_STATUS, respBuffer,  RESPID_JOIN_STATUS, sizeof(respBuffer), AT_GET, NULL, 250);

    bool bJOINED = strstr(respBuffer, "JOINED")!=NULL;
    bool bJOINING = strstr(respBuffer, "JOINING")!=NULL;
    bool bFAILED = strstr(respBuffer, "JOIN FAIL")!=NULL;

    static uint32_t checkCount = 0;
    #if 0 //Old code that had some issues
    while (bJOINING && (!bJOINED || !bFAILED) && (checkCount<5)){
    if (!USB_CDC_Connected()){
        ESP_LOGI("MKL62BA Driver", "Light sleep while waiting on Join status...");
        //xSemaphoreTake(mtx_radioInUseLockout, portMAX_DELAY);
        enableSleepPowerPinHolds();
        esp_sleep_enable_timer_wakeup(2000 * 1000);
        esp_light_sleep_start();
        disableSleepPowerPinHolds();
        //xSemaphoreGive(mtx_radioInUseLockout);
    }else{
        ESP_LOGI("MKL62BA Driver", "Skipping light sleep since USB CDC is connected!");
        delayMicroseconds(2000 * 1000);
    }
        //vTaskDelay(pdMS_TO_TICKS(750));
        joinCMDResult = sendAndReceive(AT_CMD_JOIN_STATUS, respBuffer,  RESPID_JOIN_STATUS, sizeof(respBuffer), AT_GET, NULL, 200);
        bJOINED = strstr(respBuffer, "JOINED")!=NULL;
        bJOINING = strstr(respBuffer, "JOINING")!=NULL;
        bFAILED = strstr(respBuffer, "JOIN FAIL")!=NULL;
        checkCount++;
    }
    #endif

    if (bJOINED){
        ESP_LOGI(LOGGING_TAG, "LoRaWAN network connected!");
        NetworkIsConnected = true;
        updateMTU();

    }else if (bJOINING){
        NetworkIsConnected = false; 
        sz_mtu = 0;
        #if LOG_LORA_INFO
        ESP_LOGI(LOGGING_TAG, "LoRaWAN module still connecting!");
        #endif
    }else if (strstr(respBuffer, "JOIN FAIL")!=NULL){
        #if LOG_LORA_INFO
        ESP_LOGW(LOGGING_TAG, "Join failed, Trying to reset Modem");
        bool resetCommandResult = sendAndReceive(AT_CMD_RESET, respBuffer,  RESPID_RESET, sizeof(respBuffer), AT_CMD, NULL, 500);

        #endif
        sz_mtu = 0;
        NetworkIsConnected = false;
    }
    
    #endif

    #ifndef NODE_LORA_AUTOJOIN_ACTIVE
    bool joinCMDResult = sendAndReceive(AT_CMD_JOIN_NETWORK, respBuffer,  RESPID_JOIN_NETWORK, sizeof(respBuffer), AT_CMD, NULL, 10000);
    if(joinCMDResult){
        //ESP_LOGI(LOGGING_TAG, "\r\n\r\n<MKL62 Connect Response>\r\n\r\n%s\r\n", respBuffer);
        if (strstr(respBuffer, "JOINED")!=NULL){
            NetworkIsConnected = true;
            sz_mtu = 242; //Skipping LCR Stuff to try things
        }else{
            NetworkIsConnected = false;
            sz_mtu = 242;
        }
    }
    #endif
    
    clearBuffer(respBuffer, LORA_HEX_BUFFER_SIZE);

    return NetworkIsConnected;
    #endif
}

void MKL62BA_DRIVER::resetChip(){
    sendAndReceive(AT_CMD_RESET, respBuffer,  RESPID_NONE, sizeof(respBuffer), AT_CMD, NULL, 2000);
    vTaskDelay(pdMS_TO_TICKS(250));
    sendCommand(AT_CMD_TEST,AT_CMD, NULL);
    readResponse(respBuffer,LORA_HEX_BUFFER_SIZE ,"LORA REGION:", 300,true); //Expecting to see system start message with LORA REGION:
}
bool MKL62BA_DRIVER::networkConnected(){
    //ESP_LOGI(LOGGING_TAG, "Connected? %s",NetworkIsConnected? "YES":"NO");
    if (NetworkIsConnected) return true;

    bool connected = false;
    sz_mtu = 242; //Skipping LCR Stuff to try things
    
    #if !LORA_FAKE_CONNECTION
    clearBuffer(respBuffer, LORA_HEX_BUFFER_SIZE);
    if (sendAndReceive(AT_CMD_JOIN_STATUS, respBuffer,  RESPID_JOIN_STATUS, sizeof(respBuffer), AT_GET, NULL,500) && strstr(respBuffer, "JOINED")){
        ESP_LOGI(LOGGING_TAG, "Is joined to network.");
        connected = true;
        updateMTU();
    }else{
        //Problem with Join status command
        sz_mtu = 0;
        ESP_LOGI(LOGGING_TAG, "Is NOT joined to network.");
        connected = false;
    }
    NetworkIsConnected = connected;
    return connected;
    #endif

    #if LORA_FAKE_CONNECTION
    sz_mtu = 242;
    return true;
    #endif
} 

bool MKL62BA_DRIVER::isInitialized(){
    /*
    Bug notice:  There is something in the proceeding call stack that fails saying the response isn't detected even though it is in the response buffer.
    */
    if (!isReady){
        //Once powered on, it seems to wait for anything over serial before spamming back the boot info
        sendCommand(AT_CMD_TEST,AT_CMD, NULL);
        readResponse(respBuffer,LORA_HEX_BUFFER_SIZE ,"LORA REGION:", 500,true); //Expecting to see system start message with LORA REGION:
        isReady = responseACKorOK(respBuffer);
        clearRxBuffer();
    }
    return isReady;
}

void MKL62BA_DRIVER::clearBuffer(void* buffPtr, uint16_t buffSize){
    memset(buffPtr, 0, buffSize);
}
/*** end of file ***/