/** @file MKL62BA_DRIVER.cpp
 * 
 * @brief A driver module for the MKL62BA LoRaWAN radio module.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */ 
#ifndef MKL62BA_DRIVER_HPP
#define MKL62BA_DRIVER_HPP

#include <Arduino.h>
#include <HardwareSerial.h>
#include "MKL62BA_AT_COMMANDS.hpp"
#include "../PLATFORM_PKT_TYPES.h"
#include <cstring>

//Pulls in the APPEUI from this
#include "../CONFIG/NODE_CONFIG.h"


#define MAX_MKL62BA_COMMAND_LENGTH 256
#define MAX_MKL62BA_RESPONSE_LENGTH 512 + 32 +1

enum MKL62_BaudRate {
    BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
    BAUD_57600 = 57600,
    BAUD_115200 = 115200
};

enum LoraCommand {
    LORA_INIT,
    LORA_TXPKT,
    LORA_PING
};

enum SerialCommandType{
    AT_CMD,
    AT_GET,
    AT_SET
};

class MKL62BA_DRIVER {
public:
    MKL62BA_DRIVER(HardwareSerial &serial, int txPin, int rxPin, MKL62_BaudRate baudRate);
    void begin();
    void sendCommand(const char *command, SerialCommandType cmdType, const char * setValue);
    bool sendDataHex(char *dataBuffer, uint16_t dataLen);
    void readResponse(char *response, size_t maxLength, const char *respid, uint32_t timeout, bool isolateResponse);
    //void readResponse(char *response, size_t maxLength, const char *respid);
    bool sendAndReceive(const char *command, char *response, const char * respid, size_t maxLength, SerialCommandType cmdType, const char * setValue, uint32_t timeout);
    bool wrappedSendAndReceive(const char* cmd, char* respBuffer, const char* respId, size_t maxLength, SerialCommandType atCmdType, const char* setValue, uint32_t timeout, const char* errorMsg);
    //bool sendAndReceive(const char *command, char *response, const char * respid, size_t maxLength, SerialCommandType cmdType, const char * setValue);
    void initializeModule();
    void getModuleInformation(char *devEUI, char *appEUI, char *appKEY, char *region, char *version, uint16_t buffSize);
    bool connect();
    void resetChip();
    bool sendDataPacket(char *dataPacket, uint16_t dataLen);
    bool networkConnected();
    bool joinNetwork();
    bool isInitialized();

private:
    
    void clearBuffer(void* buffPtr, uint16_t buffSize);
    bool updateMTU();
    void clearRxBuffer();
    HardwareSerial &loRaSerial;
    int txPin;
    int rxPin;
    MKL62_BaudRate baudRate;
    bool responseACKorOK(const char *respBuffer);

    char cmdBuffer[LORA_HEX_BUFFER_SIZE];  //Holds a formatted string to send over UART
    uint16_t cmdBufferSize;                //Actual size of string data in buffer to send
    char respBuffer[LORA_HEX_BUFFER_SIZE]; //Holds responses from the modem received over UART
    uint16_t respBufferSize;               //Actual size of string data recieved
    bool isReady;
    uint8_t sz_mtu; // Maximum transmissable data packet size

    static bool NetworkIsConnected;

    const static uint32_t MKL62_MAX_TIME_OUT = 15000;
};

#endif // MKL62BA_DRIVER_HPP

/*** end of file ***/