/** @file main.hpp
 * 
 * @brief An integrated CO2, Temperature, Humidity wireless sensor node platform based on the ESP32-C3.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for Polysense Solutions Inc., by David Stewart.  All rights reserved.
 */ 
#include <Preferences.h>

// Function declarations
void saveVariablesToNVS();
void loadVariablesFromNVS();
void reconnectUSB();

#ifndef __MAIN_HPP
#define __MAIN_HPP

//Includes
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <soc/rtc_wdt.h>
#include <soc/rtc_cntl_reg.h>
#include "esp_sleep.h"
#include "esp_system.h"

#include <Wire.h>
#include "DRIVERS/K33_SENSOR.hpp"
#include "DRIVERS/MKL62BA_DRIVER.hpp"

#include "CONFIG/NODE_CONFIG.h"
#include "PLATFORM_PKT_TYPES.h"

#include "UTILS/DBG_INTERFACE_CMDS.hpp"

#include <HardwareSerial.h>
#include <stdint.h>



#include <Preferences.h>
#include <SensirionI2cSht4x.h>
 
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "UTILS/RTC_SysStat.hpp"
#include "TASK/RTC_SCHEDULER.hpp"

//PMU Task
//#include "TASK/PMUTask.hpp"

#if MANU_WIFI_MAC_INFO
#include "UTILS/NODE_WIFI.hpp"
char MAC_STRING[20];
#endif

#if MANU_ESP_CHIP_INFO
#include "UTILS/ESP_INFO.hpp"
char ESP_INFO_STRING[100];
#endif

/*
    Serial port configuration for ESP_LOG outputs
*/
#if USE_DEBUG_SERIAL_SHIM
#include "UTILS/DualPrintShim.hpp" // DualPrintShim automatically outputs Logs to both ports
//On ESP32 C3 with -D ARDUINO_USB_CDC_ON_BOOT=1 set in platformIO, 'Serial' refers to HWCDC (USB Serial)
DualPrintShim cereal(Serial0, Serial);
#endif

#if USE_USB_SERIAL_CONN_TASK
#include "UTILS/USB_CONN.hpp"
#endif

#if !USE_DEBUG_SERIAL_SHIM
HardwareSerial cereal(0);
#endif


/*
    Data Structures & Enums exclusively used in main.cpp
*/

enum TX_PKT_TYPE{
    MSG_FACTORY_RESET,
    MSG_PRE_ZERO_CAL,
    MSG_PRE_BACK_CAL,
    MSG_POST_ZERO_CAL,
    MSG_POST_BACK_CAL,
    MSG_BACK_FAILED,
    MSG_ZERO_FAILED,
    MSG_DEV_INFO,
    MSG_NODE_DATA
};


//Task Loops
void CO2_Task(void *pvParameters);
void TempHumid_Task(void *pvParameters);
void Tx_Task(void *pvParameters);
void Sensor_WD_Task(void *pvParameters);

#if USE_USB_DEBUG_INTERFACE
void DBGSerialTask(void *parameter);
void processCommand(String command);
void printMemoryInfo();
#endif

//Task Initializers
bool initSHT40Task();
bool initCO2Task();
bool initTxTask();

void taskInitPhase1();
void taskInitPhase2();

void dumpSystemStatsToSerial();

bool isVeryFirstBoot();
void startCO2Sensor();

void prepPinsForSleep(void);

void serialInitPhase1();
void taskInitPhase1(void);
void EarlyPinInit(void);

void getManufacturingInfo();
void factoryReset();

void earlyPowerPinInit();

void setupADC1();
uint16_t readBatteryVoltageMillivolts();

void K33_WaitAndCalibrate(bool isZero);
void updateLastBootInfo(); //Update last boot reason and last boot battery voltage
bool Tx_Aux_Packet(TX_PKT_TYPE mType);
size_t dataToHexString(void *data, size_t dataSize, char *hexString, size_t hexStringSize);

void updateRTC_K33ErrorCounts(K33SensorError_t errK33);
void deepSleepForever();

void sendDeviceStats();
void initToZero(void *struct_ptr, size_t struct_size);

/*      Debug related things, should be removed from shipped product!       */



/*
    Global variables
*/
//Used to store specific node configuration information in a non-volatile way
Preferences nvStorage;

// System Runtime information, placed in RTC memory with NOINIT Attribute to preserve status across deep sleep wakes
RTC_SysStat rtcSysStat;


uint32_t batt_mVolts_AtBoot;

//ADC Parameters
esp_adc_cal_characteristics_t *adc_chars;
const adc_bits_width_t width = ADC_WIDTH_BIT_12;
const adc_atten_t atten = ADC_ATTEN_DB_12;
const adc_unit_t unit = ADC_UNIT_1;


//Main I2C bus, used for K33 and SHT40
TwoWire K33_BUS = TwoWire(0);
 
SensirionI2cSht4x SHT40_Sensor;


static char errorMessage[64];
static int16_t error;
// Global variables

#if LORA_PACKET_BUFFER_IS_HEX
char TxBuffer[LORA_HEX_BUFFER_SIZE] = {0};
uint16_t txBuffLen = 0;
#endif

volatile uint64_t SHT40_SampleReadyBy = 0;
volatile uint64_t K33_SAMPLE_READY_BY = 0;
volatile uint64_t LAST_BOOT_RTC_TIME = 0;

SemaphoreHandle_t mtx_I2C;
SemaphoreHandle_t mtx_radioInUseLockout;
SemaphoreHandle_t mtx_dataReady_SHT40;
SemaphoreHandle_t mtx_dataReady_K33;

//TaskHandle_t taskHandle_PMU; // Defined in PMUTask.hpp
TaskHandle_t taskHandle_K33CO2 = NULL;
TaskHandle_t taskHandle_SHT40 = NULL;
TaskHandle_t taskHandle_Tx = NULL;
TaskHandle_t taskHandle_RTCScheduler = NULL;

#if USE_USB_DEBUG_INTERFACE
void ParseCMDInputToCMDBuffer(String input);
TaskHandle_t serialTaskHandle = NULL;
volatile bool DBG_MANUAL_MODE_ACTIVE = false;
char DBG_cmdInputBuffer[200] = {0};
#endif

HardwareSerial LoRaSerial(1); // UART1 handles LoRaWAN radio connection.
MKL62BA_DRIVER LoRa_Radio(LoRaSerial, LORA_TX, LORA_RX, LORA_BAUD);

K33Sensor CO2_Sensor(K33_I2C_ADR, &K33_BUS);

//Storage of average sensor values.  Not much point in moving average on something with an ultra-low duty cycle
static volatile float averageCO2 = 0;
static volatile float averageTemperature = 0;
static volatile float averageHumidity = 0;

volatile bool K33IsDone = false;

bool USB_WasConnected = false;
void sleepDeep10WithUSBRestore();
RTC_DATA_ATTR bool resumingFromDeepSleep = false;


#endif

/*** end of file ***/