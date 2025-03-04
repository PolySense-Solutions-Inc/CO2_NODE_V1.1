/** @file NODE_CONFIG.h
 * 
 * @brief Polysense Sensor Node ESP32 Platform Configuration File
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */ 

#ifndef __BOARD_DEFINITIONS
#define __BOARD_DEFINITIONS
#include <stdint.h>


/*
*   Debug & Development Specific Options
*/
#define USE_DEBUG_SERIAL_SHIM 1       //Use the doublePrintShim to send log info to both USB and GPIO8.  (NOT FOR PRODUCTION USE)
#define USE_USB_SERIAL_CONN_TASK 1    //Use a task to track USB CDC connection status and auto-update DualPrintShim
#define USE_USB_DEBUG_INTERFACE 1     //Enable interactive USB Debugging via Command / Response Serial interface


#define EXPERIMENTAL_NAP_AFTER_TX 1

#define KEEP_K33_ON_IF_USB_CONNECTED 0

#define LORA_FAKE_CONNECTION 0        //Pretend to be connected to a LoRa network, also pretend to send packets
#define BOARD_IS_HW_2_PROTO_MONSTER 0 //Relates to Battery Voltage ADC resistor values and Power Control for reading it.
#define EXTENDED_USB_STARTUP_DELAY 0  //Wait a bit longer during startups if USB is connected to more easily capture output messages
#define SEND_FACT_RESET_LORA_MSG  0   //Send a lora message when a factory reset is triggered

/*  Since the K33 has an internal filtering method they refer to as "Dynamical frac", which seems to be similar to an Exponential Moving Average
    it is likely unneccesary to sample repeatedly and perform our own moving average on the sampled values.
    Thus the below K33_WAIT_AND_SAMPLE if set to 1, allows the sensor to run its own sampling and use its own filtering.
*/
//#define K33_WAIT_AND_SAMPLE 1         // K33 is powered on and is sampled only once at time = K33_WARMUP_WAIT + K33_POLLING_INTERVAL * AVERAGING_FILTER_LENGTH_K33 

/*
    Logging Options   -- Currently somewhat broken: Fighting with PlatformIO and ESP Logging to allow selectivity per TAG...
*/ 
#define LOG_PMU_EVENTS 0              //Enable Logging of PMU Event Information
#define LOG_PMU_REQUEST_RESULTS 0     //Enable Logging of PMU Request Responses

#define LOG_LORA_INFO 1               //Enable Logging of LoRa UART Information
#define LOG_RTC_CRC_UPDATES 0         //Enable Logging of RTC System Status CRC Updates


/*  K33 Logging options*/
/**/#define LOG_K33_ERRORS 1              //Enable in-driver logging of K33 Errors
/**/#define LOG_K33_DEBUG_ENDIANESS 0     //Provide detailed information to ESP_LOG when swapping endianess
/**/#define LOG_K33_TIMING_INFO 0         //Enable K33 Driver Sensor Timing info Logging
/**/#define LOG_K33_EXTENDED_ERROR_INFO 0 //Log extended info if a value error occurs
/**/#define LOG_K33_TRANSACTION_DATA 0

/*
  Board Version Information & Boot Message
*/
#define HW_VERSION "2.01"
#define FW_VERSION "1.0"
#define BOOT_MESSAGE_BASE "\r\n\nCO2 Sensor Node HW "
#define FW_PREFIX ", FW "
#define BY_POLOGO ", By Polysense Solutions Inc. (2025)\r\n\n"
#define CONCATENATE5(P1, P2, P3, P4, P5) P1 P2 P3 P4 P5
#define BOOT_MESSAGE CONCATENATE5(BOOT_MESSAGE_BASE, HW_VERSION, FW_PREFIX, FW_VERSION, BY_POLOGO)

/*
  Power / Performance & Activity Interval Settings
*/
#define CPU_RUN_FREQUENCY_MHZ 10    // Cpu clock to run at while sampling and sending data.
#define SLEEP_TIME_NORMAL   5       //Length of time in MINUTES for a LONG sleep period.  (Normal Operation)

#define NAP_TIME      1             //Length of time in MINUTES for a SHORT sleep period

/*
  K33 Calibration Related
*/

const int16_t K33_SRT_OFFSET = 0;    // Short run time value offset for measured CO2 Value

#define DONT_ACTUALLY_CALIBRATE 1    //Set to 1 to not actually calibrate (for software development)
#define MANU_BG_CAL_DELAY 2          //Background Calibration Start Delay, in MINUTES
#define MANU_ZERO_CAL_DELAY 5        //Zero Calibration Start Delay, in MINUTES

#define MIN_KEY0_BG_CAL 2000          //Minimum time to hold KEY0 at boot to start a BG calibration, in MILLISECONDS
#define MAX_KEY0_BG_CAL 6000          //Maximum time to hold KEY0 at boot to start a BG calibration, in MILLISECONDS

#define MIN_KEY0_Z_CAL  10000          //Minimum time to hold KEY0 at boot to start a Zero calibration, in MILLISECONDS
#define MAX_KEY0_Z_CAL  15000         //Maximum time to hold KEY0 at boot to start a Zero calibration, in MILLISECONDS

/*
  Manufacturing Testing & Information
*/
#define MANU_ESP_CHIP_INFO 1          //Get ESP32 Chip info
#define MANU_WIFI_MAC_INFO 1          //Get WiFi MAC of ESP32 (Like a Serial Number)

//Extended Manufacturing Tests
#define MANU_TEST_K33 1               //Sample the K33 5 times and output the results during the test
#define MANU_TEST_SHT40 1             //Sample the Temp/Humid 5 times and output the results during the test
#define MANU_TEST_RADIO 0             //Send a test packet over LORA during test  (NOT YET IMPLEMENTED)

/*
*   LoRa Radio Settings
*/
//#define NODE_LORA_APPEUI "01:23:45:67:89:AB:CD:EF"
#define NODE_LORA_APPEUI            "00:00:00:00:00:00:00:00"
//#define NODE_LORA_APPKEY_OVERRIDE   "2B:7E:15:16:28:AE:D2:A6:AB:F7:15:88:09:CF:4F:3C" //Default from AT command datasheet
#define NODE_LORA_CLASS_OVERRIDE    "A"   //Class A is Device Initiated communications only.


//#define NODE_LORA_DEVEUI_OVERRIDE "00:00:DE:AD:BE:EF:00:00" //If a specific DEVEUI is desired, enter it here
#define NODE_LORA_TX_POWER_OVERRIDE "0"                       //If a specific TX power is desired, enter it here
//#define NODE_LORA_REGION_OVERRIDE "12"                      //If a specific TX Region is desired, enter it here
#define NODE_LORA_CH_RANGE          "8-15"                    //If a specific channel range is required, enter it here

#define NODE_LORA_AUTOJOIN_ACTIVE


//  If not using AUTO DR, be sure to set the datarate override below!
//#define NODE_LORA_AUTODR_ACTIVE           //If 1, enables automatic data rate.  If 0, use DATARATE_OVERRIDE
#define NODE_LORA_AUTODR_ACTIVE 0
#define NODE_LORA_DATARATE_OVERRIDE "3"   //Corresponds to a data packet length of 53

#define LORA_TX_PORT 12
#define LORA_TX_RETRIES 4


//*********************************     TODO - Verify this is needed
#define LORA_PKT_OVERHEAD_BUFFER 0     //Allows room for header information, assuming its not already accounted for
//*********************************

//Size of Hexified data string to send
#define LORA_HEX_BUFFER_SIZE 512 + 1 + 32        //256 Bytes + 1 Null Terminator + 32 Bytes for command string

//If data must be sent as HEXADECIMAL
#define LORA_PACKET_BUFFER_IS_HEX 1

//If direct sending of binary data as bytes is possible
//#define LORA_PACKET_BUFFER_IS_BYTES 0


/*
*   Device performance metadata
*     (Packet TX / RX + Processing not implemented yet)
*/
//#define REPORT_DEVICE_STATS_EVERY 144    //Send device status information packet every ## wakes from sleep, set to 0 to disable.

/*
  IO Settings: GPIO / I2C / PWR / UART
*/
#define GPIO8_TP ((const uint8_t)8)     //Ground @ boot for factory reset.
#define STATUS_LED ((const uint8_t)9)   //Output only.

// I2C
#define I2C_CLOCK_HZ 100000
#define I2C_SDA ((const uint8_t)4)
#define I2C_SCL ((const uint8_t)10)
#define I2C_PULLUPS ((const uint8_t)5) //Ensure ALT3V3 is on as well, since the BUS will be pulled down by the SHT40 without it!

// K33 CO2 Sensor
#define K33_I2C_ADR ((const uint8_t)0x68)
#define K33_PWR ((const uint8_t)1)

// LoRaWan Radio (MKL62BA)
#define LORA_RX ((const uint8_t)20)
#define LORA_TX ((const uint8_t)21)
#define LORA_BAUD BAUD_9600

// Battery Voltage Sense
#define VBAT_ADC_IN ((const uint8_t)3)

// Key/Button Input 0
#define KEY0_IN ((const uint8_t)0)

//Power Resource Control
#define EN_3V3_ALT ((const uint8_t)7)
#define EN_5V_BOOST ((const uint8_t)6)


/*
  ADC / Battery Monitoring Parameters
*/
// V-Bat Voltage Divider Resistors
#if !BOARD_IS_HW_2_PROTO_MONSTER
const float VBAT_DIV_R1 = 1000000.0;  // 1000K Ohms // V2 Hardware
const float VBAT_DIV_R2 = 1400000.0;  // 1400K Ohms // V2 Hardware

#endif
#if BOARD_IS_HW_2_PROTO_MONSTER
const float VBAT_DIV_R1 = 1300000.0;  
const float VBAT_DIV_R2 = 1100000.0;
#endif

const int16_t VBAT_OFFSET = 0;          // Fixed offset to partially compensate for vdrop in circuit to adc input.
const float ADC_REF_VOLTAGE = 3300.0;   // ADC reference voltage in millivolts
const int ADC_RESOLUTION = 4095;        // ADC resolution (12 bits)



/*
  Sampling Counts & Intervals
*/
#define AVERAGING_FILTER_LENGTH_K33 6   // Count of K33 CO2 Sensor samples to average together
#define AVERAGING_FILTER_LENGTH_SHT40 3   // Count of SHT40 Temp & Humidity samples to average together

#define SHT_SAMPLING_DELAY_PERIOD 500    //How long to wait between SHT40 samples
//For K33 sampling interval see K33_SENSOR.hpp


// Macro definitions Required by SHT4x Library
#ifdef SHT40_NO_ERROR
#undef SHT40_NO_ERROR
#endif
#define SHT40_NO_ERROR 0

#define SHT40_HEATER_INTERVAL 5
#define SHT40_DECREEP_TMAX 120U


//For development, if the device gets a message within 2 seconds at startup it knows to delay longer
//so that we can update the firmware without the device going to sleep and USB jtag dropping asap.
//#define USB_BOOT_WAIT_STR "USB_BOOT_WAIT"
//#define USB_MSG_TIMEOUT_MS 2000   // Timeout of 2 seconds
//#define USB_BOOT_DELAY_TIME_MS 30000  // Delay of 30 seconds

//#define USB_CHECK_INTERVAL_MS 2000   // Interval to check the serial input (1 second)
//#define REBOOT_COMMAND "REBOOTME"

#endif

/*** end of file ***/