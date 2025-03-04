/** @file K33_SENSOR.hpp
 * 
 * @brief A comprehensive I2C driver for the Sensair K33 CO2 Sensor 
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */ 


/*
 *  Regarding K33 Sensor CO2 values:
 *      A value that wraps back to 65K is the result of an integer underflow and indicates that the sensor
 *      is likely not calibrated correctly.  Since there seems to only be a way to modify the trim value via
 *      the provided Calibration functions, it will likely be necessary to calibrate it such that the maximum
 *      accuracy is achieved around the target operational CO2 PPM.
*/



/*  DATA Endianess WARNING!
  
    ESP32 stores data such as a uint16_t in Little Endian format.
    Example, suppose we store a uint16_t of value 0x1234 at memory location 0x32323200

        Memory location             Content             Note
        0x32323200                  0x34                LSB
        0x32323201                  0X12                MSB

    When we write to I2C where a uint16_t has been reinterpret_cast<uint8_t*>, we would send the LSB first!

    To help deal with this problem, a private swapEndianess(uint8_t *buffer, size_t buffLen) function has been provided.
        (See also, LOG_DEBUG_ENDIANESS macro definition below)
*/


#ifndef K33_SENSOR_HPP
#define K33_SENSOR_HPP

#include "../CONFIG/NODE_CONFIG.h"
#include <stdint.h>
#include <Wire.h>
#include "../CONFIG/NODE_CONFIG.h"
#include "K33_SENSOR_CMDS.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define the SensorState enum
enum K33SensorState {
    K33_OFF,
    K33_UNINIT,
    K33_WARMUP,
    K33_ACTIVE
};
enum K33SensorError_t {
    K33_NO_ERROR,
    K33_I2C_ERROR,
    K33_TIMEOUT_ERROR,
    K33_CHECKSUM_ERROR,
    K33_COMMAND_ERROR,
    K33_VALUE_ERROR,
    K33_VALUE_RANGE_WARNING,
    K33_DRIVER_NOINIT_ERROR,
    K33_DRIVER_INVALID_PARAMETERS,
    K33_DRIVER_FEATURE_NOT_IMPLEMENTED
};

typedef struct {
    uint32_t K33_OKCOUNT;
    uint32_t K33_TIMERRS;
    uint32_t K33_CHKERRS;
    uint32_t K33_VALERRS;
    uint32_t K33_CMDERRS;
    uint32_t K33_I2CERRS;
}K33DriverStats;

typedef struct {
    uint32_t TYPEID= 0;
    uint32_t SNUMBER = 0;
    uint8_t MMAPID = 0;
    uint8_t FMVER_T = 0;
    uint8_t FWVER_M = 0;
    uint8_t FWVER_S = 0;
    uint16_t CAL_ZERO = 0;
    int16_t CAL_TRIM = 0;
    uint16_t CAL_BCC = 0;
} K33VersionInfo;

typedef struct{
    uint8_t FWTYPE = 0;
    uint8_t FWMAIN = 0;
    uint8_t FWSUB = 0;
} K33FWInfo;

/*  Since behaviour is not well defined in datasheet, using a photodiode and transimpedance amplifier
      and Sampling 20x power on till first measurement pulse yields the following:
        Average of 2983351 us to first measurement from poweron
        Average Offset for readData 945932 us from measurement pulses
*/

// Time to wait after PowerOn before beginning data collection 
#define K33_WARMUP_WAIT (2983       /*Time till first measurement pulse*/                + \
                         945        /*Offset to the middle of measurement period*/       )//+ \
                         //2000       /*Wait for second mesurement*/                       + \
                         //-1200       /*Trim for power optimization*/                        )

// Since sensor is of type ICB 10%, a value beyond 12000 is likely erroneous (From UIP5 observations it seems to cap out at 12k)
#define K33_MAX_VAL 12000

//Time to wait before polling CO2 Again
#define K33_POLLING_INTERVAL 2000

// Time to delay after a command fails before attempting retry
#define K33_RETRY_DELAY 2000

//Time to delay after issuing a command for processing
#define K33_CMD_ISSUE_DELAY 20

//Maximum time per I2C transaction
#define K33_TRANSACTION_TIMEOUT 120


class K33Sensor {
public:
    K33Sensor(uint8_t address = K33_I2C_ADR, TwoWire *wireObject = &Wire);

    /*
        Normal operation functions
    */
    K33SensorError_t readCO2Data(uint16_t &co2_value);              //Get CO2 Data
    K33DriverStats getDriverErrorStats();                           //Get CO2 Driver Error Stats (Internal counters, no communication required)
    K33SensorError_t disableABC_Temporary();                        //Disable ABC (Writes 0 to ABC Period in RAM)
    K33SensorError_t setABC_Period(uint16_t &ABCPeriod);            //Set ABC Period (Temporary)
    K33SensorError_t getErrorStatus(uint8_t &StatusErrors);         //Get contents of RAM 0x1E, status information & errors
    
    /*Decode the above Error Status information into a string buffer*/
    void decodeK33ErrorStatusToString(uint8_t &statusErrors, \
                              uint8_t *strBuffer, uint16_t strBufferSize);
    
    void setup(uint8_t powerPin);            //Resets State Data / Sets Power Pin & Serial output
    void wake();                                                    //(UNUSED)Sends wake 'command' if sensor was sleeping
    void powerOff();                                                //Toggles Power Off via GPIO
    void powerOn();                                                 //Toggles Power ON via GPIO
    void warmup();                                                  //Waits until its ok to talk to sensor



    /*
        Utility Functions
    */
    //Configuration updaters
    K33SensorError_t disableABC_Persistent();                       //Disable ABC (Writes to EEPROM)
    K33SensorError_t enableABC_Persistent();                        //Enable ABC (Writes to EEPROM)
    
    //Sensor Information Functions
    K33SensorError_t getSensorTypeID(uint32_t &SensorTypeId);        //Get Sensor Type ID, 3 Chars
    K33SensorError_t getSerialNumber(uint32_t &SensorSerialNum);    //Get Sensor Serial Number, uint32
    K33SensorError_t getMemMapID(uint8_t &MemMapID);                //Get Sensor Memory Map, uint8
    K33SensorError_t getFWInfo(uint8_t *fwInfo);                    //Get Sensor FW information
    K33SensorError_t getMeterCtrl(uint8_t &MeterCtrl);              //MeterCtrl register used for configuration
    
    K33SensorError_t getSensorInfo(K33VersionInfo &SensorInfo);     //Get all sensor information in one struct
    
    /*Decode the above Sensor information into a string buffer*/
    void dumpSensorInfoToString(K33VersionInfo &SensorInfo, \
                              uint8_t *strBuffer, uint16_t strBufferSize);
    
    //Calibration data Getters
    K33SensorError_t get_CAL_OLD(uint16_t &OldVal);                 //Gets 'OLD' value from RAM 0x06
    K33SensorError_t get_CAL_Zero(uint16_t &ZeroVal);               //Gets 'Zero' value from EEPROM 0x38
    K33SensorError_t get_CAL_ZeroTrim(int16_t &ZeroTrimVal);       //Gets 'ZeroTrim' value from EEPROM 0x48
    K33SensorError_t get_CAL_BCC(uint16_t &BCCVal);                 //Gets 'BCC' value from EEPROM 0x3C

    //Direct Calibration Data Setter - For Development only!
    K33SensorError_t set_CAL_ZeroTrim(int16_t &ZeroTrimVal);       //Sets 'ZeroTrim' value in EEPROM 0x48

    //Factory Calibration functions
    K33SensorError_t start_FACT_CAL(bool isZeroCal);                //Initiate factory calibration (Set isZeroCal to false for Background (400ppm), true for Zero Point (0 ppm))
    
    //NOT YET IMPLEMENTED
    K33SensorError_t update_ZeroTrim_ZERO();                        //Updates ZeroTrim/Zero As per I2C Comm Guide 2_15
    K33SensorError_t update_ZeroTrim_BG();                          //Updates ZeroTrim/BCC As per I2C Comm Guide 2_15
    
private:

    /*
        Communications Functions
    */
    void delay_until_next_window();            //Delays communications to stick to the middle of the non-sampling period of the K33
    
    void issueCmd(uint8_t tarAddr, uint8_t* cmdBytes, uint8_t cmdSize);
    void requestData(uint8_t tarAddr, uint8_t* rxBuff, uint8_t reqSize);
    K33SensorError_t readBytesFromK33(uint8_t *data, uint8_t numBytes, uint16_t memAddress, bool fromEEPROM, bool swapByteOrder);
    K33SensorError_t writeBytesToK33(uint8_t *data, uint8_t numBytes, uint16_t memAddress, bool toEEPROM, bool swapByteOrder);
    
    //Utility functions to support configuration options.
    K33SensorError_t setMeterCtrl(uint8_t &MeterCtrl);
    //K33SensorError_t set_CAL_ZeroTrim(int16_t &ZeroTrimVal);       //Sets 'ZeroTrim' value in EEPROM 0x48
                    
    //I2C Interaction tracker for errors
    void K33DriverEventTracker(K33SensorError_t error_type);

    //Endianess Swap for data streams
    void swapEndianess(uint8_t *buffer, size_t buffLen);
    //Driver State Variables
    K33SensorState state;                   // Power/Init State
    uint8_t i2cAddress;                     // I2C Address
    uint8_t powerPin;                       // Power Pin
    TwoWire *wire;                          // I2C Bus Instance to use
    K33DriverStats sensorStats;             // Error counters
    uint32_t PowerOnTime;                   // Timestamp of Sensor Power On

    bool sensorLibInit;                     // Are the state variables reset?
    
    portMUX_TYPE K33_Comms_Mux = portMUX_INITIALIZER_UNLOCKED;
};

#endif

/*** end of file ***/