/** @file K33_SENSOR_CMDS.hpp
 * 
 * @brief Memory addresses and Command Sequence bytes for the K33 CO2 Sensor Driver
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */ 

#ifndef K33_CMD_DEFINITIONS_hpp
#define K33_CMD_DEFINITIONS_hpp

/*
    Command bytes
*/
#define K33_GET_RAM_CMD             0x20
#define K33_SET_RAM_CMD             0x10
#define K33_GET_EEPROM_CMD          0x40
#define K33_SET_EEPROM_CMD          0x30

#define K33_SPECIAL_CMD_CAL_BG      {0x7C,0x07}
#define K33_SPECIAL_CMD_CAL_ZERO    {0x7C,0x06}

/*
    Memory addresses in RAM
*/
#define K33_RAM_ADDR_CAL_OLD        0x0006
#define K33_RAM_ADDR_CO2_DATA       0x0008

#define K33_RAM_ADDR_ERR_DATA       0x001E
/*  Error Data
    Bit     Description
    7       Space Temp out of range (N/A)
    6       Memory Error
    5       CO2 Out of Range
    4       Detector Temp Out of range
    3       (BLANK)
    2       Temp/Humid Sensor Comm Error
    1       CO2 Measurement Error (Offset Regulation)
    0       Fatal Error
*/

#define K33_RAM_ADDR_ID_TYPE        0x002C
#define K33_RAM_ADDR_ID_SNUM        0x0028
#define K33_RAM_ADDR_ID_MMAP        0x002F

#define K33_RAM_ADDR_ABC_PERIOD     0x0040

#define K33_RAM_ADDR_FW_TYPE        0x0062
#define K33_RAM_ADDR_FW_MAIN        0x0063
#define K33_RAM_ADDR_FW_SUB         0x0064


//K33-ICB Meters with memory maps <= 8
#define K33_RAM_ADDR_SPECIAL_CMD_1   0x0067

//K33-ICB Meters with memory maps > 8
#define K33_RAM_ADDR_SPECIAL_CMD_2   0x0032

/*
    Memory addresses in EEPROM
*/
#define K33_EEPROM_ADDR_I2C_ADR     0x0000
#define K33_EEPROM_ADDR_METERCTRL   0x0003
#define K33_EEPROM_ADDR_CAL_ZERO    0x0038
#define K33_EEPROM_ADDR_CAL_BCC     0x003C
#define K33_EEPROM_ADDR_CAL_ZTRIM   0x0048



/*
    Untested and probably not supported.  (Documentation is ambiguous and mentions to this functionality are scattered.)
    (From CO2-Engine-BLG_ELG configuration guide Rev 1_02.docx)
*/
#define K33_CMD_GET_STATUS_OTHER    {0x21, 0x00, 0x1D, 0x3E}        //Get Meter Status (Jumperstate @b7, Force measurement @b6, Single Measurement @b5)
#define K33_CMD_FORCE_STOP          {0x11, 0x00, 0x60, 0x30, 0xA1}  //Stop CO2 Measurements
#define K33_CMD_FORCE_START         {0x11, 0x00, 0x60, 0x31, 0xA2}  //Start CO2 Measurements
#define K33_CMD_START_SINGLE        {0x11, 0x00, 0x60, 0x35, 0xA6}  //Start a Single CO2 Measurement


#endif /*K33_CMD_DEFINITIONS_hpp*/

/*** end of file ***/