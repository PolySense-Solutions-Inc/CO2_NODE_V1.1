/** @file PLATFORM_PKT_TYPES.h
 * 
 * @brief Data structures of the various LoRaWAN packet data types for the Polysense Solutions ESP32 Sensor Node Platform.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */ 

#ifndef __PLATFORM_PKT_TYPES
#define __PLATFORM_PKT_TYPES
#include <stdint.h>

enum PacketID{
    PKT_CAL = 0xCA,
    PKT_DAT = 0xDA,
    PKT_INF = 0xF0
};
/*
    Data Packet
    Uses attribute packed to enforce smaller size.  Should be 21 bytes!
*/
/*typedef struct __attribute__((packed)){
    uint8_t PKT_Type;                   //Should be PKT_DAT
    // Run time metadata from Current Sampling Period!
    uint32_t K33_VAL :  4;              // K33 Invalid value errors                 (15 Max)
    uint32_t I2C_ERR :  4;              // I2C Bus Errors                           (15 Max)
    uint32_t K33_CHK : 3;               // K33 Checksum errors                      (7 Max)
    uint32_t Curr_BootReason :4 ;       // Current Bootup Reason                    (<10)
    uint32_t Last_BootReason :4 ;       // Previous Bootup Reason                   (<10)
    uint32_t CRC_MISMATCH_FLAG: 1;      // RTC CRC Mismatched @ Boot
    uint32_t Wake_Count : 12;           // Wakeup counter, should increment w/ rollover @ 28.4 days
    
    //Battery Measurements
    uint16_t Curr_Batt_mVolts;  // Current Battery (mVolts)
    uint16_t Prev_Batt_mVolts;  // Previous Bootup Battery (mVolts)

    //Actual data from sensors
    float filteredTemp;
    float filteredHumidity;
    float filteredCO2;
}Node_Data_pkt;*/


/*
    Data Packet
    Uses attribute packed to enforce smaller size.  Should be 21 bytes!
*/
typedef struct __attribute__((packed)){
    uint8_t PKT_Type;                   //Should be PKT_DAT
    // Run time metadata from Current Sampling Period!
    uint8_t K33_VAL;                   // K33 Invalid value errors
    uint8_t I2C_ERR;                   // I2C Bus Errors
    uint8_t K33_CHK;                   // K33 Checksum errors
    uint8_t Curr_BootReason :4 ;       // Current Bootup Reason                    (<10)
    uint8_t Last_BootReason :4 ;       // Previous Bootup Reason                   (<10)
    uint16_t Wake_Count : 15;          // Wakeup counter, should increment w/ rollover @ 28.4 days
    uint16_t CRC_MISMATCH_FLAG: 1;     // RTC CRC Mismatched @ Boot
    //Battery Measurements
    uint16_t Curr_Batt_mVolts;  // Current Battery (mVolts)
    uint16_t Prev_Batt_mVolts;  // Previous Bootup Battery (mVolts)

    //Actual data from sensors
    float filteredTemp;
    float filteredHumidity;
    float filteredCO2;
}Node_Data_pkt;

/*
    System Status Packets
*/
typedef struct __attribute__((packed)){
    uint8_t PKT_Type;                  //Should be PKT_INF
    
    uint8_t Curr_BootReason :4 ;       // Current Bootup Reason                     (<10)
    uint8_t Last_BootReason :4 ;       // Previous Bootup Reason                    (<10)
    uint8_t CRC_MISMATCH_FLAG: 1;      // RTC CRC Mismatched @ Boot

    uint32_t K33_VAL;               // K33 Invalid value errors                     (Full count)
    uint32_t I2C_ERR;               // I2C Bus Errors                               (Full count)
    uint32_t I2C_REINITS;           // I2C Bus Restarts (Contention Issues)         (Full count)
    
    // ESP32 Reset Reason Counters
    uint32_t RST_SW;                // Software Resets                              (Full count)
    uint32_t RST_PANIC;             // Panic Resets                                 (Full count)
    uint32_t RST_ALL_WDT;           // All WDT Resets                               (Full count)
    uint32_t RST_DEEPSLEEP;         // Deep Sleep Resumptions                       (Full count)
    uint32_t RST_BROWNOUT;          // Brownout Resets                              (Full count)
    
} __attribute__((packed)) Node_Info_pkt;


typedef struct {
    uint8_t PKT_Type;                    //Should be PKT_CAL
    
    uint8_t CAL_ZERO_ARMED :1;
    uint8_t CAL_ZERO_DONE :1;
    uint8_t CAL_ZERO_OK :1;
    uint8_t CAL_BACK_ARMED :1;
    uint8_t CAL_BACK_DONE :1;
    uint8_t CAL_BACK_OK :1;
    uint8_t FACT_RESET :1;
    uint8_t UNUSED_1 :1;
    
    uint16_t Curr_Batt_mVolts;
    uint16_t CAL_Zero;
    uint16_t CAL_BCC;
    int16_t CAL_Trim;
    
    uint16_t CO2_Readings[5];
    
    float filteredTemp;
    float filteredHumidity;

} Node_Cal_pkt;

#endif

/*end of file*/