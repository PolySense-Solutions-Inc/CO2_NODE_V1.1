/** @file DBG_INTERFACE_CMDS.hpp
 * 
 * @brief Commands for a debug interface for development purposes.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */

#ifndef DBG_INTERFACE_CMDS
#define DBG_INTERFACE_CMDS

#define DBG_CMD_HELP                 "HELP"

// System Commands
#define DBG_CMD_SYS_REBOOT           "SYS REBOOT"
#define DBG_CMD_SYS_GET_BATT         "SYS VBAT?"
#define DBG_CMD_SYS_MEM_STATS        "SYS MEM?"
#define DBG_CMD_SYS_DUMP_RTC         "SYS RTC?"
#define DBG_CMD_SYS_RESET_RTC        "SYS !RTC"
#define DBG_CMD_SYS_FIRST_BOOT       "SYS CLEAR!"
//#define DBG_CMD_SYS_ENABLE_SLEEP     "SYS CANSLEEP=1"
//#define DBG_CMD_SYS_DISABLE_SLEEP    "SYS CANSLEEP=0"

#define DBG_CMD_SYS_EN_MANUAL_MODE      "``"
//To exit manual mode, reboot the system!


// PMU (Power Management Unit) Commands
#define DBG_CMD_PMU_5V_ON            "PMU BOOST=1"
#define DBG_CMD_PMU_5V_OFF           "PMU BOOST=0"
#define DBG_CMD_PMU_3V3_ON           "PMU ALT3V3=1"
#define DBG_CMD_PMU_3V3_OFF          "PMU ALT3V3=0"
#define DBG_CMD_PMU_LIGHT_SLEEP      "PMU LSLEEP"
#define DBG_CMD_PMU_DEEP_SLEEP       "PMU DSLEEP"
#define DBG_CMD_PMU_I2C_ON           "PMU I2C=1"
#define DBG_CMD_PMU_I2C_OFF          "PMU I2C=0"
#define DBG_CMD_PMU_GET_STATUS       "PMU ALL?"
#define DBG_CMD_PMU_SET_NOUSERS      "PMU !ALL"


// LoRa Commands
#define DBG_CMD_LORA_REINIT          "LRA INIT"
#define DBG_CMD_LORA_JOIN            "LRA JOIN"
#define DBG_CMD_LORA_CTEST           "LRA CTEST"
#define DBG_CMD_LORA_TXTEST          "LRA TXTEST"
#define DBG_CMD_LORA_GET_INFO        "LRA INFO?"
#define DBG_CMD_LORA_ISSUE_CMD       "LRA >>"
// K33 Sensor Commands
#define DBG_CMD_K33_START            "K33 START"
#define DBG_CMD_K33_STOP             "K33 STOP"
#define DBG_CMD_K33_GET_CO2          "K33 CO2?"
#define DBG_CMD_K33_CAL_ZERO         "K33 CAL ZERO"
#define DBG_CMD_K33_CAL_BG           "K33 CAL BG"
#define DBG_CMD_K33_GET_INFO         "K33 ALL?"
#define DBG_CMD_K33_GET_TRIM         "K33 TRIM?"
#define DBG_CMD_K33_GET_BCC          "K33 BCC?"
#define DBG_CMD_K33_GET_ZERO         "K33 ZERO?"
#define DBG_CMD_K33_GET_OLD          "K33 OLD?"
//#define DBG_CMD_K33_SET_TRIM         "K33 TRIM="
#define DBG_CMD_K33_FCAL_ZERO        "K33 START CAL ZERO"
#define DBG_CMD_K33_FCAL_BG          "K33 START CAL BG"

//#define DBG_CMD_K33_SET_BCC

// SHT40 Sensor Commands
//#define DBG_CMD_SHT40_GET_TEMP       "SHT40 T?"
//#define DBG_CMD_SHT40_GET_HUMID      "SHT40 H?"
#define DBG_CMD_SHT40_GET_ALL      "SHT40 ALL?"
#define DBG_CMD_SHT40_HEAT_H_LONG    "SHT40 HT HL?"
#define DBG_CMD_SHT40_HEAT_M_SHORT    "SHT40 HT MS?"


const char *debugCommandsWithExplanations = 
"Available Debug Commands:\n"
"\n"
"System Commands:\n"
"SYS REBOOT         - Reboots the system.\n"
"SYS VBAT?          - Queries the system battery voltage.\n"
"SYS MEM?           - Displays memory usage statistics.\n"
"SYS RTC?           - Dumps the current RTC Runtime Stats.\n" 
"SYS !RTC           - Resets the RTC Runtime Stats.\n"
"SYS CLEAR!         - Clears the first-boot state.\n"
"\n"
"PMU (Power Management Unit) Commands:\n"
"PMU BOOST=1        - Enables 5V boost power supply.\n"
"PMU BOOST=0        - Disables 5V boost power supply.\n"
"PMU ALT3V3=1       - Enables alternate 3.3V power supply.\n"
"PMU ALT3V3=0       - Disables alternate 3.3V power supply.\n"
"PMU LSLEEP         - Initiates light sleep mode.\n"
"PMU DSLEEP         - Initiates deep sleep mode.\n"
"PMU I2C=1          - Turns on the I2C interface.\n"
"PMU I2C=0          - Turns off the I2C interface.\n"
"PMU ALL?           - Queries all PMU statuses.\n"
"PMU !ALL           - Resets all PMU user-defined settings.\n"
"\n"
"LoRa Commands:\n"
"LRA INIT           - Reinitializes the LoRa module.\n"
"LRA JOIN           - Joins the configured LoRa network.\n"
"LRA CTEST          - Conducts a connectivity test.\n"
"LRA TXTEST         - Sends a test transmission.\n"
"LRA INFO?          - Retrieves LoRa module information.\n"
"LRA >>             - Issues a raw command to the LoRa module.\n"
"\n"
"K33 Sensor Commands:\n"
"K33 START          - Starts the K33 sensor.\n"
"K33 STOP           - Stops the K33 sensor.\n"
"K33 CO2?           - Queries the current CO2 reading.\n"
"K33 CAL ZERO       - Calibrates the sensor to zero CO2.\n"
"K33 CAL BG         - Calibrates the sensor for background CO2.\n"
"K33 ALL?           - Retrieves all sensor data.\n"
"K33 TRIM?          - Queries trim data.\n"
"K33 BCC?           - Retrieves baseline calibration coefficient.\n"
"K33 ZERO?          - Queries zero-point calibration.\n"
"K33 OLD?           - Retrieves old data from the sensor.\n"
"K33 START CAL ZERO - Initiates a forced zero calibration.\n"
"K33 START CAL BG   - Initiates a forced background calibration.\n"
"\n"
"SHT40 Sensor Commands:\n"
"SHT40 ALL?         - Retrieves all sensor readings (temperature and humidity).\n"
"SHT40 HOTTEST      - Activates the sensor's heater at maximum power for a long duration.\n";




#endif // DBG_INTERFACE_CMDS

/*** end of file ***/