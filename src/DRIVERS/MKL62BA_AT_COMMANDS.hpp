/** @file MKL62BA_AT_COMMANDS.hpp
 * 
 * @brief Commands and response strings for the MKL62BA LoRaWAN radio module.
 *
 * @par       
 * COPYRIGHT NOTICE: (c) 2024, Developed for PolySense Solutions Inc, by David Stewart.  All rights reserved.
 */ 

#ifndef MKL62BA_AT_COMMANDS_HPP
#define MKL62BA_AT_COMMANDS_HPP


// General AT Commands
#define RESPID_NONE ""

#define AT_CMD_TEST "AT"
#define RESPID_TEST "+AT: "
#define AT_CMD_ECHO "AT+ATE"
#define RESPID_ECHO "+ATE: "
#define AT_CMD_BAUD "AT+BAUD"
#define RESPID_BAUD "+BAUD: "
#define AT_CMD_SLEEP "AT+SLEEP"
#define RESPID_SLEEP "+SLEEP: "
#define AT_CMD_RESET "AT+RESET"
#define RESPID_RESET "+ATZ: "
#define AT_CMD_FACTORY_RESET "AT+FACTORY"
#define RESPID_FACTORY_RESET "+FACTORY: "

// BLE AT Commands
#define AT_CMD_LADDR "AT+LADDR"
#define RESPID_LADDR "+LADDR: "
#define AT_CMD_NAME "AT+NAME"
#define RESPID_NAME "+NAME: "
#define AT_CMD_ADVI "AT+ADVI"
#define RESPID_ADVI "+ADVI: "
#define AT_CMD_UUID "AT+UUID"
#define RESPID_UUID "+UUID: "
#define AT_CMD_ADVD "AT+ADVD"
#define RESPID_ADVD "+ADVD: "
#define AT_CMD_SCAN_STD "AT+SCAN_STD"
#define RESPID_SCAN_STD "+SCAN_STD: "
#define AT_CMD_SCAN_RSSI "AT+SCAN_RSSI"
#define RESPID_SCAN_RSSI "+SCAN_RSSI: "

// LoRa Network Management AT Commands
#define AT_CMD_JOIN_MODE "AT+JOIN_MODE"
#define RESPID_JOIN_MODE "+JOIN_MODE: "
#define AT_CMD_DEVEUI "AT+DEVEUI"
#define RESPID_DEVEUI "+DEVEUI: "
#define AT_CMD_APPEUI "AT+APPEUI"
#define RESPID_APPEUI "+APPEUI: "
#define AT_CMD_APPKEY "AT+APPKEY"
#define RESPID_APPKEY "+APPKEY: "
#define AT_CMD_NWKSKEY "AT+NWKSKEY"
#define RESPID_NWKSKEY "+NWKSKEY: "
#define AT_CMD_APPSKEY "AT+APPSKEY"
#define RESPID_APPSKEY "+APPSKEY: "
#define AT_CMD_DEVADDR "AT+DEVADDR"
#define RESPID_DEVADDR "+DEVADDR: "
#define AT_CMD_REGION "AT+REGION"
#define RESPID_REGION "+REGION: "
#define AT_CMD_CLASS "AT+CLASS"
#define RESPID_CLASS "+CLASS: "
#define AT_CMD_JOIN_NETWORK "AT+JOINING"
#define RESPID_JOIN_NETWORK "+JOINING: "
#define AT_CMD_JOIN_STATUS "AT+JOIN_STD"
#define RESPID_JOIN_STATUS "+JOIN_STD: "
#define AT_CMD_AUTO_JOIN "AT+AUTO_JOIN"
#define RESPID_AUTO_JOIN "+AUTO_JOIN: "
#define AT_CMD_NWKID "AT+NWKID"
#define RESPID_NWKID "+NWKID: "
#define AT_CMD_LCR "AT+LCR"
#define RESPID_LCR "+LCR: "
#define AT_CMD_ADR "AT+ADR"
#define RESPID_ADR "+ADR: "
#define AT_CMD_TX_POWER "AT+TX_POWER"
#define RESPID_TX_POWER "+TX_POWER: "
#define AT_CMD_DATA_RATE "AT+DR"
#define RESPID_DATA_RATE "+DR: "
#define AT_CMD_CHANNEL "AT+CH"
#define RESPID_CHANNEL "+CH: "

// Send and Receive Packets AT Commands
#define AT_CMD_TX_LEN "AT+TX_LEN"
#define RESPID_TX_LEN "+TX_LEN: "
#define AT_CMD_CONFIRM "AT+CONFIRM"
#define RESPID_CONFIRM "+CONFIRM: "
#define AT_CMD_SEND_HEX "AT+SENDB"
#define RESPID_SEND_HEX "+SENDB: "
#define AT_CMD_SEND_STRING "AT+SEND"
#define RESPID_SEND_STRING "+SEND: "
#define AT_CMD_RECV_HEX "AT+RECVB"
#define RESPID_RECV_HEX "+RECVB: "
#define AT_CMD_RECV_STRING "AT+RECV"
#define RESPID_RECV_STRING "+RECV: "
#define AT_CMD_DUTY_CYCLE "AT+DUTY_CYCLE"
#define RESPID_DUTY_CYCLE "+DUTY_CYCLE: "
#define AT_CMD_UP_CNT "AT+UP_CNT"
#define RESPID_UP_CNT "+UP_CNT: "
#define AT_CMD_DOWN_CNT "AT+DOWN_CNT"
#define RESPID_DOWN_CNT "+DOWN_CNT: "

// LoRa Multicast Management AT Commands
#define AT_CMD_MC "AT+MC"
#define RESPID_MC "+MC: "
#define AT_CMD_MC_DEVADDR "AT+MC_DEVADDR"
#define RESPID_MC_DEVADDR "+MC_DEVADDR: "
#define AT_CMD_MC_NWKSKEY "AT+MC_NWKSKEY"
#define RESPID_MC_NWKSKEY "+MC_NWKSKEY: "
#define AT_CMD_MC_APPSKEY "AT+MC_APPSKEY"
#define RESPID_MC_APPSKEY "+MC_APPSKEY: "
#define AT_CMD_MC_CNT "AT+MC_CNT"
#define RESPID_MC_CNT "+MC_CNT: "

// Test AT Commands
#define AT_CMD_TEST_MODE "AT+TEST"
#define RESPID_TEST_MODE "+TEST: "
#define AT_CMD_TEST_CONF "AT+TEST_CONF"
#define RESPID_TEST_CONF "+TEST_CONF: "
#define AT_CMD_TEST_TXLORA "AT+TEST_TXLORA"
#define RESPID_TEST_TXLORA "+TEST_TXLORA: "
#define AT_CMD_TEST_RXLORA "AT+TEST_RXLORA"
#define RESPID_TEST_RXLORA "+TEST_RXLORA: "
#define AT_CMD_TEST_SCAN "AT+TEST_SCAN"
#define RESPID_TEST_SCAN "+TEST_SCAN: "
#define AT_CMD_TEST_BLE_CON "AT+TEST_BLE_CON"
#define RESPID_TEST_BLE_CON "+TEST_BLE_CON: "
#define AT_CMD_TEST_PIOXX "AT+TEST_PIOXX"
#define RESPID_TEST_PIOXX "+TEST_PIOXX: "

// Version Information AT Commands
#define AT_CMD_VERSION "AT+VER"
#define RESPID_VERSION "+VER: "

#endif // MKL62BA_AT_COMMANDS_HPP

/*** end of file ***/