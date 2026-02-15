/*
 Name:		ESP32 APRS Internet Gateway
 Created:	1-Nov-2021 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#ifndef MAIN_H
#define MAIN_H

#define VERSION "1.7"
#define VERSION_BUILD ' '

#include <Arduino.h>
#include <AX25.h>
#include <AsyncTCP.h>
#include "weather.h"

#include "config.h"
// #if defined(TTGO_T_Beam_S3_SUPREME_V3)  || defined(HELTEC_V3_GPS) || defined(HELTEC_HTIT_TRACKER) || defined(APRS_LORA_HT) || defined(APRS_LORA_DONGLE)
// #else
// #include "soc/rtc_wdt.h"
// #endif

//-------ESP32---------
#if defined(CONFIG_IDF_TARGET_ESP32)
#define ESP32
#define USE_TDM
#define USE_PDM
#endif
#if defined(CONFIG_IDF_TARGET_ESP32C3)
#define ESP32C3
#define ESP32X
#define USE_INT24_FROM_INT
#define USE_TDM
#define USE_PDM
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S2)
#  define ESP32S2
#  define ESP32X
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#  define ESP32S3
#  define ESP32X
#  define USE_TDM
#  define USE_PDM
#  define USE_PDM_RX
#endif
#if defined(CONFIG_IDF_TARGET_ESP32C6)
#  define ESP32C6
#  define ESP32X
#  define USE_TDM
#  define USE_PDM
#endif
#if defined(CONFIG_IDF_TARGET_ESP32P4)
#  define ESP32P4
#  define ESP32X
#  define USE_TDM
#  define USE_PDM
#  define USE_PDM_RX
#endif

#define WX

#define WIFI_OFF_FIX 0
#define WIFI_AP_FIX 1
#define WIFI_STA_FIX 2
#define WIFI_AP_STA_FIX 3

#define IMPLEMENTATION FIFO

#define TZ 0	 // (utc+) TZ in hours
#define DST_MN 0 // use 60mn for summer time in some countries
#define TZ_MN ((TZ) * 60)
#define TZ_SEC ((TZ) * 3600)
#define DST_SEC ((DST_MN) * 60)

#define FORMAT_SPIFFS_IF_FAILED true

#ifdef BOARD_HAS_PSRAM
#define TLMLISTSIZE 100
#define PKGLISTSIZE 30
#define PKGTXSIZE 10
#else
#define TLMLISTSIZE 5
#define PKGLISTSIZE 20
#define PKGTXSIZE 5
#endif

#define LOG_NONE 0
#define LOG_TRACKER (1 << 0)
#define LOG_IGATE (1 << 1)
#define LOG_DIGI (1 << 2)
#define LOG_WX (1 << 3)
#define LOG_STATUS (1 << 4)

#define FILTER_ALL 0				// Packet is disable all packet
#define FILTER_OBJECT (1 << 0)		// packet is an object
#define FILTER_ITEM (1 << 1)		// packet is an item
#define FILTER_MESSAGE (1 << 2)		// packet is a message
#define FILTER_WX (1 << 3)			// packet is WX data
#define FILTER_TELEMETRY (1 << 4)	// packet is telemetry
#define FILTER_QUERY (1 << 5)		// packet is a query
#define FILTER_STATUS (1 << 6)		// packet is status
#define FILTER_POSITION (1 << 7)	// packet is postion
#define FILTER_BUOY (1 << 8)		// packet is buoy
#define FILTER_MICE (1 << 9)		// packet is MIC-E
#define FILTER_THIRDPARTY (1 << 10) // packet is 3rd-party packet from INET2RF

#define RF_NONE 0
#define RF_SA868_VHF 1 // G-NiceRF SA818,SA868 VHF band 134~174 MHz
#define RF_SA868_UHF 2 // G-NiceRF SA818,SA868 UHF band 400~470 MHz
#define RF_SA868_350 3 // G-NiceRF SA818,SA868 350 band frequency：320-400MHz
#define RF_SR_1WV 4	   // SUNRISE SR110V,FRS-1WV VHF band 136~174 MHz
#define RF_SR_1WU 5	   // SUNRISE SR110U,FRS-1WU UHF band 400~470 MHz
#define RF_SR_1W350 6  // SUNRISE SR350P 350 band frequency：350-390MHz
#define RF_SR_2WVS 7   // SUNRISE SR120V,SR_2WVS VHF band 136~174 MHz
#define RF_SR_2WUS 8   // SUNRISE SR120U,SR_2WUS UHF band 400~470 MHz

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define TXCH_TCP 0
#define TXCH_RF 1
#define TXCH_DIGI 2
#define TXCH_3PTY 3

#define RF_CHANNEL	(1<<0)
#define INET_CHANNEL	(1<<1)
#define TNC_CHANNEL	(1<<2)

#if defined(HELTEC_HTIT_TRACKER)
#define ST7735_LED_K_Pin 21
#elif defined(APRS_LORA_DONGLE)
#define ST7735_LED_K_Pin 16
#endif

// APRS data type identifiers
#define RADIOLIB_APRS_DATA_TYPE_POSITION_NO_TIME_NO_MSG "!"
#define RADIOLIB_APRS_DATA_TYPE_GPS_RAW "$"
#define RADIOLIB_APRS_DATA_TYPE_ITEM ")"
#define RADIOLIB_APRS_DATA_TYPE_TEST ","
#define RADIOLIB_APRS_DATA_TYPE_POSITION_TIME_NO_MSG "/"
#define RADIOLIB_APRS_DATA_TYPE_MSG ":"
#define RADIOLIB_APRS_DATA_TYPE_OBJECT ";"
#define RADIOLIB_APRS_DATA_TYPE_STATION_CAPABILITES "<"
#define RADIOLIB_APRS_DATA_TYPE_POSITION_NO_TIME_MSG "="
#define RADIOLIB_APRS_DATA_TYPE_STATUS ">"
#define RADIOLIB_APRS_DATA_TYPE_QUERY "?"
#define RADIOLIB_APRS_DATA_TYPE_POSITION_TIME_MSG "@"
#define RADIOLIB_APRS_DATA_TYPE_TELEMETRY "T"
#define RADIOLIB_APRS_DATA_TYPE_MAIDENHEAD_BEACON "["
#define RADIOLIB_APRS_DATA_TYPE_WEATHER_REPORT "_"
#define RADIOLIB_APRS_DATA_TYPE_USER_DEFINED "{"
#define RADIOLIB_APRS_DATA_TYPE_THIRD_PARTY "}"

/*!
  \defgroup mic_e_message_types Mic-E message types.
  \{
*/

/*! \brief Mic-E "Off duty" message. */
#define RADIOLIB_APRS_MIC_E_TYPE_OFF_DUTY 0b00000111

/*! \brief Mic-E "En route" message. */
#define RADIOLIB_APRS_MIC_E_TYPE_EN_ROUTE 0b00000110

/*! \brief Mic-E "In service" message. */
#define RADIOLIB_APRS_MIC_E_TYPE_IN_SERVICE 0b00000101

/*! \brief Mic-E "Returning" message. */
#define RADIOLIB_APRS_MIC_E_TYPE_RETURNING 0b00000100

/*! \brief Mic-E "Commited" message. */
#define RADIOLIB_APRS_MIC_E_TYPE_COMMITTED 0b00000011

/*! \brief Mic-E special message. */
#define RADIOLIB_APRS_MIC_E_TYPE_SPECIAL 0b00000010

/*! \brief Mic-E priority message. */
#define RADIOLIB_APRS_MIC_E_TYPE_PRIORITY 0b00000001

/*! \brief Mic-E emergency message. */
#define RADIOLIB_APRS_MIC_E_TYPE_EMERGENCY 0b00000000

/*!
  \}
*/

// magic offset applied to encode extra bits in the Mic-E destination field
#define RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET 25

// Mic-E data types
#define RADIOLIB_APRS_MIC_E_GPS_DATA_CURRENT '`'
#define RADIOLIB_APRS_MIC_E_GPS_DATA_OLD '\''

// Mic-E telemetry flags
#define RADIOLIB_APRS_MIC_E_TELEMETRY_LEN_2 '`'
#define RADIOLIB_APRS_MIC_E_TELEMETRY_LEN_5 '\''

// alias for unused altitude in Mic-E
#define RADIOLIB_APRS_MIC_E_ALTITUDE_UNUSED -1000000

#ifdef __cplusplus
extern "C" {
#endif

#if defined(MBEDTLS_MD5_ALT)
#include "md/esp_md.h"

#define mbedtls_md5_init                        esp_md5_init
#define mbedtls_md5_update_ret                      esp_md5_update
#define mbedtls_md5_finish_ret                      esp_md5_finish
#define mbedtls_md5_starts_ret                      esp_md5_starts

#define mbedtls_md5_free                        esp_md5_free
#define mbedtls_md5_clone                       esp_md5_clone
#define mbedtls_internal_md5_process            esp_md5_process

#endif /* MBEDTLS_MD5_ALT */
#ifdef __cplusplus
}
#endif

#define ets_printf	log_d 

typedef struct igateTLM_struct
{
	uint16_t Sequence;
	unsigned long ParmTimeout;
	unsigned long TeleTimeout;
	uint8_t RF2INET;
	uint8_t INET2RF;
	uint8_t RX;
	uint8_t TX;
	uint8_t DROP;
} igateTLMType;

typedef struct
{
	time_t time;
	char calsign[11];
	char object[10];
	char ssid[5];
	bool channel;
	unsigned int pkg;
	uint16_t type;
	uint8_t symbol;
	int16_t audio_level;
	float rssi;
	float snr;
	float freqErr;
	char *raw;
	size_t length;
	//char raw[256];
} pkgListType;

typedef struct statisticStruct
{
	uint32_t allCount;
	uint32_t tncCount;
	uint32_t isCount;
	uint32_t locationCount;
	uint32_t wxCount;
	uint32_t digiCount;
	uint32_t errorCount;
	uint32_t dropCount;
	uint32_t rf2inet;
	uint32_t inet2rf;
	uint32_t txCount;
	uint32_t rxCount;
} statusType;

typedef struct digiTLM_struct
{
	unsigned int Sequence;
	unsigned int ParmTimeout;
	unsigned int TeleTimeout;
	unsigned char RxPkts;
	unsigned char TxPkts;
	unsigned char DropRx;
	unsigned char ErPkts;
} digiTLMType;

typedef struct dataTLM_struct
{
	unsigned int Sequence;
	unsigned long ParmTimeout;
	unsigned long TeleTimeout;
	uint8_t A1;
	uint8_t A2;
	uint8_t A3;
	uint8_t A4;
	uint8_t A5;
	uint8_t BITS;
} dataTLMType;

typedef struct Telemetry_struct
{
	time_t time;
	char callsign[10];
	char PARM[5][10];
	char UNIT[5][10];
	float VAL[5];
	float RAW[5];
	float EQNS[15];
	uint8_t BITS;
	uint8_t BITS_FLAG;
	bool EQNS_FLAG;
} TelemetryType;

typedef struct txQueue_struct
{
	bool Active;
	uint8_t Channel;
	long timeStamp;
	int Delay;
	size_t length;
	char Info[350];
} txQueueType;

typedef struct txDispStruct
{
	uint8_t tx_ch;
	char name[12];
	char info[50];
} txDisp;

typedef struct pppStruct
{
	char manufacturer[30];
	char model[20];
	char imei[20];
	char oper[20];	
	char imsi[20];
	uint32_t ip;
	uint32_t dns;
	uint32_t gateway;
	uint32_t netmask;	
	int rssi;
} pppType;

#ifdef OLED
const unsigned char LOGO[] PROGMEM =
	{
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
		0xC0, 0x00, 0x00, 0xC0, 0xC0, 0xE0, 0xE0, 0xF0, 0xF8, 0xF8,
		0xFC, 0xFC, 0xFE, 0x7E, 0x7F, 0x7C, 0x70, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF8, 0xF8, 0xF8, 0xFC,
		0x7C, 0x3C, 0x3C, 0x18, 0x83, 0x9F, 0x9F, 0x9F, 0x8F, 0x8F,
		0xCC, 0x41, 0x63, 0x13, 0x01, 0x81, 0xE1, 0x21, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
		0x07, 0x07, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
		0x08, 0x08, 0x08, 0x09, 0x19, 0x19, 0x09, 0x08, 0x08, 0x0C,
		0x04, 0x86, 0x83, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x8C, 0x80, 0x84, 0xC2, 0x80, 0x07, 0x08,
		0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF8, 0x3E, 0x43, 0x7D,
		0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E,
		0x7E, 0x7E, 0x7E, 0x7E, 0x79, 0x07, 0x7C, 0xF0, 0xC0, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE7, 0xC3, 0xC3, 0xC3, 0xFF,
		0xFF, 0xFF, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0xFF, 0xFF, 0xFF,
		0xE7, 0xC3, 0xC3, 0xC3, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x03, 0x1F, 0x3F, 0x3F, 0x3F, 0x1F, 0x03, 0x03, 0x03, 0x03,
		0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x1F,
		0x3F, 0x3F, 0x3F, 0x1F, 0x03, 0x00, 0x00, 0x00};
#endif

const char PARM[] = {"PARM.RF->INET,INET->RF,DigiRpt,TX2RF,DropRx"};
const char UNIT[] = {"UNIT.Pkts,Pkts,Pkts,Pkts,Pkts"};
const char EQNS[] = {"EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0"};

const float ctcss[] = {0, 67, 71.9, 74.4, 77, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100, 103.5, 107.2, 110.9, 114.8, 118.8, 123, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8, 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3};
const float wifiPwr[12][2] = {{-4, -1}, {8, 2}, {20, 5}, {28, 7}, {34, 8.5}, {44, 11}, {52, 13}, {60, 15}, {68, 17}, {74, 18.5}, {76, 19}, {78, 19.5}};
const char RF_TYPE[9][11] = {"NONE", "SA868_VHF", "SA868_UHF", "SA868_350", "SR110V_VHF", "SR110U_UHF", "SR350P", "SR120V_VHF", "SR120U_UHF"};
const unsigned long baudrate[] = {2400, 4800, 9600, 19200, 2880, 38400, 57600, 76800, 115200, 230400, 460800, 576000, 921600};
const char GNSS_PORT[5][6] = {"NONE", "UART0", "UART1", "UART2", "TCP"};
const char TNC_PORT[5][6] = {"NONE", "UART0", "UART1","UART2", "USB"};
const char TNC_MODE[4][6] = {"NONE", "KISS", "TNC2", "YAESU"};
const char WX_PORT[7][11] = {"NONE", "UART0_CSV", "UART1_CSV", "UART2_CSV", "MODBUS","SENSOR","TCP/UDP"};
const char MODEM_TYPE[4][17] = {"AFSK_300", "AFSK_1200","AFSK_1200v23","GFSK9600(G3RUH)"};
const char FX25_MODE[3][6] = {"NONE","RX","RX+TX"};
const char PWR_MODE[3][10] = {"MODE A", "MODE B","MODE C"};
const char WX_SENSOR[26][19]={"Wind Course","Wind Speed","Wind Gust","Temperature","Rain 1hr","Rain 24hr","Rain Midnight","Humidity","Barometric","Luminosity","Snow","Soil Temperature","Soil Humidity","Water Temperature","Water TDS","Water Level","PM 2.5","PM 10","Co2","CH2O","TVOC","UV","SOUND","VBAT","IBAT","VSOLAR"};
const char MIC_E_MSG[8][10] = {"Emergency", "Priority", "Special", "Committed", "Returning", "InService", "En Route", "Off Duty"};

uint8_t checkSum(uint8_t *ptr, size_t count);
//void saveEEPROM();
void defaultConfig();
String getValue(String data, char separator, int index);
boolean isValidNumber(String str);
void taskSerial(void *pvParameters);
void taskGPS(void *pvParameters);
void taskAPRS(void *pvParameters);
void taskAPRSPoll(void *pvParameters);
void taskNetwork(void *pvParameters);
//void taskTNC(void *pvParameters);
void sort(pkgListType a[], int size);
void sortPkgDesc(pkgListType a[], int size);
//int processPacket(String &tnc2);
int digiProcess(AX25Msg &Packet);
void printTime();
//int popTNC2Raw(int &ret);
//int pushTNC2Raw(int raw);
//int pkgListUpdate(char *call, char *raw, uint16_t type, bool channel, uint16_t audioLvl);
int pkgList_Find(char *call,char *object, uint16_t type);
int pkgList_Find(char *call, uint16_t type);
int pkgList_Find(char *call);
pkgListType getPkgList(int idx);
//String myBeacon(String Path);
int tlmList_Find(char *call);
int tlmListOld();
TelemetryType getTlmList(int idx);
//void powerSave();
//void powerWakeup();
//bool powerStatus();
int packet2Raw(String &tnc2, AX25Msg &Packet);
//bool waitResponse(String &data, String rsp = "\r\n", uint32_t timeout = 1000);
//String sendIsAckMsg(String toCallSign, char *msgId);
bool pkgTxPush(const char *info, size_t len, int dly,uint8_t Ch);
//bool pkgTxUpdate(const char *info, int delay);
void dispWindow(String line, uint8_t mode, bool filter);
void dispTxWindow(txDisp txs);
void systemDisp();
void pkgCountDisp();
void pkgLastDisp();
void statisticsDisp();
String getTimeStamp();
void DD_DDDDDtoDDMMSS(float DD_DDDDD, int *DD, int *MM, int *SS);
String getPath(int idx);
void GPS_INIT();
void gpsDisp();
void radioDisp();
void wifiDisp();
void sensorDisp();
void convertSecondsToDHMS(char *dmhs,unsigned long totalSeconds);
void wifiConnection();

#endif