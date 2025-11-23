/*
 Name:		ESP32APRS T-TWR Plus
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#include "main.h"
#ifndef WEBSERVICE_H
#define WEBSERVICE_H

#include <Update.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <time.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>
#include "LITTLEFS.h"
#include "message.h"

typedef struct timeZoneName
{
	float tz;
	const char *name;
}timeZoneName;

const timeZoneName tzList[40] PROGMEM= {
	{-12.00, "(GMT -12:00) Eniwetok, Kwajalein"},
	{-11.00, "(GMT -11:00) Midway Island, Samoa"},
	{-10.00, "(GMT -10:00) Hawaii"},
	{-09.50, "(GMT -9:30) Taiohae"},
	{-09.00, "(GMT -9:00) Alaska"},
	{-08.00, "(GMT -8:00) Pacific Time (US &amp; Canada)"},
	{-07.00, "(GMT -7:00) Mountain Time (US &amp; Canada)"},
	{-06.00, "(GMT -6:00) Central Time (US &amp; Canada), Mexico City"},
	{-05.00, "(GMT -5:00) Eastern Time (US &amp; Canada), Bogota, Lima"},
	{-04.50, "(GMT -4:30) Caracas"},
	{-04.00, "(GMT -4:00) Atlantic Time (Canada), Caracas, La Paz"},
	{-03.50, "(GMT -3:30) Newfoundland"},
	{-03.00, "(GMT -3:00) Brazil, Buenos Aires, Georgetown"},
	{-02.00, "(GMT -2:00) Mid-Atlantic"},
	{-01.00, "(GMT -1:00) Azores, Cape Verde Islands"},
	{+00.00, "(GMT) Western Europe Time, London, Lisbon, Casablanca"},
	{+01.00, "(GMT +1:00) Brussels, Copenhagen, Madrid, Paris"},
	{+02.00, "(GMT +2:00) Kaliningrad, South Africa"},
	{+03.00, "(GMT +3:00) Baghdad, Riyadh, Moscow, St. Petersburg"},
	{+03.50, "(GMT +3:30) Tehran"},
	{+04.00, "(GMT +4:00) Abu Dhabi, Muscat, Baku, Tbilisi"},
	{+04.50, "(GMT +4:30) Kabul"},
	{+05.00, "(GMT +5:00) Ekaterinburg, Islamabad, Karachi, Tashkent"},
	{+05.50, "(GMT +5:30) Bombay, Calcutta, Madras, New Delhi"},
	{+05.75, "(GMT +5:45) Kathmandu, Pokhara"},
	{+06.00, "(GMT +6:00) Almaty, Dhaka, Colombo"},
	{+06.50, "(GMT +6:30) Yangon, Mandalay"},
	{+07.00, "(GMT +7:00) Bangkok, Hanoi, Jakarta"},
	{+08.00, "(GMT +8:00) Beijing, Perth, Singapore, Hong Kong"},
	{+08.75, "(GMT +8:45) Eucla"},
	{+09.00, "(GMT +9:00) Tokyo, Seoul, Osaka, Sapporo, Yakutsk"},
	{+09.50, "(GMT +9:30) Adelaide, Darwin"},
	{+10.00, "(GMT +10:00) Eastern Australia, Guam, Vladivostok"},
	{+10.50, "(GMT +10:30) Lord Howe Island"},
	{+11.00, "(GMT +11:00) Magadan, Solomon Islands, New Caledonia"},
	{+11.50, "(GMT +11:30) Norfolk Island"},
	{+12.00, "(GMT +12:00) Auckland, Wellington, Fiji, Kamchatka"},
	{+12.75, "(GMT +12:45) Chatham Islands"},
	{+13.00, "(GMT +13:00) Apia, Nukualofa"},
	{+14.00, "(GMT +14:00) Line Islands, Tokelau"},
};

const char ADC_ATTEN[4][19] = {"0dB (100-950mV)", "2.5dB (100-1250mV)", "6dB (150-1750mV)", "11dB (150-2450mV)"};

#define SYSTEM_LEN 6
#define SYSTEM_BIT_LEN 9
#define PATH_LEN 17
const char SYSTEM_NAME[SYSTEM_LEN][20] = {"NONE", "All Count", "RF2INET", "INET2RF","DIGI","All Drop"};
const char SYSTEM_BITS_NAME[9][20] = {"NONE", "IGATE Mode", "DIGI Mode", "WEATHER Mode","SAT Status","INET Status","VPN Status","GSM Status","FX.25 Status"};
const char PATH_NAME[PATH_LEN][15] = {"OFF", "DST-TRACE 1", "DST-TRACE 2", "DST-TRACE 3", "DST-TRACE 4", "TRACE1-1", "TRACE2-2", "TRACE3-3", "WIDE1-1","RFONLY","RELAY","GATE","ECHO","UserDefine 1","UserDefine 2","UserDefine 3","UserDefine 4"};


// ใช้ตัวแปรโกลบอลในไฟล์ main.cpp
extern statusType status;
extern digiTLMType digiTLM;
extern Configuration config;
extern TaskHandle_t taskNetworkHandle;
extern TaskHandle_t taskAPRSHandle;
extern TaskHandle_t taskAPRSPollHandle;
extern TaskHandle_t taskSerialHandle;
extern TaskHandle_t taskGPSHandle;
extern TaskHandle_t taskSensorHandle;
extern time_t systemUptime;
extern String RF_VERSION;
extern pkgListType *pkgList;
extern msgType *msgQueue;
extern TinyGPSPlus gps;
extern float vbat;
extern WiFiClient aprsClient;
extern bool initInterval;
extern bool webServiceBegin;
extern fs::LITTLEFSFS LITTLEFS;
extern double VBat;
extern double TempNTC;
extern bool lastHeard_Flag;
extern unsigned long lastHeardTimeout;
extern SensorData sen[SENSOR_NUMBER];
extern uint16_t TLM_SEQ;
extern uint16_t IGATE_TLM_SEQ;
extern uint16_t DIGI_TLM_SEQ;
extern unsigned long StandByTick;

#ifdef __cplusplus
extern "C"
{
#endif
	uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

void serviceHandle();
void setHTML(byte page);
void handle_root();
void handle_setting();
void handle_service();
void handle_system();
void handle_firmware();
void handle_default();
void webService();
void handle_radio();
extern void RF_MODULE(bool boot);
//void handle_ws(String Raw,uint16_t mVrms);
void handle_ws(char *Raw,size_t len,uint16_t mVrms);
void handle_ws_gnss(char *nmea);
void handle_ws_gnss(char *nmea, size_t size);
String event_lastHeard(bool gethtml=false);
String event_chatMessage(bool gethtml=false);

#endif
