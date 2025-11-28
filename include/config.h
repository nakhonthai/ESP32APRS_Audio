/*
 Name:		ESP32 APRS Internet Gateway
 Created:	1-Nov-2021 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "sensor.h"

#define COMMENT_SIZE 25
#define STATUS_SIZE 50

#define WX_SENSOR_NUM 26

#define ACTIVATE_OFF 0				// Packet is disable all packet
#define ACTIVATE_TRACKER (1 << 0)		// packet is an object
#define ACTIVATE_IGATE (1 << 1)		// packet is an item
#define ACTIVATE_DIGI (1 << 2)		// packet is a message
#define ACTIVATE_WX (1 << 3)			// packet is WX data
#define ACTIVATE_TELEMETRY (1 << 4)	// packet is telemetry
#define ACTIVATE_QUERY (1 << 5)		// packet is a query
#define ACTIVATE_STATUS (1 << 6)		// packet is status
#define ACTIVATE_WIFI (1 << 7)		// packet is wifi


// #include <FS.h>
// #include <SD.h>
// #include <SPIFFS.h>
// #include "soc/rtc_wdt.h"
// #include <AX25.h>

// #include "HardwareSerial.h"
// #include "EEPROM.h"
#define RF_SX1231	1
#define RF_SX1233	2
#define RF_SX1261	3
#define RF_SX1262	4
#define RF_SX1268	5
#define RF_SX1272	6
#define RF_SX1273	7
#define RF_SX1276	8
#define RF_SX1278	9
#define RF_SX1279	10
#define RF_SX1280	11
#define RF_SX1281	12
#define RF_SX1282	13

#define RF_SX123x	21
#define RF_SX126x	22
#define RF_SX127x	23
#define RF_SX128x	24

#define RF_MODE_OFF		0
#define RF_MODE_LoRa 	1
#define RF_MODE_G3RUH 	2
#define RF_MODE_AIS 	3
#define RF_MODE_GFSK	4
#define RF_MODE_DPRS	5

#define MODE_A	0
#define MODE_B	1
#define MODE_C	2


typedef struct wifi_struct
{
	bool enable;
	char wifi_ssid[32];
	char wifi_pass[63];
} wifiSTA;

typedef struct Config_Struct
{
	float timeZone;
	bool synctime;
	bool title;

	// WiFi/BT/RF
	uint8_t wifi_mode; // WIFI_AP,WIFI_STA,WIFI_AP_STA,WIFI_OFF
	int8_t wifi_power;
	//--WiFi Client
	wifiSTA wifi_sta[5];
	// bool wifi_client;
	// char wifi_ssid[32];
	// char wifi_pass[63];
	//--WiFi AP
	// bool wifi_ap;
	uint8_t wifi_ap_ch;
	char wifi_ap_ssid[32];
	char wifi_ap_pass[63];

		//--Blue Tooth
		bool bt_slave;
		bool bt_master;
		uint8_t bt_mode;
		char bt_uuid[37];
		char bt_uuid_rx[37];
		char bt_uuid_tx[37];
		char bt_name[20];
		uint32_t bt_pin;
		uint8_t bt_power;

	//--RF Module
	bool rf_en;
	uint8_t rf_type;
	float freq_rx;
	float freq_tx;
	int offset_rx;
	int offset_tx;
	int tone_rx;
	int tone_tx;
	uint8_t band;
	uint8_t sql_level;
	bool rf_power;
	uint8_t volume;
	uint8_t mic;
	

	// IGATE
	bool igate_en;
	bool rf2inet;
	bool inet2rf;
	bool igate_loc2rf;
	bool igate_loc2inet;
	uint16_t rf2inetFilter;
	uint16_t inet2rfFilter;
	//--APRS-IS
	uint8_t aprs_ssid;
	uint16_t aprs_port;
	char aprs_mycall[10];
	char aprs_host[20];
	char aprs_passcode[6];
	char aprs_moniCall[10];
	char aprs_filter[30];
	//--Position
	bool igate_bcn;
	bool igate_gps;
	bool igate_timestamp;
	float igate_lat;
	float igate_lon;
	float igate_alt;
	uint16_t igate_interval;
	char igate_symbol[3] = "N&";
	char igate_object[10];
	char igate_phg[8];
	uint8_t igate_path;
	char igate_comment[COMMENT_SIZE];
	uint16_t igate_sts_interval;
	char igate_status[STATUS_SIZE];
	//--Filter

	// DIGI REPEATER
	bool digi_en;
	bool digi_auto;
	bool digi_loc2rf;
	bool digi_loc2inet;
	bool digi_timestamp;
	uint8_t digi_ssid;
	char digi_mycall[10];
	uint8_t digi_path;
	uint16_t digi_delay; // ms
	uint16_t digiFilter;
	//--Position
	bool digi_bcn;
	//bool digi_compress = false;
	//bool digi_altitude = false;
	bool digi_gps;
	float digi_lat;
	float digi_lon;
	float digi_alt;
	uint16_t digi_interval;
	char digi_symbol[3] = "N&";
	char digi_phg[8];
	char digi_comment[COMMENT_SIZE];
	uint16_t digi_sts_interval;
	char digi_status[STATUS_SIZE];

// TRACKER
	bool trk_en;
	bool trk_loc2rf;
	bool trk_loc2inet;
	bool trk_timestamp;
	uint8_t trk_ssid;
	char trk_mycall[10];
	uint8_t trk_path;
	//--Position
	bool trk_gps;
	float trk_lat;
	float trk_lon;
	float trk_alt;
	uint16_t trk_interval = 60;
	bool trk_smartbeacon = false;
	bool trk_compress = false;
	bool trk_altitude = false;
	bool trk_log = false;
	bool trk_rssi = false;
	bool trk_sat = false;
	bool trk_dx = false;
	uint16_t trk_hspeed = 120;
	uint8_t trk_lspeed = 2;
	uint8_t trk_maxinterval = 15;
	uint8_t trk_mininterval = 5;
	uint8_t trk_minangle = 25;
	uint16_t trk_slowinterval = 600;
	char trk_symbol[3] = "\\>";
	char trk_symmove[3] = "/>";
	char trk_symstop[3] = "\\>";
	// char trk_btext[17] = "";
	char trk_comment[COMMENT_SIZE];
	char trk_item[10] = "";
	uint16_t trk_sts_interval;
	char trk_status[STATUS_SIZE];

	// WX
	bool wx_en;
	bool wx_2rf;
	bool wx_2inet;
	bool wx_timestamp;
	uint8_t wx_ssid;
	char wx_mycall[10];
	uint8_t wx_path;
	bool wx_gps;
	float wx_lat;
	float wx_lon;
	float wx_alt;
	uint16_t wx_interval;
	//int8_t wx_channel = 0;
	//uint8_t wx_type[32]; //Sensor number 32
	uint32_t wx_flage;
	char wx_object[10];
	char wx_comment[COMMENT_SIZE];

	// Telemetry 0
	bool tlm0_en;
	bool tlm0_2rf;
	bool tlm0_2inet;
	uint8_t tlm0_ssid;
	char tlm0_mycall[10];
	uint8_t tlm0_path;
	uint16_t tlm0_data_interval;
	uint16_t tlm0_info_interval;
	char tlm0_PARM[13][10];
	char tlm0_UNIT[13][8];
	float tlm0_EQNS[5][3];
	uint8_t tlm0_BITS_Active;	
	char tlm0_comment[COMMENT_SIZE];
	uint8_t tml0_data_channel[13];

	// Telemetry 1
	bool tlm1_en;
	bool tlm1_2rf;
	bool tlm1_2inet;
	uint8_t tlm1_ssid;
	char tlm1_mycall[10];
	uint8_t tlm1_path;
	uint16_t tlm1_data_interval;
	uint16_t tlm1_info_interval;
	char tlm1_PARM[13][10];
	char tlm1_UNIT[13][8];
	float tlm1_EQNS[5][3];
	uint8_t tlm1_BITS_Active;	
	char tlm1_comment[COMMENT_SIZE];
	uint8_t tml1_data_channel[13];

	// OLED DISPLAY
	bool oled_enable;
	int oled_timeout;
	uint8_t dim;
	uint8_t contrast;
	uint8_t startup;

	// Display
	unsigned int dispDelay;
	unsigned int filterDistant;
	bool h_up = true;
	bool tx_display = true;
	bool rx_display = true;
	uint16_t dispFilter;
	bool dispRF;
	bool dispINET;

	// AFSK,TNC
	bool audio_hpf;
	bool audio_lpf;
	uint8_t preamble;
	uint8_t modem_type;
	uint8_t fx25_mode;
	uint16_t tx_timeslot;
	char ntp_host[20];

	// VPN wiregurad
	bool vpn;
	bool modem;
	uint16_t wg_port;
	char wg_peer_address[32];
	char wg_local_address[16];
	char wg_netmask_address[16];
	char wg_gw_address[16];
	char wg_public_key[45];
	char wg_private_key[45];

	char http_username[32];
	char http_password[64];

	char path[4][72];

	// GNSS
	bool gnss_enable;
	int8_t gnss_pps_gpio = -1;
	int8_t gnss_channel = 0;	
	uint16_t gnss_tcp_port;
	char gnss_tcp_host[20];
	char gnss_at_command[30];

	// RF Module GPIO
	unsigned long rf_baudrate;
	int8_t rf_tx_gpio = 13;
	int8_t rf_rx_gpio = 14;
	int8_t rf_sql_gpio = 33;
	int8_t rf_pd_gpio = 27;
	int8_t rf_pwr_gpio = 12;
	int8_t rf_ptt_gpio = 32;
	bool rf_sql_active = 0;
	bool rf_pd_active = 1;
	bool rf_pwr_active = 0;
	bool rf_ptt_active = 0;
	int8_t adc_gpio = 1;
	int8_t dac_gpio = 18;
	int8_t adc_sel_gpio = -1;
	int8_t dac_sel_gpio = 17;
	uint8_t adc_atten = 0;
	uint16_t adc_dc_offset;


	bool i2c_enable;
	int8_t i2c_sda_pin = -1;
	int8_t i2c_sck_pin = -1;
	int8_t i2c_rst_pin = -1;
	uint32_t i2c_freq = 400000;
	bool i2c1_enable;
	int8_t i2c1_sda_pin = -1;
	int8_t i2c1_sck_pin = -1;
	uint32_t i2c1_freq = 100000;

	bool onewire_enable = false;
	int8_t onewire_gpio = -1;

	bool uart0_enable = false;
	unsigned long uart0_baudrate;
	int8_t uart0_tx_gpio = -1;
	int8_t uart0_rx_gpio = -1;
	int8_t uart0_rts_gpio = -1;

	bool uart1_enable = false;
	unsigned long uart1_baudrate;
	int8_t uart1_tx_gpio = -1;
	int8_t uart1_rx_gpio = -1;
	int8_t uart1_rts_gpio = -1;

	// #if SOC_UART_NUM > 2
	bool uart2_enable = false;
	unsigned long uart2_baudrate;
	int8_t uart2_tx_gpio = -1;
	int8_t uart2_rx_gpio = -1;
	// #endif

	bool modbus_enable = false;
	uint8_t modbus_address = 0;
	int8_t modbus_channel = -1;
	int8_t modbus_de_gpio = -1;

	bool counter0_enable = false;
	bool counter0_active = 0;
	int8_t counter0_gpio = -1;

	bool counter1_enable = false;
	bool counter1_active = 0;
	int8_t counter1_gpio = -1;

	bool ext_tnc_enable = false;
	int8_t ext_tnc_channel = 0;
	int8_t ext_tnc_mode = 0;

	// Sleep mode
	bool pwr_en;
	uint8_t pwr_mode;
	uint16_t pwr_sleep_interval; //sec
	uint16_t pwr_stanby_delay; //sec
	uint8_t pwr_sleep_activate;
	int8_t pwr_gpio=-1;
	bool pwr_active = 1;
	bool disp_flip;
	uint8_t disp_brightness;

	uint16_t log=0;

	SensorInfo sensor[SENSOR_NUMBER];
	
	bool trk_tlm_avg[5];
	uint8_t trk_tlm_sensor[5];
	uint8_t trk_tlm_precision[5];
	float trk_tlm_offset[5];
	char trk_tlm_PARM[5][10];
	char trk_tlm_UNIT[5][8];
	float trk_tlm_EQNS[5][3];

	bool digi_tlm_avg[5];
	uint8_t digi_tlm_sensor[5];
	uint8_t digi_tlm_precision[5];
	float digi_tlm_offset[5];
	char digi_tlm_PARM[5][10];
	char digi_tlm_UNIT[5][8];
	float digi_tlm_EQNS[5][3];

	bool igate_tlm_avg[5];
	uint8_t igate_tlm_sensor[5];
	uint8_t igate_tlm_precision[5];
	float igate_tlm_offset[5];
	char igate_tlm_PARM[5][10];
	char igate_tlm_UNIT[5][8];
	float igate_tlm_EQNS[5][3];

	bool wx_sensor_enable[WX_SENSOR_NUM];
	bool wx_sensor_avg[WX_SENSOR_NUM];
	uint8_t wx_sensor_ch[WX_SENSOR_NUM];

	bool ppp_enable = false;
	char ppp_apn[32];
	char ppp_pin[8];
	int8_t ppp_rst_gpio = -1;
	int8_t ppp_tx_gpio = -1;
	int8_t ppp_rx_gpio = -1;
	int8_t ppp_rts_gpio = -1;
	int8_t ppp_cts_gpio = -1;
	int8_t ppp_dtr_gpio = -1;
	int8_t ppp_ri_gpio = -1;
	bool ppp_rst_active = 0;
	uint16_t ppp_rst_delay = 1000;
	int8_t ppp_pwr_gpio = -1;
	bool ppp_pwr_active = 0;
	uint8_t ppp_serial = 1;
	unsigned long ppp_serial_baudrate = 115200;
	uint8_t ppp_model = 0;
	uint8_t ppp_flow_ctrl = 0;
	bool ppp_gnss = false;
	bool ppp_napt = true;

	// MQTT Config
#ifdef MQTT	
	bool en_mqtt;
	char mqtt_host[63];
	char mqtt_topic[63];
	char mqtt_subscribe[63];
	char mqtt_user[32];
	char mqtt_pass[63];
	uint16_t mqtt_port;
	uint16_t mqtt_topic_flag;
	uint16_t mqtt_subscribe_flag;
#endif

	uint8_t trk_mice_type;
	uint8_t trk_tlm_interval;
	uint8_t digi_tlm_interval;
	uint8_t igate_tlm_interval;
	uint8_t wx_tlm_interval;
	char host_name[32];	
	uint16_t reset_timeout; // minute
	bool at_cmd_mqtt;
	bool at_cmd_msg;
	bool at_cmd_bluetooth;
	uint8_t at_cmd_uart;

	//Message
	bool msg_enable;
	char msg_mycall[10];
	uint8_t msg_path;
	bool msg_rf;
	bool msg_inet;
	bool msg_encrypt;
	char msg_key[33];
	uint8_t msg_retry;
	uint16_t msg_interval;

} Configuration;

bool saveConfiguration(const char *filename, const Configuration &config);
bool loadConfiguration(const char *filename, Configuration &config);

#endif