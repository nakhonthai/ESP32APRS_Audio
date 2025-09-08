/*
 Name:		ESP32APRS
 Created:	25-12-2024
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
*/

#include "config.h"
#include <LITTLEFS.h>
#include <SPIFFS.h>
#include "FS.h"
#include <ArduinoJson.h>
//#include <StreamUtils.h>
#include "AFSK.h"
#include <esp_task_wdt.h>

extern fs::LITTLEFSFS LITTLEFS;

extern TaskHandle_t taskAPRSHandle;
extern TaskHandle_t taskAPRSPollHandle;

//JsonDocument doc;

extern int8_t adcEn;
extern int8_t dacEn;

// Saves the configuration to a file
bool saveConfiguration(const char *filename, const Configuration &config)
{
    // Delete existing file, otherwise the configuration is appended to the file
    //AFSK_TimerEnable(false);
    // if (LITTLEFS.exists(filename))
    // {
    //     LITTLEFS.remove(filename);
    //     log_d("Remove file %s",filename);
    // }

    // Open file for writing
    // File file = LITTLEFS.open(filename, FILE_WRITE);
    // if (!file)
    // {
    //     log_d("Failed to create file");
    //     return false;
    // }
    
    log_d("Create file %s",filename);
    //AFSK_TimerEnable(false);
    // Allocate a temporary JsonDocument
    //adcEn=-1;
    //delay(500);
    //StaticJsonDocument<6000> doc;    
    JsonDocument doc;
    
    // Set the values in the document
    doc["txTimeSlot"] = config.tx_timeslot;
    doc["syncTime"] = config.synctime;
    doc["timeZone"] = config.timeZone;
    doc["ntpHost"] = config.ntp_host;
    doc["WiFiMode"] = (signed char)config.wifi_mode;
    doc["WiFiPwr"] = config.wifi_power;
    doc["WiFiAPCH"] = config.wifi_ap_ch;
    doc["WiFiAP_SSID"] = config.wifi_ap_ssid;
    doc["WiFiAP_PASS"] = config.wifi_ap_pass;
    JsonArray wifiArray = doc["WiFiSTA"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
    {
        wifiArray.add(config.wifi_sta[i].enable);
        wifiArray.add(config.wifi_sta[i].wifi_ssid);
        wifiArray.add(config.wifi_sta[i].wifi_pass);
    }

    doc["fx25Mode"] = config.fx25_mode;
    doc["rfEnable"] = config.rf_en;
    doc["rfType"] = config.rf_type;
    doc["rfModem"] = config.modem_type;    
    doc["rfPreamble"] = config.preamble;
    doc["rfFreqRX"] = config.freq_rx;
    doc["rfFreqTX"] = config.freq_tx;
    doc["rfToneRX"] = config.tone_rx;
    doc["rfToneTX"] = config.tone_tx;
    doc["rfSql"] = config.sql_level;
    doc["rfVolume"] = config.volume;
    doc["rfBand"] = config.band;
    doc["rfPwr"] = config.rf_power;
    doc["audioLPF"] = config.audio_lpf;
    // doc["rfFreq"] = config.rf_freq;
    // doc["rfOffset"] = config.rf_freq_offset;
    // doc["rfBW"] = config.rf_bw;
    // doc["rfSF"] = config.rf_sf;
    // doc["rfCR"] = config.rf_cr;
    // doc["rfSync"] = config.rf_sync;
    // doc["rfPwr"] = config.rf_power;
    // doc["rfPream"] = config.rf_preamable;
    // doc["rfLNA"] = config.rf_lna;
    // doc["rfMode"] = config.rf_mode;
    // doc["rfAX25"] = config.rf_ax25;
    // doc["rfBR"] = config.rf_br;
    // doc["rfShaping"] = config.rf_shaping;
    // doc["rfEncoding"] = config.rf_encoding;

    // IGate group
    doc["igateEn"] = config.igate_en;
    doc["igateBcn"] = config.igate_bcn;
    doc["rf2inet"] = config.rf2inet;
    doc["inet2rf"] = config.inet2rf;
    doc["igatePos2rf"] = config.igate_loc2rf;
    doc["igatePos2inet"] = config.igate_loc2inet;
    doc["rf2inetFilter"] = config.rf2inetFilter;
    doc["inet2rfFiltger"] = config.inet2rfFilter;

    doc["igateSSID"] = config.aprs_ssid;
    doc["igatePort"] = config.aprs_port;
    doc["igateMycall"] = config.aprs_mycall;
    doc["igateHost"] = config.aprs_host;
    // doc["igatePassCode"]=config.aprs_passcode;
    doc["igateFilter"] = config.aprs_filter;
    doc["igateGPS"] = config.igate_gps;
    doc["igateLAT"] = config.igate_lat;
    doc["igateLON"] = config.igate_lon;
    doc["igateALT"] = config.igate_alt;
    doc["igateINV"] = config.igate_interval;
    doc["igateSymbol"] = config.igate_symbol;
    doc["igateObject"] = config.igate_object;
    doc["igatePHG"] = config.igate_phg;
    doc["igatePath"] = config.igate_path;
    doc["igateComment"] = config.igate_comment;
    doc["igateSTSIntv"] = config.igate_sts_interval;
    doc["igateStatus"] = config.igate_status;
    JsonArray igateTlmAvg = doc["igateTlmAvg"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        igateTlmAvg.add(config.igate_tlm_avg[i]);
    JsonArray igateTlmSen = doc["igateTlmSen"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        igateTlmSen.add(config.igate_tlm_sensor[i]);
    JsonArray igateTlmPrec = doc["igateTlmPrec"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        igateTlmPrec.add(config.igate_tlm_precision[i]);
    JsonArray igateTlmOffset = doc["igateTlmOffset"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        igateTlmOffset.add(config.igate_tlm_offset[i]);
    JsonArray igateTlmPARM = doc["igateTlmPARM"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        igateTlmPARM.add(config.igate_tlm_PARM[i]);
    JsonArray igateTlmUNIT = doc["igateTlmUNIT"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        igateTlmUNIT.add(config.igate_tlm_UNIT[i]);
    JsonArray igateTlmEQNS = doc["igateTlmEQNS"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
    {
        igateTlmEQNS.add(config.igate_tlm_EQNS[i][0]);
        igateTlmEQNS.add(config.igate_tlm_EQNS[i][1]);
        igateTlmEQNS.add(config.igate_tlm_EQNS[i][2]);
    }

    // Digi group
    doc["digiEn"] = config.digi_en;
    doc["digiPos2rf"] = config.digi_loc2rf;
    doc["digiPos2inet"] = config.digi_loc2inet;
    doc["digiTime"] = config.digi_timestamp;
    doc["digiSSID"] = config.digi_ssid;
    doc["digiMycall"] = config.digi_mycall;
    doc["digiPath"] = config.digi_path;
    doc["digiDelay"] = config.digi_delay;
    doc["digiFilter"] = config.digiFilter;
    doc["digiBcn"] = config.digi_bcn;
    doc["digiAlt"] = config.digi_alt;
    doc["digiGPS"] = config.digi_gps;
    doc["digiLAT"] = config.digi_lat;
    doc["digiLON"] = config.digi_lon;
    doc["digiINV"] = config.digi_interval;
    doc["digiSymbol"] = config.digi_symbol;
    doc["digiPHG"] = config.digi_phg;
    doc["digiComment"] = config.digi_comment;
    doc["digiSTSIntv"] = config.digi_sts_interval;
    doc["digiStatus"] = config.digi_status;
    JsonArray digiTlmAvg = doc["digiTlmAvg"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        digiTlmAvg.add(config.digi_tlm_avg[i]);
    JsonArray digiTlmSen = doc["digiTlmSen"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        digiTlmSen.add(config.digi_tlm_sensor[i]);
    JsonArray digiTlmPrec = doc["digiTlmPrec"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        digiTlmPrec.add(config.digi_tlm_precision[i]);
    JsonArray digiTlmOffset = doc["digiTlmOffset"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        digiTlmOffset.add(config.digi_tlm_offset[i]);
    JsonArray digiTlmPARM = doc["digiTlmPARM"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        digiTlmPARM.add(config.digi_tlm_PARM[i]);
    JsonArray digiTlmUNIT = doc["digiTlmUNIT"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        digiTlmUNIT.add(config.digi_tlm_UNIT[i]);
    JsonArray digiTlmEQNS = doc["digiTlmEQNS"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
    {
        digiTlmEQNS.add(config.digi_tlm_EQNS[i][0]);
        digiTlmEQNS.add(config.digi_tlm_EQNS[i][1]);
        digiTlmEQNS.add(config.digi_tlm_EQNS[i][2]);
    }

    // Tracker group
    doc["trkEn"] = config.trk_en;
    doc["trkPos2rf"] = config.trk_loc2rf;
    doc["trkPos2inet"] = config.trk_loc2inet;
    doc["trkTime"] = config.trk_timestamp;
    doc["trkSSID"] = config.trk_ssid;
    doc["trkMycall"] = config.trk_mycall;
    doc["trkPath"] = config.trk_path;
    doc["trkGPS"] = config.trk_gps;
    doc["trkLAT"] = config.trk_lat;
    doc["trkLON"] = config.trk_lon;
    doc["trkALT"] = config.trk_alt;
    doc["trkINV"] = config.trk_interval;
    doc["trkSmart"] = config.trk_smartbeacon;
    doc["trkCompress"] = config.trk_compress;
    doc["trkOptAlt"] = config.trk_altitude;
    doc["trkLog"] = config.trk_log;
    doc["trkOptRSSI"] = config.trk_rssi;
    doc["trkLSpeed"] = config.trk_lspeed;
    doc["trkHSpeed"] = config.trk_hspeed;
    doc["trkMaxInv"] = config.trk_maxinterval;
    doc["trkMinInv"] = config.trk_mininterval;
    doc["trkMinDir"] = config.trk_minangle;
    doc["trkSlowInv"] = config.trk_slowinterval;
    doc["trkSymbol"] = config.trk_symbol;
    doc["trkSymbolMove"] = config.trk_symmove;
    doc["trkSymbolStop"] = config.trk_symstop;
    doc["trkItem"] = config.trk_item;
    doc["trkComment"] = config.trk_comment;
    doc["trkSTSIntv"] = config.trk_sts_interval;
    doc["trkStatus"] = config.trk_status;
    JsonArray trkTlmAvg = doc["trkTlmAvg"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        trkTlmAvg.add(config.trk_tlm_avg[i]);
    JsonArray trkTlmSen = doc["trkTlmSen"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        trkTlmSen.add(config.trk_tlm_sensor[i]);
    JsonArray trkTlmPrec = doc["trkTlmPrec"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        trkTlmPrec.add(config.trk_tlm_precision[i]);
    JsonArray trkTlmOffset = doc["trkTlmOffset"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        trkTlmOffset.add(config.trk_tlm_offset[i]);
    JsonArray trkTlmPARM = doc["trkTlmPARM"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        trkTlmPARM.add(config.trk_tlm_PARM[i]);
    JsonArray trkTlmUNIT = doc["trkTlmUNIT"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
        trkTlmUNIT.add(config.trk_tlm_UNIT[i]);
    JsonArray trkTlmEQNS = doc["trkTlmEQNS"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
    {
        trkTlmEQNS.add(config.trk_tlm_EQNS[i][0]);
        trkTlmEQNS.add(config.trk_tlm_EQNS[i][1]);
        trkTlmEQNS.add(config.trk_tlm_EQNS[i][2]);
    }

    // WX group
    doc["wxEn"] = config.wx_en;
    doc["wxTx2rf"] = config.wx_2rf;
    doc["wxTx2inet"] = config.wx_2inet;
    doc["wxTime"] = config.wx_timestamp;
    doc["wxSSID"] = config.wx_ssid;
    doc["wxMycall"] = config.wx_mycall;
    doc["wxPath"] = config.wx_path;
    doc["wxGPS"] = config.wx_gps;
    doc["wxLAT"] = config.wx_lat;
    doc["wxLON"] = config.wx_lon;
    doc["wxALT"] = config.wx_alt;
    doc["wxInv"] = config.wx_interval;
    doc["wxFlage"] = config.wx_flage;
    doc["wxObject"] = config.wx_object;
    doc["wxComment"] = config.wx_comment;

    JsonArray wxSenEn = doc["wxSenEn"].to<JsonArray>();
    for (int i = 0; i < WX_SENSOR_NUM; i++)
    {
        wxSenEn.add(config.wx_sensor_enable[i]);
    }
    JsonArray wxSenAvg = doc["wxSenAvg"].to<JsonArray>();
    for (int i = 0; i < WX_SENSOR_NUM; i++)
    {
        wxSenAvg.add(config.wx_sensor_avg[i]);
    }
    JsonArray wxSenCH = doc["wxSenCH"].to<JsonArray>();
    for (int i = 0; i < WX_SENSOR_NUM; i++)
    {
        wxSenCH.add(config.wx_sensor_ch[i]);
    }

    // Telemetry
    doc["tlmEn"] = config.tlm0_en;
    doc["tlmTx2rf"] = config.tlm0_2rf;
    doc["tlmTx2inet"] = config.tlm0_2inet;
    doc["tlmSSID"] = config.tlm0_ssid;
    doc["tlmMycall"] = config.tlm0_mycall;
    doc["tlmPath"] = config.tlm0_path;
    doc["tlmInfoInv"] = config.tlm0_info_interval;
    doc["tlmDataInv"] = config.tlm0_data_interval;
    doc["tlmBIT"] = config.tlm0_BITS_Active;
    // doc["tlmDataCH"]=config.tml0_data_channel;
    JsonArray tlmEQNS = doc["tlmEQNS"].to<JsonArray>();
    for (int i = 0; i < 5; i++)
    {
        tlmEQNS.add(config.tlm0_EQNS[i][0]);
        tlmEQNS.add(config.tlm0_EQNS[i][1]);
        tlmEQNS.add(config.tlm0_EQNS[i][2]);
    }
    JsonArray tlmPARM = doc["tlmPARM"].to<JsonArray>();
    for (int i = 0; i < 13; i++)
    {
        tlmPARM.add(config.tlm0_PARM[i]);
    }
    JsonArray tlmUNIT = doc["tlmUNIT"].to<JsonArray>();
    for (int i = 0; i < 13; i++)
    {
        tlmUNIT.add(config.tlm0_UNIT[i]);
    }
    JsonArray tlmDataCH = doc["tlmDataCH"].to<JsonArray>();
    for (int i = 0; i < 13; i++)
    {
        tlmDataCH.add(config.tml0_data_channel[i]);
    }

    // OLED Display
    doc["dspEn"] = config.oled_enable;
    doc["dspTOut"] = config.oled_timeout;
    doc["dspDim"] = config.dim;
    doc["dspContrast"] = config.contrast;
    doc["dspBright"] = config.disp_brightness;
    doc["dspStartUp"] = config.startup;
    doc["dspDelay"] = config.dispDelay;
    doc["dspDxFilter"] = config.filterDistant;
    doc["dspHUp"] = config.h_up;
    doc["dspTX"] = config.tx_display;
    doc["dspRX"] = config.rx_display;
    doc["dspFilter"] = config.dispFilter;
    doc["dspRF"] = config.dispRF;
    doc["dspINET"] = config.dispINET;
    doc["dspFlip"] = config.disp_flip;

    // VPN Wireguard
    doc["vpnEn"] = config.vpn;
    doc["vpnPort"] = config.wg_port;
    doc["vpnPeer"] = config.wg_peer_address;
    doc["vpnLocal"] = config.wg_local_address;
    doc["vpnNetmark"] = config.wg_netmask_address;
    doc["vpnGW"] = config.wg_gw_address;
    doc["vpnPubKey"] = config.wg_public_key;
    doc["vpnPriKey"] = config.wg_private_key;

    // System
    doc["httpUser"] = config.http_username;
    doc["httpPass"] = config.http_password;

    JsonArray path = doc["path"].to<JsonArray>();
    for (int i = 0; i < 4; i++)
    {
        path.add(config.path[i]);
    }

    // MOD GNSS group
    doc["gnssEn"] = config.gnss_enable;
    doc["gnssCH"] = config.gnss_channel;
    doc["gnssPPS"] = config.gnss_pps_gpio;
    doc["gnssTCPPort"] = config.gnss_tcp_port;
    doc["gnssTCPHost"] = config.gnss_tcp_host;
    doc["gnssAT"] = config.gnss_at_command;

    // MOD RF group
    doc["rfTx"] = config.rf_tx_gpio;
    doc["rfRx"] = config.rf_rx_gpio;
    doc["rfSQL"] = config.rf_sql_gpio;
    doc["rfPD"] = config.rf_pd_gpio;
    doc["rfPWR"] = config.rf_pwr_gpio;
    doc["rfPTT"] = config.rf_ptt_gpio;
    doc["rfSQLAct"] = config.rf_sql_active;
    doc["rfPDAct"] = config.rf_pd_active;
    doc["rfPWRAct"] = config.rf_pwr_active;
    doc["rfPTTAct"] = config.rf_ptt_active;
    doc["adcAtten"] = config.adc_atten;
    doc["adcOffset"] = config.adc_dc_offset;
    doc["rfBaudrate"] = config.rf_baudrate;

    // doc["rfRST"] = config.rf_reset_gpio;
    // doc["rfDIO0"] = config.rf_dio0_gpio;
    // doc["rfDIO1"] = config.rf_dio1_gpio;
    // doc["rfDIO2"] = config.rf_dio2_gpio;
    // doc["rfNSS"] = config.rf_nss_gpio;
    // doc["rfSCK"] = config.rf_sclk_gpio;
    // doc["rfMISO"] = config.rf_miso_gpio;
    // doc["rfMOSI"] = config.rf_mosi_gpio;
    // doc["rfTxAct"] = config.rf_tx_active;
    // doc["rfRxAct"] = config.rf_rx_active;
    // doc["rfRSTAct"] = config.rf_reset_active;
    // doc["rfNSSAct"] = config.rf_nss_active;
    // MOD I2C group
    doc["i2cEn"] = config.i2c_enable;
    doc["i2cSDA"] = config.i2c_sda_pin;
    doc["i2cSCK"] = config.i2c_sck_pin;
    // doc["i2cRST"]=config.i2c_rst_pin;
    doc["i2cFreq"] = config.i2c_freq;
    doc["i2c1En"] = config.i2c1_enable;
    doc["i2c1SDA"] = config.i2c1_sda_pin;
    doc["i2c1SCK"] = config.i2c1_sck_pin;
    doc["i2c1Freq"] = config.i2c1_freq;
    // MOD 1-Wire group
    doc["oneWireEn"] = config.onewire_enable;
    doc["oneWireIO"] = config.onewire_gpio;

    // MOD UART
    doc["uart0En"] = config.uart0_enable;
    doc["uart0BR"] = config.uart0_baudrate;
    doc["uart0TX"] = config.uart0_tx_gpio;
    doc["uart0RX"] = config.uart0_rx_gpio;
    doc["uart0RTS"] = config.uart0_rts_gpio;
    doc["uart1En"] = config.uart1_enable;
    doc["uart1BR"] = config.uart1_baudrate;
    doc["uart1TX"] = config.uart1_tx_gpio;
    doc["uart1RX"] = config.uart1_rx_gpio;
    doc["uart1RTS"] = config.uart1_rts_gpio;
    // MOD Modbus
    doc["modbusEn"] = config.modbus_enable;
    doc["modbusAddr"] = config.modbus_address;
    doc["modbusCh"] = config.modbus_channel;
    doc["modbusDE"] = config.modbus_de_gpio;
    // MOD Counter
    doc["cnt0En"] = config.counter0_enable;
    doc["cnt0Act"] = config.counter0_active;
    doc["cnt0IO"] = config.counter0_gpio;
    doc["cnt1En"] = config.counter1_enable;
    doc["cnt1Act"] = config.counter1_active;
    doc["cnt1IO"] = config.counter1_gpio;
    // MOD External TNC
    doc["extTNCEn"] = config.ext_tnc_enable;
    doc["extTNCCh"] = config.ext_tnc_channel;
    doc["extTNCMode"] = config.ext_tnc_mode;

    // Power control
    doc["pwrEn"] = config.pwr_en;
    doc["pwrMode"] = config.pwr_mode;
    doc["pwrSleep"] = config.pwr_sleep_interval;
    doc["pwrStanby"] = config.pwr_stanby_delay;
    doc["pwrSleepAct"] = config.pwr_sleep_activate;
    doc["pwrIO"] = config.pwr_gpio;
    doc["pwrIOAct"] = config.pwr_active;

    doc["logFile"] = config.log;

    JsonArray sensor = doc["Sensor"].to<JsonArray>();
    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        sensor.add(config.sensor[i].enable);
        sensor.add(config.sensor[i].port);
        sensor.add(config.sensor[i].address);
        sensor.add(config.sensor[i].samplerate);
        sensor.add(config.sensor[i].averagerate);
        sensor.add(config.sensor[i].eqns[0]);
        sensor.add(config.sensor[i].eqns[1]);
        sensor.add(config.sensor[i].eqns[2]);
        sensor.add(config.sensor[i].type);
        sensor.add(config.sensor[i].parm);
        sensor.add(config.sensor[i].unit);
    }

     // PPP Modem
    doc["pppEn"] = config.ppp_enable;
    doc["pppAPN"] = config.ppp_apn;
    doc["pppRST"] = config.ppp_rst_gpio;
    doc["pppRSTAct"] = config.ppp_rst_active;
    doc["pppTX"] = config.ppp_tx_gpio;
    doc["pppRX"] = config.ppp_rx_gpio;
    doc["pppRTS"] = config.ppp_rts_gpio;
    doc["pppDTR"] = config.ppp_dtr_gpio;
    doc["pppCTS"] = config.ppp_cts_gpio;
    doc["pppRI"] = config.ppp_ri_gpio;
    doc["pppPWR"] = config.ppp_pwr_gpio;
    doc["pppPWRAct"] = config.ppp_pwr_active;
    doc["pppRSTDelay"] = config.ppp_rst_delay;
    doc["pppPin"] = config.ppp_pin;
    doc["pppSerial"] = config.ppp_serial;
    doc["pppSerialBaudrate"] = config.ppp_serial_baudrate;
    doc["pppModel"] = config.ppp_model;
    doc["pppFlow"] = config.ppp_flow_ctrl;
    doc["pppGNSS"] = config.ppp_gnss;

    doc["mqttEnable"] = config.en_mqtt;
    doc["mqttHost"] = config.mqtt_host;
    doc["mqttTopic"] = config.mqtt_topic;
    doc["mqttSub"] = config.mqtt_subscribe;
    doc["mqttPort"] = config.mqtt_port;

    doc["trkMicEType"] = config.trk_mice_type;
    doc["trkTlmInv"] = config.trk_tlm_interval;
    doc["digiTlmInv"] = config.digi_tlm_interval;
    doc["igateTlmInv"] = config.igate_tlm_interval;
    doc["hostName"] = config.host_name;

    // Serialize JSON to file
    File file = LITTLEFS.open(filename, FILE_WRITE);
    if (file)
    {
        int8_t adcEnBak=adcEn;
        int8_t dacEnBak=dacEn;
        //adcEn=-1;
        //dacEn=-1;
        
        //delay(500);
        if (serializeJson(doc, file) == 0)
        {
            log_d("Failed to write to file");
        }else{
            log_d("Write file configuration susses.");
        }
        // Close the file
        file.close();
        delay(100);
        //adcEn=1;
        dacEn=0;
    }
    doc.clear();
    return true;
}

// Loads the configuration from a file
bool loadConfiguration(const char *filename, Configuration &config)
{

    if (!LITTLEFS.exists(filename))
    {
        log_d("File %s not found.", filename);
        return false;
    }

    // Open file for reading
    File f = LITTLEFS.open(filename);
    if (f)
    {
        // Open file for reading
        log_d("Loading %s file. Size=%d", filename, f.size());

        // String str=f.readString();
        // log_d("%s",str.c_str());
        //  Allocate a temporary JsonDocument
        JsonDocument doc;

        // Deserialize the JSON document
        DeserializationError error = deserializeJson(doc, f);
        f.close();
        if (error)
        {
            log_d("Failed to read file, using default configuration");
            return false;
        }

        config.tx_timeslot = doc["txTimeSlot"] | 2000;
        config.synctime = doc["syncTime"];
        config.timeZone = doc["timeZone"];
        strlcpy(config.ntp_host, doc["ntpHost"], sizeof(config.ntp_host));
        config.wifi_mode = doc["WiFiMode"];
        config.wifi_power = doc["WiFiPwr"];
        config.wifi_ap_ch = doc["WiFiAPCH"];
        strlcpy(config.wifi_ap_ssid, doc["WiFiAP_SSID"] | "ESP32LoRa", sizeof(config.wifi_ap_ssid));
        strlcpy(config.wifi_ap_pass, doc["WiFiAP_PASS"] | "aprsthnetwork", sizeof(config.wifi_ap_pass));
        // log_d("Load WiFiAP: SSID=%s Pass=%s",config.wifi_ap_ssid,config.wifi_ap_pass);
        for (int i = 0; i < 5; i++)
        {
            config.wifi_sta[i].enable = doc["WiFiSTA"][i * 3];
            strlcpy(config.wifi_sta[i].wifi_ssid, doc["WiFiSTA"][(i * 3) + 1] | "APRSTH", sizeof(config.wifi_sta[i].wifi_ssid));
            strlcpy(config.wifi_sta[i].wifi_pass, doc["WiFiSTA"][(i * 3) + 2] | "aprsthnetwork", sizeof(config.wifi_sta[i].wifi_pass));
            // if(config.wifi_sta[i].enable)
            // log_d("Load WiFiSTA[%i]: SSID=%s Pass=%s",i,config.wifi_sta[i].wifi_ssid,config.wifi_sta[i].wifi_pass);
        }

        config.fx25_mode = doc["fx25Mode"];
        config.rf_en = doc["rfEnable"];
        config.rf_type = doc["rfType"];
        config.rf_power = doc["rfPwr"];
        config.modem_type = doc["rfModem"];
        config.preamble = doc["rfPreamble"];
        config.freq_rx = doc["rfFreqRX"];
        config.freq_tx = doc["rfFreqTX"];
        config.tone_rx = doc["rfToneRX"];
        config.tone_tx = doc["rfToneTX"];
        config.sql_level = doc["rfSql"];
        config.volume = doc["rfVolume"];
        config.band = doc["rfBand"];
        config.audio_lpf = doc["audioLPF"];
        // config.rf_freq = doc["rfFreq"];
        // config.rf_freq_offset = doc["rfOffset"];
        // config.rf_bw = doc["rfBW"];
        // config.rf_sf = doc["rfSF"];
        // config.rf_cr = doc["rfCR"];
        // config.rf_sync = doc["rfSync"];
        // config.rf_power = doc["rfPwr"];
        // config.rf_preamable = doc["rfPream"];
        // config.rf_lna = doc["rfLNA"];
        // config.rf_mode = doc["rfMode"];
        // config.rf_ax25 = doc["rfAX25"];
        // config.rf_br = doc["rfBR"];
        // config.rf_shaping = doc["rfShaping"];
        // config.rf_encoding = doc["rfEncoding"];

        // IGate group
        config.igate_en = doc["igateEn"];
        config.igate_bcn = doc["igateBcn"];
        config.rf2inet = doc["rf2inet"];
        config.inet2rf = doc["inet2rf"];
        config.igate_loc2rf = doc["igatePos2rf"];
        config.igate_loc2inet = doc["igatePos2inet"];
        config.rf2inetFilter = doc["rf2inetFilter"];
        config.inet2rfFilter = doc["inet2rfFiltger"];

        config.aprs_ssid = doc["igateSSID"];
        config.aprs_port = doc["igatePort"];
        strlcpy(config.aprs_mycall, doc["igateMycall"] | "NOCALL", sizeof(config.aprs_mycall));
        strlcpy(config.aprs_host, doc["igateHost"] | "", sizeof(config.aprs_host));
        strlcpy(config.aprs_filter, doc["igateFilter"] | "", sizeof(config.aprs_filter));
        config.igate_gps = doc["igateGPS"];
        config.igate_lat = doc["igateLAT"];
        config.igate_lon = doc["igateLON"];
        config.igate_alt = doc["igateALT"];
        config.igate_interval = doc["igateINV"];
        strlcpy(config.igate_symbol, doc["igateSymbol"] | "", sizeof(config.igate_symbol));
        strlcpy(config.igate_object, doc["igateObject"] | "", sizeof(config.igate_object));
        strlcpy(config.igate_phg, doc["igatePHG"] | "", sizeof(config.igate_phg));
        strlcpy(config.igate_comment, doc["igateComment"] | "", sizeof(config.igate_comment));
        strlcpy(config.igate_status, doc["igateStatus"] | "", sizeof(config.igate_status));
        config.igate_sts_interval = doc["igateSTSIntv"];
        config.igate_path = doc["igatePath"];

        for (int i = 0; i < 5; i++)
            config.igate_tlm_avg[i] = doc["igateTlmAvg"][i];
        for (int i = 0; i < 5; i++)
            config.igate_tlm_sensor[i] = doc["igateTlmSen"][i];
        for (int i = 0; i < 5; i++)
            config.igate_tlm_precision[i] = doc["igateTlmPrec"][i];
        for (int i = 0; i < 5; i++)
            config.igate_tlm_offset[i] = doc["igateTlmOffset"][i];
        for (int i = 0; i < 5; i++)
            strlcpy(config.igate_tlm_PARM[i], doc["igateTlmPARM"][i] | "", sizeof(config.igate_tlm_PARM[i]));
        for (int i = 0; i < 5; i++)
            strlcpy(config.igate_tlm_UNIT[i], doc["igateTlmUNIT"][i] | "", sizeof(config.igate_tlm_UNIT[i]));
        for (int i = 0; i < 5; i++)
        {
            config.igate_tlm_EQNS[i][0] = doc["igateTlmEQNS"][i * 3];
            config.igate_tlm_EQNS[i][1] = doc["igateTlmEQNS"][(i * 3) + 1];
            config.igate_tlm_EQNS[i][2] = doc["igateTlmEQNS"][(i * 3) + 2];
        }
        // Digi group
        config.digi_en = doc["digiEn"];
        config.digi_loc2rf = doc["digiPos2rf"];
        config.digi_loc2inet = doc["digiPos2inet"];
        config.digi_timestamp = doc["digiTime"];
        config.digi_ssid = doc["digiSSID"];
        strlcpy(config.digi_mycall, doc["digiMycall"] | "", sizeof(config.digi_mycall));
        config.digi_path = doc["digiPath"];
        config.digi_delay = doc["digiDelay"];
        config.digiFilter = doc["digiFilter"];
        config.digi_bcn = doc["digiBcn"];
        config.digi_alt = doc["digiAlt"];
        config.digi_gps = doc["digiGPS"];
        config.digi_lat = doc["digiLAT"];
        config.digi_lon = doc["digiLON"];
        config.digi_interval = doc["digiINV"];
        strlcpy(config.digi_symbol, doc["digiSymbol"] | "", sizeof(config.digi_symbol));
        strlcpy(config.digi_phg, doc["digiPHG"] | "", sizeof(config.digi_phg));
        strlcpy(config.digi_comment, doc["digiComment"] | "", sizeof(config.digi_comment));
        strlcpy(config.digi_status, doc["digiStatus"] | "", sizeof(config.digi_status));
        config.digi_sts_interval = doc["digiSTSIntv"];
        for (int i = 0; i < 5; i++)
            config.digi_tlm_avg[i] = doc["digiTlmAvg"][i];
        for (int i = 0; i < 5; i++)
            config.digi_tlm_sensor[i] = doc["digiTlmSen"][i];
        for (int i = 0; i < 5; i++)
            config.digi_tlm_precision[i] = doc["digiTlmPrec"][i];
        for (int i = 0; i < 5; i++)
            config.digi_tlm_offset[i] = doc["digiTlmOffset"][i];
        for (int i = 0; i < 5; i++)
            strlcpy(config.digi_tlm_PARM[i], doc["digiTlmPARM"][i] | "", sizeof(config.digi_tlm_PARM[i]));
        for (int i = 0; i < 5; i++)
            strlcpy(config.digi_tlm_UNIT[i], doc["digiTlmUNIT"][i] | "", sizeof(config.digi_tlm_UNIT[i]));
        for (int i = 0; i < 5; i++)
        {
            config.digi_tlm_EQNS[i][0] = doc["digiTlmEQNS"][i * 3];
            config.digi_tlm_EQNS[i][1] = doc["digiTlmEQNS"][(i * 3) + 1];
            config.digi_tlm_EQNS[i][2] = doc["digiTlmEQNS"][(i * 3) + 2];
        }
        // Tracker group
        config.trk_en = doc["trkEn"];
        config.trk_loc2rf = doc["trkPos2rf"];
        config.trk_loc2inet = doc["trkPos2inet"];
        config.trk_timestamp = doc["trkTime"];
        config.trk_ssid = doc["trkSSID"];
        strlcpy(config.trk_mycall, doc["trkMycall"] | "", sizeof(config.trk_mycall));
        config.trk_path = doc["trkPath"];
        config.trk_gps = doc["trkGPS"];
        config.trk_lat = doc["trkLAT"];
        config.trk_lon = doc["trkLON"];
        config.trk_alt = doc["trkALT"];
        config.trk_interval = doc["trkINV"];
        config.trk_smartbeacon = doc["trkSmart"];
        config.trk_compress = doc["trkCompress"];
        config.trk_altitude = doc["trkOptAlt"];
        config.trk_log = doc["trkLog"];
        config.trk_rssi = doc["trkOptRSSI"];
        config.trk_lspeed = doc["trkLSpeed"];
        config.trk_hspeed = doc["trkHSpeed"];
        config.trk_maxinterval = doc["trkMaxInv"];
        config.trk_mininterval = doc["trkMinInv"];
        config.trk_minangle = doc["trkMinDir"];
        config.trk_slowinterval = doc["trkSlowInv"];
        strlcpy(config.trk_symbol, doc["trkSymbol"] | "", sizeof(config.trk_symbol));
        strlcpy(config.trk_symmove, doc["trkSymbolMove"] | "", sizeof(config.trk_symmove));
        strlcpy(config.trk_symstop, doc["trkSymbolStop"] | "", sizeof(config.trk_symstop));
        strlcpy(config.trk_item, doc["trkItem"] | "", sizeof(config.trk_item));
        strlcpy(config.trk_comment, doc["trkComment"] | "", sizeof(config.trk_comment));
        strlcpy(config.trk_status, doc["trkStatus"] | "", sizeof(config.trk_status));
        config.trk_sts_interval = doc["trkSTSIntv"];
        for (int i = 0; i < 5; i++)
            config.trk_tlm_avg[i] = doc["trkTlmAvg"][i];
        for (int i = 0; i < 5; i++)
            config.trk_tlm_sensor[i] = doc["trkTlmSen"][i];
        for (int i = 0; i < 5; i++)
            config.trk_tlm_precision[i] = doc["trkTlmPrec"][i];
        for (int i = 0; i < 5; i++)
            config.trk_tlm_offset[i] = doc["trkTlmOffset"][i];
        for (int i = 0; i < 5; i++)
            strlcpy(config.trk_tlm_PARM[i], doc["trkTlmPARM"][i] | "", sizeof(config.trk_tlm_PARM[i]));
        for (int i = 0; i < 5; i++)
            strlcpy(config.trk_tlm_UNIT[i], doc["trkTlmUNIT"][i] | "", sizeof(config.trk_tlm_UNIT[i]));
        for (int i = 0; i < 5; i++)
        {
            config.trk_tlm_EQNS[i][0] = doc["trkTlmEQNS"][i * 3];
            config.trk_tlm_EQNS[i][1] = doc["trkTlmEQNS"][(i * 3) + 1];
            config.trk_tlm_EQNS[i][2] = doc["trkTlmEQNS"][(i * 3) + 2];
        }
        // WX group
        config.wx_en = doc["wxEn"];
        config.wx_2rf = doc["wxTx2rf"];
        config.wx_2inet = doc["wxTx2inet"];
        config.wx_timestamp = doc["wxTime"];
        config.wx_ssid = doc["wxSSID"];
        strlcpy(config.wx_mycall, doc["wxMycall"] | "", sizeof(config.wx_mycall));
        config.wx_path = doc["wxPath"];
        config.wx_gps = doc["wxGPS"];
        config.wx_lat = doc["wxLAT"];
        config.wx_lon = doc["wxLON"];
        config.wx_alt = doc["wxALT"];
        config.wx_interval = doc["wxInv"];
        config.wx_flage = doc["wxFlage"];
        strlcpy(config.wx_object, doc["wxObject"] | "", sizeof(config.wx_object));
        strlcpy(config.wx_comment, doc["wxComment"] | "", sizeof(config.wx_comment));

        for (int i = 0; i < WX_SENSOR_NUM; i++)
        {
            config.wx_sensor_enable[i] = doc["wxSenEn"][i];
        }
        for (int i = 0; i < WX_SENSOR_NUM; i++)
        {
            config.wx_sensor_avg[i] = doc["wxSenAvg"][i];
        }
        for (int i = 0; i < WX_SENSOR_NUM; i++)
        {
            config.wx_sensor_ch[i] = doc["wxSenCH"][i];
        }

        // Telemetry
        config.tlm0_en = doc["tlmEn"];
        config.tlm0_2rf = doc["tlmTx2rf"];
        config.tlm0_2inet = doc["tlmTx2inet"];
        config.tlm0_ssid = doc["tlmSSID"];
        strlcpy(config.tlm0_mycall, doc["tlmMycall"] | "", sizeof(config.tlm0_mycall));
        config.tlm0_path = doc["tlmPath"];
        config.tlm0_info_interval = doc["tlmInfoInv"];
        config.tlm0_data_interval = doc["tlmDataInv"];
        config.tlm0_BITS_Active = doc["tlmBIT"];

        for (int i = 0; i < 5; i++)
        {
            config.tlm0_EQNS[i][0] = doc["tlmEQNS"][i * 3];
            config.tlm0_EQNS[i][1] = doc["tlmEQNS"][(i * 3) + 1];
            config.tlm0_EQNS[i][2] = doc["tlmEQNS"][(i * 3) + 2];
        }
        for (int i = 0; i < 13; i++)
        {
            strlcpy(config.tlm0_PARM[i], doc["tlmPARM"][i] | "", sizeof(config.tlm0_PARM[i]));
        }
        for (int i = 0; i < 13; i++)
        {
            strlcpy(config.tlm0_UNIT[i], doc["tlmUNIT"][i] | "", sizeof(config.tlm0_UNIT[i]));
        }
        for (int i = 0; i < 13; i++)
        {
            config.tml0_data_channel[i] = doc["tlmDataCH"][i];
        }
        // OLED Display
        config.oled_enable = doc["dspEn"];
        config.oled_timeout = doc["dspTOut"];
        config.dim = doc["dspDim"];
        config.contrast = doc["dspContrast"];
        config.disp_brightness = doc["dspBright"];
        config.startup = doc["dspStartUp"];
        config.dispDelay = doc["dspDelay"];
        config.filterDistant = doc["dspDxFilter"];
        config.h_up = doc["dspHUp"];
        config.tx_display = doc["dspTX"];
        config.rx_display = doc["dspRX"];
        config.dispFilter = doc["dspFilter"];
        config.dispRF = doc["dspRF"];
        config.dispINET = doc["dspINET"];
        config.disp_flip = doc["dspFlip"];

        // VPN Wireguard
        config.vpn = doc["vpnEn"];
        config.wg_port = doc["vpnPort"];
        strlcpy(config.wg_peer_address, doc["vpnPeer"] | "", sizeof(config.wg_peer_address));
        strlcpy(config.wg_local_address, doc["vpnLocal"] | "", sizeof(config.wg_local_address));
        strlcpy(config.wg_netmask_address, doc["vpnNetmark"] | "", sizeof(config.wg_netmask_address));
        strlcpy(config.wg_gw_address, doc["vpnGW"] | "", sizeof(config.wg_gw_address));
        strlcpy(config.wg_public_key, doc["vpnPubKey"] | "", sizeof(config.wg_public_key));
        strlcpy(config.wg_private_key, doc["vpnPriKey"] | "", sizeof(config.wg_private_key));

        // System
        strlcpy(config.http_username, doc["httpUser"] | "", sizeof(config.http_username));
        strlcpy(config.http_password, doc["httpPass"] | "", sizeof(config.http_password));

        for (int i = 0; i < 4; i++)
        {
            strlcpy(config.path[i], doc["path"][i] | "", sizeof(config.path[i]));
        }

        // MOD GNSS group
        config.gnss_enable = doc["gnssEn"];
        config.gnss_channel = doc["gnssCH"];
        config.gnss_pps_gpio = doc["gnssPPS"];
        config.gnss_tcp_port = doc["gnssTCPPort"];
        strlcpy(config.gnss_tcp_host, doc["gnssTCPHost"] | "", sizeof(config.gnss_tcp_host));
        strlcpy(config.gnss_at_command, doc["gnssAT"] | "", sizeof(config.gnss_at_command));

        // MOD RF group
        config.rf_tx_gpio = doc["rfTx"];
        config.rf_rx_gpio = doc["rfRx"];
        config.rf_sql_gpio = doc["rfSQL"];
        config.rf_pd_gpio = doc["rfPD"];
        config.rf_pwr_gpio = doc["rfPWR"];
        config.rf_ptt_gpio = doc["rfPTT"];
        config.rf_sql_active = doc["rfSQLAct"];
        config.rf_pd_active = doc["rfPDAct"];
        config.rf_pwr_active = doc["rfPWRAct"];
        config.rf_ptt_active = doc["rfPTTAct"];
        config.rf_baudrate = doc["rfBaudrate"];
        config.adc_atten = doc["adcAtten"];
        config.adc_dc_offset = doc["adcOffset"];          

        // config.rf_reset_gpio = doc["rfRST"];
        // config.rf_dio0_gpio = doc["rfDIO0"];
        // config.rf_dio1_gpio = doc["rfDIO1"];
        // config.rf_dio2_gpio = doc["rfDIO2"];
        // config.rf_nss_gpio = doc["rfNSS"];
        // config.rf_sclk_gpio = doc["rfSCK"];
        // config.rf_miso_gpio = doc["rfMISO"];
        // config.rf_mosi_gpio = doc["rfMOSI"];
        // config.rf_tx_active = doc["rfTxAct"];
        // config.rf_rx_active = doc["rfRxAct"];
        // config.rf_reset_active = doc["rfRSTAct"];
        // config.rf_nss_active = doc["rfNSSAct"];
        // MOD I2C group
        config.i2c_enable = doc["i2cEn"];
        config.i2c_sda_pin = doc["i2cSDA"];
        config.i2c_sck_pin = doc["i2cSCK"];
        // doc["i2cRST"]=config.i2c_rst_pin;
        config.i2c_freq = doc["i2cFreq"];
        config.i2c1_enable = doc["i2c1En"];
        config.i2c1_sda_pin = doc["i2c1SDA"];
        config.i2c1_sck_pin = doc["i2c1SCK"];
        config.i2c1_freq = doc["i2c1Freq"];
        // MOD 1-Wire group
        config.onewire_enable = doc["oneWireEn"];
        config.onewire_gpio = doc["oneWireIO"];

        // MOD UART
        config.uart0_enable = doc["uart0En"];
        config.uart0_baudrate = doc["uart0BR"];
        config.uart0_tx_gpio = doc["uart0TX"];
        config.uart0_rx_gpio = doc["uart0RX"];
        config.uart0_rts_gpio = doc["uart0RTS"];
        config.uart1_enable = doc["uart1En"];
        config.uart1_baudrate = doc["uart1BR"];
        config.uart1_tx_gpio = doc["uart1TX"];
        config.uart1_rx_gpio = doc["uart1RX"];
        config.uart1_rts_gpio = doc["uart1RTS"];
        // MOD Modbus
        config.modbus_enable = doc["modbusEn"];
        config.modbus_address = doc["modbusAddr"];
        config.modbus_channel = doc["modbusCh"];
        config.modbus_de_gpio = doc["modbusDE"];
        // MOD Counter
        config.counter0_enable = doc["cnt0En"];
        config.counter0_active = doc["cnt0Act"];
        config.counter0_gpio = doc["cnt0IO"];
        config.counter1_enable = doc["cnt1En"];
        config.counter1_active = doc["cnt1Act"];
        config.counter1_gpio = doc["cnt1IO"];
        // MOD External TNC
        config.ext_tnc_enable = doc["extTNCEn"];
        config.ext_tnc_channel = doc["extTNCCh"];
        config.ext_tnc_mode = doc["extTNCMode"];

        // Power control
        config.pwr_en = doc["pwrEn"];
        config.pwr_mode = doc["pwrMode"];
        config.pwr_sleep_interval = doc["pwrSleep"];
        config.pwr_stanby_delay = doc["pwrStanby"];
        config.pwr_sleep_activate = doc["pwrSleepAct"];
        config.pwr_gpio = doc["pwrIO"];
        config.pwr_active = doc["pwrIOAct"];

        config.log = doc["logFile"];

        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            config.sensor[i].enable = doc["Sensor"][(i * 11) + 0];
            config.sensor[i].port = doc["Sensor"][(i * 11) + 1];
            config.sensor[i].address = doc["Sensor"][(i * 11) + 2];
            config.sensor[i].samplerate = doc["Sensor"][(i * 11) + 3];
            config.sensor[i].averagerate = doc["Sensor"][(i * 11) + 4];
            config.sensor[i].eqns[0] = doc["Sensor"][(i * 11) + 5];
            config.sensor[i].eqns[1] = doc["Sensor"][(i * 11) + 6];
            config.sensor[i].eqns[2] = doc["Sensor"][(i * 11) + 7];
            config.sensor[i].type = doc["Sensor"][(i * 11) + 8];
            strlcpy(config.sensor[i].parm, doc["Sensor"][(i * 11) + 9] | "", sizeof(config.sensor[i].parm));
            strlcpy(config.sensor[i].unit, doc["Sensor"][(i * 11) + 10] | "", sizeof(config.sensor[i].unit));
        }

        // PPP Modem
        config.ppp_enable = doc["pppEn"];
        strlcpy(config.ppp_apn, doc["pppAPN"] | "", sizeof(config.ppp_apn));
        strlcpy(config.ppp_pin, doc["pppPin"] | "", sizeof(config.ppp_pin));
        config.ppp_rst_gpio = doc["pppRST"];
        config.ppp_rst_active = doc["pppRSTAct"];
        config.ppp_rst_delay = doc["pppRSTDelay"];
        config.ppp_tx_gpio = doc["pppTX"];
        config.ppp_rx_gpio = doc["pppRX"];
        config.ppp_rts_gpio = doc["pppRTS"];
        config.ppp_dtr_gpio = doc["pppDTR"];
        config.ppp_cts_gpio = doc["pppCTS"];
        config.ppp_ri_gpio = doc["pppRI"];
        config.ppp_pwr_gpio = doc["pppPWR"];
        config.ppp_pwr_active = doc["pppPWRAct"];   
        config.ppp_serial = doc["pppSerial"];
        config.ppp_serial_baudrate = doc["pppSerialBaudrate"];
        config.ppp_model = doc["pppModel"];
        config.ppp_flow_ctrl = doc["pppFlowCtrl"];
        config.ppp_gnss = doc["pppGNSS"];

        config.en_mqtt = doc["mqttEnable"];
        strlcpy(config.mqtt_host, doc["mqttHost"] | "", sizeof(config.mqtt_host));
        strlcpy(config.mqtt_topic, doc["mqttTopic"] | "", sizeof(config.mqtt_topic));
        strlcpy(config.mqtt_subscribe, doc["mqttSub"] | "", sizeof(config.mqtt_subscribe));
        config.mqtt_port = doc["mqttPort"];

        config.trk_mice_type = doc["trkMicEType"];
        config.trk_tlm_interval = doc["trkTlmInv"];
        config.digi_tlm_interval = doc["digiTlmInv"];
        config.igate_tlm_interval = doc["igateTlmInv"];
        strlcpy(config.host_name, doc["hostName"] | "", sizeof(config.host_name));

        // Close the file (Curiously, File's destructor doesn't close the file)
        // f.close();
        return true;
    }
    else
    {
        log_d("Can't load %s file.", filename);
    }
    return false;
}