#include <Arduino.h>
#include "handleATCommand.h"
#include "config.h"
#include <WiFi.h>

extern Configuration config;

String handleATCommand(String cmd)
{
    cmd.trim();
    if (!cmd.startsWith("AT"))
        return "";
    if (cmd == "AT")
        return "OK";

    if (cmd == "AT+RESET" || cmd == "AT+RESTART")
    {
        log_d("CMD Reset System");
        delay(3000);
        esp_restart();
    }

    // if(cmd == "AT+VERSION?")
    // {
    //     String ver = "Firmware Version: " + String(FIRMWARE_VERSION);
    //     ver += ", SDK Version: " + String(ESP.getSdkVersion());
    //     ver += ", Core Version: " + String(ARDUINO_ESP32_RELEASE);
    //     return ver;
    // }

    if (cmd == "AT+CHIPID?")
    {
        uint64_t chipid = ESP.getEfuseMac();
        String id = String((uint16_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);
        id.toUpperCase();
        return id;
    }

    if (cmd == "AT+SAVECONFIG")
    {
        if (saveConfiguration("/default.cfg", config))
        {
            return "Configuration Saved";
        }
        else
        {
            return "Failed to Save Configuration";
        }
    }

    if (cmd == "LOADCONFIG")
    {
        if (loadConfiguration("/default.cfg", config))
        {
            return "Configuration Loaded";
        }
        else
        {
            return "Failed to Load Configuration";
        }
    }

    if (cmd == "AT+WIFI_DISCONNECT")
    {
        WiFi.disconnect(true);
        return "WiFi Disconnected";
    }

    if (cmd == "AT+WIFI_CONNECT")
    {
        WiFi.reconnect();
        return "WiFi Reconnecting";
    }

    if (cmd == "AT+WIFI_STATUS?")
    {
        String status = "WiFi Status: ";
        if (WiFi.status() == WL_CONNECTED)
        {
            status += "Connected";
        }
        else
        {
            status += "Disconnected";
        }
        return status;
    }

    if (cmd == "AT+WIFI_SCAN")
    {
        int16_t n = WiFi.scanNetworks();
        String scanResult = "Scan Complete: " + String(n) + " networks found.\n";
        for (int i = 0; i < n; ++i)
        {
            scanResult += String(i + 1) + ": ";
            scanResult += WiFi.SSID(i);
            scanResult += ", RSSI: " + String(WiFi.RSSI(i)) + "dBm";
            scanResult += ", BSSID: " + WiFi.BSSIDstr(i);
            scanResult += ", Channel: " + String(WiFi.channel(i));
            scanResult += ", Encryption: " + String((int)WiFi.encryptionType(i)) + "\n";
        }
        WiFi.scanDelete();
        return scanResult;
    }

    if (cmd == "AT+WIFI?")
    {
        String mode = "";
        if (config.wifi_mode == WIFI_AP)
            mode = "AP";
        else if (config.wifi_mode == WIFI_STA)
            mode = "STA";
        else if (config.wifi_mode == WIFI_AP_STA)
            mode = "AP+STA";
        else
            mode = "OFF";
        mode = "Mode:" + mode + ",SSID:" + String(WiFi.SSID()) + ",RSSI:" + String(WiFi.RSSI()) + "dBm";
        return mode;
    }

    if (cmd == "AT+MODE?")
    {
        String mode = "MODE: ";
        if (config.igate_en)
            mode += "IGATE";
        if (config.digi_en)
            mode += ",DIGI";
        if (config.trk_en)
            mode += ",TRK";
        if (config.wx_en)
            mode += ",WX";
        if (config.tlm0_en)
            mode += ",TLM0";
        return mode;
    }

    if (cmd == "AT+TIME?")
    {
        struct tm tmstruct;
        char strTime[20];
        tmstruct.tm_year = 0;
        getLocalTime(&tmstruct, 100);
        sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
        return String(strTime);
    }
    else if (cmd.startsWith("AT+TIME="))
    {
        cmd.remove(0, 8);
        cmd.replace("\"", "");
        char strTime[20];
        strncpy(strTime, cmd.c_str(), sizeof(strTime));
        String date = getValue(strTime, ' ', 0);
        String time = getValue(strTime, ' ', 1);
        int yyyy = getValue(date, '-', 0).toInt();
        int mm = getValue(date, '-', 1).toInt();
        int dd = getValue(date, '-', 2).toInt();
        int hh = getValue(time, ':', 0).toInt();
        int ii = getValue(time, ':', 1).toInt();
        int ss = getValue(time, ':', 2).toInt();
        // int ss = 0;

        tmElements_t timeinfo;
        timeinfo.Year = yyyy - 1970;
        timeinfo.Month = mm;
        timeinfo.Day = dd;
        timeinfo.Hour = hh;
        timeinfo.Minute = ii;
        timeinfo.Second = ss;
        time_t timeStamp = makeTime(timeinfo);
        time_t rtc = timeStamp - (config.timeZone * 3600);
        timeval tv = {rtc, 0};
        timezone tz = {(0) + DST_MN, 0};
        settimeofday(&tv, &tz);
        log_d("Set Time: %s", strTime);
        return "Set Time: " + String(strTime);
    }

    // Generate AT-Command from script python

    if (cmd == "AT+TIMEZONE?")
        return String(config.timeZone, 6);
    else if (cmd.startsWith("AT+TIMEZONE="))
    {
        config.timeZone = cmd.substring(12).toFloat();
        return "OK";
    }

    if (cmd == "AT+SYNCTIME?")
        return String(config.synctime ? "1" : "0");
    else if (cmd == "AT+SYNCTIME=1")
    {
        config.synctime = true;
        return "OK";
    }
    else if (cmd == "AT+SYNCTIME=0")
    {
        config.synctime = false;
        return "OK";
    }

    if (cmd == "AT+TITLE?")
        return String(config.title ? "1" : "0");
    else if (cmd == "AT+TITLE=1")
    {
        config.title = true;
        return "OK";
    }
    else if (cmd == "AT+TITLE=0")
    {
        config.title = false;
        return "OK";
    }

    if (cmd == "AT+WIFI_MODE?")
        return String(config.wifi_mode);
    else if (cmd.startsWith("AT+WIFI_MODE="))
    {
        config.wifi_mode = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+WIFI_POWER?")
        return String(config.wifi_power);
    else if (cmd.startsWith("AT+WIFI_POWER="))
    {
        config.wifi_power = cmd.substring(14).toInt();
        return "OK";
    }

    // ---- WiFi STA 0 ----
    if (cmd == "AT+WIFI0EN?")
        return String(config.wifi_sta[0].enable ? "1" : "0");
    else if (cmd == "AT+WIFI0EN=1")
    {
        config.wifi_sta[0].enable = true;
        return "OK";
    }
    else if (cmd == "AT+WIFI0EN=0")
    {
        config.wifi_sta[0].enable = false;
        return "OK";
    }
    else if (cmd == "AT+WIFI0SSID?")
        return String(config.wifi_sta[0].wifi_ssid);
    else if (cmd.startsWith("AT+WIFI0SSID="))
    {
        cmd.remove(0, 13 + String(0).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[0].wifi_ssid, cmd.c_str(), sizeof(config.wifi_sta[0].wifi_ssid));
        return "OK";
    }
    else if (cmd == "AT+WIFI0PASS?")
        return String(config.wifi_sta[0].wifi_pass);
    else if (cmd.startsWith("AT+WIFI0PASS="))
    {
        cmd.remove(0, 13 + String(0).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[0].wifi_pass, cmd.c_str(), sizeof(config.wifi_sta[0].wifi_pass));
        return "OK";
    }

    // ---- WiFi STA 1 ----
    if (cmd == "AT+WIFI1EN?")
        return String(config.wifi_sta[1].enable ? "1" : "0");
    else if (cmd == "AT+WIFI1EN=1")
    {
        config.wifi_sta[1].enable = true;
        return "OK";
    }
    else if (cmd == "AT+WIFI1EN=0")
    {
        config.wifi_sta[1].enable = false;
        return "OK";
    }
    else if (cmd == "AT+WIFI1SSID?")
        return String(config.wifi_sta[1].wifi_ssid);
    else if (cmd.startsWith("AT+WIFI1SSID="))
    {
        cmd.remove(0, 13 + String(1).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[1].wifi_ssid, cmd.c_str(), sizeof(config.wifi_sta[1].wifi_ssid));
        return "OK";
    }
    else if (cmd == "AT+WIFI1PASS?")
        return String(config.wifi_sta[1].wifi_pass);
    else if (cmd.startsWith("AT+WIFI1PASS="))
    {
        cmd.remove(0, 13 + String(1).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[1].wifi_pass, cmd.c_str(), sizeof(config.wifi_sta[1].wifi_pass));
        return "OK";
    }

    // ---- WiFi STA 2 ----
    if (cmd == "AT+WIFI2EN?")
        return String(config.wifi_sta[2].enable ? "1" : "0");
    else if (cmd == "AT+WIFI2EN=1")
    {
        config.wifi_sta[2].enable = true;
        return "OK";
    }
    else if (cmd == "AT+WIFI2EN=0")
    {
        config.wifi_sta[2].enable = false;
        return "OK";
    }
    else if (cmd == "AT+WIFI2SSID?")
        return String(config.wifi_sta[2].wifi_ssid);
    else if (cmd.startsWith("AT+WIFI2SSID="))
    {
        cmd.remove(0, 13 + String(2).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[2].wifi_ssid, cmd.c_str(), sizeof(config.wifi_sta[2].wifi_ssid));
        return "OK";
    }
    else if (cmd == "AT+WIFI2PASS?")
        return String(config.wifi_sta[2].wifi_pass);
    else if (cmd.startsWith("AT+WIFI2PASS="))
    {
        cmd.remove(0, 13 + String(2).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[2].wifi_pass, cmd.c_str(), sizeof(config.wifi_sta[2].wifi_pass));
        return "OK";
    }

    // ---- WiFi STA 3 ----
    if (cmd == "AT+WIFI3EN?")
        return String(config.wifi_sta[3].enable ? "1" : "0");
    else if (cmd == "AT+WIFI3EN=1")
    {
        config.wifi_sta[3].enable = true;
        return "OK";
    }
    else if (cmd == "AT+WIFI3EN=0")
    {
        config.wifi_sta[3].enable = false;
        return "OK";
    }
    else if (cmd == "AT+WIFI3SSID?")
        return String(config.wifi_sta[3].wifi_ssid);
    else if (cmd.startsWith("AT+WIFI3SSID="))
    {
        cmd.remove(0, 13 + String(3).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[3].wifi_ssid, cmd.c_str(), sizeof(config.wifi_sta[3].wifi_ssid));
        return "OK";
    }
    else if (cmd == "AT+WIFI3PASS?")
        return String(config.wifi_sta[3].wifi_pass);
    else if (cmd.startsWith("AT+WIFI3PASS="))
    {
        cmd.remove(0, 13 + String(3).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[3].wifi_pass, cmd.c_str(), sizeof(config.wifi_sta[3].wifi_pass));
        return "OK";
    }

    // ---- WiFi STA 4 ----
    if (cmd == "AT+WIFI4EN?")
        return String(config.wifi_sta[4].enable ? "1" : "0");
    else if (cmd == "AT+WIFI4EN=1")
    {
        config.wifi_sta[4].enable = true;
        return "OK";
    }
    else if (cmd == "AT+WIFI4EN=0")
    {
        config.wifi_sta[4].enable = false;
        return "OK";
    }
    else if (cmd == "AT+WIFI4SSID?")
        return String(config.wifi_sta[4].wifi_ssid);
    else if (cmd.startsWith("AT+WIFI4SSID="))
    {
        cmd.remove(0, 13 + String(4).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[4].wifi_ssid, cmd.c_str(), sizeof(config.wifi_sta[4].wifi_ssid));
        return "OK";
    }
    else if (cmd == "AT+WIFI4PASS?")
        return String(config.wifi_sta[4].wifi_pass);
    else if (cmd.startsWith("AT+WIFI4PASS="))
    {
        cmd.remove(0, 13 + String(4).length());
        cmd.replace("\"", "");
        strncpy(config.wifi_sta[4].wifi_pass, cmd.c_str(), sizeof(config.wifi_sta[4].wifi_pass));
        return "OK";
    }

    if (cmd == "AT+WIFI_AP_CH?")
        return String(config.wifi_ap_ch);
    else if (cmd.startsWith("AT+WIFI_AP_CH="))
    {
        config.wifi_ap_ch = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+WIFI_AP_SSID?")
        return String(config.wifi_ap_ssid);
    else if (cmd.startsWith("AT+WIFI_AP_SSID="))
    {
        cmd.remove(0, 16);
        cmd.replace("\"", "");
        strncpy(config.wifi_ap_ssid, cmd.c_str(), sizeof(config.wifi_ap_ssid));
        return "OK";
    }

    if (cmd == "AT+WIFI_AP_PASS?")
        return String(config.wifi_ap_pass);
    else if (cmd.startsWith("AT+WIFI_AP_PASS="))
    {
        cmd.remove(0, 16);
        cmd.replace("\"", "");
        strncpy(config.wifi_ap_pass, cmd.c_str(), sizeof(config.wifi_ap_pass));
        return "OK";
    }

    if (cmd == "AT+BT_SLAVE?")
        return String(config.bt_slave ? "1" : "0");
    else if (cmd == "AT+BT_SLAVE=1")
    {
        config.bt_slave = true;
        return "OK";
    }
    else if (cmd == "AT+BT_SLAVE=0")
    {
        config.bt_slave = false;
        return "OK";
    }

    if (cmd == "AT+BT_MASTER?")
        return String(config.bt_master ? "1" : "0");
    else if (cmd == "AT+BT_MASTER=1")
    {
        config.bt_master = true;
        return "OK";
    }
    else if (cmd == "AT+BT_MASTER=0")
    {
        config.bt_master = false;
        return "OK";
    }

    if (cmd == "AT+BT_MODE?")
        return String(config.bt_mode);
    else if (cmd.startsWith("AT+BT_MODE="))
    {
        config.bt_mode = cmd.substring(11).toInt();
        return "OK";
    }

    if (cmd == "AT+BT_UUID?")
        return String(config.bt_uuid);
    else if (cmd.startsWith("AT+BT_UUID="))
    {
        cmd.remove(0, 11);
        cmd.replace("\"", "");
        strncpy(config.bt_uuid, cmd.c_str(), sizeof(config.bt_uuid));
        return "OK";
    }

    if (cmd == "AT+BT_UUID_RX?")
        return String(config.bt_uuid_rx);
    else if (cmd.startsWith("AT+BT_UUID_RX="))
    {
        cmd.remove(0, 14);
        cmd.replace("\"", "");
        strncpy(config.bt_uuid_rx, cmd.c_str(), sizeof(config.bt_uuid_rx));
        return "OK";
    }

    if (cmd == "AT+BT_UUID_TX?")
        return String(config.bt_uuid_tx);
    else if (cmd.startsWith("AT+BT_UUID_TX="))
    {
        cmd.remove(0, 14);
        cmd.replace("\"", "");
        strncpy(config.bt_uuid_tx, cmd.c_str(), sizeof(config.bt_uuid_tx));
        return "OK";
    }

    if (cmd == "AT+BT_NAME?")
        return String(config.bt_name);
    else if (cmd.startsWith("AT+BT_NAME="))
    {
        cmd.remove(0, 11);
        cmd.replace("\"", "");
        strncpy(config.bt_name, cmd.c_str(), sizeof(config.bt_name));
        return "OK";
    }

    if (cmd == "AT+BT_PIN?")
        return String(config.bt_pin);
    else if (cmd.startsWith("AT+BT_PIN="))
    {
        config.bt_pin = cmd.substring(10).toInt();
        return "OK";
    }

    if (cmd == "AT+BT_POWER?")
        return String(config.bt_power);
    else if (cmd.startsWith("AT+BT_POWER="))
    {
        config.bt_power = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+RF_EN?")
        return String(config.rf_en ? "1" : "0");
    else if (cmd == "AT+RF_EN=1")
    {
        config.rf_en = true;
        return "OK";
    }
    else if (cmd == "AT+RF_EN=0")
    {
        config.rf_en = false;
        return "OK";
    }

    if (cmd == "AT+RF_TYPE?")
        return String(config.rf_type);
    else if (cmd.startsWith("AT+RF_TYPE="))
    {
        config.rf_type = cmd.substring(11).toInt();
        return "OK";
    }

    if (cmd == "AT+FREQ_RX?")
        return String(config.freq_rx, 6);
    else if (cmd.startsWith("AT+FREQ_RX="))
    {
        config.freq_rx = cmd.substring(11).toFloat();
        return "OK";
    }

    if (cmd == "AT+FREQ_TX?")
        return String(config.freq_tx, 6);
    else if (cmd.startsWith("AT+FREQ_TX="))
    {
        config.freq_tx = cmd.substring(11).toFloat();
        return "OK";
    }

    if (cmd == "AT+OFFSET_RX?")
        return String(config.offset_rx);
    else if (cmd.startsWith("AT+OFFSET_RX="))
    {
        config.offset_rx = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+OFFSET_TX?")
        return String(config.offset_tx);
    else if (cmd.startsWith("AT+OFFSET_TX="))
    {
        config.offset_tx = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+TONE_RX?")
        return String(config.tone_rx);
    else if (cmd.startsWith("AT+TONE_RX="))
    {
        config.tone_rx = cmd.substring(11).toInt();
        return "OK";
    }

    if (cmd == "AT+TONE_TX?")
        return String(config.tone_tx);
    else if (cmd.startsWith("AT+TONE_TX="))
    {
        config.tone_tx = cmd.substring(11).toInt();
        return "OK";
    }

    if (cmd == "AT+BAND?")
        return String(config.band);
    else if (cmd.startsWith("AT+BAND="))
    {
        config.band = cmd.substring(8).toInt();
        return "OK";
    }

    if (cmd == "AT+SQL_LEVEL?")
        return String(config.sql_level);
    else if (cmd.startsWith("AT+SQL_LEVEL="))
    {
        config.sql_level = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+RF_POWER?")
        return String(config.rf_power ? "1" : "0");
    else if (cmd == "AT+RF_POWER=1")
    {
        config.rf_power = true;
        return "OK";
    }
    else if (cmd == "AT+RF_POWER=0")
    {
        config.rf_power = false;
        return "OK";
    }

    if (cmd == "AT+VOLUME?")
        return String(config.volume);
    else if (cmd.startsWith("AT+VOLUME="))
    {
        config.volume = cmd.substring(10).toInt();
        return "OK";
    }

    if (cmd == "AT+MIC?")
        return String(config.mic);
    else if (cmd.startsWith("AT+MIC="))
    {
        config.mic = cmd.substring(7).toInt();
        return "OK";
    }

    if (cmd == "AT+IGATE_EN?")
        return String(config.igate_en ? "1" : "0");
    else if (cmd == "AT+IGATE_EN=1")
    {
        config.igate_en = true;
        return "OK";
    }
    else if (cmd == "AT+IGATE_EN=0")
    {
        config.igate_en = false;
        return "OK";
    }

    if (cmd == "AT+RF2INET?")
        return String(config.rf2inet ? "1" : "0");
    else if (cmd == "AT+RF2INET=1")
    {
        config.rf2inet = true;
        return "OK";
    }
    else if (cmd == "AT+RF2INET=0")
    {
        config.rf2inet = false;
        return "OK";
    }

    if (cmd == "AT+INET2RF?")
        return String(config.inet2rf ? "1" : "0");
    else if (cmd == "AT+INET2RF=1")
    {
        config.inet2rf = true;
        return "OK";
    }
    else if (cmd == "AT+INET2RF=0")
    {
        config.inet2rf = false;
        return "OK";
    }

    if (cmd == "AT+IGATE_LOC2RF?")
        return String(config.igate_loc2rf ? "1" : "0");
    else if (cmd == "AT+IGATE_LOC2RF=1")
    {
        config.igate_loc2rf = true;
        return "OK";
    }
    else if (cmd == "AT+IGATE_LOC2RF=0")
    {
        config.igate_loc2rf = false;
        return "OK";
    }

    if (cmd == "AT+IGATE_LOC2INET?")
        return String(config.igate_loc2inet ? "1" : "0");
    else if (cmd == "AT+IGATE_LOC2INET=1")
    {
        config.igate_loc2inet = true;
        return "OK";
    }
    else if (cmd == "AT+IGATE_LOC2INET=0")
    {
        config.igate_loc2inet = false;
        return "OK";
    }

    if (cmd == "AT+RF2INETFILTER?")
        return String(config.rf2inetFilter);
    else if (cmd.startsWith("AT+RF2INETFILTER="))
    {
        config.rf2inetFilter = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+INET2RFFILTER?")
        return String(config.inet2rfFilter);
    else if (cmd.startsWith("AT+INET2RFFILTER="))
    {
        config.inet2rfFilter = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+APRS_SSID?")
        return String(config.aprs_ssid);
    else if (cmd.startsWith("AT+APRS_SSID="))
    {
        config.aprs_ssid = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+APRS_PORT?")
        return String(config.aprs_port);
    else if (cmd.startsWith("AT+APRS_PORT="))
    {
        config.aprs_port = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+APRS_MYCALL?")
        return String(config.aprs_mycall);
    else if (cmd.startsWith("AT+APRS_MYCALL="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.aprs_mycall, cmd.c_str(), sizeof(config.aprs_mycall));
        return "OK";
    }

    if (cmd == "AT+APRS_HOST?")
        return String(config.aprs_host);
    else if (cmd.startsWith("AT+APRS_HOST="))
    {
        cmd.remove(0, 13);
        cmd.replace("\"", "");
        strncpy(config.aprs_host, cmd.c_str(), sizeof(config.aprs_host));
        return "OK";
    }

    if (cmd == "AT+APRS_PASSCODE?")
        return String(config.aprs_passcode);
    else if (cmd.startsWith("AT+APRS_PASSCODE="))
    {
        cmd.remove(0, 17);
        cmd.replace("\"", "");
        strncpy(config.aprs_passcode, cmd.c_str(), sizeof(config.aprs_passcode));
        return "OK";
    }

    if (cmd == "AT+APRS_MONICALL?")
        return String(config.aprs_moniCall);
    else if (cmd.startsWith("AT+APRS_MONICALL="))
    {
        cmd.remove(0, 17);
        cmd.replace("\"", "");
        strncpy(config.aprs_moniCall, cmd.c_str(), sizeof(config.aprs_moniCall));
        return "OK";
    }

    if (cmd == "AT+APRS_FILTER?")
        return String(config.aprs_filter);
    else if (cmd.startsWith("AT+APRS_FILTER="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.aprs_filter, cmd.c_str(), sizeof(config.aprs_filter));
        return "OK";
    }

    if (cmd == "AT+IGATE_BCN?")
        return String(config.igate_bcn ? "1" : "0");
    else if (cmd == "AT+IGATE_BCN=1")
    {
        config.igate_bcn = true;
        return "OK";
    }
    else if (cmd == "AT+IGATE_BCN=0")
    {
        config.igate_bcn = false;
        return "OK";
    }

    if (cmd == "AT+IGATE_GPS?")
        return String(config.igate_gps ? "1" : "0");
    else if (cmd == "AT+IGATE_GPS=1")
    {
        config.igate_gps = true;
        return "OK";
    }
    else if (cmd == "AT+IGATE_GPS=0")
    {
        config.igate_gps = false;
        return "OK";
    }

    if (cmd == "AT+IGATE_TIMESTAMP?")
        return String(config.igate_timestamp ? "1" : "0");
    else if (cmd == "AT+IGATE_TIMESTAMP=1")
    {
        config.igate_timestamp = true;
        return "OK";
    }
    else if (cmd == "AT+IGATE_TIMESTAMP=0")
    {
        config.igate_timestamp = false;
        return "OK";
    }

    if (cmd == "AT+IGATE_LAT?")
        return String(config.igate_lat, 6);
    else if (cmd.startsWith("AT+IGATE_LAT="))
    {
        config.igate_lat = cmd.substring(13).toFloat();
        return "OK";
    }

    if (cmd == "AT+IGATE_LON?")
        return String(config.igate_lon, 6);
    else if (cmd.startsWith("AT+IGATE_LON="))
    {
        config.igate_lon = cmd.substring(13).toFloat();
        return "OK";
    }

    if (cmd == "AT+IGATE_ALT?")
        return String(config.igate_alt, 6);
    else if (cmd.startsWith("AT+IGATE_ALT="))
    {
        config.igate_alt = cmd.substring(13).toFloat();
        return "OK";
    }

    if (cmd == "AT+IGATE_INTERVAL?")
        return String(config.igate_interval);
    else if (cmd.startsWith("AT+IGATE_INTERVAL="))
    {
        config.igate_interval = cmd.substring(18).toInt();
        return "OK";
    }

    if (cmd == "AT+IGATE_SYMBOL?")
        return String(config.igate_symbol);
    else if (cmd.startsWith("AT+IGATE_SYMBOL="))
    {
        cmd.remove(0, 16);
        cmd.replace("\"", "");
        strncpy(config.igate_symbol, cmd.c_str(), sizeof(config.igate_symbol));
        return "OK";
    }

    if (cmd == "AT+IGATE_OBJECT?")
        return String(config.igate_object);
    else if (cmd.startsWith("AT+IGATE_OBJECT="))
    {
        cmd.remove(0, 16);
        cmd.replace("\"", "");
        strncpy(config.igate_object, cmd.c_str(), sizeof(config.igate_object));
        return "OK";
    }

    if (cmd == "AT+IGATE_PHG?")
        return String(config.igate_phg);
    else if (cmd.startsWith("AT+IGATE_PHG="))
    {
        cmd.remove(0, 13);
        cmd.replace("\"", "");
        strncpy(config.igate_phg, cmd.c_str(), sizeof(config.igate_phg));
        return "OK";
    }

    if (cmd == "AT+IGATE_PATH?")
        return String(config.igate_path);
    else if (cmd.startsWith("AT+IGATE_PATH="))
    {
        config.igate_path = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+IGATE_COMMENT?")
        return String(config.igate_comment);
    else if (cmd.startsWith("AT+IGATE_COMMENT="))
    {
        cmd.remove(0, 17);
        cmd.replace("\"", "");
        strncpy(config.igate_comment, cmd.c_str(), sizeof(config.igate_comment));
        return "OK";
    }

    if (cmd == "AT+IGATE_STS_INTERVAL?")
        return String(config.igate_sts_interval);
    else if (cmd.startsWith("AT+IGATE_STS_INTERVAL="))
    {
        config.igate_sts_interval = cmd.substring(22).toInt();
        return "OK";
    }

    if (cmd == "AT+IGATE_STATUS?")
        return String(config.igate_status);
    else if (cmd.startsWith("AT+IGATE_STATUS="))
    {
        cmd.remove(0, 16);
        cmd.replace("\"", "");
        strncpy(config.igate_status, cmd.c_str(), sizeof(config.igate_status));
        return "OK";
    }

    if (cmd == "AT+DIGI_EN?")
        return String(config.digi_en ? "1" : "0");
    else if (cmd == "AT+DIGI_EN=1")
    {
        config.digi_en = true;
        return "OK";
    }
    else if (cmd == "AT+DIGI_EN=0")
    {
        config.digi_en = false;
        return "OK";
    }

    if (cmd == "AT+DIGI_AUTO?")
        return String(config.digi_auto ? "1" : "0");
    else if (cmd == "AT+DIGI_AUTO=1")
    {
        config.digi_auto = true;
        return "OK";
    }
    else if (cmd == "AT+DIGI_AUTO=0")
    {
        config.digi_auto = false;
        return "OK";
    }

    if (cmd == "AT+DIGI_LOC2RF?")
        return String(config.digi_loc2rf ? "1" : "0");
    else if (cmd == "AT+DIGI_LOC2RF=1")
    {
        config.digi_loc2rf = true;
        return "OK";
    }
    else if (cmd == "AT+DIGI_LOC2RF=0")
    {
        config.digi_loc2rf = false;
        return "OK";
    }

    if (cmd == "AT+DIGI_LOC2INET?")
        return String(config.digi_loc2inet ? "1" : "0");
    else if (cmd == "AT+DIGI_LOC2INET=1")
    {
        config.digi_loc2inet = true;
        return "OK";
    }
    else if (cmd == "AT+DIGI_LOC2INET=0")
    {
        config.digi_loc2inet = false;
        return "OK";
    }

    if (cmd == "AT+DIGI_TIMESTAMP?")
        return String(config.digi_timestamp ? "1" : "0");
    else if (cmd == "AT+DIGI_TIMESTAMP=1")
    {
        config.digi_timestamp = true;
        return "OK";
    }
    else if (cmd == "AT+DIGI_TIMESTAMP=0")
    {
        config.digi_timestamp = false;
        return "OK";
    }

    if (cmd == "AT+DIGI_SSID?")
        return String(config.digi_ssid);
    else if (cmd.startsWith("AT+DIGI_SSID="))
    {
        config.digi_ssid = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+DIGI_MYCALL?")
        return String(config.digi_mycall);
    else if (cmd.startsWith("AT+DIGI_MYCALL="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.digi_mycall, cmd.c_str(), sizeof(config.digi_mycall));
        return "OK";
    }

    if (cmd == "AT+DIGI_PATH?")
        return String(config.digi_path);
    else if (cmd.startsWith("AT+DIGI_PATH="))
    {
        config.digi_path = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+DIGI_DELAY?")
        return String(config.digi_delay);
    else if (cmd.startsWith("AT+DIGI_DELAY="))
    {
        config.digi_delay = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+DIGIFILTER?")
        return String(config.digiFilter);
    else if (cmd.startsWith("AT+DIGIFILTER="))
    {
        config.digiFilter = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+DIGI_BCN?")
        return String(config.digi_bcn ? "1" : "0");
    else if (cmd == "AT+DIGI_BCN=1")
    {
        config.digi_bcn = true;
        return "OK";
    }
    else if (cmd == "AT+DIGI_BCN=0")
    {
        config.digi_bcn = false;
        return "OK";
    }

    if (cmd == "AT+DIGI_GPS?")
        return String(config.digi_gps ? "1" : "0");
    else if (cmd == "AT+DIGI_GPS=1")
    {
        config.digi_gps = true;
        return "OK";
    }
    else if (cmd == "AT+DIGI_GPS=0")
    {
        config.digi_gps = false;
        return "OK";
    }

    if (cmd == "AT+DIGI_LAT?")
        return String(config.digi_lat, 6);
    else if (cmd.startsWith("AT+DIGI_LAT="))
    {
        config.digi_lat = cmd.substring(12).toFloat();
        return "OK";
    }

    if (cmd == "AT+DIGI_LON?")
        return String(config.digi_lon, 6);
    else if (cmd.startsWith("AT+DIGI_LON="))
    {
        config.digi_lon = cmd.substring(12).toFloat();
        return "OK";
    }

    if (cmd == "AT+DIGI_ALT?")
        return String(config.digi_alt, 6);
    else if (cmd.startsWith("AT+DIGI_ALT="))
    {
        config.digi_alt = cmd.substring(12).toFloat();
        return "OK";
    }

    if (cmd == "AT+DIGI_INTERVAL?")
        return String(config.digi_interval);
    else if (cmd.startsWith("AT+DIGI_INTERVAL="))
    {
        config.digi_interval = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+DIGI_SYMBOL?")
        return String(config.digi_symbol);
    else if (cmd.startsWith("AT+DIGI_SYMBOL="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.digi_symbol, cmd.c_str(), sizeof(config.digi_symbol));
        return "OK";
    }

    if (cmd == "AT+DIGI_PHG?")
        return String(config.digi_phg);
    else if (cmd.startsWith("AT+DIGI_PHG="))
    {
        cmd.remove(0, 12);
        cmd.replace("\"", "");
        strncpy(config.digi_phg, cmd.c_str(), sizeof(config.digi_phg));
        return "OK";
    }

    if (cmd == "AT+DIGI_COMMENT?")
        return String(config.digi_comment);
    else if (cmd.startsWith("AT+DIGI_COMMENT="))
    {
        cmd.remove(0, 16);
        cmd.replace("\"", "");
        strncpy(config.digi_comment, cmd.c_str(), sizeof(config.digi_comment));
        return "OK";
    }

    if (cmd == "AT+DIGI_STS_INTERVAL?")
        return String(config.digi_sts_interval);
    else if (cmd.startsWith("AT+DIGI_STS_INTERVAL="))
    {
        config.digi_sts_interval = cmd.substring(21).toInt();
        return "OK";
    }

    if (cmd == "AT+DIGI_STATUS?")
        return String(config.digi_status);
    else if (cmd.startsWith("AT+DIGI_STATUS="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.digi_status, cmd.c_str(), sizeof(config.digi_status));
        return "OK";
    }

    if (cmd == "AT+TRK_EN?")
        return String(config.trk_en ? "1" : "0");
    else if (cmd == "AT+TRK_EN=1")
    {
        config.trk_en = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_EN=0")
    {
        config.trk_en = false;
        return "OK";
    }

    if (cmd == "AT+TRK_LOC2RF?")
        return String(config.trk_loc2rf ? "1" : "0");
    else if (cmd == "AT+TRK_LOC2RF=1")
    {
        config.trk_loc2rf = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_LOC2RF=0")
    {
        config.trk_loc2rf = false;
        return "OK";
    }

    if (cmd == "AT+TRK_LOC2INET?")
        return String(config.trk_loc2inet ? "1" : "0");
    else if (cmd == "AT+TRK_LOC2INET=1")
    {
        config.trk_loc2inet = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_LOC2INET=0")
    {
        config.trk_loc2inet = false;
        return "OK";
    }

    if (cmd == "AT+TRK_TIMESTAMP?")
        return String(config.trk_timestamp ? "1" : "0");
    else if (cmd == "AT+TRK_TIMESTAMP=1")
    {
        config.trk_timestamp = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_TIMESTAMP=0")
    {
        config.trk_timestamp = false;
        return "OK";
    }

    if (cmd == "AT+TRK_SSID?")
        return String(config.trk_ssid);
    else if (cmd.startsWith("AT+TRK_SSID="))
    {
        config.trk_ssid = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_MYCALL?")
        return String(config.trk_mycall);
    else if (cmd.startsWith("AT+TRK_MYCALL="))
    {
        cmd.remove(0, 14);
        cmd.replace("\"", "");
        strncpy(config.trk_mycall, cmd.c_str(), sizeof(config.trk_mycall));
        return "OK";
    }

    if (cmd == "AT+TRK_PATH?")
        return String(config.trk_path);
    else if (cmd.startsWith("AT+TRK_PATH="))
    {
        config.trk_path = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_GPS?")
        return String(config.trk_gps ? "1" : "0");
    else if (cmd == "AT+TRK_GPS=1")
    {
        config.trk_gps = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_GPS=0")
    {
        config.trk_gps = false;
        return "OK";
    }

    if (cmd == "AT+TRK_LAT?")
        return String(config.trk_lat, 6);
    else if (cmd.startsWith("AT+TRK_LAT="))
    {
        config.trk_lat = cmd.substring(11).toFloat();
        return "OK";
    }

    if (cmd == "AT+TRK_LON?")
        return String(config.trk_lon, 6);
    else if (cmd.startsWith("AT+TRK_LON="))
    {
        config.trk_lon = cmd.substring(11).toFloat();
        return "OK";
    }

    if (cmd == "AT+TRK_ALT?")
        return String(config.trk_alt, 6);
    else if (cmd.startsWith("AT+TRK_ALT="))
    {
        config.trk_alt = cmd.substring(11).toFloat();
        return "OK";
    }

    if (cmd == "AT+TRK_INTERVAL?")
        return String(config.trk_interval);
    else if (cmd.startsWith("AT+TRK_INTERVAL="))
    {
        config.trk_interval = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_SMARTBEACON?")
        return String(config.trk_smartbeacon ? "1" : "0");
    else if (cmd == "AT+TRK_SMARTBEACON=1")
    {
        config.trk_smartbeacon = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_SMARTBEACON=0")
    {
        config.trk_smartbeacon = false;
        return "OK";
    }

    if (cmd == "AT+TRK_COMPRESS?")
        return String(config.trk_compress ? "1" : "0");
    else if (cmd == "AT+TRK_COMPRESS=1")
    {
        config.trk_compress = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_COMPRESS=0")
    {
        config.trk_compress = false;
        return "OK";
    }

    if (cmd == "AT+TRK_ALTITUDE?")
        return String(config.trk_altitude ? "1" : "0");
    else if (cmd == "AT+TRK_ALTITUDE=1")
    {
        config.trk_altitude = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_ALTITUDE=0")
    {
        config.trk_altitude = false;
        return "OK";
    }

    if (cmd == "AT+TRK_LOG?")
        return String(config.trk_log ? "1" : "0");
    else if (cmd == "AT+TRK_LOG=1")
    {
        config.trk_log = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_LOG=0")
    {
        config.trk_log = false;
        return "OK";
    }

    if (cmd == "AT+TRK_RSSI?")
        return String(config.trk_rssi ? "1" : "0");
    else if (cmd == "AT+TRK_RSSI=1")
    {
        config.trk_rssi = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_RSSI=0")
    {
        config.trk_rssi = false;
        return "OK";
    }

    if (cmd == "AT+TRK_SAT?")
        return String(config.trk_sat ? "1" : "0");
    else if (cmd == "AT+TRK_SAT=1")
    {
        config.trk_sat = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_SAT=0")
    {
        config.trk_sat = false;
        return "OK";
    }

    if (cmd == "AT+TRK_DX?")
        return String(config.trk_dx ? "1" : "0");
    else if (cmd == "AT+TRK_DX=1")
    {
        config.trk_dx = true;
        return "OK";
    }
    else if (cmd == "AT+TRK_DX=0")
    {
        config.trk_dx = false;
        return "OK";
    }

    if (cmd == "AT+TRK_HSPEED?")
        return String(config.trk_hspeed);
    else if (cmd.startsWith("AT+TRK_HSPEED="))
    {
        config.trk_hspeed = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_LSPEED?")
        return String(config.trk_lspeed);
    else if (cmd.startsWith("AT+TRK_LSPEED="))
    {
        config.trk_lspeed = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_MAXINTERVAL?")
        return String(config.trk_maxinterval);
    else if (cmd.startsWith("AT+TRK_MAXINTERVAL="))
    {
        config.trk_maxinterval = cmd.substring(19).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_MININTERVAL?")
        return String(config.trk_mininterval);
    else if (cmd.startsWith("AT+TRK_MININTERVAL="))
    {
        config.trk_mininterval = cmd.substring(19).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_MINANGLE?")
        return String(config.trk_minangle);
    else if (cmd.startsWith("AT+TRK_MINANGLE="))
    {
        config.trk_minangle = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_SLOWINTERVAL?")
        return String(config.trk_slowinterval);
    else if (cmd.startsWith("AT+TRK_SLOWINTERVAL="))
    {
        config.trk_slowinterval = cmd.substring(20).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_SYMBOL?")
        return String(config.trk_symbol);
    else if (cmd.startsWith("AT+TRK_SYMBOL="))
    {
        cmd.remove(0, 14);
        cmd.replace("\"", "");
        strncpy(config.trk_symbol, cmd.c_str(), sizeof(config.trk_symbol));
        return "OK";
    }

    if (cmd == "AT+TRK_SYMMOVE?")
        return String(config.trk_symmove);
    else if (cmd.startsWith("AT+TRK_SYMMOVE="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.trk_symmove, cmd.c_str(), sizeof(config.trk_symmove));
        return "OK";
    }

    if (cmd == "AT+TRK_SYMSTOP?")
        return String(config.trk_symstop);
    else if (cmd.startsWith("AT+TRK_SYMSTOP="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.trk_symstop, cmd.c_str(), sizeof(config.trk_symstop));
        return "OK";
    }

    if (cmd == "AT+TRK_COMMENT?")
        return String(config.trk_comment);
    else if (cmd.startsWith("AT+TRK_COMMENT="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.trk_comment, cmd.c_str(), sizeof(config.trk_comment));
        return "OK";
    }

    if (cmd == "AT+TRK_ITEM?")
        return String(config.trk_item);
    else if (cmd.startsWith("AT+TRK_ITEM="))
    {
        cmd.remove(0, 12);
        cmd.replace("\"", "");
        strncpy(config.trk_item, cmd.c_str(), sizeof(config.trk_item));
        return "OK";
    }

    if (cmd == "AT+TRK_STS_INTERVAL?")
        return String(config.trk_sts_interval);
    else if (cmd.startsWith("AT+TRK_STS_INTERVAL="))
    {
        config.trk_sts_interval = cmd.substring(20).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_STATUS?")
        return String(config.trk_status);
    else if (cmd.startsWith("AT+TRK_STATUS="))
    {
        cmd.remove(0, 14);
        cmd.replace("\"", "");
        strncpy(config.trk_status, cmd.c_str(), sizeof(config.trk_status));
        return "OK";
    }

    if (cmd == "AT+WX_EN?")
        return String(config.wx_en ? "1" : "0");
    else if (cmd == "AT+WX_EN=1")
    {
        config.wx_en = true;
        return "OK";
    }
    else if (cmd == "AT+WX_EN=0")
    {
        config.wx_en = false;
        return "OK";
    }

    if (cmd == "AT+WX_2RF?")
        return String(config.wx_2rf ? "1" : "0");
    else if (cmd == "AT+WX_2RF=1")
    {
        config.wx_2rf = true;
        return "OK";
    }
    else if (cmd == "AT+WX_2RF=0")
    {
        config.wx_2rf = false;
        return "OK";
    }

    if (cmd == "AT+WX_2INET?")
        return String(config.wx_2inet ? "1" : "0");
    else if (cmd == "AT+WX_2INET=1")
    {
        config.wx_2inet = true;
        return "OK";
    }
    else if (cmd == "AT+WX_2INET=0")
    {
        config.wx_2inet = false;
        return "OK";
    }

    if (cmd == "AT+WX_TIMESTAMP?")
        return String(config.wx_timestamp ? "1" : "0");
    else if (cmd == "AT+WX_TIMESTAMP=1")
    {
        config.wx_timestamp = true;
        return "OK";
    }
    else if (cmd == "AT+WX_TIMESTAMP=0")
    {
        config.wx_timestamp = false;
        return "OK";
    }

    if (cmd == "AT+WX_SSID?")
        return String(config.wx_ssid);
    else if (cmd.startsWith("AT+WX_SSID="))
    {
        config.wx_ssid = cmd.substring(11).toInt();
        return "OK";
    }

    if (cmd == "AT+WX_MYCALL?")
        return String(config.wx_mycall);
    else if (cmd.startsWith("AT+WX_MYCALL="))
    {
        cmd.remove(0, 13);
        cmd.replace("\"", "");
        strncpy(config.wx_mycall, cmd.c_str(), sizeof(config.wx_mycall));
        return "OK";
    }

    if (cmd == "AT+WX_PATH?")
        return String(config.wx_path);
    else if (cmd.startsWith("AT+WX_PATH="))
    {
        config.wx_path = cmd.substring(11).toInt();
        return "OK";
    }

    if (cmd == "AT+WX_GPS?")
        return String(config.wx_gps ? "1" : "0");
    else if (cmd == "AT+WX_GPS=1")
    {
        config.wx_gps = true;
        return "OK";
    }
    else if (cmd == "AT+WX_GPS=0")
    {
        config.wx_gps = false;
        return "OK";
    }

    if (cmd == "AT+WX_LAT?")
        return String(config.wx_lat, 6);
    else if (cmd.startsWith("AT+WX_LAT="))
    {
        config.wx_lat = cmd.substring(10).toFloat();
        return "OK";
    }

    if (cmd == "AT+WX_LON?")
        return String(config.wx_lon, 6);
    else if (cmd.startsWith("AT+WX_LON="))
    {
        config.wx_lon = cmd.substring(10).toFloat();
        return "OK";
    }

    if (cmd == "AT+WX_ALT?")
        return String(config.wx_alt, 6);
    else if (cmd.startsWith("AT+WX_ALT="))
    {
        config.wx_alt = cmd.substring(10).toFloat();
        return "OK";
    }

    if (cmd == "AT+WX_INTERVAL?")
        return String(config.wx_interval);
    else if (cmd.startsWith("AT+WX_INTERVAL="))
    {
        config.wx_interval = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+WX_FLAGE?")
        return String(config.wx_flage);
    else if (cmd.startsWith("AT+WX_FLAGE="))
    {
        config.wx_flage = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+WX_OBJECT?")
        return String(config.wx_object);
    else if (cmd.startsWith("AT+WX_OBJECT="))
    {
        cmd.remove(0, 13);
        cmd.replace("\"", "");
        strncpy(config.wx_object, cmd.c_str(), sizeof(config.wx_object));
        return "OK";
    }

    if (cmd == "AT+WX_COMMENT?")
        return String(config.wx_comment);
    else if (cmd.startsWith("AT+WX_COMMENT="))
    {
        cmd.remove(0, 14);
        cmd.replace("\"", "");
        strncpy(config.wx_comment, cmd.c_str(), sizeof(config.wx_comment));
        return "OK";
    }

    if (cmd == "AT+TLM0_EN?")
        return String(config.tlm0_en ? "1" : "0");
    else if (cmd == "AT+TLM0_EN=1")
    {
        config.tlm0_en = true;
        return "OK";
    }
    else if (cmd == "AT+TLM0_EN=0")
    {
        config.tlm0_en = false;
        return "OK";
    }

    if (cmd == "AT+TLM0_2RF?")
        return String(config.tlm0_2rf ? "1" : "0");
    else if (cmd == "AT+TLM0_2RF=1")
    {
        config.tlm0_2rf = true;
        return "OK";
    }
    else if (cmd == "AT+TLM0_2RF=0")
    {
        config.tlm0_2rf = false;
        return "OK";
    }

    if (cmd == "AT+TLM0_2INET?")
        return String(config.tlm0_2inet ? "1" : "0");
    else if (cmd == "AT+TLM0_2INET=1")
    {
        config.tlm0_2inet = true;
        return "OK";
    }
    else if (cmd == "AT+TLM0_2INET=0")
    {
        config.tlm0_2inet = false;
        return "OK";
    }

    if (cmd == "AT+TLM0_SSID?")
        return String(config.tlm0_ssid);
    else if (cmd.startsWith("AT+TLM0_SSID="))
    {
        config.tlm0_ssid = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM0_MYCALL?")
        return String(config.tlm0_mycall);
    else if (cmd.startsWith("AT+TLM0_MYCALL="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.tlm0_mycall, cmd.c_str(), sizeof(config.tlm0_mycall));
        return "OK";
    }

    if (cmd == "AT+TLM0_PATH?")
        return String(config.tlm0_path);
    else if (cmd.startsWith("AT+TLM0_PATH="))
    {
        config.tlm0_path = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM0_DATA_INTERVAL?")
        return String(config.tlm0_data_interval);
    else if (cmd.startsWith("AT+TLM0_DATA_INTERVAL="))
    {
        config.tlm0_data_interval = cmd.substring(22).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM0_INFO_INTERVAL?")
        return String(config.tlm0_info_interval);
    else if (cmd.startsWith("AT+TLM0_INFO_INTERVAL="))
    {
        config.tlm0_info_interval = cmd.substring(22).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM0_BITS_ACTIVE?")
        return String(config.tlm0_BITS_Active);
    else if (cmd.startsWith("AT+TLM0_BITS_ACTIVE="))
    {
        config.tlm0_BITS_Active = cmd.substring(20).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM0_COMMENT?")
        return String(config.tlm0_comment);
    else if (cmd.startsWith("AT+TLM0_COMMENT="))
    {
        cmd.remove(0, 16);
        cmd.replace("\"", "");
        strncpy(config.tlm0_comment, cmd.c_str(), sizeof(config.tlm0_comment));
        return "OK";
    }

    // if (cmd == "AT+TML0_DATA_CHANNEL?")
    //     return String(config.tml0_data_channel);
    // else if (cmd.startsWith("AT+TML0_DATA_CHANNEL="))
    // {
    //     config.tml0_data_channel = cmd.substring(21).toInt();
    //     return "OK";
    // }

    if (cmd == "AT+TLM1_EN?")
        return String(config.tlm1_en ? "1" : "0");
    else if (cmd == "AT+TLM1_EN=1")
    {
        config.tlm1_en = true;
        return "OK";
    }
    else if (cmd == "AT+TLM1_EN=0")
    {
        config.tlm1_en = false;
        return "OK";
    }

    if (cmd == "AT+TLM1_2RF?")
        return String(config.tlm1_2rf ? "1" : "0");
    else if (cmd == "AT+TLM1_2RF=1")
    {
        config.tlm1_2rf = true;
        return "OK";
    }
    else if (cmd == "AT+TLM1_2RF=0")
    {
        config.tlm1_2rf = false;
        return "OK";
    }

    if (cmd == "AT+TLM1_2INET?")
        return String(config.tlm1_2inet ? "1" : "0");
    else if (cmd == "AT+TLM1_2INET=1")
    {
        config.tlm1_2inet = true;
        return "OK";
    }
    else if (cmd == "AT+TLM1_2INET=0")
    {
        config.tlm1_2inet = false;
        return "OK";
    }

    if (cmd == "AT+TLM1_SSID?")
        return String(config.tlm1_ssid);
    else if (cmd.startsWith("AT+TLM1_SSID="))
    {
        config.tlm1_ssid = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM1_MYCALL?")
        return String(config.tlm1_mycall);
    else if (cmd.startsWith("AT+TLM1_MYCALL="))
    {
        cmd.remove(0, 15);
        cmd.replace("\"", "");
        strncpy(config.tlm1_mycall, cmd.c_str(), sizeof(config.tlm1_mycall));
        return "OK";
    }

    if (cmd == "AT+TLM1_PATH?")
        return String(config.tlm1_path);
    else if (cmd.startsWith("AT+TLM1_PATH="))
    {
        config.tlm1_path = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM1_DATA_INTERVAL?")
        return String(config.tlm1_data_interval);
    else if (cmd.startsWith("AT+TLM1_DATA_INTERVAL="))
    {
        config.tlm1_data_interval = cmd.substring(22).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM1_INFO_INTERVAL?")
        return String(config.tlm1_info_interval);
    else if (cmd.startsWith("AT+TLM1_INFO_INTERVAL="))
    {
        config.tlm1_info_interval = cmd.substring(22).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM1_BITS_ACTIVE?")
        return String(config.tlm1_BITS_Active);
    else if (cmd.startsWith("AT+TLM1_BITS_ACTIVE="))
    {
        config.tlm1_BITS_Active = cmd.substring(20).toInt();
        return "OK";
    }

    if (cmd == "AT+TLM1_COMMENT?")
        return String(config.tlm1_comment);
    else if (cmd.startsWith("AT+TLM1_COMMENT="))
    {
        cmd.remove(0, 16);
        cmd.replace("\"", "");
        strncpy(config.tlm1_comment, cmd.c_str(), sizeof(config.tlm1_comment));
        return "OK";
    }

    // if (cmd == "AT+TML1_DATA_CHANNEL?")
    //     return String(config.tml1_data_channel);
    // else if (cmd.startsWith("AT+TML1_DATA_CHANNEL="))
    // {
    //     config.tml1_data_channel = cmd.substring(21).toInt();
    //     return "OK";
    // }

    if (cmd == "AT+OLED_ENABLE?")
        return String(config.oled_enable ? "1" : "0");
    else if (cmd == "AT+OLED_ENABLE=1")
    {
        config.oled_enable = true;
        return "OK";
    }
    else if (cmd == "AT+OLED_ENABLE=0")
    {
        config.oled_enable = false;
        return "OK";
    }

    if (cmd == "AT+OLED_TIMEOUT?")
        return String(config.oled_timeout);
    else if (cmd.startsWith("AT+OLED_TIMEOUT="))
    {
        config.oled_timeout = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+DIM?")
        return String(config.dim);
    else if (cmd.startsWith("AT+DIM="))
    {
        config.dim = cmd.substring(7).toInt();
        return "OK";
    }

    if (cmd == "AT+CONTRAST?")
        return String(config.contrast);
    else if (cmd.startsWith("AT+CONTRAST="))
    {
        config.contrast = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+STARTUP?")
        return String(config.startup);
    else if (cmd.startsWith("AT+STARTUP="))
    {
        config.startup = cmd.substring(11).toInt();
        return "OK";
    }

    if (cmd == "AT+H_UP?")
        return String(config.h_up ? "1" : "0");
    else if (cmd == "AT+H_UP=1")
    {
        config.h_up = true;
        return "OK";
    }
    else if (cmd == "AT+H_UP=0")
    {
        config.h_up = false;
        return "OK";
    }

    if (cmd == "AT+TX_DISPLAY?")
        return String(config.tx_display ? "1" : "0");
    else if (cmd == "AT+TX_DISPLAY=1")
    {
        config.tx_display = true;
        return "OK";
    }
    else if (cmd == "AT+TX_DISPLAY=0")
    {
        config.tx_display = false;
        return "OK";
    }

    if (cmd == "AT+RX_DISPLAY?")
        return String(config.rx_display ? "1" : "0");
    else if (cmd == "AT+RX_DISPLAY=1")
    {
        config.rx_display = true;
        return "OK";
    }
    else if (cmd == "AT+RX_DISPLAY=0")
    {
        config.rx_display = false;
        return "OK";
    }

    if (cmd == "AT+DISPFILTER?")
        return String(config.dispFilter);
    else if (cmd.startsWith("AT+DISPFILTER="))
    {
        config.dispFilter = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+DISPRF?")
        return String(config.dispRF ? "1" : "0");
    else if (cmd == "AT+DISPRF=1")
    {
        config.dispRF = true;
        return "OK";
    }
    else if (cmd == "AT+DISPRF=0")
    {
        config.dispRF = false;
        return "OK";
    }

    if (cmd == "AT+DISPINET?")
        return String(config.dispINET ? "1" : "0");
    else if (cmd == "AT+DISPINET=1")
    {
        config.dispINET = true;
        return "OK";
    }
    else if (cmd == "AT+DISPINET=0")
    {
        config.dispINET = false;
        return "OK";
    }

    if (cmd == "AT+AUDIO_HPF?")
        return String(config.audio_hpf ? "1" : "0");
    else if (cmd == "AT+AUDIO_HPF=1")
    {
        config.audio_hpf = true;
        return "OK";
    }
    else if (cmd == "AT+AUDIO_HPF=0")
    {
        config.audio_hpf = false;
        return "OK";
    }

    if (cmd == "AT+AUDIO_LPF?")
        return String(config.audio_lpf ? "1" : "0");
    else if (cmd == "AT+AUDIO_LPF=1")
    {
        config.audio_lpf = true;
        return "OK";
    }
    else if (cmd == "AT+AUDIO_LPF=0")
    {
        config.audio_lpf = false;
        return "OK";
    }

    if (cmd == "AT+PREAMBLE?")
        return String(config.preamble);
    else if (cmd.startsWith("AT+PREAMBLE="))
    {
        config.preamble = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+MODEM_TYPE?")
        return String(config.modem_type);
    else if (cmd.startsWith("AT+MODEM_TYPE="))
    {
        config.modem_type = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+FX25_MODE?")
        return String(config.fx25_mode);
    else if (cmd.startsWith("AT+FX25_MODE="))
    {
        config.fx25_mode = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+TX_TIMESLOT?")
        return String(config.tx_timeslot);
    else if (cmd.startsWith("AT+TX_TIMESLOT="))
    {
        config.tx_timeslot = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+NTP_HOST?")
        return String(config.ntp_host);
    else if (cmd.startsWith("AT+NTP_HOST="))
    {
        cmd.remove(0, 12);
        cmd.replace("\"", "");
        strncpy(config.ntp_host, cmd.c_str(), sizeof(config.ntp_host));
        return "OK";
    }

    if (cmd == "AT+VPN?")
        return String(config.vpn ? "1" : "0");
    else if (cmd == "AT+VPN=1")
    {
        config.vpn = true;
        return "OK";
    }
    else if (cmd == "AT+VPN=0")
    {
        config.vpn = false;
        return "OK";
    }

    if (cmd == "AT+MODEM?")
        return String(config.modem ? "1" : "0");
    else if (cmd == "AT+MODEM=1")
    {
        config.modem = true;
        return "OK";
    }
    else if (cmd == "AT+MODEM=0")
    {
        config.modem = false;
        return "OK";
    }

    if (cmd == "AT+WG_PORT?")
        return String(config.wg_port);
    else if (cmd.startsWith("AT+WG_PORT="))
    {
        config.wg_port = cmd.substring(11).toInt();
        return "OK";
    }

    if (cmd == "AT+WG_PEER_ADDRESS?")
        return String(config.wg_peer_address);
    else if (cmd.startsWith("AT+WG_PEER_ADDRESS="))
    {
        cmd.remove(0, 19);
        cmd.replace("\"", "");
        strncpy(config.wg_peer_address, cmd.c_str(), sizeof(config.wg_peer_address));
        return "OK";
    }

    if (cmd == "AT+WG_LOCAL_ADDRESS?")
        return String(config.wg_local_address);
    else if (cmd.startsWith("AT+WG_LOCAL_ADDRESS="))
    {
        cmd.remove(0, 20);
        cmd.replace("\"", "");
        strncpy(config.wg_local_address, cmd.c_str(), sizeof(config.wg_local_address));
        return "OK";
    }

    if (cmd == "AT+WG_NETMASK_ADDRESS?")
        return String(config.wg_netmask_address);
    else if (cmd.startsWith("AT+WG_NETMASK_ADDRESS="))
    {
        cmd.remove(0, 22);
        cmd.replace("\"", "");
        strncpy(config.wg_netmask_address, cmd.c_str(), sizeof(config.wg_netmask_address));
        return "OK";
    }

    if (cmd == "AT+WG_GW_ADDRESS?")
        return String(config.wg_gw_address);
    else if (cmd.startsWith("AT+WG_GW_ADDRESS="))
    {
        cmd.remove(0, 17);
        cmd.replace("\"", "");
        strncpy(config.wg_gw_address, cmd.c_str(), sizeof(config.wg_gw_address));
        return "OK";
    }

    if (cmd == "AT+WG_PUBLIC_KEY?")
        return String(config.wg_public_key);
    else if (cmd.startsWith("AT+WG_PUBLIC_KEY="))
    {
        cmd.remove(0, 17);
        cmd.replace("\"", "");
        strncpy(config.wg_public_key, cmd.c_str(), sizeof(config.wg_public_key));
        return "OK";
    }

    if (cmd == "AT+WG_PRIVATE_KEY?")
        return String(config.wg_private_key);
    else if (cmd.startsWith("AT+WG_PRIVATE_KEY="))
    {
        cmd.remove(0, 18);
        cmd.replace("\"", "");
        strncpy(config.wg_private_key, cmd.c_str(), sizeof(config.wg_private_key));
        return "OK";
    }

    if (cmd == "AT+HTTP_USERNAME?")
        return String(config.http_username);
    else if (cmd.startsWith("AT+HTTP_USERNAME="))
    {
        cmd.remove(0, 17);
        cmd.replace("\"", "");
        strncpy(config.http_username, cmd.c_str(), sizeof(config.http_username));
        return "OK";
    }

    if (cmd == "AT+HTTP_PASSWORD?")
        return String(config.http_password);
    else if (cmd.startsWith("AT+HTTP_PASSWORD="))
    {
        cmd.remove(0, 17);
        cmd.replace("\"", "");
        strncpy(config.http_password, cmd.c_str(), sizeof(config.http_password));
        return "OK";
    }

    if (cmd == "AT+GNSS_ENABLE?")
        return String(config.gnss_enable ? "1" : "0");
    else if (cmd == "AT+GNSS_ENABLE=1")
    {
        config.gnss_enable = true;
        return "OK";
    }
    else if (cmd == "AT+GNSS_ENABLE=0")
    {
        config.gnss_enable = false;
        return "OK";
    }

    if (cmd == "AT+GNSS_PPS_GPIO?")
        return String(config.gnss_pps_gpio);
    else if (cmd.startsWith("AT+GNSS_PPS_GPIO="))
    {
        config.gnss_pps_gpio = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+GNSS_CHANNEL?")
        return String(config.gnss_channel);
    else if (cmd.startsWith("AT+GNSS_CHANNEL="))
    {
        config.gnss_channel = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+GNSS_TCP_PORT?")
        return String(config.gnss_tcp_port);
    else if (cmd.startsWith("AT+GNSS_TCP_PORT="))
    {
        config.gnss_tcp_port = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+GNSS_TCP_HOST?")
        return String(config.gnss_tcp_host);
    else if (cmd.startsWith("AT+GNSS_TCP_HOST="))
    {
        cmd.remove(0, 17);
        cmd.replace("\"", "");
        strncpy(config.gnss_tcp_host, cmd.c_str(), sizeof(config.gnss_tcp_host));
        return "OK";
    }

    if (cmd == "AT+GNSS_AT_COMMAND?")
        return String(config.gnss_at_command);
    else if (cmd.startsWith("AT+GNSS_AT_COMMAND="))
    {
        cmd.remove(0, 19);
        cmd.replace("\"", "");
        strncpy(config.gnss_at_command, cmd.c_str(), sizeof(config.gnss_at_command));
        return "OK";
    }

    if (cmd == "AT+RF_TX_GPIO?")
        return String(config.rf_tx_gpio);
    else if (cmd.startsWith("AT+RF_TX_GPIO="))
    {
        config.rf_tx_gpio = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+RF_RX_GPIO?")
        return String(config.rf_rx_gpio);
    else if (cmd.startsWith("AT+RF_RX_GPIO="))
    {
        config.rf_rx_gpio = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+RF_SQL_GPIO?")
        return String(config.rf_sql_gpio);
    else if (cmd.startsWith("AT+RF_SQL_GPIO="))
    {
        config.rf_sql_gpio = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+RF_PD_GPIO?")
        return String(config.rf_pd_gpio);
    else if (cmd.startsWith("AT+RF_PD_GPIO="))
    {
        config.rf_pd_gpio = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+RF_PWR_GPIO?")
        return String(config.rf_pwr_gpio);
    else if (cmd.startsWith("AT+RF_PWR_GPIO="))
    {
        config.rf_pwr_gpio = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+RF_PTT_GPIO?")
        return String(config.rf_ptt_gpio);
    else if (cmd.startsWith("AT+RF_PTT_GPIO="))
    {
        config.rf_ptt_gpio = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+RF_SQL_ACTIVE?")
        return String(config.rf_sql_active ? "1" : "0");
    else if (cmd == "AT+RF_SQL_ACTIVE=1")
    {
        config.rf_sql_active = true;
        return "OK";
    }
    else if (cmd == "AT+RF_SQL_ACTIVE=0")
    {
        config.rf_sql_active = false;
        return "OK";
    }

    if (cmd == "AT+RF_PD_ACTIVE?")
        return String(config.rf_pd_active ? "1" : "0");
    else if (cmd == "AT+RF_PD_ACTIVE=1")
    {
        config.rf_pd_active = true;
        return "OK";
    }
    else if (cmd == "AT+RF_PD_ACTIVE=0")
    {
        config.rf_pd_active = false;
        return "OK";
    }

    if (cmd == "AT+RF_PWR_ACTIVE?")
        return String(config.rf_pwr_active ? "1" : "0");
    else if (cmd == "AT+RF_PWR_ACTIVE=1")
    {
        config.rf_pwr_active = true;
        return "OK";
    }
    else if (cmd == "AT+RF_PWR_ACTIVE=0")
    {
        config.rf_pwr_active = false;
        return "OK";
    }

    if (cmd == "AT+RF_PTT_ACTIVE?")
        return String(config.rf_ptt_active ? "1" : "0");
    else if (cmd == "AT+RF_PTT_ACTIVE=1")
    {
        config.rf_ptt_active = true;
        return "OK";
    }
    else if (cmd == "AT+RF_PTT_ACTIVE=0")
    {
        config.rf_ptt_active = false;
        return "OK";
    }

    if (cmd == "AT+ADC_GPIO?")
        return String(config.adc_gpio);
    else if (cmd.startsWith("AT+ADC_GPIO="))
    {
        config.adc_gpio = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+DAC_GPIO?")
        return String(config.dac_gpio);
    else if (cmd.startsWith("AT+DAC_GPIO="))
    {
        config.dac_gpio = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+ADC_SEL_GPIO?")
        return String(config.adc_sel_gpio);
    else if (cmd.startsWith("AT+ADC_SEL_GPIO="))
    {
        config.adc_sel_gpio = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+DAC_SEL_GPIO?")
        return String(config.dac_sel_gpio);
    else if (cmd.startsWith("AT+DAC_SEL_GPIO="))
    {
        config.dac_sel_gpio = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+ADC_ATTEN?")
        return String(config.adc_atten);
    else if (cmd.startsWith("AT+ADC_ATTEN="))
    {
        config.adc_atten = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+ADC_DC_OFFSET?")
        return String(config.adc_dc_offset);
    else if (cmd.startsWith("AT+ADC_DC_OFFSET="))
    {
        config.adc_dc_offset = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+I2C_ENABLE?")
        return String(config.i2c_enable ? "1" : "0");
    else if (cmd == "AT+I2C_ENABLE=1")
    {
        config.i2c_enable = true;
        return "OK";
    }
    else if (cmd == "AT+I2C_ENABLE=0")
    {
        config.i2c_enable = false;
        return "OK";
    }

    if (cmd == "AT+I2C_SDA_PIN?")
        return String(config.i2c_sda_pin);
    else if (cmd.startsWith("AT+I2C_SDA_PIN="))
    {
        config.i2c_sda_pin = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+I2C_SCK_PIN?")
        return String(config.i2c_sck_pin);
    else if (cmd.startsWith("AT+I2C_SCK_PIN="))
    {
        config.i2c_sck_pin = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+I2C_RST_PIN?")
        return String(config.i2c_rst_pin);
    else if (cmd.startsWith("AT+I2C_RST_PIN="))
    {
        config.i2c_rst_pin = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+I2C_FREQ?")
        return String(config.i2c_freq);
    else if (cmd.startsWith("AT+I2C_FREQ="))
    {
        config.i2c_freq = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+I2C1_ENABLE?")
        return String(config.i2c1_enable ? "1" : "0");
    else if (cmd == "AT+I2C1_ENABLE=1")
    {
        config.i2c1_enable = true;
        return "OK";
    }
    else if (cmd == "AT+I2C1_ENABLE=0")
    {
        config.i2c1_enable = false;
        return "OK";
    }

    if (cmd == "AT+I2C1_SDA_PIN?")
        return String(config.i2c1_sda_pin);
    else if (cmd.startsWith("AT+I2C1_SDA_PIN="))
    {
        config.i2c1_sda_pin = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+I2C1_SCK_PIN?")
        return String(config.i2c1_sck_pin);
    else if (cmd.startsWith("AT+I2C1_SCK_PIN="))
    {
        config.i2c1_sck_pin = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+I2C1_FREQ?")
        return String(config.i2c1_freq);
    else if (cmd.startsWith("AT+I2C1_FREQ="))
    {
        config.i2c1_freq = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+ONEWIRE_ENABLE?")
        return String(config.onewire_enable ? "1" : "0");
    else if (cmd == "AT+ONEWIRE_ENABLE=1")
    {
        config.onewire_enable = true;
        return "OK";
    }
    else if (cmd == "AT+ONEWIRE_ENABLE=0")
    {
        config.onewire_enable = false;
        return "OK";
    }

    if (cmd == "AT+ONEWIRE_GPIO?")
        return String(config.onewire_gpio);
    else if (cmd.startsWith("AT+ONEWIRE_GPIO="))
    {
        config.onewire_gpio = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+UART0_ENABLE?")
        return String(config.uart0_enable ? "1" : "0");
    else if (cmd == "AT+UART0_ENABLE=1")
    {
        config.uart0_enable = true;
        return "OK";
    }
    else if (cmd == "AT+UART0_ENABLE=0")
    {
        config.uart0_enable = false;
        return "OK";
    }

    if (cmd == "AT+UART0_TX_GPIO?")
        return String(config.uart0_tx_gpio);
    else if (cmd.startsWith("AT+UART0_TX_GPIO="))
    {
        config.uart0_tx_gpio = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+UART0_RX_GPIO?")
        return String(config.uart0_rx_gpio);
    else if (cmd.startsWith("AT+UART0_RX_GPIO="))
    {
        config.uart0_rx_gpio = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+UART0_RTS_GPIO?")
        return String(config.uart0_rts_gpio);
    else if (cmd.startsWith("AT+UART0_RTS_GPIO="))
    {
        config.uart0_rts_gpio = cmd.substring(18).toInt();
        return "OK";
    }

    if (cmd == "AT+UART1_ENABLE?")
        return String(config.uart1_enable ? "1" : "0");
    else if (cmd == "AT+UART1_ENABLE=1")
    {
        config.uart1_enable = true;
        return "OK";
    }
    else if (cmd == "AT+UART1_ENABLE=0")
    {
        config.uart1_enable = false;
        return "OK";
    }

    if (cmd == "AT+UART1_TX_GPIO?")
        return String(config.uart1_tx_gpio);
    else if (cmd.startsWith("AT+UART1_TX_GPIO="))
    {
        config.uart1_tx_gpio = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+UART1_RX_GPIO?")
        return String(config.uart1_rx_gpio);
    else if (cmd.startsWith("AT+UART1_RX_GPIO="))
    {
        config.uart1_rx_gpio = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+UART1_RTS_GPIO?")
        return String(config.uart1_rts_gpio);
    else if (cmd.startsWith("AT+UART1_RTS_GPIO="))
    {
        config.uart1_rts_gpio = cmd.substring(18).toInt();
        return "OK";
    }

    if (cmd == "AT+MODBUS_ENABLE?")
        return String(config.modbus_enable ? "1" : "0");
    else if (cmd == "AT+MODBUS_ENABLE=1")
    {
        config.modbus_enable = true;
        return "OK";
    }
    else if (cmd == "AT+MODBUS_ENABLE=0")
    {
        config.modbus_enable = false;
        return "OK";
    }

    if (cmd == "AT+MODBUS_ADDRESS?")
        return String(config.modbus_address);
    else if (cmd.startsWith("AT+MODBUS_ADDRESS="))
    {
        config.modbus_address = cmd.substring(18).toInt();
        return "OK";
    }

    if (cmd == "AT+MODBUS_CHANNEL?")
        return String(config.modbus_channel);
    else if (cmd.startsWith("AT+MODBUS_CHANNEL="))
    {
        config.modbus_channel = cmd.substring(18).toInt();
        return "OK";
    }

    if (cmd == "AT+MODBUS_DE_GPIO?")
        return String(config.modbus_de_gpio);
    else if (cmd.startsWith("AT+MODBUS_DE_GPIO="))
    {
        config.modbus_de_gpio = cmd.substring(18).toInt();
        return "OK";
    }

    if (cmd == "AT+COUNTER0_ENABLE?")
        return String(config.counter0_enable ? "1" : "0");
    else if (cmd == "AT+COUNTER0_ENABLE=1")
    {
        config.counter0_enable = true;
        return "OK";
    }
    else if (cmd == "AT+COUNTER0_ENABLE=0")
    {
        config.counter0_enable = false;
        return "OK";
    }

    if (cmd == "AT+COUNTER0_ACTIVE?")
        return String(config.counter0_active ? "1" : "0");
    else if (cmd == "AT+COUNTER0_ACTIVE=1")
    {
        config.counter0_active = true;
        return "OK";
    }
    else if (cmd == "AT+COUNTER0_ACTIVE=0")
    {
        config.counter0_active = false;
        return "OK";
    }

    if (cmd == "AT+COUNTER0_GPIO?")
        return String(config.counter0_gpio);
    else if (cmd.startsWith("AT+COUNTER0_GPIO="))
    {
        config.counter0_gpio = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+COUNTER1_ENABLE?")
        return String(config.counter1_enable ? "1" : "0");
    else if (cmd == "AT+COUNTER1_ENABLE=1")
    {
        config.counter1_enable = true;
        return "OK";
    }
    else if (cmd == "AT+COUNTER1_ENABLE=0")
    {
        config.counter1_enable = false;
        return "OK";
    }

    if (cmd == "AT+COUNTER1_ACTIVE?")
        return String(config.counter1_active ? "1" : "0");
    else if (cmd == "AT+COUNTER1_ACTIVE=1")
    {
        config.counter1_active = true;
        return "OK";
    }
    else if (cmd == "AT+COUNTER1_ACTIVE=0")
    {
        config.counter1_active = false;
        return "OK";
    }

    if (cmd == "AT+COUNTER1_GPIO?")
        return String(config.counter1_gpio);
    else if (cmd.startsWith("AT+COUNTER1_GPIO="))
    {
        config.counter1_gpio = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+EXT_TNC_ENABLE?")
        return String(config.ext_tnc_enable ? "1" : "0");
    else if (cmd == "AT+EXT_TNC_ENABLE=1")
    {
        config.ext_tnc_enable = true;
        return "OK";
    }
    else if (cmd == "AT+EXT_TNC_ENABLE=0")
    {
        config.ext_tnc_enable = false;
        return "OK";
    }

    if (cmd == "AT+EXT_TNC_CHANNEL?")
        return String(config.ext_tnc_channel);
    else if (cmd.startsWith("AT+EXT_TNC_CHANNEL="))
    {
        config.ext_tnc_channel = cmd.substring(19).toInt();
        return "OK";
    }

    if (cmd == "AT+EXT_TNC_MODE?")
        return String(config.ext_tnc_mode);
    else if (cmd.startsWith("AT+EXT_TNC_MODE="))
    {
        config.ext_tnc_mode = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+PWR_EN?")
        return String(config.pwr_en ? "1" : "0");
    else if (cmd == "AT+PWR_EN=1")
    {
        config.pwr_en = true;
        return "OK";
    }
    else if (cmd == "AT+PWR_EN=0")
    {
        config.pwr_en = false;
        return "OK";
    }

    if (cmd == "AT+PWR_MODE?")
        return String(config.pwr_mode);
    else if (cmd.startsWith("AT+PWR_MODE="))
    {
        config.pwr_mode = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+PWR_SLEEP_INTERVAL?")
        return String(config.pwr_sleep_interval);
    else if (cmd.startsWith("AT+PWR_SLEEP_INTERVAL="))
    {
        config.pwr_sleep_interval = cmd.substring(22).toInt();
        return "OK";
    }

    if (cmd == "AT+PWR_STANBY_DELAY?")
        return String(config.pwr_stanby_delay);
    else if (cmd.startsWith("AT+PWR_STANBY_DELAY="))
    {
        config.pwr_stanby_delay = cmd.substring(20).toInt();
        return "OK";
    }

    if (cmd == "AT+PWR_SLEEP_ACTIVATE?")
        return String(config.pwr_sleep_activate);
    else if (cmd.startsWith("AT+PWR_SLEEP_ACTIVATE="))
    {
        config.pwr_sleep_activate = cmd.substring(22).toInt();
        return "OK";
    }

    if (cmd == "AT+PWR_GPIO?")
        return String(config.pwr_gpio);
    else if (cmd.startsWith("AT+PWR_GPIO="))
    {
        config.pwr_gpio = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+PWR_ACTIVE?")
        return String(config.pwr_active ? "1" : "0");
    else if (cmd == "AT+PWR_ACTIVE=1")
    {
        config.pwr_active = true;
        return "OK";
    }
    else if (cmd == "AT+PWR_ACTIVE=0")
    {
        config.pwr_active = false;
        return "OK";
    }

    if (cmd == "AT+DISP_FLIP?")
        return String(config.disp_flip ? "1" : "0");
    else if (cmd == "AT+DISP_FLIP=1")
    {
        config.disp_flip = true;
        return "OK";
    }
    else if (cmd == "AT+DISP_FLIP=0")
    {
        config.disp_flip = false;
        return "OK";
    }

    if (cmd == "AT+DISP_BRIGHTNESS?")
        return String(config.disp_brightness);
    else if (cmd.startsWith("AT+DISP_BRIGHTNESS="))
    {
        config.disp_brightness = cmd.substring(19).toInt();
        return "OK";
    }

    if (cmd == "AT+LOG?")
        return String(config.log);
    else if (cmd.startsWith("AT+LOG="))
    {
        config.log = cmd.substring(7).toInt();
        return "OK";
    }

    // if (cmd == "AT+TRK_TLM_AVG?")
    //     return String(config.trk_tlm_avg ? "1" : "0");
    // else if (cmd == "AT+TRK_TLM_AVG=1")
    // {
    //     config.trk_tlm_avg = true;
    //     return "OK";
    // }
    // else if (cmd == "AT+TRK_TLM_AVG=0")
    // {
    //     config.trk_tlm_avg = false;
    //     return "OK";
    // }

    // if (cmd == "AT+TRK_TLM_SENSOR?")
    //     return String(config.trk_tlm_sensor);
    // else if (cmd.startsWith("AT+TRK_TLM_SENSOR="))
    // {
    //     config.trk_tlm_sensor = cmd.substring(18).toInt();
    //     return "OK";
    // }

    // if (cmd == "AT+TRK_TLM_PRECISION?")
    //     return String(config.trk_tlm_precision);
    // else if (cmd.startsWith("AT+TRK_TLM_PRECISION="))
    // {
    //     config.trk_tlm_precision = cmd.substring(21).toInt();
    //     return "OK";
    // }

    // if (cmd == "AT+TRK_TLM_OFFSET?")
    //     return String(config.trk_tlm_offset, 6);
    // else if (cmd.startsWith("AT+TRK_TLM_OFFSET="))
    // {
    //     config.trk_tlm_offset = cmd.substring(18).toFloat();
    //     return "OK";
    // }

    // if (cmd == "AT+DIGI_TLM_AVG?")
    //     return String(config.digi_tlm_avg ? "1" : "0");
    // else if (cmd == "AT+DIGI_TLM_AVG=1")
    // {
    //     config.digi_tlm_avg = true;
    //     return "OK";
    // }
    // else if (cmd == "AT+DIGI_TLM_AVG=0")
    // {
    //     config.digi_tlm_avg = false;
    //     return "OK";
    // }

    // if (cmd == "AT+DIGI_TLM_SENSOR?")
    //     return String(config.digi_tlm_sensor);
    // else if (cmd.startsWith("AT+DIGI_TLM_SENSOR="))
    // {
    //     config.digi_tlm_sensor = cmd.substring(19).toInt();
    //     return "OK";
    // }

    // if (cmd == "AT+DIGI_TLM_PRECISION?")
    //     return String(config.digi_tlm_precision);
    // else if (cmd.startsWith("AT+DIGI_TLM_PRECISION="))
    // {
    //     config.digi_tlm_precision = cmd.substring(22).toInt();
    //     return "OK";
    // }

    // if (cmd == "AT+DIGI_TLM_OFFSET?")
    //     return String(config.digi_tlm_offset, 6);
    // else if (cmd.startsWith("AT+DIGI_TLM_OFFSET="))
    // {
    //     config.digi_tlm_offset = cmd.substring(19).toFloat();
    //     return "OK";
    // }

    // if (cmd == "AT+IGATE_TLM_AVG?")
    //     return String(config.igate_tlm_avg ? "1" : "0");
    // else if (cmd == "AT+IGATE_TLM_AVG=1")
    // {
    //     config.igate_tlm_avg = true;
    //     return "OK";
    // }
    // else if (cmd == "AT+IGATE_TLM_AVG=0")
    // {
    //     config.igate_tlm_avg = false;
    //     return "OK";
    // }

    // if (cmd == "AT+IGATE_TLM_SENSOR?")
    //     return String(config.igate_tlm_sensor);
    // else if (cmd.startsWith("AT+IGATE_TLM_SENSOR="))
    // {
    //     config.igate_tlm_sensor = cmd.substring(20).toInt();
    //     return "OK";
    // }

    // if (cmd == "AT+IGATE_TLM_PRECISION?")
    //     return String(config.igate_tlm_precision);
    // else if (cmd.startsWith("AT+IGATE_TLM_PRECISION="))
    // {
    //     config.igate_tlm_precision = cmd.substring(23).toInt();
    //     return "OK";
    // }

    // if (cmd == "AT+IGATE_TLM_OFFSET?")
    //     return String(config.igate_tlm_offset, 6);
    // else if (cmd.startsWith("AT+IGATE_TLM_OFFSET="))
    // {
    //     config.igate_tlm_offset = cmd.substring(20).toFloat();
    //     return "OK";
    // }

    // if (cmd == "AT+WX_SENSOR_ENABLE?")
    //     return String(config.wx_sensor_enable ? "1" : "0");
    // else if (cmd == "AT+WX_SENSOR_ENABLE=1")
    // {
    //     config.wx_sensor_enable = true;
    //     return "OK";
    // }
    // else if (cmd == "AT+WX_SENSOR_ENABLE=0")
    // {
    //     config.wx_sensor_enable = false;
    //     return "OK";
    // }

    // if (cmd == "AT+WX_SENSOR_AVG?")
    //     return String(config.wx_sensor_avg ? "1" : "0");
    // else if (cmd == "AT+WX_SENSOR_AVG=1")
    // {
    //     config.wx_sensor_avg = true;
    //     return "OK";
    // }
    // else if (cmd == "AT+WX_SENSOR_AVG=0")
    // {
    //     config.wx_sensor_avg = false;
    //     return "OK";
    // }

    // if (cmd == "AT+WX_SENSOR_CH?")
    //     return String(config.wx_sensor_ch);
    // else if (cmd.startsWith("AT+WX_SENSOR_CH="))
    // {
    //     config.wx_sensor_ch = cmd.substring(16).toInt();
    //     return "OK";
    // }

    if (cmd == "AT+PPP_ENABLE?")
        return String(config.ppp_enable ? "1" : "0");
    else if (cmd == "AT+PPP_ENABLE=1")
    {
        config.ppp_enable = true;
        return "OK";
    }
    else if (cmd == "AT+PPP_ENABLE=0")
    {
        config.ppp_enable = false;
        return "OK";
    }

    if (cmd == "AT+PPP_APN?")
        return String(config.ppp_apn);
    else if (cmd.startsWith("AT+PPP_APN="))
    {
        cmd.remove(0, 11);
        cmd.replace("\"", "");
        strncpy(config.ppp_apn, cmd.c_str(), sizeof(config.ppp_apn));
        return "OK";
    }

    if (cmd == "AT+PPP_PIN?")
        return String(config.ppp_pin);
    else if (cmd.startsWith("AT+PPP_PIN="))
    {
        cmd.remove(0, 11);
        cmd.replace("\"", "");
        strncpy(config.ppp_pin, cmd.c_str(), sizeof(config.ppp_pin));
        return "OK";
    }

    if (cmd == "AT+PPP_RST_GPIO?")
        return String(config.ppp_rst_gpio);
    else if (cmd.startsWith("AT+PPP_RST_GPIO="))
    {
        config.ppp_rst_gpio = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_TX_GPIO?")
        return String(config.ppp_tx_gpio);
    else if (cmd.startsWith("AT+PPP_TX_GPIO="))
    {
        config.ppp_tx_gpio = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_RX_GPIO?")
        return String(config.ppp_rx_gpio);
    else if (cmd.startsWith("AT+PPP_RX_GPIO="))
    {
        config.ppp_rx_gpio = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_RTS_GPIO?")
        return String(config.ppp_rts_gpio);
    else if (cmd.startsWith("AT+PPP_RTS_GPIO="))
    {
        config.ppp_rts_gpio = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_CTS_GPIO?")
        return String(config.ppp_cts_gpio);
    else if (cmd.startsWith("AT+PPP_CTS_GPIO="))
    {
        config.ppp_cts_gpio = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_DTR_GPIO?")
        return String(config.ppp_dtr_gpio);
    else if (cmd.startsWith("AT+PPP_DTR_GPIO="))
    {
        config.ppp_dtr_gpio = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_RI_GPIO?")
        return String(config.ppp_ri_gpio);
    else if (cmd.startsWith("AT+PPP_RI_GPIO="))
    {
        config.ppp_ri_gpio = cmd.substring(15).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_RST_ACTIVE?")
        return String(config.ppp_rst_active ? "1" : "0");
    else if (cmd == "AT+PPP_RST_ACTIVE=1")
    {
        config.ppp_rst_active = true;
        return "OK";
    }
    else if (cmd == "AT+PPP_RST_ACTIVE=0")
    {
        config.ppp_rst_active = false;
        return "OK";
    }

    if (cmd == "AT+PPP_RST_DELAY?")
        return String(config.ppp_rst_delay);
    else if (cmd.startsWith("AT+PPP_RST_DELAY="))
    {
        config.ppp_rst_delay = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_PWR_GPIO?")
        return String(config.ppp_pwr_gpio);
    else if (cmd.startsWith("AT+PPP_PWR_GPIO="))
    {
        config.ppp_pwr_gpio = cmd.substring(16).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_PWR_ACTIVE?")
        return String(config.ppp_pwr_active ? "1" : "0");
    else if (cmd == "AT+PPP_PWR_ACTIVE=1")
    {
        config.ppp_pwr_active = true;
        return "OK";
    }
    else if (cmd == "AT+PPP_PWR_ACTIVE=0")
    {
        config.ppp_pwr_active = false;
        return "OK";
    }

    if (cmd == "AT+PPP_SERIAL?")
        return String(config.ppp_serial);
    else if (cmd.startsWith("AT+PPP_SERIAL="))
    {
        config.ppp_serial = cmd.substring(14).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_MODEL?")
        return String(config.ppp_model);
    else if (cmd.startsWith("AT+PPP_MODEL="))
    {
        config.ppp_model = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_FLOW_CTRL?")
        return String(config.ppp_flow_ctrl);
    else if (cmd.startsWith("AT+PPP_FLOW_CTRL="))
    {
        config.ppp_flow_ctrl = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+PPP_GNSS?")
        return String(config.ppp_gnss ? "1" : "0");
    else if (cmd == "AT+PPP_GNSS=1")
    {
        config.ppp_gnss = true;
        return "OK";
    }
    else if (cmd == "AT+PPP_GNSS=0")
    {
        config.ppp_gnss = false;
        return "OK";
    }

#ifdef MQTT
    if (cmd == "AT+EN_MQTT?")
        return String(config.en_mqtt ? "1" : "0");
    else if (cmd == "AT+EN_MQTT=1")
    {
        config.en_mqtt = true;
        return "OK";
    }
    else if (cmd == "AT+EN_MQTT=0")
    {
        config.en_mqtt = false;
        return "OK";
    }

    if (cmd == "AT+MQTT_HOST?")
        return String(config.mqtt_host);
    else if (cmd.startsWith("AT+MQTT_HOST="))
    {
        cmd.remove(0, 13);
        cmd.replace("\"", "");
        strncpy(config.mqtt_host, cmd.c_str(), sizeof(config.mqtt_host));
        return "OK";
    }

    if (cmd == "AT+MQTT_TOPIC?")
        return String(config.mqtt_topic);
    else if (cmd.startsWith("AT+MQTT_TOPIC="))
    {
        cmd.remove(0, 14);
        cmd.replace("\"", "");
        strncpy(config.mqtt_topic, cmd.c_str(), sizeof(config.mqtt_topic));
        return "OK";
    }

    if (cmd == "AT+MQTT_SUBSCRIBE?")
        return String(config.mqtt_subscribe);
    else if (cmd.startsWith("AT+MQTT_SUBSCRIBE="))
    {
        cmd.remove(0, 18);
        cmd.replace("\"", "");
        strncpy(config.mqtt_subscribe, cmd.c_str(), sizeof(config.mqtt_subscribe));
        return "OK";
    }

    if (cmd == "AT+MQTT_USER?")
        return String(config.mqtt_user);
    else if (cmd.startsWith("AT+MQTT_USER="))
    {
        cmd.remove(0, 13);
        cmd.replace("\"", "");
        strncpy(config.mqtt_user, cmd.c_str(), sizeof(config.mqtt_user));
        return "OK";
    }

    if (cmd == "AT+MQTT_PASS?")
        return String(config.mqtt_pass);
    else if (cmd.startsWith("AT+MQTT_PASS="))
    {
        cmd.remove(0, 13);
        cmd.replace("\"", "");
        strncpy(config.mqtt_pass, cmd.c_str(), sizeof(config.mqtt_pass));
        return "OK";
    }

    if (cmd == "AT+MQTT_PORT?")
        return String(config.mqtt_port);
    else if (cmd.startsWith("AT+MQTT_PORT="))
    {
        config.mqtt_port = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+MQTT_TOPIC_FLAG?")
        return String(config.mqtt_topic_flag);
    else if (cmd.startsWith("AT+MQTT_TOPIC_FLAG="))
    {
        config.mqtt_topic_flag = cmd.substring(19).toInt();
        return "OK";
    }

    if (cmd == "AT+MQTT_SUBSCRIBE_FLAG?")
        return String(config.mqtt_subscribe_flag);
    else if (cmd.startsWith("AT+MQTT_SUBSCRIBE_FLAG="))
    {
        config.mqtt_subscribe_flag = cmd.substring(23).toInt();
        return "OK";
    }

#endif
    if (cmd == "AT+TRK_MICE_TYPE?")
        return String(config.trk_mice_type);
    else if (cmd.startsWith("AT+TRK_MICE_TYPE="))
    {
        config.trk_mice_type = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+TRK_TLM_INTERVAL?")
        return String(config.trk_tlm_interval);
    else if (cmd.startsWith("AT+TRK_TLM_INTERVAL="))
    {
        config.trk_tlm_interval = cmd.substring(20).toInt();
        return "OK";
    }

    if (cmd == "AT+DIGI_TLM_INTERVAL?")
        return String(config.digi_tlm_interval);
    else if (cmd.startsWith("AT+DIGI_TLM_INTERVAL="))
    {
        config.digi_tlm_interval = cmd.substring(21).toInt();
        return "OK";
    }

    if (cmd == "AT+IGATE_TLM_INTERVAL?")
        return String(config.igate_tlm_interval);
    else if (cmd.startsWith("AT+IGATE_TLM_INTERVAL="))
    {
        config.igate_tlm_interval = cmd.substring(22).toInt();
        return "OK";
    }

    if (cmd == "AT+WX_TLM_INTERVAL?")
        return String(config.wx_tlm_interval);
    else if (cmd.startsWith("AT+WX_TLM_INTERVAL="))
    {
        config.wx_tlm_interval = cmd.substring(19).toInt();
        return "OK";
    }

    if (cmd == "AT+HOST_NAME?")
        return String(config.host_name);
    else if (cmd.startsWith("AT+HOST_NAME="))
    {
        cmd.remove(0, 13);
        cmd.replace("\"", "");
        strncpy(config.host_name, cmd.c_str(), sizeof(config.host_name));
        return "OK";
    }

    if (cmd == "AT+RESET_TIMEOUT?")
        return String(config.reset_timeout);
    else if (cmd.startsWith("AT+RESET_TIMEOUT="))
    {
        config.reset_timeout = cmd.substring(17).toInt();
        return "OK";
    }

    if (cmd == "AT+MSG_ENABLE?")
        return String(config.msg_enable ? "1" : "0");
    else if (cmd == "AT+MSG_ENABLE=1")
    {
        config.msg_enable = true;
        return "OK";
    }
    else if (cmd == "AT+MSG_ENABLE=0")
    {
        config.msg_enable = false;
        return "OK";
    }

    if (cmd == "AT+MSG_MYCALL?")
        return String(config.msg_mycall);
    else if (cmd.startsWith("AT+MSG_MYCALL="))
    {
        cmd.remove(0, 14);
        cmd.replace("\"", "");
        strncpy(config.msg_mycall, cmd.c_str(), sizeof(config.msg_mycall));
        return "OK";
    }

    if (cmd == "AT+MSG_PATH?")
        return String(config.msg_path);
    else if (cmd.startsWith("AT+MSG_PATH="))
    {
        config.msg_path = cmd.substring(12).toInt();
        return "OK";
    }

    if (cmd == "AT+MSG_RF?")
        return String(config.msg_rf ? "1" : "0");
    else if (cmd == "AT+MSG_RF=1")
    {
        config.msg_rf = true;
        return "OK";
    }
    else if (cmd == "AT+MSG_RF=0")
    {
        config.msg_rf = false;
        return "OK";
    }

    if (cmd == "AT+MSG_INET?")
        return String(config.msg_inet ? "1" : "0");
    else if (cmd == "AT+MSG_INET=1")
    {
        config.msg_inet = true;
        return "OK";
    }
    else if (cmd == "AT+MSG_INET=0")
    {
        config.msg_inet = false;
        return "OK";
    }

    if (cmd == "AT+MSG_ENCRYPT?")
        return String(config.msg_encrypt ? "1" : "0");
    else if (cmd == "AT+MSG_ENCRYPT=1")
    {
        config.msg_encrypt = true;
        return "OK";
    }
    else if (cmd == "AT+MSG_ENCRYPT=0")
    {
        config.msg_encrypt = false;
        return "OK";
    }

    if (cmd == "AT+MSG_KEY?")
        return String(config.msg_key);
    else if (cmd.startsWith("AT+MSG_KEY="))
    {
        cmd.remove(0, 11);
        cmd.replace("\"", "");
        strncpy(config.msg_key, cmd.c_str(), sizeof(config.msg_key));
        return "OK";
    }

    if (cmd == "AT+MSG_RETRY?")
        return String(config.msg_retry);
    else if (cmd.startsWith("AT+MSG_RETRY="))
    {
        config.msg_retry = cmd.substring(13).toInt();
        return "OK";
    }

    if (cmd == "AT+MSG_INTERVAL?")
        return String(config.msg_interval);
    else if (cmd.startsWith("AT+MSG_INTERVAL="))
    {
        config.msg_interval = cmd.substring(16).toInt();
        return "OK";
    }

    return "ERR";
}
