/*
 Name:		ESP32 APRS Internet Gateway
 Created:	1-Nov-2021 14:27:23
 Author:	HS5TQA/Atten
 Support IS: host:aprs.dprns.com port:14580
 Support IS monitor: http://aprs.dprns.com:14501
 Support in LINE Group APRS Only
*/

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "main.h"
#include <LibAPRSesp.h>
#include <limits.h>
#include <KISS.h>
#include "webservice.h"
#include <WiFiUdp.h>
#include "ESP32Ping.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include "cppQueue.h"
#include "digirepeater.h"
#include "igate.h"
#include "wireguardif.h"
#include "wireguard.h"
#include "driver/pcnt.h"

#include <TinyGPS++.h>
#include <pbuf.h>
#include <parse_aprs.h>

#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSerifItalic9pt7b.h>
#include "Font/Seven_Segment24pt7b.h"

#include "wireguard_vpn.h"
#include <WiFiUdp.h>

#include <WiFiClientSecure.h>

#include <SoftwareSerial.h>
// SoftwareSerial ss(-1, -1);

#include <ModbusMaster.h>

#include "sensor.h"

#include <rom/spi_flash.h>
#include "FS.h"
#include <LITTLEFS.h>

#include <time.h>

#ifdef MQTT
#include <PubSubClient.h>
#endif

#include "AFSK.h"

#ifdef PPPOS
#include <PPP.h>
#define PPP_MODEM_APN "internet"
#define PPP_MODEM_PIN "0000" // or NULL
#define PPP_MODEM_RST_DELAY 500
#define PPP_MODEM_RST 13
#define PPP_MODEM_RST_LOW true // active LOW
#define PPP_MODEM_TX 12
#define PPP_MODEM_RX 11
#define PPP_MODEM_RTS -1
#define PPP_MODEM_CTS -1
#define PPP_MODEM_FC ESP_MODEM_FLOW_CONTROL_NONE
#define PPP_MODEM_MODEL PPP_MODEM_SIM800
#endif

#define CONFIG_I2C_ENABLE_DEBUG_LOG 0

#define EEPROM_SIZE 4096

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
#ifdef __XTENSA__
#define BOOT_PIN 9
#else
#define BOOT_PIN 0
#endif
#endif

#include <Wire.h>
#ifndef __XTENSA__
TwoWire Wire1 = TwoWire(1);
#endif
#ifdef OLED
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>

#if defined(SSD1306_72x40)
#define SCREEN_WIDTH 72  // OLED display width, in pixels
#define SCREEN_HEIGHT 40 // OLED display height, in pixels
#else
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#endif

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#ifdef SH1106
#include <Adafruit_SH1106.h>
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SH1106 display(OLED_RESET);
#else
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1
#ifdef SSD1306_72x40
#define SSD1306_NO_SPLASH
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#else
#define SCREEN_ADDRESS 0x3C //(0x3C for 32-pixel-tall displays, 0x3D for all others).
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

#endif

#endif

#define FORMAT_LITTLEFS_IF_FAILED true
extern fs::LITTLEFSFS LITTLEFS;

bool i2c_busy = false;

struct pbuf_t aprs;
ParseAPRS aprsParse;

TinyGPSPlus gps;

// instantiate ModbusMaster object
ModbusMaster modbus;

// #ifdef __XTENSA__
// #define BOOT_PIN 0
// #else
// #define BOOT_PIN 9
// #endif

#define VBAT_PIN 35
// #define WIRE 4
//  #define POWER_PIN 12
//  #define PULLDOWN_PIN 27
//  #define SQL_PIN 33

// #define MODEM_PWRKEY 5
// #define MODEM_TX 17
// #define MODEM_RX 16
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel *strip=NULL;

#ifdef STRIP_PIN
#define LED_TX -1
#define LED_RX -1
// portMUX_TYPE ledMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR LED_Status(uint8_t r, uint8_t g, uint8_t b)
{
    // portENTER_CRITICAL_ISR(&ledMux);          // ISR start
    strip->setPixelColor(0, strip->Color(r, g, b));
    strip->show();
    // portEXIT_CRITICAL_ISR(&ledMux);
}
#else
#define LED_TX 4
#define LED_RX 2
void LED_Status(uint8_t r, uint8_t g, uint8_t b)
{
    // portENTER_CRITICAL_ISR(&ledMux);          // ISR start
    if (LED_TX > -1)
    {
        if (r > 0)
            digitalWrite(LED_TX, HIGH);
        else
            digitalWrite(LED_TX, LOW);
    }
    if (LED_RX > -1)
    {
        if (g > 0)
            digitalWrite(LED_RX, HIGH);
        else
            digitalWrite(LED_RX, LOW);
    }
    // portEXIT_CRITICAL_ISR(&ledMux);
}
#endif

#define PPP_APN "internet"
#define PPP_USER ""
#define PPP_PASS ""


const char *str_status[] = {
    "IDLE_STATUS",
    "NO_SSID_AVAIL",
    "SCAN_COMPLETED",
    "CONNECTED",
    "CONNECT_FAILED",
    "CONNECTION_LOST",
    "DISCONNECTED"};

WiFiMulti wifiMulti;

time_t systemUptime = 0;
time_t wifiUptime = 0;

uint8_t Sleep_Activate = 0;
unsigned long StandByTick = 0;

bool lastHeard_Flag = 0;

boolean KISS = false;
bool aprsUpdate = false;

long timeSleep = 0;

boolean gotPacket = false;
AX25Msg incomingPacket;

bool lastPkg = false;
bool afskSync = false;
float dBV = 0;
int mVrms = 0;
float rssi = 0;
float snr = 0;
float freqErr = 0;

extern int8_t adcEn;
extern int8_t dacEn;

long timeNetwork, timeAprs, timeGui;
long autoResetTimeout = 0;

// cppQueue PacketBuffer(sizeof(AX25Msg), 5, IMPLEMENTATION); // Instantiate queue
#ifdef OLED
cppQueue dispBuffer(300, 5, IMPLEMENTATION);
cppQueue queTxDisp(sizeof(txDisp), 5, IMPLEMENTATION); // Instantiate queue

void pushTxDisp(uint8_t ch, const char *name, char *info)
{
    if (!config.tx_display)
        return;

    txDisp pkg;

    pkg.tx_ch = ch;
    strcpy(pkg.name, name);
    strcpy(pkg.info, info);
    queTxDisp.push(&pkg); // ใส่แพ็จเก็จจาก TNC ลงคิวบัพเฟอร์
}
#endif

statusType status;
RTC_DATA_ATTR igateTLMType igateTLM;
RTC_DATA_ATTR dataTLMType systemTLM;
txQueueType *txQueue;

RTC_DATA_ATTR double LastLat, LastLng;
RTC_DATA_ATTR time_t lastTimeStamp;
RTC_DATA_ATTR uint32_t COUNTER0_RAW;
RTC_DATA_ATTR uint32_t COUNTER1_RAW;

extern RTC_DATA_ATTR uint8_t digiCount;

String RF_VERSION;

Configuration config;

pkgListType *pkgList;

TelemetryType *Telemetry;

RTC_DATA_ATTR double VBat;
RTC_DATA_ATTR bool VBat_Flag = false;
RTC_DATA_ATTR double TempNTC;

RTC_NOINIT_ATTR uint16_t TLM_SEQ;
RTC_NOINIT_ATTR uint16_t IGATE_TLM_SEQ;
RTC_NOINIT_ATTR uint16_t DIGI_TLM_SEQ;

TaskHandle_t taskNetworkHandle;
TaskHandle_t taskAPRSHandle;
TaskHandle_t taskAPRSPollHandle;
TaskHandle_t taskSerialHandle;
TaskHandle_t taskGPSHandle;
TaskHandle_t taskSensorHandle;

unsigned long timerNetwork, timerNetwork_old;
unsigned long timerAPRS, timerAPRS_old;
unsigned long timerGPS, timerGPS_old;
unsigned long timerSerial, timerSerial_old;

// SoftwareSerial SerialRF;
EspSoftwareSerial::UART SerialRF;

bool firstGpsTime = true;
time_t startTime = 0;

#ifdef BLUETOOTH
#if !defined(CONFIG_IDF_TARGET_ESP32)
#include <NuSerial.hpp>
#include <NimBLEDevice.h>
#else
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif
#endif


// Set your Static IP address for wifi AP
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 254);
IPAddress subnet(255, 255, 255, 0);

IPAddress vpn_IP(192, 168, 44, 195);

int pkgTNC_count = 0;

xQueueHandle pcnt_evt_queue;              // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; // user's ISR service handle

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct
{
    int unit;                // the PCNT unit that originated an interrupt
    uint32_t status;         // information on the event type that caused the interrupt
    unsigned long timeStamp; // The time the event occured
} pcnt_evt_t;

unsigned long upTimeStamp = 0;
void convertSecondsToDHMS(char *dmhs, unsigned long totalSeconds)
{
    // Calculate days, hours, minutes, and seconds
    unsigned int days = totalSeconds / 86400;           // 86400 seconds in a day
    unsigned int hours = (totalSeconds % 86400) / 3600; // 3600 seconds in an hour
    unsigned int minutes = (totalSeconds % 3600) / 60;  // 60 seconds in a minute
    unsigned int seconds = totalSeconds % 60;

    sprintf(dmhs, "%dD[%d:%d:%d]", days, hours, minutes, seconds);
    // Print the result in D:H:M:S format
    // Serial.print("Time: ");
    // Serial.print(days);
    // Serial.print("D ");
    // Serial.print(hours);
    // Serial.print("H ");
    // Serial.print(minutes);
    // Serial.print("M ");
    // Serial.print(seconds);
    // Serial.println("S");
}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * and timestamp to the main program using a queue.
 */
// static void pcnt_intr_handler(void *arg)
// {
//     unsigned long currentMillis = millis(); // Time at instant ISR was called
//     uint32_t intr_status = 0xFFFF; //PCNT.int_st.val;
//     int i = 0;
//     pcnt_evt_t evt;
//     portBASE_TYPE HPTaskAwoken = pdFALSE;

//     for (i = 0; i < PCNT_UNIT_MAX; i++)
//     {
//         if (intr_status & (BIT(i)))
//         {
//             evt.unit = i;
//             /* Save the PCNT event type that caused an interrupt
//                to pass it to the main program */
//             evt.status = PCNT.status_unit[i].val;
//             evt.timeStamp = currentMillis;
//             PCNT.int_clr.val = BIT(i);
//             xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
//             if (HPTaskAwoken == pdTRUE)
//             {
//                 portYIELD_FROM_ISR();
//             }
//         }
//     }
// }

/* Initialize PCNT functions for one channel:
 *  - configure and initialize PCNT with pos-edge counting
 *  - set up the input filter
 *  - set up the counter events to watch
 * Variables:
 * UNIT - Pulse Counter #, INPUT_SIG - Signal Input Pin, INPUT_CTRL - Control Input Pin,
 * Channel - Unit input channel, H_LIM - High Limit, L_LIM - Low Limit,
 * THRESH1 - configurable limit 1, THRESH0 - configurable limit 2,
 */
// void pcnt_init_channel(pcnt_unit_t PCNT_UNIT, int PCNT_INPUT_SIG_IO, bool ACTIVE = false, int PCNT_INPUT_CTRL_IO = PCNT_PIN_NOT_USED, pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0, int PCNT_H_LIM_VAL = 65535, int PCNT_L_LIM_VAL = 0, int PCNT_THRESH1_VAL = 50, int PCNT_THRESH0_VAL = -50)
// {
//     /* Prepare configuration for the PCNT unit */
//     pcnt_config_t pcnt_config;
//     // Set PCNT input signal and control GPIOs
//     pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
//     pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;
//     pcnt_config.channel = PCNT_CHANNEL;
//     pcnt_config.unit = PCNT_UNIT;
//     // What to do on the positive / negative edge of pulse input?
//     if (ACTIVE)
//     {
//         pcnt_config.pos_mode = PCNT_COUNT_INC; // Count up on the positive edge
//         pcnt_config.neg_mode = PCNT_COUNT_DIS; // Keep the counter value on the negative edge
//     }
//     else
//     {
//         pcnt_config.neg_mode = PCNT_COUNT_INC; // Count up on the positive edge
//         pcnt_config.pos_mode = PCNT_COUNT_DIS; // Keep the counter value on the negative edge
//     }
//     // What to do when control input is low or high?
//     pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
//     pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
//     // Set the maximum and minimum limit values to watch
//     pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
//     pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;

//     /* Initialize PCNT unit */
//     pcnt_unit_config(&pcnt_config);
//     /* Configure and enable the input filter */
//     // pcnt_set_filter_value(PCNT_UNIT, 100);
//     // pcnt_filter_enable(PCNT_UNIT);

//     /* Set threshold 0 and 1 values and enable events to watch */
//     // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
//     // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_1);
//     // pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
//     // pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_0);
//     /* Enable events on zero, maximum and minimum limit values */
//     pcnt_event_enable(PCNT_UNIT, PCNT_EVT_ZERO);
//     pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);

//     /* Initialize PCNT's counter */
//     pcnt_counter_pause(PCNT_UNIT);
//     pcnt_counter_clear(PCNT_UNIT);
//     /* Register ISR handler and enable interrupts for PCNT unit */
//     // pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
//     // pcnt_intr_enable(PCNT_UNIT);

//     /* Everything is set up, now go to counting */
//     pcnt_counter_resume(PCNT_UNIT);
//     // pcnt_counter_resume(PCNT_UNIT_1);
// }

/* Count RPM Function - takes first timestamp and last timestamp,
number of pulses, and pulses per revolution */
// int countRPM(int firstTime, int lastTime, int pulseTotal, int pulsePerRev)
// {
//     int timeDelta = (lastTime - firstTime); // lastTime - firstTime
//     if (timeDelta <= 0)
//     { // This means we've gotten something wrong
//         return -1;
//     }
//     return ((60000 * (pulseTotal / pulsePerRev)) / timeDelta);
// }

RTC_DATA_ATTR bool gps_mode;
char EVENT_TX_POSITION = 0;
unsigned char SB_SPEED = 0, SB_SPEED_OLD = 0;
int16_t SB_HEADING = 0;
uint16_t tx_interval = 0, igate_tx_interval, digi_tx_interval; // How often we transmit, in seconds
unsigned int tx_counter, igate_tx_counter, digi_tx_counter;    // Incremented every second
int16_t last_heading, heading_change;                          // Keep track of corner pegging
uint16_t trk_interval = 15;

void smartbeacon(void)
{
    // Adaptive beacon rate
    if (SB_SPEED >= config.trk_hspeed)
    {
        tx_interval = (uint16_t)config.trk_maxinterval;
    }
    else
    {
        if (SB_SPEED <= config.trk_lspeed)
        {
            // Speed is below lower limit - use minimum rate
            tx_interval = (uint16_t)config.trk_slowinterval;
        }
        else
        {
            // In between, do SmartBeaconing calculations
            if (SB_SPEED <= 0)
                SB_SPEED = 1;
            tx_interval = (trk_interval * config.trk_hspeed) / SB_SPEED;
            if (tx_interval < 5)
                tx_interval = 5;
            if (tx_interval > config.trk_maxinterval)
                tx_interval = (uint16_t)config.trk_maxinterval;

            // Corner pegging - note that we use two-degree units
            if (last_heading > SB_HEADING)
                heading_change = last_heading - SB_HEADING;
            else
                heading_change = SB_HEADING - last_heading;
            if (heading_change > 90)
                heading_change = 180 - heading_change;
            if (heading_change > config.trk_minangle)
            {
                // Make sure last heading is updated during turns to avoid immedate xmit
                if ((tx_counter > config.trk_mininterval) && (SB_SPEED > 1))
                {
                    EVENT_TX_POSITION = 3;
                }
                else
                {
                    // last_heading = SB_HEADING;
                    tx_interval = config.trk_mininterval;
                }
            }
        }
    }
}

// (2^31 / 180) / 380926 semicircles per Base 91 unit
static unsigned long b91val[4] = {23601572L, 259358L, 2851L, 32L};
// Constants for converting lat/lon to semicircles
static long valtable[] = {1193046471L, 119304647L, 11930465L, 1988411L, 198841L, 19884L, 1988L, 199L, 20L, 2L};

// Takes degrees and fractional minutes and returns 2^31/180 degrees
// West and South values are negative
long semicircles(char *degstr, char hemi)
{
    char dgt = 0, *p;
    long ltemp = 0;
    if (degstr[4] == '.')
        dgt++; // Skip hundreds place if we know we're doing latitude
    p = degstr;
    for (; dgt < 10; p++)
    {
        if (*p == '.')
            continue;
        if (!isdigit(*p))
            break;
        ltemp += (*p & 0x0f) * valtable[dgt];
        dgt++;
    }
    if (hemi)
        return -ltemp;
    return ltemp;
}

// Converts semicircles to Base 91 units (must be properly biased first)
// Non-reentrant use of ltemp, but we only call this function in one place
void base91encode(long ltemp, char *s)
{
    // (2^31 / 180) / 380926 semicircles per Base 91 unit
    unsigned char c;

    for (c = 0; c < 4; c++)
    {
        s[c] = '!';
        while (ltemp >= b91val[c])
        {
            // reset_watchdog;
            ltemp -= b91val[c];
            s[c]++;
        }
    }
}

void telemetry_base91(char *cdata, char *output, size_t outputsize)
{
    int x, d1, d2;
    int i = 0;
    // Returns first token
    char *token = strtok(cdata, ",");
    output[i++] = '|';
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (token != NULL)
    {
        // printf(" % s\n", token);
        x = atoi(token);
        if (x > 8280)
            x = 8280;
        if (x < 0)
            x = 0;
        d1 = int(x / 91);
        d2 = x % 91;
        output[i++] = d1 + 33;
        output[i++] = d2 + 33;
        token = strtok(NULL, ",");
    }
    output[i++] = '|';
    outputsize = i;
}

/// Degrees to radians.
#define DEG2RAD(x) (x / 360 * 2 * PI)
/// Radians to degrees.
#define RAD2DEG(x) (x * (180 / PI))

double direction(double lon0, double lat0, double lon1, double lat1)
{
    double direction;

    /* Convert degrees into radians. */
    lon0 = DEG2RAD(lon0);
    lat0 = DEG2RAD(lat0);
    lon1 = DEG2RAD(lon1);
    lat1 = DEG2RAD(lat1);

    /* Direction from Aviation Formulary V1.42 by Ed Williams by way of
     * http://mathforum.org/library/drmath/view/55417.html */
    direction = atan2(sin(lon1 - lon0) * cos(lat1), cos(lat0) * sin(lat1) - sin(lat0) * cos(lat1) * cos(lon1 - lon0));
    if (direction < 0)
    {
        /* Make direction positive. */
        direction += 2 * PI;
    }

    return RAD2DEG(direction);
}

double distance(double lon0, double lat0, double lon1, double lat1)
{
    double dlon;
    double dlat;
    double a, c;
    /* Convert degrees into radians. */
    lon0 = DEG2RAD(lon0);
    lat0 = DEG2RAD(lat0);
    lon1 = DEG2RAD(lon1);
    lat1 = DEG2RAD(lat1);

    /* Use the haversine formula for distance calculation
     * http://mathforum.org/library/drmath/view/51879.html */
    dlon = lon1 - lon0;
    dlat = lat1 - lat0;
    a = pow(sin(dlat / 2), 2) + cos(lat0) * cos(lat1) * pow(sin(dlon / 2), 2);
    c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return c * 6366.71; /* in kilometers */
}

void deg2lat(char *dmm, double deg)
{
    char sign;
    if (deg > 0.0F)
    {
        sign = 'N';
    }
    else
    {
        sign = 'S';
        deg *= -1;
    }

    uint id = (uint)floor(deg);
    uint im = (uint)((deg - (double)id) * 60);
    uint imm = (uint)round((((deg - (double)id) * 60) - (double)im) * 100);
    sprintf(dmm, "%02d%02d.%02d%c", id, im, imm, sign);
}

void deg2lon(char *dmm, double deg)
{
    char sign;
    if (deg > 0.0F)
    {
        sign = 'E';
    }
    else
    {
        sign = 'W';
        deg *= -1;
    }
    uint id = (uint)floor(deg);
    uint im = (uint)((deg - (double)id) * 60);
    uint imm = (uint)round((((deg - (double)id) * 60) - (double)im) * 100);
    sprintf(dmm, "%03d%02d.%02d%c", id, im, imm, sign);
}

String deg2lat(double deg)
{
    char sign;
    if (deg > 0.0F)
    {
        sign = 'N';
    }
    else
    {
        sign = 'S';
        deg *= -1;
    }

    uint id = (uint)floor(deg);
    uint im = (uint)((deg - (double)id) * 60);
    uint imm = (uint)round((((deg - (double)id) * 60) - (double)im) * 100);
    char dmm[10];
    sprintf(dmm, "%02d%02d.%02d%c", id, im, imm, sign);
    return String(dmm);
}

String deg2lon(double deg)
{
    char sign;
    if (deg > 0.0F)
    {
        sign = 'E';
    }
    else
    {
        sign = 'W';
        deg *= -1;
    }
    uint id = (uint)floor(deg);
    uint im = (uint)((deg - (double)id) * 60);
    uint imm = (uint)round((((deg - (double)id) * 60) - (double)im) * 100);
    char dmm[10];
    sprintf(dmm, "%03d%02d.%02d%c", id, im, imm, sign);
    return String(dmm);
}

time_t setGpsTime()
{
    time_t time;
    tmElements_t timeinfo;
    if (gps.time.isValid())
    {
        timeinfo.Year = (gps.date.year()) - 1970;
        timeinfo.Month = gps.date.month();
        timeinfo.Day = gps.date.day();
        timeinfo.Hour = gps.time.hour();
        timeinfo.Minute = gps.time.minute();
        timeinfo.Second = gps.time.second();
        time_t timeStamp = makeTime(timeinfo);
        time = timeStamp + config.timeZone * SECS_PER_HOUR;
        setTime(time);
        // setTime(timeinfo.Hour,timeinfo.Minute,timeinfo.Second,timeinfo.Day, timeinfo.Month, timeinfo.Year);
        return time;
    }
    return 0;
}

time_t getGpsTime()
{
    time_t time;
    tmElements_t timeinfo;
    if (gps.time.isValid())
    {
        timeinfo.Year = (gps.date.year()) - 1970;
        timeinfo.Month = gps.date.month();
        timeinfo.Day = gps.date.day();
        timeinfo.Hour = gps.time.hour();
        timeinfo.Minute = gps.time.minute();
        timeinfo.Second = gps.time.second();
        time_t timeStamp = makeTime(timeinfo);
        time = timeStamp + config.timeZone * SECS_PER_HOUR;
        return time;
    }
    return 0;
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length();

    for (int i = 0; i <= maxIndex && found <= index; i++)
    {
        if (data.charAt(i) == separator || i == maxIndex)
        {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
} // END

boolean isValidNumber(String str)
{
    for (int i = 0; i < str.length(); i++)
    {
        if (isDigit(str.charAt(i)))
            return true;
    }
    return false;
}

uint8_t checkSum(uint8_t *ptr, size_t count)
{
    uint8_t lrc, tmp;
    uint16_t i;
    lrc = 0;
    for (i = 0; i < count; i++)
    {
        tmp = *ptr++;
        lrc = lrc ^ tmp;
    }
    return lrc;
}

// void saveEEPROM()
// {
//     uint8_t chkSum = 0;
//     byte *ptr;
//     ptr = (byte *)&config;
//     EEPROM.writeBytes(1, ptr, sizeof(Configuration));
//     chkSum = checkSum(ptr, sizeof(Configuration));
//     EEPROM.write(0, chkSum);
//     EEPROM.commit();
// #ifdef DEBUG
//     Serial.print("Save EEPROM ChkSUM=");
//     Serial.println(chkSum, HEX);
// #endif
// }

void logTracker(double lat, double lon, double speed, double course)
{
    char data[200];
    char datetime[30];
    char dfName[30];
    bool header = false;
    double dist;
    time_t nowTime;

    if (gps.time.isValid())
    {
        time_t timeGps = getGpsTime(); // Local gps time
        time(&nowTime);
        int tdiff = abs(timeGps - nowTime);
        if (timeGps > 1700000000 && tdiff > 2) // && timeGps < 2347462800)
        {
            nowTime = timeGps;
            setTime(nowTime);
            time_t rtc = nowTime - (time_t)(config.timeZone * (float)SECS_PER_HOUR);
            timeval tv = {rtc, 0};
            timezone tz = {(int)(config.timeZone * (float)SECS_PER_HOUR), 0};
            settimeofday(&tv, &tz);
        }
    }
    else
    {
        time(&nowTime);
    }

    String col;
    struct tm tmstruct;
    getLocalTime(&tmstruct, 100);
    sprintf(dfName, "/trk_%02d%d.csv", (tmstruct.tm_mon) + 1, (tmstruct.tm_year) + 1900);

    if (lastTimeStamp == 0)
        lastTimeStamp = nowTime;
    time_t tdiff = nowTime - lastTimeStamp;
    double nowLat = gps.location.lat();
    double nowLng = gps.location.lng();
    double spd = gps.speed.kmph();

    // dist = distance(LastLng, LastLat, nowLng, nowLat);

    if (spd < 5)
    {

        dist = gps.distanceBetween(LastLng, LastLat, nowLng, nowLat);
        course = direction(LastLng, LastLat, nowLng, nowLat);
        if (dist > 50.0F)
            dist = 0;
        if (tdiff > 10 && (nowTime > lastTimeStamp))
            speed = dist / ((double)tdiff / 3600);
        else
            speed = 0.0F;

        if (speed > 5)
        {
            speed = gps.speed.kmph();
            course = gps.course.deg();
        }
    }
    LastLat = nowLat;
    LastLng = nowLng;
    lastTimeStamp = nowTime;
    // if (!waterTempFlag)
    // 		{
    // ds18b20.setWaitForConversion(false); // makes it async
    // ds18b20.requestTemperatures();
    // ds18b20.setWaitForConversion(true);
    // waterTemperature = ds18b20.getTempCByIndex(0);
    // if (waterTemperature > 0)
    // 	waterTempFlag = true;
    //}

    if (!LITTLEFS.exists(dfName))
    {
        header = true;
    }

    File f = LITTLEFS.open(dfName, "a");
#ifdef DEBUG
    log_d("====== Writing to data file =========");
    log_d("Open File %s", dfName);
#endif
    if (!f)
    {
#ifdef DEBUG
        log_d("Data file open failed");
#endif
    }
    else
    {
        if (header)
        {
            // col = "\"dd/mm/yyyy h:m\",\"Weight Origin(Kg)\",\"Weight(Kg)\",\"Water Drain(L)\",\"Water Height(mm)\",\"Temperature(C)\",\"Raw\"";
            col = "dd/mm/yyyy h:m:s";
            col += ",Latitude";
            col += ",Longitude";
            col += ",Speed(kph)";
            col += ",Course(°)";
            for (int i = 0; i < 5; i++)
            {
                if (config.trk_tlm_sensor[i] > 0)
                {
                    col += "," + String(config.trk_tlm_PARM[i]) + "(" + String(config.trk_tlm_UNIT[i]) + ")";
                }
            }
            f.println(col);
        }
        sprintf(datetime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
        sprintf(data, "%s,%0.5f,%0.5f,%0.2f,%d", datetime, lat, lon, speed, (int)course);
        for (int s = 0; s < 5; s++)
        {
            if (config.trk_tlm_sensor[s] == 0)
            {
                continue;
                // strcat(tlm_data, "0");
            }
            else
            {
                strcat(data, ",");
                int sen_idx = config.trk_tlm_sensor[s] - 1;
                double val = 0;
                if (sen[sen_idx].visable)
                {
                    if (config.trk_tlm_avg[s] && (sen[sen_idx].timeAvg > 0))
                        val = sen[sen_idx].average;
                    else
                        val = sen[sen_idx].sample;
                }
                strcat(data, String(val, 2).c_str());
            }
        }
        f.println(data);
        f.close();
#ifdef DEBUG
        log_d("Data file updated");
#endif
    }
}

void logIGate(double lat, double lon, double speed, double course)
{
    char data[200];
    char datetime[30];
    char dfName[30];
    bool header = false;
    double dist;
    time_t nowTime;

    if (gps.time.isValid())
    {
        time_t timeGps = getGpsTime(); // Local gps time
        time(&nowTime);
        int tdiff = abs(timeGps - nowTime);
        if (timeGps > 1700000000 && tdiff > 2) // && timeGps < 2347462800)
        {
            nowTime = timeGps;
            setTime(nowTime);
            time_t rtc = nowTime - (time_t)(config.timeZone * (float)SECS_PER_HOUR);
            timeval tv = {rtc, 0};
            timezone tz = {(int)(config.timeZone * (float)SECS_PER_HOUR), 0};
            settimeofday(&tv, &tz);
        }
    }
    else
    {
        time(&nowTime);
    }

    String col;
    struct tm tmstruct;
    getLocalTime(&tmstruct, 100);
    sprintf(dfName, "/igate_%02d%d.csv", (tmstruct.tm_mon) + 1, (tmstruct.tm_year) + 1900);

    if (lastTimeStamp == 0)
        lastTimeStamp = nowTime;
    time_t tdiff = nowTime - lastTimeStamp;
    double nowLat = gps.location.lat();
    double nowLng = gps.location.lng();
    double spd = gps.speed.kmph();

    // dist = distance(LastLng, LastLat, nowLng, nowLat);

    if (spd < 5)
    {

        dist = gps.distanceBetween(LastLng, LastLat, nowLng, nowLat);
        course = direction(LastLng, LastLat, nowLng, nowLat);
        if (dist > 50.0F)
            dist = 0;
        if (tdiff > 10 && (nowTime > lastTimeStamp))
            speed = dist / ((double)tdiff / 3600);
        else
            speed = 0.0F;

        if (speed > 5)
        {
            speed = gps.speed.kmph();
            course = gps.course.deg();
        }
    }
    LastLat = nowLat;
    LastLng = nowLng;
    lastTimeStamp = nowTime;

    if (!LITTLEFS.exists(dfName))
    {
        header = true;
    }

    File f = LITTLEFS.open(dfName, "a");
#ifdef DEBUG
    log_d("====== Writing to data file =========");
    log_d("Open File %s", dfName);
#endif
    if (!f)
    {
#ifdef DEBUG
        log_d("Data file open failed");
#endif
    }
    else
    {
        if (header)
        {
            // col = "\"dd/mm/yyyy h:m\",\"Weight Origin(Kg)\",\"Weight(Kg)\",\"Water Drain(L)\",\"Water Height(mm)\",\"Temperature(C)\",\"Raw\"";
            col = "dd/mm/yyyy h:m:s";
            col += ",Latitude";
            col += ",Longitude";
            col += ",Speed(kph)";
            col += ",Course(°)";
            for (int i = 0; i < 5; i++)
            {
                if (config.igate_tlm_sensor[i] > 0)
                {
                    col += "," + String(config.igate_tlm_PARM[i]) + "(" + String(config.igate_tlm_UNIT[i]) + ")";
                }
            }
            f.println(col);
        }
        sprintf(datetime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
        sprintf(data, "%s,%0.5f,%0.5f,%0.2f,%d", datetime, lat, lon, speed, (int)course);
        for (int s = 0; s < 5; s++)
        {
            if (config.igate_tlm_sensor[s] == 0)
            {
                continue;
                // strcat(tlm_data, "0");
            }
            else
            {
                strcat(data, ",");
                int sen_idx = config.igate_tlm_sensor[s] - 1;
                double val = 0;
                if (sen[sen_idx].visable)
                {
                    if (config.igate_tlm_avg[s] && (sen[sen_idx].timeAvg > 0))
                        val = sen[sen_idx].average;
                    else
                        val = sen[sen_idx].sample;
                }
                strcat(data, String(val, 2).c_str());
            }
        }
        f.println(data);
        f.close();
#ifdef DEBUG
        log_d("Data file updated");
#endif
    }
}

void logDigi(double lat, double lon, double speed, double course)
{
    char data[200];
    char datetime[30];
    char dfName[30];
    bool header = false;
    double dist;
    time_t nowTime;

    if (gps.time.isValid())
    {
        time_t timeGps = getGpsTime(); // Local gps time
        time(&nowTime);
        int tdiff = abs(timeGps - nowTime);
        if (timeGps > 1700000000 && tdiff > 2) // && timeGps < 2347462800)
        {
            nowTime = timeGps;
            setTime(nowTime);
            time_t rtc = nowTime - (time_t)(config.timeZone * (float)SECS_PER_HOUR);
            timeval tv = {rtc, 0};
            timezone tz = {(int)(config.timeZone * (float)SECS_PER_HOUR), 0};
            settimeofday(&tv, &tz);
        }
    }
    else
    {
        time(&nowTime);
    }

    String col;
    struct tm tmstruct;
    getLocalTime(&tmstruct, 100);
    sprintf(dfName, "/digi_%02d%d.csv", (tmstruct.tm_mon) + 1, (tmstruct.tm_year) + 1900);

    if (lastTimeStamp == 0)
        lastTimeStamp = nowTime;
    time_t tdiff = nowTime - lastTimeStamp;
    double nowLat = gps.location.lat();
    double nowLng = gps.location.lng();
    double spd = gps.speed.kmph();

    // dist = distance(LastLng, LastLat, nowLng, nowLat);

    if (spd < 5)
    {

        dist = gps.distanceBetween(LastLng, LastLat, nowLng, nowLat);
        course = direction(LastLng, LastLat, nowLng, nowLat);
        if (dist > 50.0F)
            dist = 0;
        if (tdiff > 10 && (nowTime > lastTimeStamp))
            speed = dist / ((double)tdiff / 3600);
        else
            speed = 0.0F;

        if (speed > 5)
        {
            speed = gps.speed.kmph();
            course = gps.course.deg();
        }
    }
    LastLat = nowLat;
    LastLng = nowLng;
    lastTimeStamp = nowTime;

    if (!LITTLEFS.exists(dfName))
    {
        header = true;
    }

    File f = LITTLEFS.open(dfName, "a");
#ifdef DEBUG
    log_d("====== Writing to data file =========");
    log_d("Open File %s", dfName);
#endif
    if (!f)
    {
#ifdef DEBUG
        log_d("Data file open failed");
#endif
    }
    else
    {
        if (header)
        {
            // col = "\"dd/mm/yyyy h:m\",\"Weight Origin(Kg)\",\"Weight(Kg)\",\"Water Drain(L)\",\"Water Height(mm)\",\"Temperature(C)\",\"Raw\"";
            col = "dd/mm/yyyy h:m:s";
            col += ",Latitude";
            col += ",Longitude";
            col += ",Speed(kph)";
            col += ",Course(°)";
            for (int i = 0; i < 5; i++)
            {
                if (config.digi_tlm_sensor[i] > 0)
                {
                    col += "," + String(config.digi_tlm_PARM[i]) + "(" + String(config.digi_tlm_UNIT[i]) + ")";
                }
            }
            f.println(col);
        }
        sprintf(datetime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
        sprintf(data, "%s,%0.5f,%0.5f,%0.2f,%d", datetime, lat, lon, speed, (int)course);
        for (int s = 0; s < 5; s++)
        {
            if (config.digi_tlm_sensor[s] == 0)
            {
                continue;
                // strcat(tlm_data, "0");
            }
            else
            {
                strcat(data, ",");
                int sen_idx = config.digi_tlm_sensor[s] - 1;
                double val = 0;
                if (sen[sen_idx].visable)
                {
                    if (config.digi_tlm_avg[s] && (sen[sen_idx].timeAvg > 0))
                        val = sen[sen_idx].average;
                    else
                        val = sen[sen_idx].sample;
                }
                strcat(data, String(val, 2).c_str());
            }
        }
        f.println(data);
        f.close();
#ifdef DEBUG
        log_d("Data file updated");
#endif
    }
}

void logWeather(double lat, double lon, double speed, double course)
{
    char data[500];
    char datetime[30];
    char dfName[30];
    bool header = false;
    double dist;
    time_t nowTime;

    if (gps.time.isValid())
    {
        time_t timeGps = getGpsTime(); // Local gps time
        time(&nowTime);
        int tdiff = abs(timeGps - nowTime);
        if (timeGps > 1700000000 && tdiff > 2) // && timeGps < 2347462800)
        {
            nowTime = timeGps;
            setTime(nowTime);
            time_t rtc = nowTime - (time_t)(config.timeZone * (float)SECS_PER_HOUR);
            timeval tv = {rtc, 0};
            timezone tz = {(int)(config.timeZone * (float)SECS_PER_HOUR), 0};
            settimeofday(&tv, &tz);
        }
    }
    else
    {
        time(&nowTime);
    }

    String col;
    struct tm tmstruct;
    getLocalTime(&tmstruct, 100);
    sprintf(dfName, "/wx_%02d%d.csv", (tmstruct.tm_mon) + 1, (tmstruct.tm_year) + 1900);

    if (lastTimeStamp == 0)
        lastTimeStamp = nowTime;
    time_t tdiff = nowTime - lastTimeStamp;
    double nowLat = gps.location.lat();
    double nowLng = gps.location.lng();
    double spd = gps.speed.kmph();

    // dist = distance(LastLng, LastLat, nowLng, nowLat);

    if (spd < 5)
    {

        dist = gps.distanceBetween(LastLng, LastLat, nowLng, nowLat);
        course = direction(LastLng, LastLat, nowLng, nowLat);
        if (dist > 50.0F)
            dist = 0;
        if (tdiff > 10 && (nowTime > lastTimeStamp))
            speed = dist / ((double)tdiff / 3600);
        else
            speed = 0.0F;

        if (speed > 5)
        {
            speed = gps.speed.kmph();
            course = gps.course.deg();
        }
    }
    LastLat = nowLat;
    LastLng = nowLng;
    lastTimeStamp = nowTime;

    if (!LITTLEFS.exists(dfName))
    {
        header = true;
    }

    File f = LITTLEFS.open(dfName, "a");
#ifdef DEBUG
    log_d("====== Writing to data file =========");
    log_d("Open File %s", dfName);
#endif
    if (!f)
    {
#ifdef DEBUG
        log_d("Data file open failed");
#endif
    }
    else
    {
        if (header)
        {
            // col = "\"dd/mm/yyyy h:m\",\"Weight Origin(Kg)\",\"Weight(Kg)\",\"Water Drain(L)\",\"Water Height(mm)\",\"Temperature(C)\",\"Raw\"";
            col = "dd/mm/yyyy h:m:s";
            col += ",Latitude";
            col += ",Longitude";
            col += ",Speed(kph)";
            col += ",Course(°)";
            for (int s = 0; s < WX_SENSOR_NUM; s++)
            {
                int senIdx = config.wx_sensor_ch[s];
                if ((config.wx_sensor_enable[s] == 0) || (senIdx == 0))
                {
                    continue;
                }
                else
                {
                    senIdx -= 1;
                    col += "," + String(String(WX_SENSOR[s])) + "(" + String(config.sensor[senIdx].unit) + ")";
                }
            }
            f.println(col);
        }
        sprintf(datetime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
        sprintf(data, "%s,%0.5f,%0.5f,%0.2f,%d", datetime, lat, lon, speed, (int)course);
        for (int s = 0; s < WX_SENSOR_NUM; s++)
        {
            int senIdx = config.wx_sensor_ch[s];
            if ((config.wx_sensor_enable[s] == 0) || (senIdx == 0))
            {
                continue;
            }
            else
            {
                strcat(data, ",");
                senIdx -= 1;
                double val = 0;
                if (sen[senIdx].visable)
                {
                    if ((config.wx_sensor_avg[s] && (config.sensor[senIdx].averagerate <= config.sensor[senIdx].samplerate)) || (sen[senIdx].timeAvg == 0))
                        val = sen[senIdx].sample;
                    else
                        val = sen[senIdx].average;
                }
                strcat(data, String(val, 2).c_str());
            }
        }
        f.println(data);
        f.close();
#ifdef DEBUG
        log_d("Data file updated");
#endif
    }
}

void defaultConfig()
{
    log_d("Default configure mode!");
    config.synctime = true;
    config.timeZone = 7;
    config.tx_timeslot = 2000; // ms

    config.wifi_mode = WIFI_AP_STA_FIX;
    config.wifi_power = 44; // WIFI_POWER_11dBm
    config.wifi_ap_ch = 6;
    config.wifi_sta[0].enable = true;
    sprintf(config.wifi_sta[0].wifi_ssid, "APRSTH");
    sprintf(config.wifi_sta[0].wifi_pass, "aprsthnetwork");
    for (int i = 1; i < 5; i++)
    {
        config.wifi_sta[i].enable = false;
        config.wifi_sta[i].wifi_ssid[0] = 0;
        config.wifi_sta[i].wifi_pass[0] = 0;
    }
    sprintf(config.wifi_ap_ssid, "ESP32APRS_Audio");
    sprintf(config.wifi_ap_pass, "aprsthnetwork");

        // Blutooth
    config.bt_slave = false;
    config.bt_master = false;
    config.bt_mode = 1; // 0-None,1-TNC2RAW,2-KISS
    config.bt_power = 3;
    sprintf(config.bt_name, "ESP32APRS_Audio");
    config.bt_pin = 0;
#if !defined(CONFIG_IDF_TARGET_ESP32)
    // Bluetooth BLE
    sprintf(config.bt_uuid, "00000001-ba2a-46c9-ae49-01b0961f68bb");
    sprintf(config.bt_uuid_rx, "00000002-ba2a-46c9-ae49-01b0961f68bb");
    sprintf(config.bt_uuid_tx, "00000003-ba2a-46c9-ae49-01b0961f68bb");
#endif

    //--RF Module
    config.rf_en = false;
    config.rf_type = RF_SA868_VHF;
    config.freq_rx = 144.3900;
    config.freq_tx = 144.3900;
    config.offset_rx = 0;
    config.offset_tx = 0;
    config.tone_rx = 0;
    config.tone_tx = 0;
    config.band = 0;
    config.sql_level = 1;
    config.rf_power = LOW;
    config.volume = 6;
    config.mic = 8;
    config.modem_type = 1;

#ifdef ESP32C3_MINI
    //config.wifi_power = 74;
    config.rf_tx_gpio = -1;
    config.rf_rx_gpio = -1;
    config.rf_sql_gpio = -1;
    config.rf_pd_gpio = -1;
    config.rf_pwr_gpio = -1;
    config.rf_ptt_gpio = 3;
    config.rf_sql_active = 0;
    config.rf_pd_active = 1;
    config.rf_pwr_active = 1;
    config.rf_ptt_active = 0;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    config.rf_tx_gpio = 17;
    config.rf_rx_gpio = 18;
    config.rf_sql_gpio = -1;
    config.rf_pd_gpio = 4;
    config.rf_pwr_gpio = -1;
    config.rf_ptt_gpio = 5;
    config.rf_sql_active = 0;
    config.rf_pd_active = 1;
    config.rf_pwr_active = 1;
    config.rf_ptt_active = 1;    
#else
    config.rf_tx_gpio = 13;
    config.rf_rx_gpio = 14;
    config.rf_sql_gpio = -1;
    config.rf_pd_gpio = 27;
    config.rf_pwr_gpio = -1;
    config.rf_ptt_gpio = 32;
    config.rf_sql_active = 0;
    config.rf_pd_active = 1;
    config.rf_pwr_active = 1;
    config.rf_ptt_active = 1;
#endif
    config.adc_atten = 0;
    config.adc_dc_offset = 600;
    config.rf_baudrate = 9600;

    config.audio_hpf = false;
    config.audio_lpf = false;
    config.preamble = 3;
    sprintf(config.ntp_host, "ntp.dprns.com");

    // IGATE
    config.igate_bcn = false;
    config.igate_en = false;
    config.rf2inet = true;
    config.inet2rf = false;
    config.igate_loc2rf = false;
    config.igate_loc2inet = true;
    config.rf2inetFilter = 0xFFF; // All
    config.inet2rfFilter = config.digiFilter = FILTER_OBJECT | FILTER_ITEM | FILTER_MESSAGE | FILTER_MICE | FILTER_POSITION | FILTER_WX;
    //--APRS-IS
    config.aprs_ssid = 1;
    config.aprs_port = 14580;
    sprintf(config.aprs_mycall, "NOCALL");
    sprintf(config.aprs_host, "aprs.dprns.com");
    memset(config.aprs_passcode, 0, sizeof(config.aprs_passcode));
    sprintf(config.aprs_moniCall, "%s-%d", config.aprs_mycall, config.aprs_ssid);
    sprintf(config.aprs_filter, "m/0");
    //--Position
    config.igate_gps = false;
    config.igate_lat = 13.7555;
    config.igate_lon = 100.4930;
    config.igate_alt = 0;
    config.igate_interval = 600;
    sprintf(config.igate_symbol, "A&");
    memset(config.igate_object, 0, sizeof(config.igate_object));
    memset(config.igate_phg, 0, sizeof(config.igate_phg));
    config.igate_path = 8;
    sprintf(config.igate_comment, "");
    sprintf(config.igate_status, "https://github.com/nakhonthai/ESP32APRS_Audio");
    config.igate_sts_interval = 1800;

    // DIGI REPEATER
    config.digi_en = false;
    config.digi_auto = false;
    config.digi_loc2rf = true;
    config.digi_loc2inet = false;
    config.digi_ssid = 3;
    config.digi_timestamp = false;
    sprintf(config.digi_mycall, "NOCALL");
    config.digi_path = 8;
    //--Position
    config.digi_gps = false;
    config.digi_lat = 13.7555;
    config.digi_lon = 100.4930;
    config.digi_alt = 0;
    config.digi_interval = 600;
    config.igate_timestamp = false;
    config.digi_delay = 0;
    config.digiFilter = FILTER_OBJECT | FILTER_ITEM | FILTER_MESSAGE | FILTER_MICE | FILTER_POSITION | FILTER_WX;

    sprintf(config.digi_symbol, "A#");
    memset(config.digi_phg, 0, sizeof(config.digi_phg));
    sprintf(config.digi_comment, "");
    sprintf(config.digi_status, "https://github.com/nakhonthai/ESP32APRS_Audio");
    config.digi_sts_interval = 1800;

    // Tracker
    config.trk_en = false;
    config.trk_loc2rf = true;
    config.trk_loc2inet = false;
    config.trk_rssi = false;
    config.trk_sat = false;
    config.trk_dx = false;
    config.trk_ssid = 7;
    config.trk_timestamp = false;
    sprintf(config.trk_mycall, "NOCALL");
    config.trk_path = 2;

    //--Position
    config.trk_gps = false;
    config.trk_lat = 13.7555;
    config.trk_lon = 100.4930;
    config.trk_alt = 0;
    config.trk_interval = 600;
    // Smart beacon
    config.trk_smartbeacon = true;
    config.trk_compress = true;
    config.trk_altitude = true;
    config.trk_log = true;
    config.trk_hspeed = 120;
    config.trk_lspeed = 5;
    config.trk_maxinterval = 30;
    config.trk_mininterval = 5;
    config.trk_minangle = 25;
    config.trk_slowinterval = 600;

    sprintf(config.trk_symbol, "/[");
    sprintf(config.trk_symmove, "/>");
    sprintf(config.trk_symstop, "\\>");
    // sprintf(config.trk_btext, "");
    sprintf(config.trk_mycall, "NOCALL");
    sprintf(config.trk_comment, "");
    memset(config.trk_item, 0, sizeof(config.trk_item));
    sprintf(config.trk_status, "https://github.com/nakhonthai/ESP32APRS_Audio");
    config.trk_sts_interval = 1800;

    // WX
    config.wx_en = false;
    config.wx_2rf = true;
    config.wx_2inet = true;
    // config.wx_channel = 0;
    config.wx_ssid = 13;
    sprintf(config.wx_mycall, "NOCALL");
    config.wx_path = 8;
    sprintf(config.wx_comment, "WX MODE");
    memset(config.wx_object, 0, sizeof(config.wx_object));
    config.wx_gps = false;
    config.wx_lat = 13.7555;
    config.wx_lon = 100.4930;
    config.wx_alt = 0;
    config.wx_interval = 600;
    config.wx_flage = 0;

    // Telemetry_0
    config.tlm0_en = false;
    config.tlm0_2rf = true;
    config.tlm0_2inet = true;
    config.tlm0_ssid = 0;
    sprintf(config.tlm0_mycall, "NOCALL");
    config.tlm0_path = 0;
    config.tlm0_data_interval = 600;
    config.tlm0_info_interval = 3600;
    sprintf(config.tlm0_PARM[0], "RF->INET");
    sprintf(config.tlm0_PARM[1], "INET->RF");
    sprintf(config.tlm0_PARM[2], "Repeater");
    sprintf(config.tlm0_PARM[3], "AllCount");
    sprintf(config.tlm0_PARM[4], "AllDrop");
    sprintf(config.tlm0_PARM[5], "IGATE");
    sprintf(config.tlm0_PARM[6], "DIGI");
    sprintf(config.tlm0_PARM[7], "WX");
    sprintf(config.tlm0_PARM[8], "SAT");
    sprintf(config.tlm0_PARM[9], "INET");
    sprintf(config.tlm0_PARM[10], "VPN");
    sprintf(config.tlm0_PARM[11], "4G");
    sprintf(config.tlm0_PARM[12], "FX.25");

    for (int i = 0; i < 5; i++)
    {
        sprintf(config.tlm0_UNIT[i], "Pkts");
        config.tlm0_EQNS[i][0] = 0; // a av2 + bv + c
        config.tlm0_EQNS[i][1] = 1; // b
        config.tlm0_EQNS[i][2] = 0; // c
    }
    sprintf(config.tlm0_UNIT[5], "En");
    sprintf(config.tlm0_UNIT[6], "En");
    sprintf(config.tlm0_UNIT[7], "En");
    sprintf(config.tlm0_UNIT[8], "En");
    sprintf(config.tlm0_UNIT[9], "ON");
    sprintf(config.tlm0_UNIT[10], "ON");
    sprintf(config.tlm0_UNIT[11], "ON");
    sprintf(config.tlm0_UNIT[12], "ON");
    config.tlm0_BITS_Active = 0xFF;
    config.tml0_data_channel[0] = 2;
    config.tml0_data_channel[1] = 3;
    config.tml0_data_channel[2] = 4;
    config.tml0_data_channel[3] = 1;
    config.tml0_data_channel[4] = 5;
    config.tml0_data_channel[5] = 1;
    config.tml0_data_channel[6] = 2;
    config.tml0_data_channel[7] = 3;
    config.tml0_data_channel[8] = 4;
    config.tml0_data_channel[9] = 5;
    config.tml0_data_channel[10] = 6;
    config.tml0_data_channel[11] = 7;
    config.tml0_data_channel[12] = 8;
    sprintf(config.tlm0_comment, "SYSTEM STATUS");

// memset(config.wx_type, 0, sizeof(config.wx_type));

// OLED DISPLAY
#ifdef OLED
    config.oled_enable = true;
#else
    config.oled_enable = false;
#endif
    config.oled_timeout = 60;
    config.dim = 0;
    config.contrast = 0;
    config.startup = 0;

    // Display
    config.disp_brightness = 250;
    config.disp_flip = false;
    config.dispDelay = 3; // Popup display 3 sec
    config.dispRF = true;
    config.dispINET = false;
    config.filterDistant = 0;
    config.dispFilter = FILTER_OBJECT | FILTER_ITEM | FILTER_MESSAGE | FILTER_MICE | FILTER_POSITION | FILTER_WX | FILTER_STATUS | FILTER_BUOY | FILTER_QUERY;
    config.h_up = true;
    config.tx_display = true;
    config.rx_display = true;

    // afsk,TNC
    sprintf(config.ntp_host, "ntp.dprns.com");

    sprintf(config.path[0], "WIDE1-1");
    sprintf(config.path[1], "WIDE1-1,WIDE2-1");
    sprintf(config.path[2], "TRACK3-3");
    sprintf(config.path[3], "RS0ISS");

    // VPN Wireguard
    config.vpn = false;
    config.wg_port = 51820;
    sprintf(config.wg_peer_address, "vpn.dprns.com");
    sprintf(config.wg_local_address, "192.168.1.2");
    sprintf(config.wg_netmask_address, "255.255.255.0");
    sprintf(config.wg_gw_address, "192.168.1.1");
    sprintf(config.wg_public_key, "");
    sprintf(config.wg_private_key, "");

    sprintf(config.http_username, "admin");
    sprintf(config.http_password, "admin");

    // config.gnss_baudrate = 9600;
    config.gnss_enable = false;
    // config.gnss_tx_gpio = 19;
    // config.gnss_rx_gpio = 18;
    // config.gnss_pps_gpio = -1;
    config.gnss_tcp_port = 8080;
    sprintf(config.gnss_tcp_host, "192.168.0.1");
    memset(config.gnss_at_command, 0, sizeof(config.gnss_at_command));

    config.sensor[0].enable = false;
    config.sensor[0].port = 0;
    config.sensor[0].address = 2;
    config.sensor[0].samplerate = 10;
    config.sensor[0].averagerate = 600;
    config.sensor[0].eqns[0] = 0; // a
    config.sensor[0].eqns[1] = 1; // b
    config.sensor[0].eqns[2] = 0; // c
    config.sensor[0].type = SENSOR_TEMPERATURE;
    sprintf(config.sensor[0].parm, "Temperature");
    sprintf(config.sensor[0].unit, "°C");

    config.sensor[1].enable = false;
    config.sensor[1].port = 0;
    config.sensor[1].address = 2;
    config.sensor[1].samplerate = 10;
    config.sensor[1].averagerate = 600;
    config.sensor[1].eqns[0] = 0; // a
    config.sensor[1].eqns[1] = 1; // b
    config.sensor[1].eqns[2] = 0; // c
    config.sensor[1].type = SENSOR_HUMIDITY;
    sprintf(config.sensor[1].parm, "Humidity");
    sprintf(config.sensor[1].unit, "%%RH");

    config.sensor[2].enable = false;
    config.sensor[2].port = 0;
    config.sensor[2].address = 2;
    config.sensor[2].samplerate = 10;
    config.sensor[2].averagerate = 600;
    config.sensor[2].eqns[0] = 0; // a
    config.sensor[2].eqns[1] = 1; // b
    config.sensor[2].eqns[2] = 0; // c
    config.sensor[2].type = SENSOR_PM25;
    sprintf(config.sensor[2].parm, "PM2.5");
    sprintf(config.sensor[2].unit, "μg/m³");

    config.sensor[3].enable = false;
    config.sensor[3].port = 0;
    config.sensor[3].address = 2;
    config.sensor[3].samplerate = 10;
    config.sensor[3].averagerate = 600;
    config.sensor[3].eqns[0] = 0; // a
    config.sensor[3].eqns[1] = 1; // b
    config.sensor[3].eqns[2] = 0; // c
    config.sensor[3].type = SENSOR_PM100;
    sprintf(config.sensor[3].parm, "PM10.0");
    sprintf(config.sensor[3].unit, "μg/m³");

    config.sensor[4].enable = false;
    config.sensor[4].port = 0;
    config.sensor[4].address = 2;
    config.sensor[4].samplerate = 10;
    config.sensor[4].averagerate = 600;
    config.sensor[4].eqns[0] = 0; // a
    config.sensor[4].eqns[1] = 1; // b
    config.sensor[4].eqns[2] = 0; // c
    config.sensor[4].type = SENSOR_CO2;
    sprintf(config.sensor[4].parm, "CO2");
    sprintf(config.sensor[4].unit, "ppm");

    config.sensor[5].enable = false;
    config.sensor[5].port = 0;
    config.sensor[5].address = 2;
    config.sensor[5].samplerate = 10;
    config.sensor[5].averagerate = 600;
    config.sensor[5].eqns[0] = 0; // a
    config.sensor[5].eqns[1] = 1; // b
    config.sensor[5].eqns[2] = 0; // c
    config.sensor[5].type = SENSOR_CH2O;
    sprintf(config.sensor[5].parm, "CH2O");
    sprintf(config.sensor[5].unit, "μg/m³");

    config.sensor[6].enable = false;
    config.sensor[6].port = 0;
    config.sensor[6].address = 2;
    config.sensor[6].samplerate = 10;
    config.sensor[6].averagerate = 600;
    config.sensor[6].eqns[0] = 0; // a
    config.sensor[6].eqns[1] = 1; // b
    config.sensor[6].eqns[2] = 0; // c
    config.sensor[6].type = SENSOR_TVOC;
    sprintf(config.sensor[6].parm, "TVOC");
    sprintf(config.sensor[6].unit, "μg/m³");

    config.sensor[7].enable = false;
    config.sensor[7].port = 0;
    config.sensor[7].address = 2;
    config.sensor[7].samplerate = 10;
    config.sensor[7].averagerate = 600;
    config.sensor[7].eqns[0] = 0; // a
    config.sensor[7].eqns[1] = 1; // b
    config.sensor[7].eqns[2] = 0; // c
    config.sensor[7].type = SENSOR_PRESSURE;
    sprintf(config.sensor[7].parm, "Pressure");
    sprintf(config.sensor[7].unit, "hPa(mBar)");

    config.sensor[8].enable = false;
    config.sensor[8].port = 0;
    config.sensor[8].address = 2;
    config.sensor[8].samplerate = 10;
    config.sensor[8].averagerate = 600;
    config.sensor[8].eqns[0] = 0;   // a
    config.sensor[8].eqns[1] = 0.2; // b
    config.sensor[8].eqns[2] = 0;   // c
    config.sensor[8].type = SENSOR_RAIN;
    sprintf(config.sensor[8].parm, "Rain");
    sprintf(config.sensor[8].unit, "mm.");

    config.sensor[9].enable = false;
    config.sensor[9].port = 0;
    config.sensor[9].address = 2;
    config.sensor[9].samplerate = 10;
    config.sensor[9].averagerate = 600;
    config.sensor[9].eqns[0] = 0; // a
    config.sensor[9].eqns[1] = 1; // b
    config.sensor[9].eqns[2] = 0; // c
    config.sensor[9].type = SENSOR_WIND_SPD;
    sprintf(config.sensor[9].parm, "Wind Speed");
    sprintf(config.sensor[9].unit, "kPh");

// #ifdef CORE_DEBUG_LEVEL
//     config.uart0_enable = false;
//     config.uart0_baudrate = 115200;
//     config.uart0_rx_gpio = 20;
//     config.uart0_tx_gpio = 21;
//     config.uart0_rts_gpio = -1;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    config.uart0_enable = false;
    config.uart0_baudrate = 115200;
    config.uart0_rx_gpio = 44;
    config.uart0_tx_gpio = 43;
    config.uart0_rts_gpio = -1;    
#else
    config.uart0_enable = false;
    config.uart0_baudrate = 9600;
    config.uart0_rx_gpio = 3;
    config.uart0_tx_gpio = 1;
    config.uart0_rts_gpio = -1;
#endif

#ifdef ESP32C3_MINI
    config.uart1_enable = false;
    config.uart1_baudrate = 9600;
    config.uart1_rx_gpio = 21;
    config.uart1_tx_gpio = 20;
    config.uart1_rts_gpio = -1;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    config.uart1_enable = false;
    config.uart1_baudrate = 9600;
    config.uart1_rx_gpio = 18;
    config.uart1_tx_gpio = 17;
    config.uart1_rts_gpio = -1;
#else
    config.uart1_enable = false;
    config.uart1_baudrate = 9600;
    config.uart1_rx_gpio = 18;
    config.uart1_tx_gpio = 19;
    config.uart1_rts_gpio = -1;
#endif

    // config.uart2_enable = false;
    // config.uart2_baudrate = 9600;
    // config.uart2_rx_gpio = 16;
    // config.uart2_tx_gpio = 17;
    // config.uart2_rts_gpio = -1;

    config.modbus_enable = false;
    config.modbus_de_gpio = -1;
    config.modbus_address = 0;
    config.modbus_channel = 0;

    config.onewire_enable = false;
    config.onewire_gpio = -1;

    config.pwr_en = false;
    config.pwr_mode = MODE_A;        // A=Continue,B=Wait for receive,C=Send and sleep
    config.pwr_sleep_interval = 600; // sec
    config.pwr_stanby_delay = 300;   // sec
    config.pwr_sleep_activate = ACTIVATE_TRACKER | ACTIVATE_WIFI;
    config.pwr_gpio = -1;
    config.pwr_active = 1;

    for (int i = 0; i < 5; i++)
    {
        config.trk_tlm_avg[i] = false;
        config.trk_tlm_sensor[i] = 0;
        config.trk_tlm_precision[i] = 0;
        config.trk_tlm_offset[i] = 0;
        memset(config.trk_tlm_UNIT[i], 0, 8);
        memset(config.trk_tlm_PARM[i], 0, 10);
        config.trk_tlm_EQNS[i][0] = 0; // a av2 + bv + c
        config.trk_tlm_EQNS[i][1] = 1; // b
        config.trk_tlm_EQNS[i][2] = 0; // c
    }

    for (int i = 0; i < 5; i++)
    {
        config.digi_tlm_avg[i] = false;
        config.digi_tlm_sensor[i] = 0;
        config.digi_tlm_precision[i] = 0;
        config.digi_tlm_offset[i] = 0;
        memset(config.digi_tlm_UNIT[i], 0, 8);
        memset(config.digi_tlm_PARM[i], 0, 10);
        config.digi_tlm_EQNS[i][0] = 0; // a av2 + bv + c
        config.digi_tlm_EQNS[i][1] = 1; // b
        config.digi_tlm_EQNS[i][2] = 0; // c
    }

    for (int i = 0; i < 5; i++)
    {
        config.igate_tlm_avg[i] = false;
        config.igate_tlm_sensor[i] = 0;
        config.igate_tlm_precision[i] = 0;
        config.igate_tlm_offset[i] = 0;
        memset(config.igate_tlm_UNIT[i], 0, 8);
        memset(config.igate_tlm_PARM[i], 0, 10);
        config.igate_tlm_EQNS[i][0] = 0; // a av2 + bv + c
        config.igate_tlm_EQNS[i][1] = 1; // b
        config.igate_tlm_EQNS[i][2] = 0; // c
    }

    for (int i = 0; i < WX_SENSOR_NUM; i++)
    {
        config.wx_sensor_enable[i] = false;
        config.wx_sensor_avg[i] = true;
        config.wx_sensor_ch[i] = 0;
    }

#ifdef OLED
#ifdef ESP32C3_MINI
    config.i2c_enable = true;
    config.i2c_sda_pin = 5;
    config.i2c_sck_pin = 6;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    config.i2c_enable = true;
    config.i2c_sda_pin = 42;
    config.i2c_sck_pin = 41;
#else
    config.i2c_enable = true;
    config.i2c_sda_pin = 21;
    config.i2c_sck_pin = 22;
#endif
#else
    config.i2c_enable = false;
    config.i2c_sda_pin = -1;
    config.i2c_sck_pin = -1;
#endif
    config.i2c_freq = 400000;
    config.i2c1_enable = false;
    config.i2c1_sda_pin = -1;
    config.i2c1_sck_pin = -1;
    config.i2c1_freq = 100000;

    config.counter0_enable = false;
    config.counter0_active = 0;
    config.counter0_gpio = -1;

    config.counter1_enable = false;
    config.counter1_active = 0;
    config.counter1_gpio = -1;

    config.ext_tnc_enable = false;
    config.ext_tnc_channel = 0;
    config.ext_tnc_mode = 2;

    sprintf(config.path[0], "TRACE2-2");
    sprintf(config.path[1], "WIDE1-1");
    sprintf(config.path[2], "WIDE1-1,WIDE2-1");
    sprintf(config.path[3], "RFONLY");

    config.log = 0;
#ifdef MQTT
    config.en_mqtt = false;
    sprintf(config.mqtt_host, "mqtt.nakhonthai.net");
    char strCID[13];
	uint64_t chipid = ESP.getEfuseMac();
	sprintf(strCID, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
    sprintf(config.mqtt_topic, "/%s/TX",strCID);
    sprintf(config.mqtt_subscribe, "/%s/RX",strCID);
    config.mqtt_user[0]=0;
    config.mqtt_pass[0]=0;
    config.mqtt_port = 1883;
#endif

    config.ppp_enable = false;
    sprintf(config.ppp_apn, "internet");
    sprintf(config.ppp_pin, "0000");
    config.ppp_serial = 0;
    config.ppp_serial_baudrate = 115200;
    config.ppp_rx_gpio = -1;
    config.ppp_tx_gpio = -1;
    config.ppp_rst_gpio = -1;
    config.ppp_rts_gpio = -1;
    config.ppp_rst_active = 0;
    config.ppp_dtr_gpio = -1;
    config.ppp_cts_gpio = -1;
    config.ppp_ri_gpio = -1;
    config.ppp_rst_delay = 1000;
    config.ppp_pwr_gpio = -1;
    config.ppp_pwr_active = 1;

    config.trk_mice_type = 7;
    config.trk_tlm_interval = 0;
    config.digi_tlm_interval = 0;
    config.igate_tlm_interval = 0;
    config.wx_tlm_interval = 0;
    sprintf(config.host_name, "ESP32APRS_Audio");

    config.fx25_mode = 2; //Used modem mode FX.25 RX+TX
}

unsigned long NTP_Timeout;
unsigned long pingTimeout;

bool psramBusy = false;

bool waitPSRAM(bool state)
{
#ifdef BOARD_HAS_PSRAM
    if (state)
    {
        int i = 0;
        while (psramBusy)
        {
            delay(1);
            if (i++ > 1000)
            {
                log_e("PSRAM Busy Timeout");
                return false;
            }
        }
        return true;
    }
    else
    {
        psramBusy = false;
        return true;
    }
#else
    return true;
#endif
}

const char *lastTitle = "LAST HEARD";

int tlmList_Find(char *call)
{
    int i;
    for (i = 0; i < TLMLISTSIZE; i++)
    {
        if (strstr(Telemetry[i].callsign, call) != NULL)
            return i;
    }
    return -1;
}

int tlmListOld()
{
    int i, ret = 0;
    time_t minimum = Telemetry[0].time;
    for (i = 1; i < TLMLISTSIZE; i++)
    {
        if (Telemetry[i].time < minimum)
        {
            minimum = Telemetry[i].time;
            ret = i;
        }
        if (Telemetry[i].time > now())
            Telemetry[i].time = 0;
    }
    return ret;
}

int pkgList_Find(char *call)
{
    int i;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (strstr(pkgList[(int)i].calsign, call) != NULL)
            return i;
    }
    return -1;
}

int pkgList_Find(char *call, uint16_t type)
{
    int i;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[i].type == type)
        {
            if (strstr(pkgList[i].calsign, call) != NULL)
            {
                return i;
            }
        }
    }
    return -1;
}

int pkgList_Find(char *call, char *object, uint16_t type)
{
    int i;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[i].type == type)
        {
            if (strnstr(pkgList[i].calsign, call, strlen(call)) != NULL)
            {
                if (strnstr(pkgList[i].object, object, strlen(object)) != NULL)
                    return i;
            }
        }
    }
    return -1;
}

int pkgListOld()
{
    int i, ret = -1;
    time_t minimum = time(NULL) + 86400; // pkgList[0].time;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[(int)i].time < minimum)
        {
            minimum = pkgList[(int)i].time;
            ret = i;
        }
    }
    return ret;
}

void sort(pkgListType a[], int size)
{
    pkgListType t;
    char *ptr1;
    char *ptr2;
    char *ptr3;
    ptr1 = (char *)&t;
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    for (int i = 0; i < (size - 1); i++)
    {
        for (int o = 0; o < (size - (i + 1)); o++)
        {
            if (a[o].time < a[o + 1].time)
            {
                ptr2 = (char *)&a[o];
                ptr3 = (char *)&a[o + 1];
                memcpy(ptr1, ptr2, sizeof(pkgListType));
                memcpy(ptr2, ptr3, sizeof(pkgListType));
                memcpy(ptr3, ptr1, sizeof(pkgListType));
            }
        }
    }
    psramBusy = false;
}

void sortPkgDesc(pkgListType a[], int size)
{
    pkgListType t;
    char *ptr1;
    char *ptr2;
    char *ptr3;
    ptr1 = (char *)&t;
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    for (int i = 0; i < (size - 1); i++)
    {
        for (int o = 0; o < (size - (i + 1)); o++)
        {
            if (a[o].pkg < a[o + 1].pkg)
            {
                ptr2 = (char *)&a[o];
                ptr3 = (char *)&a[o + 1];
                memcpy(ptr1, ptr2, sizeof(pkgListType));
                memcpy(ptr2, ptr3, sizeof(pkgListType));
                memcpy(ptr3, ptr1, sizeof(pkgListType));
            }
        }
    }
    psramBusy = false;
}

uint16_t pkgType(const char *raw)
{
    uint16_t type = 0;
    char packettype = 0;
    const char *body;
    // int paclen = strlen(raw);
    char *ptr;

    if (*raw == 0)
        return 0;

    packettype = (char)raw[0];
    body = &raw[1];

    switch (packettype)
    {
    case '$': // NMEA
        type |= FILTER_POSITION;
        break;
    case 0x27: /* ' */
    case 0x60: /* ` */
        type |= FILTER_POSITION;
        type |= FILTER_MICE;
        break;
    case '!':
    case '=':
        type |= FILTER_POSITION;
        if (body[18] == '_' || body[10] == '_')
        {
            type |= FILTER_WX;
            break;
        }
    case '/':
    case '@':
        type |= FILTER_POSITION;
        if (body[25] == '_' || body[16] == '_')
        {
            type |= FILTER_WX;
            break;
        }
        if (strchr(body, 'r') != NULL)
        {
            if (strchr(body, 'g') != NULL)
            {
                if (strchr(body, 't') != NULL)
                {
                    if (strchr(body, 'P') != NULL)
                    {
                        type |= FILTER_WX;
                    }
                }
            }
        }
        break;
    case ':':
        if (body[9] == ':' &&
            (memcmp(body + 10, "PARM", 4) == 0 ||
             memcmp(body + 10, "UNIT", 4) == 0 ||
             memcmp(body + 10, "EQNS", 4) == 0 ||
             memcmp(body + 10, "BITS", 4) == 0))
        {
            type |= FILTER_TELEMETRY;
        }
        else
        {
            type |= FILTER_MESSAGE;
        }
        break;
    case '{': // User defind
    case '<': // statcapa
    case '>':
        type |= FILTER_STATUS;
        break;
    case '?':
        type |= FILTER_QUERY;
        break;
    case ';':
        type |= FILTER_OBJECT;
        if (body[35] == '_')
            type |= FILTER_WX;            
        break;
    case ')':
        type |= FILTER_ITEM;
        break;
    case '}':
        type |= FILTER_THIRDPARTY;
        ptr = strchr(raw, ':');
        if (ptr != NULL)
        {
            ptr++;
            type |= pkgType(ptr);
        }
        break;
    case 'T':
        type |= FILTER_TELEMETRY;
        break;
    case '#': /* Peet Bros U-II Weather Station */
    case '*': /* Peet Bros U-I  Weather Station */
    case '_': /* Weather report without position */
        type |= FILTER_WX;
        break;
    default:
        type = 0;
        break;
    }
    return type;
}

// uint16_t TNC2Raw[PKGLISTSIZE];
// int raw_count = 0, raw_idx_rd = 0, raw_idx_rw = 0;

// int pushTNC2Raw(int raw)
// {
//   if (raw < 0)
//     return -1;
//   if (raw_count > PKGLISTSIZE)
//     return -1;
//   if (++raw_idx_rw >= PKGLISTSIZE)
//     raw_idx_rw = 0;
//   TNC2Raw[raw_idx_rw] = raw;
//   raw_count++;
//   return raw_count;
// }

// int popTNC2Raw(int &ret)
// {
//   String raw = "";
//   int idx = 0;
//   if (raw_count <= 0)
//     return -1;
//   if (++raw_idx_rd >= PKGLISTSIZE)
//     raw_idx_rd = 0;
//   idx = TNC2Raw[raw_idx_rd];
//   if (idx < PKGLISTSIZE)
//     ret = idx;
//   if (raw_count > 0)
//     raw_count--;
//   return raw_count;
// }

pkgListType getPkgList(int idx)
{
    pkgListType ret;
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    memset(&ret, 0, sizeof(pkgListType));
    if (idx < PKGLISTSIZE)
        memcpy(&ret, &pkgList[idx], sizeof(pkgListType));
    psramBusy = false;
    return ret;
}

int pkgListUpdate(char *call, char *raw, uint16_t type, bool channel, uint16_t audioLvl)
{
    size_t len;
    if (*call == 0)
        return -1;
    if (*raw == 0)
        return -1;

    // int start_info = strchr(':',0);

    char callsign[11];
    char object[10];
    size_t sz = strlen(call);
    memset(callsign, 0, 11);
    if (sz > 10)
        sz = 10;
    // strncpy(callsign, call, sz);
    memcpy(callsign, call, sz);

#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    int i = -1;

    memset(object, 0, sizeof(object));
    if (type & FILTER_ITEM)
    {
        int x = 0;
        char *body = strchr(raw, ':');
        if (body != NULL)
        {
            body += 2;

            for (int z = 0; z < 9 && body[z] != '!' && body[z] != '_'; z++)
            {
                if (body[z] < 0x20 || body[z] > 0x7e)
                {
                    log_d("\titem name has unprintable characters");
                    break; /* non-printable */
                }
                object[x++] = body[z];
            }
        }
        i = pkgList_Find(callsign, object, type);
    }
    else if (type & FILTER_OBJECT)
    {
        int x = 0;
        char *body = strchr(raw, ':');
        // log_d("body=%s",body);
        if (body != NULL)
        {
            body += 2;
            for (int z = 0; z < 9; z++)
            {
                if (body[z] < 0x30 || body[z] > 0x7A)
                {
                    log_d("\tobject name has unprintable characters");
                    break; // non-printable
                }
                object[x++] = body[z];
                // if (raw[i] != ' ')
                //     namelen = i;
            }
        }
        i = pkgList_Find(callsign, object, type);
    }
    else
    {
        i = pkgList_Find(callsign, type);
    }

    if (i > PKGLISTSIZE)
    {
        psramBusy = false;
        return -1;
    }
    if (i > -1)
    { // Found call in old pkg
        if ((channel == 1) || (channel == pkgList[i].channel))
        {
            pkgList[i].time = time(NULL);
            pkgList[i].pkg++;
            pkgList[i].type = type;
            // memcpy(pkgList[i].object,object,sizeof(object));
            if (channel == 0)
            {
                pkgList[i].audio_level = (int16_t)audioLvl;
                //     pkgList[i].rssi = rssi;
                //     pkgList[i].snr = snr;
                //     pkgList[i].freqErr = freqErr;
            }
            else
            {
                pkgList[i].rssi = -140;
                pkgList[i].snr = 0;
                pkgList[i].freqErr = 0;
                pkgList[i].audio_level = 0;
            }
            len = strlen(raw);
            pkgList[i].length = len + 1;
            if (pkgList[i].raw != NULL)
            {
                pkgList[i].raw = (char *)realloc(pkgList[i].raw, pkgList[i].length);
            }
            else
            {
                pkgList[i].raw = (char *)calloc(pkgList[i].length, sizeof(char));
            }
            if (pkgList[i].raw)
            {
                memset(pkgList[i].raw, 0, pkgList[i].length);
                memcpy(pkgList[i].raw, raw, len);
                pkgList[i].raw[len] = 0;
                log_d("Update: pkgList_idx=%d callsign:%s object:%s", i, callsign, object);
            }
        }
    }
    else
    {
        i = pkgListOld(); // Search free in array
        if (i > PKGLISTSIZE || i < 0)
        {
            psramBusy = false;
            return -1;
        }
        // memset(&pkgList[i], 0, sizeof(pkgListType));
        pkgList[i].channel = channel;
        pkgList[i].time = time(NULL);
        pkgList[i].pkg = 1;
        pkgList[i].type = type;
        if (strlen(object) > 3)
        {
            memcpy(pkgList[i].object, object, 9);
            pkgList[i].object[9] = 0;
        }
        else
        {
            memset(pkgList[i].object, 0, sizeof(pkgList[i].object));
        }
        if (channel == 0)
        {
            pkgList[i].audio_level = (int16_t)audioLvl;
            //     pkgList[i].rssi = rssi;
            //     pkgList[i].snr = snr;
            //     pkgList[i].freqErr = freqErr;
        }
        else
        {
            pkgList[i].rssi = -140;
            pkgList[i].snr = 0;
            pkgList[i].freqErr = 0;
            pkgList[i].audio_level = 0;
        }
        // strcpy(pkgList[i].calsign, callsign);
        memcpy(pkgList[i].calsign, callsign, strlen(callsign));
        len = strlen(raw);
        pkgList[i].length = len + 1;
        if (pkgList[i].raw != NULL)
        {
            pkgList[i].raw = (char *)realloc(pkgList[i].raw, pkgList[i].length);
        }
        else
        {
            pkgList[i].raw = (char *)calloc(pkgList[i].length, sizeof(char));
        }
        if (pkgList[i].raw)
        {
            memset(pkgList[i].raw, 0, pkgList[i].length);
            memcpy(pkgList[i].raw, raw, len);
            pkgList[i].raw[len] = 0;
            log_d("New: pkgList_idx=%d callsign:%s object:%s", i, callsign, object);
        }
    }
    psramBusy = false;
    lastHeard_Flag = true;
    lastHeardTimeout = millis() + 1000;
    return i;
}

bool pkgTxDuplicate(AX25Msg ax25)
{
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    char callsign[12];
    for (int i = 0; i < PKGTXSIZE; i++)
    {
        if (txQueue[i].Active)
        {
            if (txQueue[i].Channel & INET_CHANNEL)
                continue;

            if (ax25.src.ssid > 0)
                sprintf(callsign, "%s-%d", ax25.src.call, ax25.src.ssid);
            else
                sprintf(callsign, "%s", ax25.src.call);
            if (strncmp(&txQueue[i].Info[0], callsign, strlen(callsign)) >= 0) // Check duplicate src callsign
            {
                char *ecs1 = strstr(txQueue[i].Info, ":");
                if (ecs1 == NULL)
                    continue;
                ;
                if (strncmp(ecs1, (const char *)ax25.info, strlen(ecs1)) >= 0)
                { // Check duplicate aprs info
                    txQueue[i].Active = false;
                    psramBusy = false;
                    return true;
                }
            }
        }
    }

    psramBusy = false;
    return false;
}

int pkgTxCount()
{
    int count = 0;
    for (int i = 0; i < PKGTXSIZE; i++)
    {
        if (txQueue[i].Active)
        {
            count++;
        }
    }
    return count;
}

bool pkgTxPush(const char *info, size_t len, int dly, uint8_t Ch)
{
    char *ecs = strstr(info, ">");
    if (ecs == NULL)
        return false;
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    // for (int i = 0; i < PKGTXSIZE; i++)
    // {
    //   if (txQueue[i].Active)
    //   {
    //     if ((strncmp(&txQueue[i].Info[0], info, info - ecs)==0)) //Check src callsign
    //     {
    //       // strcpy(&txQueue[i].Info[0], info);
    //       memset(txQueue[i].Info, 0, sizeof(txQueue[i].Info));
    //       memcpy(&txQueue[i].Info[0], info, len);
    //       txQueue[i].Delay = dly;
    //       txQueue[i].timeStamp = millis();
    //       psramBusy = false;
    //       return true;
    //     }
    //   }
    // }

    // Add
    for (int i = 0; i < PKGTXSIZE; i++)
    {
        if (txQueue[i].Active == false)
        {
            if (len > sizeof(txQueue[i].Info))
                len = sizeof(txQueue[i].Info);
            memset(txQueue[i].Info, 0, sizeof(txQueue[i].Info));
            memcpy(&txQueue[i].Info[0], info, len);
            txQueue[i].length = len;
            txQueue[i].Delay = dly;
            txQueue[i].Active = true;
            txQueue[i].timeStamp = millis();
            txQueue[i].Channel = Ch;
            break;
        }
    }
    psramBusy = false;
    return true;
}

bool pkgTxSend()
{
//   if (getReceive())
//     return false;
#ifdef BOARD_HAS_PSRAM
    while (psramBusy)
        delay(1);
    psramBusy = true;
#endif
    // char info[300];
    for (int i = 0; i < PKGTXSIZE; i++)
    {
        if (txQueue[i].Active)
        {
            int decTime = millis() - txQueue[i].timeStamp;
            if (txQueue[i].Channel & INET_CHANNEL)
            {
                if (config.igate_en == false)
                {
                    txQueue[i].Channel &= ~INET_CHANNEL;
                }
                else
                {
                    if (aprsClient.connected())
                    {
                        // status.txCount++;
                        // aprsClient.printf("%s\r\n", txQueue[i].Info); // Send packet to Inet
                        aprsClient.write(txQueue[i].Info, txQueue[i].length); // Send binary frame packet to APRS-IS (aprsc)
                        aprsClient.write("\r\n");                             // Send CR LF the end frame packet
                        txQueue[i].Channel &= ~INET_CHANNEL;
                        log_d("TX->INET: %s", txQueue[i].Info);
                        continue;
                    }
                }
            }
            if (decTime > txQueue[i].Delay)
            {
                if (txQueue[i].Channel & RF_CHANNEL)
                {

                    psramBusy = false;
                    if(config.rf_en){
                        if ((config.rf_type == RF_SR_1WV) || (config.rf_type == RF_SR_1WU) || (config.rf_type == RF_SR_1W350))
                        {
                            digitalWrite(config.rf_pwr_gpio, LOW);
                            if (config.rf_power ^ !config.rf_pwr_active)
                                pinMode(config.rf_pwr_gpio, OPEN_DRAIN);
                            else
                                pinMode(config.rf_pwr_gpio, OUTPUT);
                        }
                        else
                        {
                            digitalWrite(config.rf_pwr_gpio, config.rf_power ^ !config.rf_pwr_active); // ON RF Power H/L
                        }
                    }
                    status.txCount++;
                    log_d("TX->RF[%i]: %s\n", txQueue[i].length, txQueue[i].Info);
                    APRS_setPreamble(config.preamble * 100); // Send packet to RF
                    APRS_sendTNC2Pkt((uint8_t *)txQueue[i].Info, txQueue[i].length);
                    igateTLM.TX++;

                    // free(info);
                    //  for (int i = 0; i < 100; i++)
                    //  {
                    //      // if (digitalRead(config.rf_ptt_gpio) ^ config.rf_ptt_active)
                    //      if (!getTransmit())
                    //          break;
                    //      delay(50); // TOT 5sec
                    //  }
                    // digitalWrite(config.rf_pwr_gpio, !config.rf_pwr_active); // OFF RF Power H/L
                    // pinMode(config.rf_pwr_gpio, OUTPUT);
                    txQueue[i].Channel &= ~RF_CHANNEL;
                }
            }

            if ((txQueue[i].Channel == 0) || (decTime > 60000))
            {
                txQueue[i].Channel = 0;
                txQueue[i].Active = false;
            }
        }
    }
    psramBusy = false;
    return true;
}

uint8_t *packetData;
// ฟังชั่นถูกเรียกมาจาก ax25_decode
// uint8_t swap=0;
void aprs_msg_callback(struct AX25Msg *msg)
{
    AX25Msg pkg;

    memcpy(&pkg, msg, sizeof(AX25Msg));
    int timeout = 0;
    // while (PacketBuffer.isFull())
    // {
    //     log_d("PacketBuffer is Full!");
    //     delay(100);
    //     if (++timeout > 10)
    //         return;
    // }
    // if(++swap%3 == 0){
    // sprintf((char *)pkg.info,"`l-hl }>/`\"3q}144.937MHz Pirate Pilot on board!!!_%%");
    // pkg.len=52;
    // sprintf(pkg.src.call,"E25KFR");
    // pkg.src.ssid=9;
    // sprintf(pkg.dst.call,"QS4WU3");
    // pkg.dst.ssid=0;
    // pkg.rpt_count=1;
    // sprintf(pkg.rpt_list[0].call,"WIDE1");
    // pkg.rpt_list[0].ssid=1;
    // }

    // PacketBuffer.push(&pkg); // ใส่แพ็จเก็จจาก TNC ลงคิวบัพเฟอร์
    status.rxCount++;
}

void printTime()
{
    struct tm tmstruct;
    getLocalTime(&tmstruct, 5000);
    // Serial.print("[");
    // Serial.print(tmstruct.tm_hour);
    // Serial.print(":");
    // Serial.print(tmstruct.tm_min);
    // Serial.print(":");
    // Serial.print(tmstruct.tm_sec);
    // Serial.print("]");
}

uint8_t gwRaw[PKGLISTSIZE][66];
uint8_t gwRawSize[PKGLISTSIZE];
int gwRaw_count = 0, gwRaw_idx_rd = 0, gwRaw_idx_rw = 0;

void pushGwRaw(uint8_t *raw, uint8_t size)
{
    if (gwRaw_count > PKGLISTSIZE)
        return;
    if (++gwRaw_idx_rw >= PKGLISTSIZE)
        gwRaw_idx_rw = 0;
    if (size > 65)
        size = 65;
    memcpy(&gwRaw[gwRaw_idx_rw][0], raw, size);
    gwRawSize[gwRaw_idx_rw] = size;
    gwRaw_count++;
}

uint8_t popGwRaw(uint8_t *raw)
{
    uint8_t size = 0;
    if (gwRaw_count <= 0)
        return 0;
    if (++gwRaw_idx_rd >= PKGLISTSIZE)
        gwRaw_idx_rd = 0;
    size = gwRawSize[gwRaw_idx_rd];
    memcpy(raw, &gwRaw[gwRaw_idx_rd][0], size);
    if (gwRaw_count > 0)
        gwRaw_count--;
    return size;
}

bool SA868_waitResponse(String &data, String rsp, uint32_t timeout)
{
    uint32_t startMillis = millis();
    data.clear();
    do
    {
        while (SerialRF.available() > 0)
        {
            int8_t ch = SerialRF.read();
            data += static_cast<char>(ch);
            if (rsp.length() && data.endsWith(rsp))
            {
                return true;
            }
            // esp_task_wdt_reset();
        }
        delay(1);
        // esp_task_wdt_reset();
    } while (millis() - startMillis < timeout);
    return false;
}

int SA868_getRSSI()
{
    String data;
    int rssi;

    // Serial.printf("AT+RSSI?\r\n");
    SerialRF.printf("AT+RSSI?\r\n");
    if (SA868_waitResponse(data, "\r\n", 1000))
    {
        // Serial.println(INFO + data);
        String rssi = data.substring(data.indexOf("RSSI=") + strlen("RSSI="), data.indexOf("\r\n"));
        rssi = rssi.toInt();
        return rssi.toInt();
    }
    else
    {
        // timeout or error
        return 0;
    }
}

String SA868_getVERSION()
{
    String data;
    int rssi;

    SerialRF.printf("AT+VERSION\r\n");
    if (SA868_waitResponse(data, "\r\n", 1000))
    {
        String version = data.substring(0, data.indexOf("\r\n"));
        return version;
    }
    else
    {
        // timeout or error
        return "-";
    }
}

String FRS_getVERSION()
{
    String data;
    String version;

    SerialRF.printf("AT+DMOVERQ\r\n");
    if (SA868_waitResponse(data, "\r\n", 1000))
    {
        int st = data.indexOf("DMOVERQ:");
        if (st > 0)
        {
            version = data.substring(st + 8, data.indexOf("\r\n"));
        }
        else
        {
            version = "Not found";
        }
        return version;
    }
    else
    {
        // timeout or error
        return "-";
    }
}

unsigned long SA818_Timeout = 0;

void RF_MODULE_SLEEP()
{
    digitalWrite(config.rf_pwr_gpio, !config.rf_pwr_active);
    digitalWrite(config.rf_pd_gpio, LOW);
    // PMU.disableDC3();
}

String hexToString(String hex)
{ // for String to HEX conversion
    String text = "";
    for (int k = 0; k < hex.length(); k++)
    {
        if (k % 2 != 0)
        {
            char temp[3];
            sprintf(temp, "%c%c", hex[k - 1], hex[k]);
            int number = (int)strtol(temp, NULL, 16);
            text += char(number);
        }
    }
    return text;
}

String ctcssToHex(unsigned int decValue, int section)
{ // use to convert the CTCSS reading which RF module needed
    if (decValue == 7777)
    {
        return hexToString("FF");
    }
    String d1d0 = String(decValue);
    if (decValue < 1000)
    {
        d1d0 = "0" + d1d0;
    }
    if (section == 1)
    {
        return hexToString(d1d0.substring(2, 4));
    }
    else
    {
        return hexToString(d1d0.substring(0, 2));
    }
}

void RF_MODULE(bool boot)
{
    String data;
    if (config.rf_en == false)
    {
        RF_MODULE_SLEEP();
        return;
    }
    if (config.rf_type == RF_NONE)
        return;

    if (config.rf_rx_gpio == -1 || config.rf_tx_gpio == -1 || config.rf_pd_gpio == -1)
    {
        log_d("FAIL! RF Module not config gpio");
        return;
    }
    log_d("RF Module %s Init", RF_TYPE[config.rf_type]);
    adcEn = -1;
    dacEn = -1;
    //! DC3 Radio & Pixels VDD , Don't change
    // PMU.setDC3Voltage(3400);
    // PMU.disableDC3();
    pinMode(config.rf_pwr_gpio, OUTPUT);
    digitalWrite(config.rf_pwr_gpio, !config.rf_pwr_active); // RF POWER LOW

    // pinMode(config.rf_ptt_gpio, OUTPUT);
    // digitalWrite(config.rf_ptt_gpio, !config.rf_ptt_active); // PTT HIGH
    pinMode(config.rf_pd_gpio, OUTPUT);
    digitalWrite(config.rf_pd_gpio, !config.rf_pd_active); // PD LOW
    delay(500);
    if (boot)
    {
        SerialRF.begin(config.rf_baudrate, EspSoftwareSerial::SWSERIAL_8N1, config.rf_rx_gpio, config.rf_tx_gpio);
        SerialRF.setTimeout(10);
    }
    digitalWrite(config.rf_pd_gpio, config.rf_pd_active); // PD HIGH

    delay(1000);
    SerialRF.write("\r\n");

    char str[200];
    String rsp = "\r\n";
    if (config.sql_level > 8)
        config.sql_level = 8;

    // esp_task_wdt_reset();
    if ((config.rf_type == RF_SR_1WV) || (config.rf_type == RF_SR_1WU) || (config.rf_type == RF_SR_1W350))
    {
        SerialRF.printf("AT+DMOCONNECT\r\n");
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
        RF_VERSION = FRS_getVERSION();
        log_d("RF Module Version %s", RF_VERSION);
        sprintf(str, "AT+DMOSETGROUP=%01d,%0.4f,%0.4f,%d,%01d,%d,0", config.band, config.freq_tx + ((float)config.offset_tx / 1000000), config.freq_rx + ((float)config.offset_rx / 1000000), config.tone_rx, config.sql_level, config.tone_tx);
        SerialRF.println(str);
        log_d("Write to SR_FRS: %s", str);
        if (SA868_waitResponse(data, rsp, 1500))
            log_d("%s", data.c_str());
        // Module auto power save setting
        SerialRF.printf("AT+DMOAUTOPOWCONTR=1\r\n");
        if (SA868_waitResponse(data, rsp, 1500))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+DMOSETVOX=0\r\n");
        if (SA868_waitResponse(data, rsp, 1500))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+DMOSETMIC=6,0\r\n");
        if (SA868_waitResponse(data, rsp, 1500))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+DMOSETVOLUME=%d\r\n", config.volume);
        if (SA868_waitResponse(data, rsp, 1500))
            log_d("%s", data.c_str());
    }
    else if ((config.rf_type == RF_SA868_VHF) || (config.rf_type == RF_SA868_UHF) || (config.rf_type == RF_SA868_350))
    {
        SerialRF.printf("AT+DMOCONNECT\r\n");
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
        RF_VERSION = SA868_getVERSION();
        log_d("RF Module Version %s", RF_VERSION);
        sprintf(str, "AT+DMOSETGROUP=%01d,%0.4f,%0.4f,%04d,%01d,%04d\r\n", config.band, config.freq_tx, config.freq_rx, config.tone_tx, config.sql_level, config.tone_rx);
        SerialRF.print(str);
        log_d("Write to SA868: %s", str);
        if (SA868_waitResponse(data, rsp, 2000))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+SETTAIL=0\r\n");
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+SETFILTER=1,1,1\r\n");
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+DMOSETVOLUME=%d\r\n", config.volume);
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
    }
    else if ((config.rf_type == RF_SR_2WVS) || (config.rf_type == RF_SR_2WUS))
    {
        uint8_t flag = 0;  // Bit0:busy lock 0:OFF,1:ON Bit1:band 0:Wide,1:Narrow
        uint8_t flag1 = 0; // Bit0: Hi/Lo 0:2W,1:0.5W  Bit1:Middle 0:2W/0.5W 1:1W
        RF_VERSION = FRS_getVERSION();
        log_d("RF Module Version %s", RF_VERSION.c_str());

        String tone_rx, tone_tx;

        if (config.tone_rx > 0)
        {
            int idx = config.tone_rx;
            if (idx < sizeof(ctcss))
            {
                int tone = (int)(ctcss[config.tone_rx] * 10.0F);
                tone_rx = ctcssToHex(tone, 1);
                tone_rx += ctcssToHex(tone, 2);
                log_d("Tone RX[%d] rx=%0X %0X", tone, tone_rx.charAt(0), tone_rx.charAt(1));
            }
        }
        else
        {
            tone_rx = ctcssToHex(7777, 1);
            tone_rx += ctcssToHex(7777, 2);
        }

        if (config.tone_tx > 0)
        {
            int idx = config.tone_tx;
            if (idx < sizeof(ctcss))
            {
                int tone = (int)(ctcss[config.tone_tx] * 10.0F);
                tone_tx = ctcssToHex(tone, 1);
                tone_tx += ctcssToHex(tone, 2);
                log_d("Tone RX[%d] tx=%0X %0X", tone, tone_tx.charAt(0), tone_tx.charAt(1));
            }
        }
        else
        {
            tone_tx = ctcssToHex(7777, 1);
            tone_tx += ctcssToHex(7777, 2);
        }

        if (config.band == 0)
            flag |= 0x02;
        if (config.rf_power == 0)
            flag1 |= 0x02;
        sprintf(str, "AT+DMOGRP=%0.5f,%0.5f,%s,%s,%d,%d\r\n", config.freq_rx, config.freq_tx, tone_rx.c_str(), tone_tx.c_str(), flag, flag1);
        log_d("Write to SR_FRS_2W: %s", str);
        SerialRF.print(str);

        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+DMOSAV=1\r\n");
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+DMOVOL=%d\r\n", config.volume);
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+DMOVOX=0\r\n");
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
        SerialRF.printf("AT+DMOFUN=%d,5,1,0,0\r\n", config.sql_level);
        if (SA868_waitResponse(data, rsp, 1000))
            log_d("%s", data.c_str());
    }
    delay(1);
    adcEn = 1;
    dacEn = -1;
}

void RF_MODULE_CHECK()
{
    while (SerialRF.available() > 0)
        SerialRF.read();
    SerialRF.println("AT+DMOCONNECT");
    delay(100);
    if (SerialRF.available() > 0)
    {
        String ret = SerialRF.readString();
        if (ret.indexOf("DMOCONNECT") > 0)
        {
            SA818_Timeout = millis();
            // Serial.println(SerialRF.readString());
            log_d("RF Module %s Activate", RF_TYPE[config.rf_type]);
        }
    }
    else
    {
        log_d("RF Module %s sleep", RF_TYPE[config.rf_type]);
        digitalWrite(config.rf_pwr_gpio, !config.rf_pwr_active);
        digitalWrite(config.rf_pd_gpio, LOW);
        delay(500);
        RF_MODULE(true);
    }
}

WiFiClient aprsClient;

boolean APRSConnect()
{
    // Serial.println("Connect TCP Server");
    String login = "";
    int cnt = 0;
    uint8_t con = aprsClient.connected();
    // Serial.println(con);
    if (con <= 0)
    {
        if (!aprsClient.connect(config.aprs_host, config.aprs_port)) // เชื่อมต่อกับเซิร์ฟเวอร์ TCP
        {
            // Serial.print(".");
            delay(1000);
            cnt++;
            if (cnt > 3) // วนร้องขอการเชื่อมต่อ 50 ครั้ง ถ้าไม่ได้ให้รีเทิร์นฟังค์ชั่นเป็น False
                return false;
        }
        // ขอเชื่อมต่อกับ aprsc
        if (strlen(config.igate_object) >= 3)
        {
            uint16_t passcode = aprsParse.passCode(config.igate_object);
            login = "user " + String(config.igate_object) + " pass " + String(passcode, DEC) + " vers ESP32APRS_Audio V" + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
        }
        else
        {
            uint16_t passcode = aprsParse.passCode(config.aprs_mycall);
            if (config.aprs_ssid == 0)
                login = "user " + String(config.aprs_mycall) + " pass " + String(passcode, DEC) + " vers ESP32APRS_Audio V" + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
            else
                login = "user " + String(config.aprs_mycall) + "-" + String(config.aprs_ssid) + " pass " + String(passcode, DEC) + " vers ESP32APRS_Audio V" + String(VERSION) + String(VERSION_BUILD) + " filter " + String(config.aprs_filter);
        }
        aprsClient.println(login);
        // Serial.println(login);
        // Serial.println("Success");
        delay(500);
    }
    return true;
}

long oledSleepTimeout = 0;
bool showDisp = false;
RTC_DATA_ATTR uint8_t curTab;

void preTransmission()
{
    digitalWrite(config.modbus_de_gpio, 1);
}

void postTransmission()
{
    digitalWrite(config.modbus_de_gpio, 0);
}

#ifdef BLUETOOTH
void bluetooth_init()
{
    if (config.bt_master == true)
    {
#if !defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32C6)
        // Initialize BLE stack and Nordic UART service
        NimBLEDevice::init(config.bt_name);
        NimBLEDevice::getAdvertising()->setName(config.bt_name);
        // BLE Auth
        if (config.bt_pin > 0)
        {
            NimBLEDevice::setSecurityAuth(true, true, true); /** bonding, MITM, BLE secure connections */
            NimBLEDevice::setSecurityPasskey(config.bt_pin);
            NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); /** Display only passkey */
            log_d("Bluetooth Security: Passkey %d", config.bt_pin);
        }
        NuSerial.begin_uuid(config.bt_uuid, config.bt_uuid_tx, config.bt_uuid_rx, 115200);
#else
#endif
    }
}
#endif

// 3 seconds WDT
#define WDT_TIMEOUT 10

bool AFSKInitAct = false;
void setup()
{
    //byte *ptr;
    // setCpuFrequencyMhz(160);
#ifdef BOARD_HAS_PSRAM
    pkgList = (pkgListType *)ps_malloc(sizeof(pkgListType) * PKGLISTSIZE);
    Telemetry = (TelemetryType *)malloc(sizeof(TelemetryType) * TLMLISTSIZE);
    txQueue = (txQueueType *)ps_malloc(sizeof(txQueueType) * PKGTXSIZE);
    // TNC2Raw = (int *)ps_malloc(sizeof(int) * PKGTXSIZE);
#else
    pkgList = (pkgListType *)malloc(sizeof(pkgListType) * PKGLISTSIZE);
    Telemetry = (TelemetryType *)malloc(sizeof(TelemetryType) * TLMLISTSIZE);
    txQueue = (txQueueType *)malloc(sizeof(txQueueType) * PKGTXSIZE);
    // TNC2Raw = (int *)malloc(sizeof(int) * PKGTXSIZE);
#endif

    memset(pkgList, 0, sizeof(pkgListType) * PKGLISTSIZE);
    memset(Telemetry, 0, sizeof(TelemetryType) * TLMLISTSIZE);
    memset(txQueue, 0, sizeof(txQueueType) * PKGTXSIZE);

    pinMode(BOOT_PIN, INPUT_PULLUP); // BOOT Button
    pinMode(LED_RX, OUTPUT);
    pinMode(LED_TX, OUTPUT);

    // Set up serial port
#ifdef CORE_DEBUG_LEVEL
    Serial.begin(115200); // debug
#else
    Serial.begin(9600); // monitor
#endif

    if (!LITTLEFS.begin(FORMAT_LITTLEFS_IF_FAILED))
    {
        log_d("LITTLEFS Mount Failed");
        // return;
    }
    else
    {
        log_d("File system mounted");
        log_d("Total space: %lu\n", LITTLEFS.totalBytes());
        log_d("Use space:  %lu\n\n", LITTLEFS.usedBytes());
    }

    Serial.println();
    Serial.println("Start ESP32APRS_Audio V" + String(VERSION));
    Serial.println("Push BOOT after 3 sec for Factory Default config.");

    // if (!EEPROM.begin(EEPROM_SIZE))
    // {
    //     Serial.println(F("failed to initialise EEPROM")); // delay(100000);
    // }
    // // ตรวจสอบคอนฟิกซ์ผิดพลาด
    // ptr = (byte *)&config;
    // EEPROM.readBytes(1, ptr, sizeof(Configuration));
    // uint8_t chkSum = checkSum(ptr, sizeof(Configuration));
    // Serial.printf("EEPROM Check %0Xh=%0Xh(%dByte)\n", EEPROM.read(0), chkSum, sizeof(Configuration));
    // if (EEPROM.read(0) != chkSum)
    // {
    //     Serial.println("CFG EEPROM Error!");
    //     Serial.println("Factory Default");
    //     defaultConfig();
    // }
    #ifdef STRIP_PIN
    strip = new Adafruit_NeoPixel(1, STRIP_PIN, NEO_GRB + NEO_KHZ800);    
    #endif

    LED_Status(255,255,255);

    if (!LITTLEFS.exists("/default.cfg"))
    {
        log_d("Factory Default");
        defaultConfig();
        saveConfiguration("/default.cfg", config);
    }
    else
    {
        if (!loadConfiguration("/default.cfg", config))
            defaultConfig();
    }

    LED_Status(255,255,255);

    #if defined(CONFIG_IDF_TARGET_ESP32S3)
    if((config.rf_ptt_gpio > 25) && (config.rf_ptt_gpio < 38))
      config.rf_ptt_gpio = 5; // GPIO25-37 are flash only on ESP32S3
    #endif

    if (config.i2c1_enable)
    {
        Wire1.begin(config.i2c1_sda_pin, config.i2c1_sck_pin, config.i2c1_freq);
    }

#ifdef OLED
        if (config.oled_enable)
        {
            i2c_busy = true;
            Wire.begin(config.i2c_sda_pin, config.i2c_sck_pin, config.i2c_freq);
// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
// display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false); // initialize with the I2C addr 0x3C (for the 128x64)
#ifdef SH1106
            display.begin(SH1106_SWITCHCAPVCC, SCREEN_ADDRESS, OLED_RESET > -1);
#else
            display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, OLED_RESET > -1, false);
#endif
            // Initialising the UI will init the display too.
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(WHITE);

            display.setTextSize(1);
            display.setFont(&FreeSansBold9pt7b);
            display.setCursor(0, 15);
            display.print("APRS");
            display.setCursor(60, 32);
            display.print("AUDIO");
            display.drawYBitmap(0, 16, LOGO, 48, 48, WHITE);
            display.drawRoundRect(52, 16, 75, 22, 3, WHITE);

            display.setFont();
            display.setTextColor(WHITE);

            display.setCursor(60, 40);
            display.printf("FW Ver %s%c", VERSION, VERSION_BUILD);
            display.setCursor(65, 5);
            display.print("Copy@2022");
            display.display();
            LED_Status(255,0,0);
            delay(1000);
            //digitalWrite(LED_TX, HIGH);            
            display.fillRect(49, 49, 50, 8, 0);
            display.setCursor(70, 50);
            display.print("3 Sec");
            display.display();
            LED_Status(0,255,0);
            delay(1000);
            //digitalWrite(LED_RX, HIGH);            
            display.fillRect(49, 49, 50, 8, 0);
            display.setCursor(70, 50);
            display.print("2 Sec");
            display.display();
            LED_Status(0,0,255);
            delay(1000);
            display.fillRect(49, 49, 50, 8, 0);
            display.setCursor(70, 50);
            display.print("1 Sec");            
            display.display();
            LED_Status(0,0,0);
            i2c_busy = false;            
        }
        else
        {
            LED_Status(200,0,0);
            delay(1000);
            LED_Status(0,200,0);
            delay(1000);
            LED_Status(0,0,200);
            delay(1000);
        }
#else
    if (config.i2c_enable)
    {
        Wire.begin(config.i2c_sda_pin, config.i2c_sck_pin, config.i2c_freq);
    }
    LED_Status(200,0,0);
            delay(1000);
            LED_Status(0,200,0);
            delay(1000);
            LED_Status(0,0,200);
            delay(1000);
#endif

    if (digitalRead(BOOT_PIN) == LOW)
    {
        defaultConfig();
        Serial.println("Manual Default configure!");
#ifdef OLED
        display.clearDisplay();
        display.setCursor(10, 22);
        display.print("Factory Reset!");
        display.display();
#endif
        while (digitalRead(BOOT_PIN) == LOW)
        {
            delay(500);
            LED_Status(0,0,0);
            delay(500);
            LED_Status(255,255,255);
        }
    }
    LED_Status(0,0,0);

    // if (config.counter0_enable)
    // {
    //     pcnt_init_channel(PCNT_UNIT_0, config.counter0_gpio, config.counter0_active); // Initialize Unit 0 to pin 4
    // }
    // if (config.counter1_enable)
    // {
    //     pcnt_init_channel(PCNT_UNIT_1, config.counter1_gpio, config.counter1_active); // Initialize Unit 0 to pin 4
    // }
    log_d("UART config");
    #ifdef ARDUINO_USB_MODE
        Serial.begin(config.uart0_baudrate);
    #endif
    if (config.uart0_enable)
    {        
        Serial0.begin(config.uart0_baudrate, SERIAL_8N1, config.uart0_rx_gpio, config.uart0_tx_gpio);
    }
    if (config.uart1_enable)
    {
        pinMode(config.uart1_rx_gpio, INPUT_PULLUP); // Set RX pin to INPUT_PULLUP
        pinMode(config.uart1_tx_gpio, OUTPUT);       // Set TX pin to OUTPUT
        Serial1.begin(config.uart1_baudrate, SERIAL_8N1, config.uart1_rx_gpio, config.uart1_tx_gpio);
    }
    // if (config.uart2_enable)
    // {
    //     Serial2.begin(config.uart2_baudrate, SERIAL_8N1, config.uart2_rx_gpio, config.uart2_tx_gpio);
    // }
    log_d("MODBUS config");
    if (config.modbus_enable)
    {
        if (config.modbus_channel == 1)
        {
            modbus.begin(config.modbus_address, Serial);
        }
        else if (config.modbus_channel == 2)
        {
            modbus.begin(config.modbus_address, Serial1);
        }
#ifdef __XTENSA__
        else if (config.modbus_channel == 3)
        {
            modbus.begin(config.modbus_address, Serial2);
        }
#endif
        if (config.modbus_channel > 0 && config.modbus_channel < 4)
        {
            // Modbus slave ID 1
            if (config.modbus_de_gpio > -1)
            {
                pinMode(config.modbus_de_gpio, OUTPUT);
                // Callbacks allow us to configure the RS485 transceiver correctly
                modbus.preTransmission(preTransmission);
                modbus.postTransmission(postTransmission);
            }
        }
    }   

    log_d("RF Module config");
    delay(10);

    
    if (config.rf_en)
        RF_MODULE(true);
    log_d("Free heap: %d", ESP.getHeapSize());

#ifdef OLED
    if (config.oled_enable == true)
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.display();
    }
#endif

    showDisp = true;
    if(curTab>7) curTab = 6;
    oledSleepTimeout = millis() + (config.oled_timeout * 1000);

    // enableLoopWDT();
    // enableCore0WDT();
    // enableCore1WDT();
    // disableCore1WDT();
//      esp_task_wdt_config_t twdt_config = {
//         .timeout_ms = 3000,                             // 3 seconds
//         .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Bitmask of all cores
//         .trigger_panic = false,
//     };
//     //#if !defined(CONFIG_IDF_TARGET_ESP32C6)
//     esp_task_wdt_init(&twdt_config); // enable panic so ESP32 restarts
// // #else
// //     esp_task_wdt_init(&twdt_config); // enable panic so ESP32 restarts
// // #endif
//     esp_task_wdt_add(NULL);

    oledSleepTimeout = millis() + (config.oled_timeout * 1000);
    AFSKInitAct = false;
    log_d("GNSS config");
    if (config.gnss_enable)
    {
        if ((config.gnss_channel > 0) && (config.gnss_channel < 4))
        {
            if (strstr("AT", config.gnss_at_command) != NULL)
            {
                if (config.gnss_channel == 1)
                {
                    Serial.println(config.gnss_at_command);
                }
                else if (config.gnss_channel == 2)
                {
                    Serial1.println(config.gnss_at_command);
                }
#if SOC_UART_NUM > 2
                else if (config.gnss_channel == 3)
                {
                    Serial2.println(config.gnss_at_command);
                }
#endif
            }
        }
    }
    log_d("Start Task");
#ifdef __XTENSA__
    //if (config.wifi_mode != 0)
    //{
        xTaskCreatePinnedToCore(
            taskNetwork,        /* Function to implement the task */
            "taskNetwork",      /* Name of the task */
            12000,              /* Stack size in words */
            NULL,               /* Task input parameter */
            1,                  /* Priority of the task */
            &taskNetworkHandle, /* Task handle. */
            1);                 /* Core where the task should run */
    //}
    xTaskCreatePinnedToCore(
        taskAPRSPoll,        /* Function to implement the task */
        "taskAPRSPoll",      /* Name of the task */
        2048,                /* Stack size in words */
        NULL,                /* Task input parameter */
        0,                   /* Priority of the task */
        &taskAPRSPollHandle, /* Task handle. */
       1);                  /* Core where the task should run */
    xTaskCreatePinnedToCore(
        taskAPRS,        /* Function to implement the task */
        "taskAPRS",      /* Name of the task */
        4096,            /* Stack size in words */
        NULL,            /* Task input parameter */
        2,               /* Priority of the task */
        &taskAPRSHandle, /* Task handle. */
        1);              /* Core where the task should run */
#else
    //if (config.wifi_mode != 0)
    //{
        xTaskCreatePinnedToCore(
            taskNetwork,        /* Function to implement the task */
            "taskNetwork",      /* Name of the task */
            12000,              /* Stack size in words */
            NULL,               /* Task input parameter */
            1,                  /* Priority of the task */
            &taskNetworkHandle, /* Task handle. */
            0);                 /* Core where the task should run */
    //}

    xTaskCreatePinnedToCore(
        taskAPRSPoll,        /* Function to implement the task */
        "taskAPRSPoll",      /* Name of the task */
        2048,                /* Stack size in words */
        NULL,                /* Task input parameter */
        0,                   /* Priority of the task */
        &taskAPRSPollHandle, /* Task handle. */
        0);                  /* Core where the task should run */
    
    xTaskCreatePinnedToCore(
        taskAPRS,        /* Function to implement the task */
        "taskAPRS",      /* Name of the task */
        6000,            /* Stack size in words */
        NULL,            /* Task input parameter */
        2,               /* Priority of the task */
        &taskAPRSHandle, /* Task handle. */
        0);              /* Core where the task should run */
#endif

    if (config.gnss_enable)
    {
        xTaskCreatePinnedToCore(
            taskGPS,        /* Function to implement the task */
            "taskGPS",      /* Name of the task */
            3072,           /* Stack size in words */
            NULL,           /* Task input parameter */
            4,              /* Priority of the task */
            &taskGPSHandle, /* Task handle. */
            0);             /* Core where the task should run */
    }

    if (config.ext_tnc_enable)
    {
        xTaskCreatePinnedToCore(
            taskSerial,        /* Function to implement the task */
            "taskSerial",      /* Name of the task */
            2048,              /* Stack size in words */
            NULL,              /* Task input parameter */
            5,                 /* Priority of the task */
            &taskSerialHandle, /* Task handle. */
            0);                /* Core where the task should run */
    }

    bool SensorEn = false;
    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if (config.sensor[i].enable)
        {
            SensorEn = true;
            break;
        }
    }

    if (SensorEn == true)
    {
        xTaskCreatePinnedToCore(
            taskSensor,        /* Function to implement the task */
            "taskSensor",      /* Name of the task */
            4096,              /* Stack size in words */
            NULL,              /* Task input parameter */
            6,                 /* Priority of the task */
            &taskSensorHandle, /* Task handle. */
            0);                /* Core where the task should run */
    }

    upTimeStamp = millis() / 1000;
    autoResetTimeout = millis() + ((long)config.reset_timeout * 60000);
}

String getTimeStamp()
{
    char strtmp[50];
    time_t now;
    time(&now);
    struct tm *info = gmtime(&now);
    sprintf(strtmp, "%02d%02d%02dz", info->tm_mday, info->tm_hour, info->tm_min);
    return String(strtmp);
}

int pkgCount = 0;

float conv_coords(float in_coords)
{
    // Initialize the location.
    float f = in_coords;
    // Get the first two digits by turning f into an integer, then doing an integer divide by 100;
    // firsttowdigits should be 77 at this point.
    int firsttwodigits = ((int)f) / 100; // This assumes that f < 10000.
    float nexttwodigits = f - (float)(firsttwodigits * 100);
    float theFinalAnswer = (float)(firsttwodigits + nexttwodigits / 60.0);
    return theFinalAnswer;
}

void DD_DDDDDtoDDMMSS(float DD_DDDDD, int *DD, int *MM, int *SS)
{
    DD_DDDDD = abs(DD_DDDDD);
    *DD = (int)DD_DDDDD;
    *MM = (int)((DD_DDDDD - *DD) * 60);
    *SS = ((DD_DDDDD - *DD) * 60 - *MM) * 100;
}

String compress_position(double nowLat, double nowLng, int alt_feed, double course, uint16_t spdKnot, char table, char symbol, bool gps)
{
    String str_comp = "";
    // String lat, lon;
    // lat = deg2lat(nowLat);
    // lon = deg2lon(nowLng);
    char lat[15], lon[15];
    memset(lat, 0, sizeof(lat));
    memset(lon, 0, sizeof(lon));
    deg2lat(lat, nowLat);
    deg2lon(lon, nowLng);
    // ESP_LOGE("GPS", "Aprs Compress");
    //  Translate from semicircles to Base91 format
    char aprs_position[15];
    long latitude = semicircles(lat, (nowLat < 0));
    long longitude = semicircles(lon, (nowLng < 0));
    long ltemp = 1073741824L - latitude; // 90 degrees - latitude
    // ESP_LOGE("GPS", "lat=%u lon=%u", latitude, longitude);
    memset(aprs_position, 0, sizeof(aprs_position));

    base91encode(ltemp, aprs_position);
    ltemp = 1073741824L + (longitude >> 1); // 180 degrees + longitude
    base91encode(ltemp, aprs_position + 4);
    // Encode heading
    uint8_t c = (uint8_t)(course / 4);
    // Scan lookup table to encode speed
    uint8_t s = (uint8_t)(log(spdKnot + 1) / log(1.08));
    if ((spdKnot <= 5) && (alt_feed > 0) && config.trk_altitude)
    {
        if (gps)
        {
            // Send Altitude
            aprs_position[11] = '!' + 0x30; // t current,GGA
            int alt = (int)alt_feed;
            int cs = (int)(log(alt) / log(1.002));
            c = (uint8_t)(cs / 91);
            s = (uint8_t)(cs - ((int)c * 91));
            if (s > 91)
                s = 91;
            aprs_position[9] = '!' + c;  // c
            aprs_position[10] = '!' + s; // s
        }
        else
        {
            // Send Range
            aprs_position[11] = '!' + 0x00; //
            aprs_position[9] = '{';         // c = {
            if (!config.rf_power)
            {
                s = 10;
            }
            else
            {
                s = 30;
            }
            aprs_position[10] = '!' + s; // s
        }
    }
    else
    {
        // Send course and speed
        aprs_position[9] = '!' + c;  // c
        aprs_position[10] = '!' + s; // s

        if (gps)
        {
            aprs_position[11] = '!' + 0x20 + 0x18 + 0x06; // t 0x20 1=current,0x18 11=RMC,0x06 110=Other tracker
        }
        else
        {
            aprs_position[11] = '!' + 0x00 + 0x18 + 0x06; // t
        }
    }
    aprs_position[12] = 0;
    // waveFlag = false;
    aprs_position[8] = symbol; // Symbol
    str_comp = String(table) + String(aprs_position);
    return str_comp;
}

#ifdef MQTT

WiFiClient espClient;
PubSubClient clientMQTT(espClient);

void mqtt_reconnect()
{
    // Loop until we're reconnected
    if (!clientMQTT.connected())
    {
        char strCID[13];
        uint64_t chipid = ESP.getEfuseMac();
        sprintf(strCID, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        char idMqtt[50];
        sprintf(idMqtt, "%s", strCID);
        // if (strlen(config.wx_object) > 3)
        // {
        //     strcat(idMqtt, "_");
        //     strcat(idMqtt, config.wx_object);
        // }
        bool res=false;
        if(config.mqtt_user[0]!=0){
            res=clientMQTT.connect(idMqtt, config.mqtt_user, config.mqtt_pass);
        }else{
            res=clientMQTT.connect(idMqtt);
        }
        if (res)
        {
            log_d("MQTT Connected");
            // Once connected, publish an announcement...
            char payload[20];
            // char topic[50];
            // if (strlen(config.wx_object) > 3)
            //     sprintf(topic, "/%s/status", config.wx_object);
            // else
            //     sprintf(topic, "/%s/status", config.wx_mycall);
            sprintf(payload, "Connected");

            clientMQTT.publish(config.mqtt_topic, payload);
            // ... and resubscribe
            clientMQTT.subscribe(config.mqtt_subscribe);
            delay(3000);
        }
        else
        {
            log_d("MQTT Failed, rc=%d ", clientMQTT.state());
        }
    }
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
    byte *payload_ptr=(byte *)calloc(1,length+1);
    if(payload_ptr==NULL){
        log_d("MQTT No memory");
        return;
    }
    memset(payload_ptr,0,length+1);
    memcpy(payload_ptr, payload, length);
    log_d("MQTT Message arrived on topic [%s] with length %d", topic, length);
    if (payload_ptr[0] == 'A' && payload_ptr[1] == 'T')
    {
        if(config.at_cmd_mqtt){
            log_d("AT-Command received: %s", payload_ptr);
            String ret = handleATCommand(String((char *)payload_ptr));
            log_d("AT-Command response: %s", ret.c_str());
            clientMQTT.publish(config.mqtt_topic, ret.c_str());        
        }
    }
    else
    {
        pkgTxPush((const char *)payload_ptr, length, 0, RF_CHANNEL);
    }
    free(payload_ptr);
}

#endif // MQTT

String compressMicE(char *destCallsign, float lat, float lon, uint16_t heading, uint16_t speed, uint8_t type, uint8_t *telem, size_t telemLen, char *grid, char *status, int32_t alt, char table, char symbol)
{
    String strRet = "";
    // sanity checks first
    if (((telemLen == 0) && (telem != NULL)) || ((telemLen != 0) && (telem == NULL)))
    {
        return strRet;
    }

    if ((telemLen != 0) && (telemLen != 2) && (telemLen != 5))
    {
        return strRet;
    }

    if ((telemLen > 0) && ((grid != NULL) || (status != NULL) || (alt != RADIOLIB_APRS_MIC_E_ALTITUDE_UNUSED)))
    {
        // can't have both telemetry and status
        return strRet;
    }

    // prepare buffers
    // char destCallsign[7];
#if !RADIOLIB_STATIC_ONLY
    size_t infoLen = 10;
    if (telemLen > 0)
    {
        infoLen += 1 + telemLen;
    }
    else
    {
        if (grid != NULL)
        {
            infoLen += strlen(grid) + 2;
        }
        if (status != NULL)
        {
            infoLen += strlen(status);
        }
        if (alt > RADIOLIB_APRS_MIC_E_ALTITUDE_UNUSED)
        {
            infoLen += 4;
        }
    }
    char *info = new char[infoLen];
#else
    char info[RADIOLIB_STATIC_ARRAY_SIZE];
#endif
    size_t infoPos = 0;

    // the following is based on APRS Mic-E implementation by https://github.com/omegat
    // as discussed in https://github.com/jgromes/RadioLib/issues/430

    // latitude first, because that is in the destination field
    float lat_abs = abs(lat);
    int lat_deg = (int)lat_abs;
    int lat_min = (lat_abs - (float)lat_deg) * 60.0f;
    int lat_hun = (((lat_abs - (float)lat_deg) * 60.0f) - lat_min) * 100.0f;
    destCallsign[0] = lat_deg / 10;
    destCallsign[1] = lat_deg % 10;
    destCallsign[2] = lat_min / 10;
    destCallsign[3] = lat_min % 10;
    destCallsign[4] = lat_hun / 10;
    destCallsign[5] = lat_hun % 10;

    // next, add the extra bits
    if (type & 0x04)
    {
        destCallsign[0] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
    }
    if (type & 0x02)
    {
        destCallsign[1] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
    }
    if (type & 0x01)
    {
        destCallsign[2] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
    }
    if (lat >= 0)
    {
        destCallsign[3] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
    }
    if (lon >= 100 || lon <= -100)
    {
        destCallsign[4] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
    }
    if (lon < 0)
    {
        destCallsign[5] += RADIOLIB_APRS_MIC_E_DEST_BIT_OFFSET;
    }
    destCallsign[6] = '\0';

    // now convert to Mic-E characters to get the "callsign"
    for (uint8_t i = 0; i < 6; i++)
    {
        if (destCallsign[i] <= 9)
        {
            destCallsign[i] += '0';
        }
        else
        {
            destCallsign[i] += ('A' - 10);
        }
    }

    // setup the information field
    info[infoPos++] = RADIOLIB_APRS_MIC_E_GPS_DATA_CURRENT;

    // encode the longtitude
    float lon_abs = abs(lon);
    int32_t lon_deg = (int32_t)lon_abs;
    int32_t lon_min = (lon_abs - (float)lon_deg) * 60.0f;
    int32_t lon_hun = (((lon_abs - (float)lon_deg) * 60.0f) - lon_min) * 100.0f;

    if (lon_deg <= 9)
    {
        info[infoPos++] = lon_deg + 118;
    }
    else if (lon_deg <= 99)
    {
        info[infoPos++] = lon_deg + 28;
    }
    else if (lon_deg <= 109)
    {
        info[infoPos++] = lon_deg + 8;
    }
    else
    {
        info[infoPos++] = lon_deg - 72;
    }

    if (lon_min <= 9)
    {
        info[infoPos++] = lon_min + 88;
    }
    else
    {
        info[infoPos++] = lon_min + 28;
    }

    info[infoPos++] = lon_hun + 28;

    // now the speed and heading - this gets really weird
    int32_t speed_hun_ten = speed / 10;
    int32_t speed_uni = speed % 10;
    int32_t head_hun = heading / 100;
    int32_t head_ten_uni = heading % 100;

    if (speed <= 199)
    {
        info[infoPos++] = speed_hun_ten + 'l';
    }
    else
    {
        info[infoPos++] = speed_hun_ten + '0';
    }

    info[infoPos++] = speed_uni * 10 + head_hun + 32;
    info[infoPos++] = head_ten_uni + 28;
    info[infoPos++] = symbol;
    info[infoPos++] = table;

    // onto the optional stuff - check telemetry first
    if (telemLen > 0)
    {
        if (telemLen == 2)
        {
            info[infoPos++] = RADIOLIB_APRS_MIC_E_TELEMETRY_LEN_2;
        }
        else
        {
            info[infoPos++] = RADIOLIB_APRS_MIC_E_TELEMETRY_LEN_5;
        }
        for (uint8_t i = 0; i < telemLen; i++)
        {
            sprintf(&(info[infoPos]), "%02X", telem[i]);
            infoPos += 2;
        }
    }
    else
    {
        if (grid != NULL)
        {
            memcpy(&(info[infoPos]), grid, strlen(grid));
            infoPos += strlen(grid);
            info[infoPos++] = '/';
            info[infoPos++] = 'G';
        }
        if (status != NULL)
        {
            info[infoPos++] = ' ';
            memcpy(&(info[infoPos]), status, strlen(status));
            infoPos += strlen(status);
        }
        if (alt > RADIOLIB_APRS_MIC_E_ALTITUDE_UNUSED)
        {
            // altitude is offset by -10 km
            int32_t alt_val = alt + 10000;

            // ... and encoded in base 91 for some reason
            info[infoPos++] = (alt_val / 8281) + 33;
            info[infoPos++] = ((alt_val % 8281) / 91) + 33;
            info[infoPos++] = ((alt_val % 8281) % 91) + 33;
            info[infoPos++] = '}';
        }
    }
    info[infoPos++] = '\0';

    strRet = String(info);
    return strRet;
}

String trk_gps_postion(String comment)
{
    String rawData = "";
    String lat, lon;
    double nowLat, nowLng;
    char rawTNC[300];
    char aprs_table, aprs_symbol;
    // char timestamp[10];
    struct tm tmstruct;
    double dist, course, speed;
    time_t nowTime;

    memset(rawTNC, 0, sizeof(rawTNC));
    // getLocalTime(&tmstruct, 5000);
    // sprintf(timestamp, "%02d%02d%02d%02d", (tmstruct.tm_mon + 1), tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min);
    time(&nowTime);
    // nowTime = gps.time.value();
    if (lastTimeStamp == 0)
        lastTimeStamp = nowTime;

    // lastTimeStamp=10000;
    // nowTime=lastTimeStamp+1800;
    time_t tdiff = nowTime - lastTimeStamp;

    // aprs_table = config.aprs_table;
    // aprs_symbol = config.aprs_symbol;
    if (config.trk_smartbeacon)
    {
        if (SB_SPEED < config.trk_lspeed)
        {
            aprs_table = config.trk_symstop[0];
            aprs_symbol = config.trk_symstop[1];
            SB_SPEED = 0;
        }
        else
        {
            aprs_table = config.trk_symmove[0];
            aprs_symbol = config.trk_symmove[1];
        }
    }
    else
    {
        aprs_table = config.trk_symbol[0];
        aprs_symbol = config.trk_symbol[1];
    }

    if (gps.location.isValid()) // && (gps.hdop.hdop() < 10.0))
    {
        nowLat = gps.location.lat();
        nowLng = gps.location.lng();

        uint16_t spdKnot;
        if (LastLng == 0 || LastLat == 0)
        {
            course = gps.course.deg();
            spdKnot = (uint16_t)gps.speed.knots();
        }
        else
        {
            dist = aprsParse.distance(LastLng, LastLat, nowLng, nowLat);
            course = aprsParse.direction(LastLng, LastLat, nowLng, nowLat);
            if (dist > 50.0F)
                dist = 0;
            if (tdiff > 10 && (nowTime > lastTimeStamp))
                speed = dist / ((double)tdiff / 3600);
            else
                speed = 0.0F;

            if (speed > 999)
                speed = 999.0F;
            // uint16_t spdMph=(uint16_t)(speed / 1.609344);
            spdKnot = (uint16_t)(speed * 0.53996F);
            if (spdKnot > 94)
                spdKnot = 0;
        }

        LastLat = nowLat;
        LastLng = nowLng;
        lastTimeStamp = nowTime;

        if (config.trk_compress)
        { // Compress DATA

            // String compPosition = compress_position(nowLat, nowLng, gps.altitude.feet(), course, spdKnot, aprs_table, aprs_symbol, (gps.satellites.value() > 3));
            //  ESP_LOGE("GPS", "Compress=%s", aprs_position);
            if ((strlen(config.trk_item) >= 3) || (config.trk_timestamp))
            {
                String compPosition = compress_position(nowLat, nowLng, gps.altitude.feet(), course, spdKnot, aprs_table, aprs_symbol, (gps.satellites.value() > 3));
                char object[10];
                memset(object, 0x20, 10);
                memcpy(object, config.trk_item, strlen(config.trk_item));
                object[9] = 0;
                if (config.trk_timestamp)
                {
                    String timeStamp = getTimeStamp();
                    sprintf(rawTNC, ";%s*%s%s", object, timeStamp, compPosition.c_str());
                }
                else
                {
                    sprintf(rawTNC, ")%s!%s", config.trk_item, compPosition.c_str());
                }
            }
            else
            {
                char destCallsign[8];
                String compPosition = compressMicE(destCallsign, nowLat, nowLng, (uint16_t)course, spdKnot, config.trk_mice_type, NULL, 0, NULL, NULL, (int32_t)gps.altitude.meters(), aprs_table, aprs_symbol);
                String tnc2Raw = "";
                char *strtmp = (char *)calloc(300, sizeof(char));
                if (strtmp)
                {
                    memset(strtmp, 0, 300);
                    if (config.trk_ssid == 0)
                        sprintf(strtmp, "%s>%s", config.trk_mycall, destCallsign);
                    else
                        sprintf(strtmp, "%s-%d>%s", config.trk_mycall, config.trk_ssid, destCallsign);
                    tnc2Raw = String(strtmp);
                    free(strtmp);
                }
                if (config.trk_path < 5)
                {
                    if (config.trk_path > 0)
                        tnc2Raw += "-" + String(config.trk_path);
                }
                else
                {
                    tnc2Raw += ",";
                    tnc2Raw += getPath(config.trk_path);
                }
                tnc2Raw += ":";
                tnc2Raw += String(compPosition);
                tnc2Raw += comment + String(config.trk_comment);
                return tnc2Raw;
            }
        }
        else
        { // None compress DATA
            int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
            char lon_ew = 'E';
            char lat_ns = 'N';
            if (nowLat < 0)
                lat_ns = 'S';
            if (nowLng < 0)
                lon_ew = 'W';
            DD_DDDDDtoDDMMSS(nowLat, &lat_dd, &lat_mm, &lat_ss);
            DD_DDDDDtoDDMMSS(nowLng, &lon_dd, &lon_mm, &lon_ss);
            char csd_spd[8];
            memset(csd_spd, 0, sizeof(csd_spd));
            sprintf(csd_spd, "%03d/%03d", (int)gps.course.deg(), (int)gps.speed.knots());
            if (strlen(config.trk_item) >= 3)
            {
                char object[10];
                memset(object, 0x20, 10);
                memcpy(object, config.trk_item, strlen(config.trk_item));
                object[9] = 0;
                if (config.trk_timestamp)
                {
                    String timeStamp = getTimeStamp();
                    sprintf(rawTNC, ";%s*%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", object, timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
                }
                else
                {
                    sprintf(rawTNC, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", config.trk_item, lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
                }
            }
            else
            {
                if (config.trk_timestamp)
                {
                    String timeStamp = getTimeStamp();
                    sprintf(rawTNC, "/%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
                }
                else
                {
                    sprintf(rawTNC, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c%s", lat_dd, lat_mm, lat_ss, lat_ns, aprs_table, lon_dd, lon_mm, lon_ss, lon_ew, aprs_symbol, csd_spd);
                }
            }
            if (config.trk_altitude)
            {

                if (gps.altitude.isValid())
                {
                    char strAltitude[10];
                    memset(strAltitude, 0, sizeof(strAltitude));
                    sprintf(strAltitude, "/A=%06d", (int)gps.altitude.feet());
                    strcat(rawTNC, strAltitude);
                }
            }
        }
    }
    else
    {
        sprintf(rawTNC, ">%s ", config.trk_item);
    }

    String tnc2Raw = "";
    char *strtmp = (char *)calloc(300, sizeof(char));
    if (strtmp)
    {
        memset(strtmp, 0, 300);
        if (config.trk_ssid == 0)
            sprintf(strtmp, "%s>APE32A", config.trk_mycall);
        else
            sprintf(strtmp, "%s-%d>APE32A", config.trk_mycall, config.trk_ssid);
        tnc2Raw = String(strtmp);
        free(strtmp);
    }
    if (config.trk_path < 5)
    {
        if (config.trk_path > 0)
            tnc2Raw += "-" + String(config.trk_path);
    }
    else
    {
        tnc2Raw += ",";
        tnc2Raw += getPath(config.trk_path);
    }
    tnc2Raw += ":";
    tnc2Raw += String(rawTNC);
    tnc2Raw += comment + String(config.trk_comment);
    return tnc2Raw;
}

String trk_fix_position(String comment)
{
    char loc[100];
    String tnc2Raw = "";
    memset(loc, 0, sizeof(loc));
    if (config.trk_compress)
    { // Compress DATA

        String compPosition = compress_position(config.trk_lat, config.trk_lon, (int)(config.trk_alt * 3.28F), 0, 0, config.trk_symbol[0], config.trk_symbol[1], true);
        // ESP_LOGE("GPS", "Compress=%s", aprs_position);
        if (strlen(config.trk_item) >= 3)
        {
            char object[10];
            memset(object, 0x20, 10);
            memcpy(object, config.trk_item, strlen(config.trk_item));
            object[9] = 0;
            if (config.trk_timestamp)
            {
                String timeStamp = getTimeStamp();
                sprintf(loc, ";%s*%s%s", object, timeStamp, compPosition.c_str());
            }
            else
            {
                sprintf(loc, ")%s!%s", config.trk_item, compPosition.c_str());
            }
        }
        else
        {
            if (config.trk_timestamp)
            {
                String timeStamp = getTimeStamp();
                sprintf(loc, "/%s%s", timeStamp, compPosition.c_str());
            }
            else
            {
                sprintf(loc, "!%s", compPosition.c_str());
            }
        }
    }
    else
    {
        int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;

        char lon_ew = 'E';
        char lat_ns = 'N';
        if (config.trk_lat < 0)
            lat_ns = 'S';
        if (config.trk_lon < 0)
            lon_ew = 'W';

        DD_DDDDDtoDDMMSS(config.trk_lat, &lat_dd, &lat_mm, &lat_ss);
        DD_DDDDDtoDDMMSS(config.trk_lon, &lon_dd, &lon_mm, &lon_ss);

        if (strlen(config.trk_item) >= 3)
        {
            char object[10];
            memset(object, 0x20, 10);
            memcpy(object, config.trk_item, strlen(config.trk_item));
            object[9] = 0;
            if (config.trk_timestamp)
            {
                String timeStamp = getTimeStamp();
                sprintf(loc, ";%s*%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", object, timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
            }
            else
            {
                sprintf(loc, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", config.trk_item, lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
            }
        }
        else
        {
            if (config.trk_timestamp)
            {
                String timeStamp = getTimeStamp();
                sprintf(loc, "/%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
            }
            else
            {
                sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.trk_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.trk_symbol[1]);
            }
        }

        if (config.trk_alt > 0)
        {
            char strAltitude[12];
            memset(strAltitude, 0, sizeof(strAltitude));
            sprintf(strAltitude, "/A=%06d", (int)(config.trk_alt * 3.28F));
            strcat(loc, strAltitude);
        }
    }

    char *strtmp = (char *)calloc(300, sizeof(char));
    if (strtmp)
    {
        memset(strtmp, 0, 300);
        if (config.trk_ssid == 0)
            sprintf(strtmp, "%s>APE32A", config.trk_mycall);
        else
            sprintf(strtmp, "%s-%d>APE32A", config.trk_mycall, config.trk_ssid);
        tnc2Raw = String(strtmp);
        free(strtmp);
    }
    if (config.trk_path < 5)
    {
        if (config.trk_path > 0)
            tnc2Raw += "-" + String(config.trk_path);
    }
    else
    {
        tnc2Raw += ",";
        tnc2Raw += getPath(config.trk_path);
    }
    tnc2Raw += ":";
    tnc2Raw += String(loc);
    tnc2Raw += comment + String(config.trk_comment);
    return tnc2Raw;
}

String igate_position(double lat, double lon, double alt, String comment)
{
    String tnc2Raw = "";
    int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
    char loc[100];
    char lon_ew = 'E';
    char lat_ns = 'N';
    if (lat < 0)
        lat_ns = 'S';
    if (lon < 0)
        lon_ew = 'W';
    memset(loc, 0, sizeof(loc));
    DD_DDDDDtoDDMMSS(lat, &lat_dd, &lat_mm, &lat_ss);
    DD_DDDDDtoDDMMSS(lon, &lon_dd, &lon_mm, &lon_ss);
    char strAltitude[12];
    memset(strAltitude, 0, sizeof(strAltitude));
    if (alt > 0)
    {
        sprintf(strAltitude, "/A=%06d", (int)(alt * 3.28F));
    }
    if (strlen(config.igate_object) >= 3)
    {
        char object[10];
        memset(object, 0x20, 10);
        memcpy(object, config.igate_object, strlen(config.igate_object));
        object[9] = 0;
        if (config.igate_timestamp)
        {
            String timeStamp = getTimeStamp();
            sprintf(loc, ";%s*%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", object, timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
        }
        else
        {
            sprintf(loc, ")%s!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", config.igate_object, lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
        }
    }
    else
    {
        if (config.igate_timestamp)
        {
            String timeStamp = getTimeStamp();
            sprintf(loc, "/%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", timeStamp.c_str(), lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
        }
        else
        {
            sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.igate_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.igate_symbol[1]);
        }
    }
    char *strtmp = (char *)calloc(300, sizeof(char));
    if (strtmp)
    {
        memset(strtmp, 0, 300);
        if (config.aprs_ssid == 0)
            sprintf(strtmp, "%s>APE32A", config.aprs_mycall);
        else
            sprintf(strtmp, "%s-%d>APE32A", config.aprs_mycall, config.aprs_ssid);
        tnc2Raw = String(strtmp);
        free(strtmp);
    }

    if (config.igate_path < 5)
    {
        if (config.igate_path > 0)
            tnc2Raw += "-" + String(config.igate_path);
    }
    else
    {
        tnc2Raw += ",";
        tnc2Raw += getPath(config.igate_path);
    }
    tnc2Raw += ":";
    tnc2Raw += String(loc);
    tnc2Raw += String(config.igate_phg) + String(strAltitude);
    // tnc2Raw += comment + " " + String(config.igate_comment);
    return tnc2Raw;
}

String digi_position(double lat, double lon, double alt, String comment)
{
    String tnc2Raw = "";
    int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
    char loc[100];
    char lon_ew = 'E';
    char lat_ns = 'N';
    if (lat < 0)
        lat_ns = 'S';
    if (lon < 0)
        lon_ew = 'W';
    memset(loc, 0, sizeof(loc));
    DD_DDDDDtoDDMMSS(lat, &lat_dd, &lat_mm, &lat_ss);
    DD_DDDDDtoDDMMSS(lon, &lon_dd, &lon_mm, &lon_ss);
    char strAltitude[12];
    memset(strAltitude, 0, sizeof(strAltitude));
    if (alt > 0)
    {
        sprintf(strAltitude, "/A=%06d", (int)(alt * 3.28F));
    }
    if (config.digi_timestamp)
    {
        String timeStamp = getTimeStamp();
        sprintf(loc, "/%s%02d%02d.%02d%c%c%03d%02d.%02d%c%c", timeStamp, lat_dd, lat_mm, lat_ss, lat_ns, config.digi_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.digi_symbol[1]);
    }
    else
    {
        sprintf(loc, "!%02d%02d.%02d%c%c%03d%02d.%02d%c%c", lat_dd, lat_mm, lat_ss, lat_ns, config.digi_symbol[0], lon_dd, lon_mm, lon_ss, lon_ew, config.digi_symbol[1]);
    }
    char *strtmp = (char *)calloc(300, sizeof(char));
    if (strtmp)
    {
        memset(strtmp, 0, 300);
        if (config.digi_ssid == 0)
            sprintf(strtmp, "%s>APE32A", config.digi_mycall);
        else
            sprintf(strtmp, "%s-%d>APE32A", config.digi_mycall, config.digi_ssid);
        tnc2Raw = String(strtmp);
        free(strtmp);
    }

    if (config.digi_path < 5)
    {
        if (config.digi_path > 0)
            tnc2Raw += "-" + String(config.digi_path);
    }
    else
    {
        tnc2Raw += ",";
        tnc2Raw += getPath(config.digi_path);
    }
    tnc2Raw += ":";
    tnc2Raw += String(loc);
    tnc2Raw += String(config.digi_phg) + String(strAltitude);
    // tnc2Raw += comment + " " + String(config.digi_comment);
    return tnc2Raw;
}

void tracker_status(char *text)
{
    char name[50];

    if (config.trk_ssid > 0)
        sprintf(name, "%s-%d>APE32A", config.trk_mycall, config.trk_ssid);
    else
        sprintf(name, "%s>APE32A", config.trk_mycall);

    String tnc2Raw = String(name);
    if (config.trk_path < 5)
    {
        if (config.trk_path > 0)
            tnc2Raw += "-" + String(config.trk_path);
    }
    else
    {
        tnc2Raw += ",";
        tnc2Raw += getPath(config.trk_path);
    }
    tnc2Raw += ":>";
    tnc2Raw += String(text);

    uint8_t SendMode = 0;
    if (config.trk_loc2rf)
        SendMode |= RF_CHANNEL;
    if (config.trk_loc2inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0, SendMode);
}

void igate_status(char *text)
{
    char name[50];

    if (config.aprs_ssid > 0)
        sprintf(name, "%s-%d>APE32A", config.aprs_mycall, config.aprs_ssid);
    else
        sprintf(name, "%s>APE32A", config.aprs_mycall);

    String tnc2Raw = String(name);
    if (config.igate_path < 5)
    {
        if (config.igate_path > 0)
            tnc2Raw += "-" + String(config.igate_path);
    }
    else
    {
        tnc2Raw += ",";
        tnc2Raw += getPath(config.igate_path);
    }
    tnc2Raw += ":>";
    tnc2Raw += String(text);

    uint8_t SendMode = 0;
    if (config.igate_loc2rf)
        SendMode |= RF_CHANNEL;
    if (config.igate_loc2inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0, SendMode);
}

void digi_status(char *text)
{
    char name[50];

    if (config.digi_ssid > 0)
        sprintf(name, "%s-%d>APE32A", config.digi_mycall, config.digi_ssid);
    else
        sprintf(name, "%s>APE32A", config.digi_mycall);

    String tnc2Raw = String(name);
    if (config.digi_path < 5)
    {
        if (config.digi_path > 0)
            tnc2Raw += "-" + String(config.digi_path);
    }
    else
    {
        tnc2Raw += ",";
        tnc2Raw += getPath(config.digi_path);
    }
    tnc2Raw += ":>";
    tnc2Raw += String(text);

    uint8_t SendMode = 0;
    if (config.digi_loc2rf)
        SendMode |= RF_CHANNEL;
    if (config.digi_loc2inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0, SendMode);
}

String wx_report(double lat, double lon, double alt, String comment)
{
    String tnc2Raw = "";
    int lat_dd, lat_mm, lat_ss, lon_dd, lon_mm, lon_ss;
    char loc[300];
    char lon_ew = 'E';
    char lat_ns = 'N';
    if (lat < 0)
        lat_ns = 'S';
    if (lon < 0)
        lon_ew = 'W';
    memset(loc, 0, sizeof(loc));
    DD_DDDDDtoDDMMSS(lat, &lat_dd, &lat_mm, &lat_ss);
    DD_DDDDDtoDDMMSS(lon, &lon_dd, &lon_mm, &lon_ss);
    // char strAltitude[12];
    // memset(strAltitude, 0, sizeof(strAltitude));
    // if (alt > 0)
    // {
    //     sprintf(strAltitude, "/A=%06d", (int)(alt * 3.28F));
    // }
    if (config.wx_gps)
    {
        if (gps.satellites.value() > 5 && gps.hdop.hdop() < 5)
            mslAltitude = alt;
    }
    else
    {
        mslAltitude = alt;
    }
    if (strlen(config.wx_object) >= 3)
    {
        char object[10];
        memset(object, 0x20, 10);
        memcpy(object, config.wx_object, strlen(config.wx_object));
        object[9] = 0;
        if (config.wx_timestamp)
        {
            String timeStamp = getTimeStamp();
            sprintf(loc, ";%s*%s%02d%02d.%02d%c/%03d%02d.%02d%c_", object, timeStamp.c_str(), lat_dd, lat_mm, lat_ss, lat_ns, lon_dd, lon_mm, lon_ss, lon_ew);
        }
        else
        {
            sprintf(loc, ")%s!%02d%02d.%02d%c/%03d%02d.%02d%c_", config.wx_object, lat_dd, lat_mm, lat_ss, lat_ns, lon_dd, lon_mm, lon_ss, lon_ew);
        }
    }
    else
    {
        if (config.wx_timestamp)
        {
            String timeStamp = getTimeStamp();
            sprintf(loc, "@%s%02d%02d.%02d%c/%03d%02d.%02d%c_", timeStamp.c_str(), lat_dd, lat_mm, lat_ss, lat_ns, lon_dd, lon_mm, lon_ss, lon_ew);
        }
        else
        {
            sprintf(loc, "!%02d%02d.%02d%c/%03d%02d.%02d%c_", lat_dd, lat_mm, lat_ss, lat_ns, lon_dd, lon_mm, lon_ss, lon_ew);
        }
    }
    char *strtmp = (char *)calloc(500, sizeof(char));
    if (strtmp)
    {
        memset(strtmp, 0, 500);


        if (config.wx_ssid == 0)
            sprintf(strtmp, "%s>APE32A", config.wx_mycall);
        else
            sprintf(strtmp, "%s-%d>APE32A", config.wx_mycall, config.wx_ssid);
        tnc2Raw = String(strtmp);
        free(strtmp);
    }

    if (config.wx_path < 5)
    {
        if (config.wx_path > 0)
            tnc2Raw += "-" + String(config.wx_path);
    }
    else
    {
        tnc2Raw += ",";
        tnc2Raw += getPath(config.wx_path);
    }
    tnc2Raw += ":";
    tnc2Raw += String(loc);
    char WxRaw[500];
    memset(WxRaw, 0, 500);
    getRawWx(&WxRaw[0]);
    tnc2Raw += String(WxRaw);
    tnc2Raw += comment + String(config.wx_comment);
    return tnc2Raw;
}

int packet2Raw(String &tnc2, AX25Msg &Packet)
{
    if (Packet.len < 1)
        return 0;
    tnc2 = String(Packet.src.call);
    if (Packet.src.ssid > 0)
    {
        tnc2 += String(F("-"));
        tnc2 += String(Packet.src.ssid);
    }
    tnc2 += String(F(">"));
    tnc2 += String(Packet.dst.call);
    if (Packet.dst.ssid > 0)
    {
        tnc2 += String(F("-"));
        tnc2 += String(Packet.dst.ssid);
    }
    for (int i = 0; i < Packet.rpt_count; i++)
    {
        if (i >= 8)
            break;
        tnc2 += String(",");
        tnc2 += String(Packet.rpt_list[i].call);
        if (Packet.rpt_list[i].ssid > 0)
        {
            tnc2 += String("-");
            tnc2 += String(Packet.rpt_list[i].ssid);
        }
        if (Packet.rpt_flags & (1 << i))
            tnc2 += "*";
    }
    tnc2 += String(F(":"));
    tnc2 += String((const char *)Packet.info);

    return tnc2.length();
}

long sendTimer = 0;
int btn_count = 0;
long timeCheck = 0;
int timeHalfSec = 0;

unsigned long timeTask;
unsigned long timeSec;
char nmea[100];
int nmea_idx = 0;
bool save_mode = false;
bool save_act = false;
bool manualWiFi = false;

void msgBox(String msg)
{
#ifdef OLED
    int x = msg.length() * 6;
    int x1 = (SCREEN_WIDTH / 2) - (x / 2);
    int y1 = (SCREEN_HEIGHT / 2);

    display.fillRect(x1 - 7, y1 - 6, x + 14, 28, BLACK);
    display.drawRect(x1 - 5, y1 - 4, x + 10, 24, WHITE);
    display.drawLine(x1 - 3, y1 + 21, x1 + (x + 10) - 4, y1 + 21, WHITE);
    display.drawLine(x1 + (x + 10) - 4, y1 - 2, x1 + (x + 10) - 4, y1 + 21, WHITE);
    display.setCursor(x1, y1 + 5);
    display.print(msg);
    display.display();
#endif
}

void loop()
{
    if (millis() > timeTask)
    {
        timeTask = millis() + 10000;
        unsigned long upT = (millis() / 1000) - upTimeStamp;
        convertSecondsToDHMS(nmea, upT);
        log_d("Task process APRS=%iuS\t NETWORK=%iuS\t GPS=%iuS\t SERIAL=%iuS\n", timerAPRS, timerNetwork, timerGPS, timerSerial);
        log_d("upTime %s Free heap: %s KB \tWiFi:%s ,RSSI:%s dBm", nmea, String((float)ESP.getFreeHeap() / 1000, 1).c_str(), String(WiFi.SSID()).c_str(), String(WiFi.RSSI()).c_str());
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    // if (digitalRead(BOOT_PIN) == LOW)
    // {
    //     btn_count++;
    //     if (btn_count > 1000) // Push BOOT 10sec
    //     {
    //         digitalWrite(LED_RX_PIN, HIGH);
    //         digitalWrite(LED_TX_PIN, HIGH);
    //     }
    // }
    // else
    // {
    //     if (btn_count > 0)
    //     {
    //         // Serial.printf("btn_count=%dms\n", btn_count * 10);
    //         if (btn_count > 1000) // Push BOOT 10sec to Factory Default
    //         {
    //             // digitalWrite(LED_RX, LOW);
    //             // digitalWrite(LED_TX, LOW);
    //             defaultConfig();
    //             log_d("SYSTEM REBOOT NOW!");
    //             esp_restart();
    //         }
    //         else
    //         {
    //             showDisp = true;
    //             timeSec = timeHalfSec = millis();
    //             // if (oledSleepTimeout > 0)
    //             //{
    //             curTab++;
    //             if (curTab > 3)
    //                 curTab = 0;
    //             //}
    //         }
    //         btn_count = 0;
    //     }
    // }
    if (digitalRead(BOOT_PIN) == LOW)
    {
        btn_count++;
        if (btn_count > 200) // Push BOOT >2sec
        {
            btn_count = 0;
            if (curTab == 0)
            {
                showDisp = true;
                timeSec = timeHalfSec = millis();
                if (gps_mode)
                    gps_mode = false;
                else
                    gps_mode = true;
                msgBox("GPS SWAP");
                while (digitalRead(BOOT_PIN) == LOW)
                {
                    delay(100);
                }
            }
            else if (curTab == 4) // System display
            {
                msgBox("POWER OFF");
                while (digitalRead(BOOT_PIN) == LOW)
                {
                    delay(100);
                }
                //PowerOff();
                esp_deep_sleep_start();
            }
            else if (curTab == 6) // Wifi display
            {
                if (manualWiFi)
                {
                    manualWiFi = false;
                    msgBox("WiFi OFF");
                    vTaskSuspend(taskNetworkHandle);
                    WiFi.disconnect(true); // Disconnect from the network
                    WiFi.persistent(false);
                    WiFi.mode(WIFI_OFF); // Switch WiFi off
                }
                else
                {
                    manualWiFi = true;
                    msgBox("WiFi ON");
                    vTaskResume(taskNetworkHandle);
                }
                while (digitalRead(BOOT_PIN) == LOW)
                {
                    delay(100);
                }
            }
            else if (curTab == 1) // Status display
            {
                msgBox("MANUAL TX");
                while (digitalRead(BOOT_PIN) == LOW)
                {
                    delay(100);
                }
                EVENT_TX_POSITION = 6;
                tx_interval = config.trk_interval;
            }
        }
        else
        {
        }
    }
    else
    {
        if (btn_count > 0)
        {
            log_d("btn_count=%d", btn_count);
            if (btn_count > 25) // Push BOOT 10sec to Factory Default
            {
                // digitalWrite(LED_RX, LOW);
                // digitalWrite(LED_TX, LOW);
                // defaultConfig();
                // log_d("SYSTEM REBOOT NOW!");
                // esp_restart();
            }
            else
            {
                StandByTick = millis() + (config.pwr_stanby_delay * 1000);
                if (save_mode)
                    save_act = true;
                showDisp = true;
                timeSec = timeHalfSec = millis();
                // if (oledSleepTimeout > 0)
                //{
                curTab++;
                if (curTab > 7)
                    curTab = 0;
                //}
                log_d("curTab=%d", curTab);
            }
        }
        btn_count = 0;
    }

#ifdef OLED
    // Popup Display
    if (config.oled_enable == true)
    {
        if (dispBuffer.getCount() > 0)
        {
            if (millis() > timeHalfSec)
            {
                char tnc2[300];
                dispBuffer.pop(&tnc2);
                dispWindow(String(tnc2), 0, false);
                timeHalfSec = millis() + (config.dispDelay * 1000);
                oledSleepTimeout = millis() + (config.oled_timeout * 1000);
            }
        }
        else
        {
            if (!getTransmit())
            {
                if (queTxDisp.getCount() > 0)
                { // have tx info display
                    txDisp txs;
                    if (queTxDisp.pop(&txs))
                    {
                        dispTxWindow(txs);
                        delay(1000);
                        // if (menuSel == 0)
                        //     curTabOld = curTab + 1;
                    }
                }
            }

            // Sleep display
            if (millis() > timeHalfSec)
            {
                if (millis() > timeSec && timeHalfSec > 0)
                {
                    timeSec = millis() + 10000;
                    showDisp = true;
                    // timeHalfSec = 0;
                    // oledSleepTimeout = millis() + (config.oled_timeout * 1000);
                }
                else
                {
                    if (millis() > oledSleepTimeout && oledSleepTimeout > 0)
                    {
                        showDisp = false;
                        timeHalfSec = 0;
                        oledSleepTimeout = 0;
                        i2c_busy = true;
                        display.clearDisplay();
                        display.display();
                        i2c_busy = false;
                    }
                }
            }
        }

        if (showDisp)
        {
            showDisp = false;
            timeHalfSec = millis();
            // oledSleepTimeout = millis() + (config.oled_timeout * 1000);
            switch (curTab)
            {
            case 1:
                statisticsDisp();
                timeSec = millis() + 10000;
                break;
            case 2:
                pkgLastDisp();
                timeSec = millis() + 10000;
                break;
            case 3:
                pkgCountDisp();
                timeSec = millis() + 10000;
                break;
            case 4:
                systemDisp();
                timeSec = millis() + 10000;
                break;
            case 5:
                radioDisp();
                timeSec = millis() + 10000;
                break;
            case 6:
                wifiDisp();
                timeSec = millis() + 10000;
                break;
            case 7:
                sensorDisp();
                timeSec = millis() + 10000;
                break;
            case 0:
                gpsDisp();
                timeSec = millis() + 1000;
            }
        }
    }
#endif

    // Tick one secound
    if (millis() > timeCheck)
    {
        // esp_task_wdt_reset();
        timeCheck = millis() + 1000;
        if (ESP.getFreeHeap() < 30000)
            esp_restart();
        // Serial.println(String(ESP.getFreeHeap()));
    
        if((config.reset_timeout > 0) && (millis() > autoResetTimeout)){
            autoResetTimeout=millis() + (config.reset_timeout*60000);
            log_d("Auto Reset System");
            vTaskSuspendAll();
            WiFi.disconnect(true); // Disconnect from the network
            WiFi.persistent(false);
            WiFi.mode(WIFI_OFF); // Switch WiFi off
            //PowerOff();
            esp_restart();
        }
    }
}

String sendIsAckMsg(String toCallSign, int msgId)
{
    char str[300];
    char call[11];
    int i;
    memset(&call[0], 0, 11);
    sprintf(call, "%s-%d", config.aprs_mycall, config.aprs_ssid);
    strcpy(&call[0], toCallSign.c_str());
    i = strlen(call);
    for (; i < 9; i++)
        call[i] = 0x20;
    memset(&str[0], 0, 300);

    sprintf(str, "%s-%d>APE32A%s::%s:ack%d", config.aprs_mycall, config.aprs_ssid, VERSION, call, msgId);
    //	client.println(str);
    return String(str);
}

void sendIsPkg(char *raw)
{
    char str[300];
    sprintf(str, "%s-%d>APE32A%s:%s", config.aprs_mycall, config.aprs_ssid, VERSION, raw);
    // client.println(str);
    String tnc2Raw = String(str);
    if (aprsClient.connected())
        aprsClient.println(tnc2Raw); // Send packet to Inet
    if (config.digi_en)
        pkgTxPush(str, strlen(str), 0, RF_CHANNEL);
}

void sendIsPkgMsg(char *raw)
{
    char str[300];
    char call[11];
    int i;
    memset(&call[0], 0, 11);
    if (config.aprs_ssid == 0)
        sprintf(call, "%s", config.aprs_mycall);
    else
        sprintf(call, "%s-%d", config.aprs_mycall, config.aprs_ssid);
    i = strlen(call);
    for (; i < 9; i++)
        call[i] = 0x20;

    if (config.aprs_ssid == 0)
        sprintf(str, "%s>APE32A::%s:%s", config.aprs_mycall, call, raw);
    else
        sprintf(str, "%s-%d>APE32A::%s:%s", config.aprs_mycall, config.aprs_ssid, call, raw);

    String tnc2Raw = String(str);
    if (aprsClient.connected())
        aprsClient.println(tnc2Raw); // Send packet to Inet
    // if (config.tnc && config.tnc_digi)
    //     pkgTxUpdate(str, 0);
    // APRS_sendTNC2Pkt(tnc2Raw); // Send packet to RF
}

void sendTelemetry_0(char *raw, bool header)
{
    char str[300];
    char call[11];
    int i;
    memset(&call[0], 0, 11);
    if (strlen(config.trk_item) > 3)
    {
        sprintf(call, "%s", config.trk_item);
    }
    else
    {
        if (config.tlm0_ssid == 0)
            sprintf(call, "%s", config.tlm0_mycall);
        else
            sprintf(call, "%s-%d", config.tlm0_mycall, config.tlm0_ssid);
    }
    i = strlen(call);
    for (; i < 9; i++)
        call[i] = 0x20;

    if (header)
    {
        if (config.tlm0_ssid == 0)
        {
            if (config.tlm0_path < 5)
            {
                if (config.tlm0_path > 0)
                    sprintf(str, "%s>APE32A-%d::%s:%s ", config.tlm0_mycall, config.tlm0_path, call, raw);
                else
                    sprintf(str, "%s>APE32A::%s:%s ", config.tlm0_mycall, call, raw);
            }
            else
            {
                sprintf(str, "%s>APE32A,%s::%s:%s ", config.tlm0_mycall, getPath(config.tlm0_path).c_str(), call, raw);
            }
        }
        else
        {
            if (config.tlm0_path < 5)
            {
                if (config.tlm0_path > 0)
                    sprintf(str, "%s-%d>APE32A-%d::%s:%s ", config.tlm0_mycall, config.tlm0_ssid, config.tlm0_path, call, raw);
                else
                    sprintf(str, "%s-%d>APE32A::%s:%s ", config.tlm0_mycall, config.tlm0_ssid, call, raw);
            }
            else
            {
                sprintf(str, "%s-%d>APE32A,%s::%s:%s ", config.tlm0_mycall, config.tlm0_ssid, getPath(config.tlm0_path).c_str(), call, raw);
            }
        }
    }
    else
    {
        if (config.tlm0_ssid == 0)
        {
            if (config.tlm0_path < 5)
            {
                if (config.tlm0_path > 0)
                    sprintf(str, "%s>APE32A-%d:%s", config.tlm0_mycall, config.tlm0_path, raw);
                else
                    sprintf(str, "%s>APE32A:%s", config.tlm0_mycall, raw);
            }
            else
            {
                sprintf(str, "%s>APE32A,%s:%s", config.tlm0_mycall, getPath(config.tlm0_path).c_str(), raw);
            }
        }
        else
        {
            if (config.tlm0_path < 5)
            {
                if (config.tlm0_path > 0)
                    sprintf(str, "%s-%d>APE32A-%d:%s", config.tlm0_mycall, config.tlm0_ssid, config.tlm0_path, raw);
                else
                    sprintf(str, "%s-%d>APE32A:%s", config.tlm0_mycall, config.tlm0_ssid, raw);
            }
            else
            {
                sprintf(str, "%s-%d>APE32A,%s:%s", config.tlm0_mycall, config.tlm0_ssid, getPath(config.tlm0_path).c_str(), raw);
            }
        }
    }

    uint8_t SendMode = 0;
    if (config.tlm0_2rf)
        SendMode |= RF_CHANNEL;
    if (config.tlm0_2inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(str, strlen(str), 0, SendMode);

#if 1
    if (config.tlm0_2rf)
    { // TLM SEND TO RF
        SendMode |= RF_CHANNEL;
        // char *rawP = (char *)malloc(rawData.length());
        //  rawData.toCharArray(rawP, rawData.length());
        // memcpy(rawP, rawData.c_str(), rawData.length());
        pkgTxPush(str, strlen(str), 0, RF_CHANNEL);
        // pushTxDisp(TXCH_RF, "TX DIGI POS", sts);
        // free(rawP);
    }
    if (config.tlm0_2inet)
    { // TLM SEND TO APRS-IS

        if (aprsClient.connected())
        {
            status.txCount++;
            aprsClient.printf("%s\r\n", str); // Send packet to Inet
            // pushTxDisp(TXCH_TCP, "TX DIGI POS", sts);
        }
    }
#endif
}

void sendTelemetry_trk(char *raw)
{
    char str[300];
    char call[11];
    int i;
    memset(&call[0], 0, 11);
    if (strlen(config.trk_item) > 3)
    {
        sprintf(call, "%s", config.trk_item);
    }
    else
    {
        if (config.trk_ssid == 0)
            sprintf(call, "%s", config.trk_mycall);
        else
            sprintf(call, "%s-%d", config.trk_mycall, config.trk_ssid);
    }
    i = strlen(call);
    for (; i < 9; i++)
        call[i] = 0x20;

    if (config.trk_ssid == 0)
    {
        if (config.trk_path < 5)
        {
            if (config.trk_path > 0)
                sprintf(str, "%s>APE32A-%d::%s:%s ", config.trk_mycall, config.trk_path, call, raw);
            else
                sprintf(str, "%s>APE32A::%s:%s ", config.trk_mycall, call, raw);
        }
        else
        {
            sprintf(str, "%s>APE32A,%s::%s:%s ", config.trk_mycall, getPath(config.trk_path).c_str(), call, raw);
        }
    }
    else
    {
        if (config.trk_path < 5)
        {
            if (config.trk_path > 0)
                sprintf(str, "%s-%d>APE32A-%d::%s:%s ", config.trk_mycall, config.trk_ssid, config.trk_path, call, raw);
            else
                sprintf(str, "%s-%d>APE32A::%s:%s ", config.trk_mycall, config.trk_ssid, call, raw);
        }
        else
        {
            sprintf(str, "%s-%d>APE32A,%s::%s:%s ", config.trk_mycall, config.trk_ssid, getPath(config.trk_path).c_str(), call, raw);
        }
    }

    uint8_t SendMode = 0;
    if (config.trk_loc2rf)
        SendMode |= RF_CHANNEL;
    if (config.trk_loc2inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(str, strlen(str), 0, SendMode);

    // if (config.trk_loc2rf)
    // { // TLM SEND TO RF
    //     pkgTxPush(str, strlen(str), 0);
    // }
    // if (config.trk_loc2inet)
    // { // TLM SEND TO APRS-IS
    //     if (aprsClient.connected())
    //     {
    //         status.txCount++;
    //         aprsClient.printf("%s\r\n", str); // Send packet to Inet
    //         delay(2000);
    //     }
    // }
}

void sendTelemetry_igate(char *raw)
{
    char str[300];
    char call[11];
    int i;
    memset(&call[0], 0, 11);
    if (strlen(config.igate_object) > 3)
    {
        sprintf(call, "%s", config.igate_object);
    }
    else
    {
        if (config.aprs_ssid == 0)
            sprintf(call, "%s", config.aprs_mycall);
        else
            sprintf(call, "%s-%d", config.aprs_mycall, config.aprs_ssid);
    }
    i = strlen(call);
    for (; i < 9; i++)
        call[i] = 0x20;

    if (config.aprs_ssid == 0)
    {
        if (config.igate_path < 5)
        {
            if (config.igate_path > 0)
                sprintf(str, "%s>APE32A-%d::%s:%s ", config.aprs_mycall, config.igate_path, call, raw);
            else
                sprintf(str, "%s>APE32A::%s:%s ", config.aprs_mycall, call, raw);
        }
        else
        {
            sprintf(str, "%s>APE32A,%s::%s:%s ", config.aprs_mycall, getPath(config.igate_path).c_str(), call, raw);
        }
    }
    else
    {
        if (config.igate_path < 5)
        {
            if (config.igate_path > 0)
                sprintf(str, "%s-%d>APE32A-%d::%s:%s ", config.aprs_mycall, config.aprs_ssid, config.igate_path, call, raw);
            else
                sprintf(str, "%s-%d>APE32A::%s:%s ", config.aprs_mycall, config.aprs_ssid, call, raw);
        }
        else
        {
            sprintf(str, "%s-%d>APE32A,%s::%s:%s ", config.aprs_mycall, config.aprs_ssid, getPath(config.igate_path).c_str(), call, raw);
        }
    }

    uint8_t SendMode = 0;
    if (config.igate_loc2rf)
        SendMode |= RF_CHANNEL;
    if (config.igate_loc2inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(str, strlen(str), 0, SendMode);
    // if (config.igate_loc2rf)
    // { // TLM SEND TO RF
    //     pkgTxPush(str, strlen(str), 0);
    // }
    // if (config.igate_loc2inet)
    // { // TLM SEND TO APRS-IS
    //     if (aprsClient.connected())
    //     {
    //         status.txCount++;
    //         aprsClient.printf("%s\r\n", str); // Send packet to Inet
    //         delay(2000);
    //     }
    // }
}

void sendTelemetry_digi(char *raw)
{
    char str[300];
    char call[11];
    int i;
    memset(&call[0], 0, 11);

    if (config.digi_ssid == 0)
        sprintf(call, "%s", config.digi_mycall);
    else
        sprintf(call, "%s-%d", config.digi_mycall, config.digi_ssid);
    i = strlen(call);
    for (; i < 9; i++)
        call[i] = 0x20;

    if (config.digi_ssid == 0)
    {
        if (config.digi_path < 5)
        {
            if (config.digi_path > 0)
                sprintf(str, "%s>APE32A-%d::%s:%s ", config.digi_mycall, config.digi_path, call, raw);
            else
                sprintf(str, "%s>APE32A::%s:%s ", config.digi_mycall, call, raw);
        }
        else
        {
            sprintf(str, "%s>APE32A,%s::%s:%s ", config.digi_mycall, getPath(config.digi_path).c_str(), call, raw);
        }
    }
    else
    {
        if (config.digi_path < 5)
        {
            if (config.digi_path > 0)
                sprintf(str, "%s-%d>APE32A-%d::%s:%s ", config.digi_mycall, config.digi_ssid, config.digi_path, call, raw);
            else
                sprintf(str, "%s-%d>APE32A::%s:%s ", config.digi_mycall, config.digi_ssid, call, raw);
        }
        else
        {
            sprintf(str, "%s-%d>APE32A,%s::%s:%s ", config.digi_mycall, config.digi_ssid, getPath(config.digi_path).c_str(), call, raw);
        }
    }

    uint8_t SendMode = 0;
    if (config.digi_loc2rf)
        SendMode |= RF_CHANNEL;
    if (config.digi_loc2inet)
        SendMode |= INET_CHANNEL;
    pkgTxPush(str, strlen(str), 0, SendMode);
    // if (config.digi_loc2rf)
    // { // TLM SEND TO RF
    //     pkgTxPush(str, strlen(str), 0);
    // }
    // if (config.digi_loc2inet)
    // { // TLM SEND TO APRS-IS
    //     if (aprsClient.connected())
    //     {
    //         status.txCount++;
    //         aprsClient.printf("%s\r\n", str); // Send packet to Inet
    //         delay(2000);
    //     }
    // }
}

RTC_DATA_ATTR statusType statOld;
uint8_t getState(int ch)
{
    int val = 0;
    switch (ch)
    {
    case 0: // none
        val = 0;
        break;
    case 1: // allCount
        val = status.allCount - statOld.allCount;
        statOld.allCount = status.allCount;
        break;
    case 2: // rf2inet
        val = status.rf2inet - statOld.rf2inet;
        statOld.rf2inet = status.rf2inet;
        break;
    case 3: // inet2rf
        val = status.inet2rf - statOld.inet2rf;
        statOld.inet2rf = status.inet2rf;
        break;
    case 4: // digi
        val = status.digiCount - statOld.digiCount;
        statOld.digiCount = status.digiCount;
        break;
    case 5: // Drop
        val = status.dropCount - statOld.dropCount;
        statOld.dropCount = status.dropCount;
        break;
    }
    if (val < 0)
        val = 0;
    return val;
}

bool getBits(int ch)
{
    bool val = false;
    switch (ch)
    {
    case 0: // none
        val = 0;
        break;
    case 1: // IGATE Enable
        val = config.igate_en;
        break;
    case 2: // DIGI Enable
        val = config.digi_en;
        break;
    case 3: // WX Enable
        val = config.wx_en;
        break;
    case 4: // Sat Enable
        val = 0;
        break;
    case 5: // APRS-IS Status
        if (aprsClient.connected())
            val = 1;
        break;
    case 6: // VPN Status
        val = wireguard_active();
        break;
    case 7: // 4G LTE
        val = 0;
        break;
    case 8: // FX.25
        val = (config.fx25_mode > 0) ? 1 : 0;
        break;
    }
    return val;
}

void getTelemetry_0()
{
    systemTLM.A1 = getState(config.tml0_data_channel[0]);
    systemTLM.A2 = getState(config.tml0_data_channel[1]);
    systemTLM.A3 = getState(config.tml0_data_channel[2]);
    systemTLM.A4 = getState(config.tml0_data_channel[3]);
    systemTLM.A5 = getState(config.tml0_data_channel[4]);
    systemTLM.BITS = 0;
    if (getBits(config.tml0_data_channel[5]))
        systemTLM.BITS |= 0x01;
    if (getBits(config.tml0_data_channel[6]))
        systemTLM.BITS |= 0x02;
    if (getBits(config.tml0_data_channel[7]))
        systemTLM.BITS |= 0x04;
    if (getBits(config.tml0_data_channel[8]))
        systemTLM.BITS |= 0x08;
    if (getBits(config.tml0_data_channel[9]))
        systemTLM.BITS |= 0x10;
    if (getBits(config.tml0_data_channel[10]))
        systemTLM.BITS |= 0x20;
    if (getBits(config.tml0_data_channel[11]))
        systemTLM.BITS |= 0x40;
    if (getBits(config.tml0_data_channel[12]))
        systemTLM.BITS |= 0x80;
}

String getPath(int idx)
{
    String ret = "";
    switch (idx)
    {
    case 0: // OFF
        ret = "";
        break;
    case 1: // DST-TRACE1
    case 2: // DST-TRACE2
    case 3: // DST-TRACE3
    case 4: // DST-TRACE4
        ret = "DST" + String(idx);
        break;
    case 5: // TRACE1-1
        ret = "TRACE1-1";
        break;
    case 6:
        ret = "TRACE2-2";
        break;
    case 7:
        ret = "TRACE3-3";
        break;
    case 8:
        ret = "WIDE1-1";
        break;
    case 9:
        ret = "RFONLY";
        break;
    case 10:
        ret = "RELAY";
        break;
    case 11:
        ret = "GATE";
        break;
    case 12:
        ret = "ECHO";
        break;
    case 13: // UserDefine1
        ret = String(config.path[0]);
        break;
    case 14: // UserDefine2
        ret = String(config.path[1]);
        break;
    case 15: // UserDefine3
        ret = String(config.path[2]);
        break;
    case 16: // UserDefine4
        ret = String(config.path[3]);
        break;
    default:
        ret = "WIDE1-1";
        break;
    }
    return ret;
}

void GPS_INIT()
{
    // SerialGPS.begin(config.gnss_baudrate, SERIAL_8N1, config.gnss_rx_gpio, config.gnss_tx_gpio);
    //  SerialGPS.begin(config.gnss_baudrate, EspSoftwareSerial::SWSERIAL_8N1, config.gnss_rx_gpio, config.gnss_tx_gpio);
    // SerialGPS.flush();
    //  while (SerialGPS.available() > 0)
    //      SerialGPS.read();
}

WiFiClient gnssClient;
// WiFiClient tncClient;

extern AsyncWebSocket ws_gnss;

// void taskGPSActive()
// {
//     int c;
//     // log_d("GNSS Init");
//     nmea_idx = 0;

//     // if (config.gnss_enable)
//     // {
//     //     if ((config.gnss_channel > 0) && (config.gnss_channel < 4))
//     //     {
//     //         if (strstr("AT", config.gnss_at_command) >= 0)
//     //         {
//     //             if (config.gnss_channel == 1)
//     //             {
//     //                 Serial.println(config.gnss_at_command);
//     //             }
//     //             else if (config.gnss_channel == 2)
//     //             {
//     //                 Serial1.println(config.gnss_at_command);
//     //             }
//     //             else if (config.gnss_channel == 3)
//     //             {
//     //                 Serial2.println(config.gnss_at_command);
//     //             }
//     //         }
//     //     }
//     // }
//     // for (;;)
//     {
//         if (config.gnss_enable)
//         {
//             if ((config.gnss_channel > 0) && (config.gnss_channel < 4))
//             {
//                 do
//                 {
//                     c = -1;
//                     if (config.gnss_channel == 1)
//                     {
//                         c = Serial.read();
//                     }
//                     else if (config.gnss_channel == 2)
//                     {
//                         c = Serial1.read();
//                     }
// #ifdef __XTENSA__
//                     else if (config.gnss_channel == 3)
//                     {
//                         c = Serial2.read();
//                     }
// #endif
//                     if (c > -1)
//                     {
//                         gps.encode((char)c);
//                         if (webServiceBegin == false)
//                         {
//                             if (nmea_idx > 195)
//                             {
//                                 nmea_idx = 0;
//                                 memset(nmea, 0, sizeof(nmea));
//                                 // SerialGNSS->flush();
//                             }
//                             else
//                             {
//                                 nmea[nmea_idx++] = (char)c;
//                                 if ((char)c == '\r' || (char)c == '\n')
//                                 {
//                                     nmea[nmea_idx] = 0;
//                                     if (nmea_idx > 5)
//                                     {
//                                         if (ws_gnss.enabled() && !ws_gnss.getClients().isEmpty())
//                                         // if (webServiceBegin == false)
//                                         {
//                                             handle_ws_gnss(nmea, nmea_idx);
//                                         }
//                                         // log_d("%s",nmea);
//                                     }
//                                     nmea_idx = 0;
//                                     break;
//                                 }
//                             }
//                         }
//                         //}
//                     }
//                     else
//                     {
//                         break;
//                     }
//                 } while (1);
//             }
//             else if (config.gnss_channel == 4)
//             { // TCP
//                 if (WiFi.isConnected())
//                 {
//                     if (!gnssClient.connected())
//                     {
//                         gnssClient.connect(config.gnss_tcp_host, config.gnss_tcp_port);
//                         log_d("GNSS TCP ReConnect to %s:%d", config.gnss_tcp_host, config.gnss_tcp_port);
//                         delay(3000);
//                     }
//                     else
//                     {
//                         while (gnssClient.available())
//                         {
//                             c = (char)gnssClient.read();
//                             // Serial.print(c);
//                             gps.encode(c);
//                             if (webServiceBegin == false)
//                             {
//                                 if (nmea_idx > 195)
//                                 {
//                                     nmea_idx = 0;
//                                     memset(nmea, 0, sizeof(nmea));
//                                 }
//                                 else
//                                 {
//                                     nmea[nmea_idx++] = c;
//                                     if (c == '\r' || c == '\n')
//                                     {
//                                         nmea[nmea_idx] = 0;
//                                         if (nmea_idx > 5)
//                                         {
//                                             if (ws_gnss.enabled() && !ws_gnss.getClients().isEmpty())
//                                             {
//                                                 handle_ws_gnss(nmea, nmea_idx);
//                                             }
//                                             // log_d("%s",nmea);
//                                         }
//                                         nmea_idx = 0;
//                                     }
//                                 }
//                             }
//                         }
//                     }
//                 }
//             }

//             if (firstGpsTime && gps.time.isValid())
//             {
//                 if (gps.time.isUpdated())
//                 {
//                     time_t timeGps = getGpsTime(); // Local gps time
//                     if (timeGps > 1700000000 && timeGps < 2347462800)
//                     {
//                         setTime(timeGps);
//                         time_t rtc = timeGps;
//                         timeval tv = {rtc, 0};
//                         timezone tz = {TZ_SEC + DST_MN, 0};
//                         settimeofday(&tv, &tz);
// #ifdef DEBUG
//                         log_d("\nSET GPS Timestamp = %u Year=%d\n", timeGps, year());
// #endif
//                         // firstGpsTime = false;
//                         firstGpsTime = false;
//                         if (startTime == 0)
//                             startTime = now();
//                     }
//                     else
//                     {
//                         startTime = 0;
//                     }
//                 }
//             }
//         }
//     }
// }

void taskGPS(void *pvParameters)
{
    int c;
    log_d("GNSS Init");
    nmea_idx = 0;

    if (config.gnss_enable)
    {
        if ((config.gnss_channel > 0) && (config.gnss_channel < 4))
        {
            if (strstr("AT", config.gnss_at_command) != NULL)
            {
                if (config.gnss_channel == 1)
                {
                    Serial.println(config.gnss_at_command);
                }
                else if (config.gnss_channel == 2)
                {
                    Serial1.println(config.gnss_at_command);
                }
#if SOC_UART_NUM > 2
                else if (config.gnss_channel == 3)
                {
                    Serial2.println(config.gnss_at_command);
                }
#endif
            }
        }
    }
    for (;;)
    {
        timerGPS = micros() - timerGPS_old;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timerGPS_old = micros();

        if (config.gnss_enable)
        {
            if ((config.gnss_channel > 0) && (config.gnss_channel < 4))
            {
                do
                {
                    c = -1;
                    if (config.gnss_channel == 1)
                    {
                        c = Serial.read();
                    }
                    else if (config.gnss_channel == 2)
                    {
                        c = Serial1.read();
                    }
#if SOC_UART_NUM > 2
                    else if (config.gnss_channel == 3)
                    {
                        c = Serial2.read();
                    }
#endif
                    if (c > -1)
                    {
                        gps.encode((char)c);
                        if (webServiceBegin == false)
                        {
                            if (nmea_idx > 99)
                            {
                                nmea_idx = 0;
                                memset(nmea, 0, sizeof(nmea));
                                // SerialGNSS->flush();
                            }
                            else
                            {
                                nmea[nmea_idx++] = (char)c;
                                if ((char)c == '\r' || (char)c == '\n')
                                {
                                    // nmea[nmea_idx++] = 0;
                                    if (nmea_idx > 10)
                                    {
                                        if (ws_gnss.enabled() && !ws_gnss.getClients().isEmpty())
                                        // if (webServiceBegin == false)
                                        {
                                            handle_ws_gnss(nmea, nmea_idx);
                                        }
                                        // log_d("%s",nmea);
                                    }
                                    nmea_idx = 0;
                                    memset(nmea, 0, sizeof(nmea));
                                    vTaskDelay(50 / portTICK_PERIOD_MS);
                                    break;
                                }
                            }
                        }
                        //}
                    }
                    else
                    {
                        break;
                    }
                } while (1);
            }
            else if (config.gnss_channel == 4)
            { // TCP
                if (WiFi.isConnected())
                {
                    if (!gnssClient.connected())
                    {
                        gnssClient.connect(config.gnss_tcp_host, config.gnss_tcp_port);
                        log_d("GNSS TCP ReConnect to %s:%d", config.gnss_tcp_host, config.gnss_tcp_port);
                        delay(3000);
                    }
                    else
                    {
                        while (gnssClient.available())
                        {
                            c = (char)gnssClient.read();
                            // Serial.print(c);
                            gps.encode(c);
                            if (webServiceBegin == false)
                            {
                                if (nmea_idx > 99)
                                {
                                    nmea_idx = 0;
                                    memset(nmea, 0, sizeof(nmea));
                                }
                                else
                                {
                                    // nmea[nmea_idx++] = c;
                                    if (c == '\r' || c == '\n')
                                    {
                                        nmea[nmea_idx++] = 0;
                                        if (nmea_idx > 10)
                                        {
                                            if (ws_gnss.enabled() && !ws_gnss.getClients().isEmpty())
                                            {
                                                handle_ws_gnss(nmea, nmea_idx);
                                            }
                                            // log_d("%s",nmea);
                                        }
                                        nmea_idx = 0;
                                        memset(nmea, 0, sizeof(nmea));
                                        vTaskDelay(50 / portTICK_PERIOD_MS);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (firstGpsTime && gps.time.isValid())
            {
                if (gps.time.isUpdated())
                {
                    time_t timeGps = getGpsTime(); // Local gps time
                    if (timeGps > 1700000000 && timeGps < 2347462800)
                    {
                        setTime(timeGps);
                        time_t rtc = timeGps;
                        timeval tv = {rtc, 0};
                        timezone tz = {TZ_SEC + DST_MN, 0};
                        settimeofday(&tv, &tz);
#ifdef DEBUG
                        log_d("\nSET GPS Timestamp = %u Year=%d\n", timeGps, year());
#endif
                        // firstGpsTime = false;
                        firstGpsTime = false;
                        if (startTime == 0)
                            startTime = now();
                    }
                    else
                    {
                        startTime = 0;
                    }
                }
            }
        }
    }
}

void taskSerial(void *pvParameters)
{
    String raw;
    int c;
    char rawP[500];
    char call[11];
    log_d("Serial task Init");
    nmea_idx = 0;
    if (config.ext_tnc_enable)
    {
        #ifdef ARDUINO_USB_MODE
            Serial.setTimeout(10);
        #endif
        if (config.ext_tnc_channel == 1)
        {
            Serial0.setTimeout(10);

        }
        else if (config.ext_tnc_channel == 2)
        {
            Serial1.setTimeout(10);
        }
#if SOC_UART_NUM > 2        
        else if (config.ext_tnc_channel == 3)
        {
            Serial2.setTimeout(10);
        }
#endif
    }
    if (config.wx_en)
    {
        //         if (config.wx_channel == 1)
        //         {
        // #if ARDUINO_USB_CDC_ON_BOOT
        //             Serial0.setTimeout(10);
        // #else
        //             Serial.setTimeout(10);
        // #endif
        //         }
        //         else if (config.wx_channel == 2)
        //         {
        //             Serial1.setTimeout(10);
        //         }
        //         else if (config.wx_channel == 3)
        //         {
        //             // Serial2.setTimeout(10);
        //         }
    }
    for (;;)
    {
        timerSerial = millis() - timerSerial_old;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timerSerial_old = micros();

        if (config.wx_en)
        {
            //             if (config.wx_channel > 0 && config.wx_channel < 4)
            //             {
            //                 String wx = "";
            //                 if (config.wx_channel == 1)
            //                 {
            // #if ARDUINO_USB_CDC_ON_BOOT
            //                     wx = Serial.readString();
            // #else
            //                     wx = Serial.readString();
            // #endif
            //                 }
            //                 else if (config.wx_channel == 2)
            //                 {
            //                     wx = Serial1.readString();
            //                 }
            //                 else if (config.wx_channel == 3)
            //                 {
            //                     // wx = Serial2.readString();
            //                 }
            //                 // if (wx!="")
            //                 //{
            //                 // while (SerialWX->available())
            //                 //{
            //                 // String wx = SerialWX->readString();
            //                 if (wx != "" && wx.indexOf("DATA:") >= 0)
            //                 {
            //                     log_d("WX Raw >> %d", wx.c_str());
            //                     getCSV2Wx(wx);
            //                 }
            //                 //}
            //                 //}
            //             }
            // else if(config.wx_channel == 4){
            //     bool result=getM702Modbus(modbus);
            // }
        }

        if (config.ext_tnc_enable && (config.ext_tnc_mode > 0 && config.ext_tnc_mode < 5))
        {
            if (config.ext_tnc_mode == 1)
            { // KISS
                // KISS MODE
                do
                {
                    c = -1;
                    if (config.ext_tnc_channel == 1)
                    {
                        if(Serial0.available()) c = Serial0.read();
                    }
                    else if (config.ext_tnc_channel == 2)
                    {
                        if(Serial1.available()) c = Serial1.read();
                    }
                    #ifdef ARDUINO_USB_MODE
                    else if (config.ext_tnc_channel == 4)
                    {
                        if(Serial.available()) c = Serial.read();
                    }
                    #endif
#if SOC_UART_NUM > 2                    
                    else if (config.ext_tnc_channel == 3)
                    {
                        if(Serial2.available()) c = Serial2.read();
                    }
#endif
                    if (c > -1)
                        kiss_serial((uint8_t)c);
                    else
                        break;
                } while (c > -1);
            }
            else if (config.ext_tnc_mode == 2)
            { // TNC2RAW
                raw.clear();
                if (config.ext_tnc_channel == 1)
                {
                     if(Serial0.available()) raw = Serial0.readStringUntil(0x0D);
                }
                else if (config.ext_tnc_channel == 2)
                {
                    if(Serial1.available()) raw = Serial1.readStringUntil(0x0D);
                }
                #ifdef ARDUINO_USB_MODE
                else if (config.ext_tnc_channel == 4)
                {
                    if(Serial.available()) raw = Serial.readStringUntil(0x0D);
                }
                #endif
#if SOC_UART_NUM > 2                
                else if (config.ext_tnc_channel == 3)
                {
                    if(Serial2.available()) raw = Serial2.readStringUntil(0x0D);
                }
#endif
                log_d("Ext TNC2RAW RX:%s", raw.c_str());
                String src_call = raw.substring(0, raw.indexOf('>'));
                if ((src_call != "") && (src_call.length() < 10) && (raw.length() < sizeof(rawP)))
                {
                    memset(call, 0, sizeof(call));
                    strcpy(call, src_call.c_str());
                    strcpy(rawP, raw.c_str());
                    uint16_t type = pkgType((const char *)rawP);
                    pkgListUpdate(call, rawP, type, 1, 0);
                    if (config.rf2inet && aprsClient.connected())
                    {
                        // RF->INET
                        aprsClient.write(&rawP[0], strlen(rawP)); // Send binary frame packet to APRS-IS (aprsc)
                        aprsClient.write("\r\n");                 // Send CR LF the end frame packet
                        igateTLM.RX++;
                    }
                }
            }
            else if (config.ext_tnc_mode == 3)
            { // YAESU FTM-350,FTM-400
                String info = "";
                if (config.ext_tnc_channel == 1)
                {
                    if(Serial0.available()) info = Serial0.readString();
                }
                else if (config.ext_tnc_channel == 2)
                {
                    if(Serial1.available()) info = Serial1.readString();
                }
                #ifdef ARDUINO_USB_MODE
                else if (config.ext_tnc_channel == 4)
                {
                    if(Serial.available()) info = Serial.readString();
                }
                #endif
#if SOC_UART_NUM > 2                
                else if (config.ext_tnc_channel == 3)
                {
                    if(Serial2.available()) info = Serial2.readString();
                }
#endif
                //  log_d("Ext Yaesu Packet >> %s",info.c_str());
                int ed = info.indexOf(" [");
                if (info != "" && ed > 10)
                {
                    raw.clear();
                    raw = info.substring(0, ed);
                    int st = info.indexOf(">:");
                    if (st > ed)
                    {
                        int idx = 0;
                        st += 2;
                        for (int i = 0; i < 5; i++)
                        {
                            if (info.charAt(st + i) == 0x0A || info.charAt(st + i) == 0x0D)
                            {
                                idx++;
                            }
                            else
                            {
                                break;
                            }
                        }
                        st += idx;
                        ed = info.indexOf(0x0D, st + 1);
                        if (ed > info.length())
                            ed = info.length();
                        if (ed > st)
                        {
                            raw += ":" + info.substring(st, ed);

                            String src_call = raw.substring(0, raw.indexOf('>'));
                            if ((src_call != "") && (src_call.length() < 11) && (raw.length() < sizeof(rawP)))
                            {
                                memset(call, 0, sizeof(call));
                                strcpy(call, src_call.c_str());
                                memset(rawP, 0, sizeof(rawP));
                                strcpy(rawP, raw.c_str());
                                log_d("Yaesu Packet: CallSign:%s RAW:%s", call, rawP);
                                // String hstr="";
                                // for(int i=0;i<raw.length();i++){
                                //     hstr+=" "+String(rawP[i],HEX);
                                // }
                                // log_d("HEX: %s",hstr.c_str());
                                uint16_t type = pkgType((const char *)rawP);
                                pkgListUpdate(call, rawP, type, 1, 0);
                                if (config.rf2inet && aprsClient.connected())
                                {
                                    // RF->INET
                                    aprsClient.write(&rawP[0], strlen(rawP)); // Send binary frame packet to APRS-IS (aprsc)
                                    aprsClient.write("\r\n");                 // Send CR LF the end frame packet
                                    // status.rf2inet++;
                                    // igateTLM.RF2INET++;
                                    igateTLM.RX++;
                                }
                            }
                        }
                    }
                }
            }
            //}
        }
    }
}

long timeSlot;
unsigned long iGatetickInterval;
unsigned long WxInterval;
bool initInterval = true;
int trkTlmInvCount = 0;
int igateTlmInvCount = 0;
int digiTlmInvCount = 0;
void taskAPRS(void *pvParameters)
{
    //	long start, stop;
    char sts[50];
    unsigned long tickInterval = 0;
    unsigned long DiGiInterval = 0;

    unsigned long igateSTSInterval = 0;
    unsigned long digiSTSInterval = 0;
    unsigned long trkSTSInterval = 0;

    uint16_t type = 0;
    bool newIGatePkg = false;
    bool newDigiPkg = false;
    uint8_t *buf;
    uint16_t size = 0;
    int8_t peak = 0;
    int8_t valley = 0;
    uint8_t signalLevel = 0;
    uint8_t fixed = 0;
    uint16_t mV = 0;

    
    // PacketBuffer.clean();
    adcEn = 0;
    dacEn = 0;

    APRS_setCallsign(config.aprs_mycall, config.aprs_ssid);
    sendTimer = millis() - (config.igate_interval * 1000) + 30000;
    igateTLM.TeleTimeout = millis() + 60000; // 1Min

    timeSlot = millis();
    timeAprs = 0;
    tx_interval = config.trk_interval;
    tx_counter = tx_interval - 10;

    initInterval = true;
    AFSKInitAct = true;
    log_d("Task APRS has been start");
    for (;;)
    {

        // if (adcEn == 1)
        // {
        //     AFSK_TimerEnable(true);
        //     adcEn = 0;
        // }
        // else if (adcEn == -1)
        // {
        //     AFSK_TimerEnable(false);
        //     adcEn = 0;
        // }

        if (dacEn == 1)
        {
            DAC_TimerEnable(true);
            dacEn = 0;
        }
        else if (dacEn == -1)
        {
            DAC_TimerEnable(false);
            dacEn = 0;
        }
        long now = millis();
        // wdtSensorTimer = now;
        // time_t timeStamp;
        // time(&timeStamp);
        if (initInterval)
        {
            tickInterval = WxInterval = DiGiInterval = igateSTSInterval = iGatetickInterval = digiSTSInterval = trkSTSInterval = millis() + 10000;
            systemTLM.ParmTimeout = millis() + 20000;
            systemTLM.TeleTimeout = millis() + 30000;
            initInterval = false;
            tx_interval = config.trk_interval;
            tx_counter = tx_interval - 10;
        }
        timerAPRS = micros() - timerAPRS_old;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timerAPRS_old = micros();

#ifdef BLUETOOTH
#if !defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32C6)
        if (NuSerial.isConnected())
        {
            if (NuSerial.available())
            {
                log_d("Bluetooth RX Data: %d Byte", NuSerial.available());
                if (config.bt_mode == 1)
                { // TNC2RAW MODE
                    String rxValue = NuSerial.readString();
                    uint8_t SendMode = TNC_CHANNEL;
                    if (config.igate_loc2rf)
                        SendMode |= RF_CHANNEL;
                    if (config.igate_loc2inet)
                        SendMode |= INET_CHANNEL;
                    pkgTxPush(rxValue.c_str(), rxValue.length(), 1, SendMode);
                }
                else if (config.bt_mode == 2)
                {
                    // KISS MODE
                    size_t num = NuSerial.available();
                    for (int i = 0; i < num; i++)
                    {
                        kiss_serial((uint8_t)NuSerial.read());
                    }
                }
            }
        }
#else
        if (SerialBT.available())
        {
            log_d("Bluetooth RX Data: %d Byte", SerialBT.available());
            if (config.bt_mode == 1)
            { // TNC2RAW MODE
                String rxValue = SerialBT.readString();
                uint8_t SendMode = TNC_CHANNEL;
                if (config.igate_loc2rf)
                    SendMode |= RF_CHANNEL;
                if (config.igate_loc2inet)
                    SendMode |= INET_CHANNEL;
                pkgTxPush(rxValue.c_str(), rxValue.length(), 1, SendMode);
            }
            else if (config.bt_mode == 2)
            {
                // KISS MODE
                size_t num = SerialBT.available();
                for (int i = 0; i < num; i++)
                {
                    kiss_serial((uint8_t)SerialBT.read());
                }
            }
        }
#endif
#endif

        // SEND RF in time slot
        // if (now > timeSlot)
        // {
        // Transmit in timeslot if enabled
        pkgTxSend();
        //     timeSlot = millis() + 100;
        // }
        Ax25TransmitBuffer(); // transmit buffer (will return if nothing to be transmitted)
        Ax25TransmitCheck();  // check for pending transmission request

        if (config.trk_en)
        { // TRACKER MODE
            if (config.trk_sts_interval > 10)
            {
                if (millis() > trkSTSInterval)
                {
                    trkSTSInterval = millis() + (config.trk_sts_interval * 1000);
                    tracker_status(config.trk_status);
                }
            }
            if (millis() > tickInterval)
            {
                tickInterval = millis() + 1000;

                tx_counter++;
                // log_d("TRACKER tx_counter=%d\t INTERVAL=%d\n", tx_counter, tx_interval);
                //   Check interval timeout
                if (config.trk_smartbeacon && config.trk_gps)
                {
                    if ((gps.satellites.value() > 3) && (gps.hdop.hdop() < 10))
                    {
                        if (tx_counter > tx_interval)
                        {
                            if (tx_counter > config.trk_mininterval)
                                EVENT_TX_POSITION = 4;
                        }
                        else
                        {
                            if (tx_counter >= (tx_interval + 5))
                            {
                                EVENT_TX_POSITION = 5;
                            }
                        }
                    }
                }
                else if (tx_counter > tx_interval)
                {
                    // if (tx_counter > config.trk_slowinterval)
                    //{
                    EVENT_TX_POSITION = 6;
                    tx_interval = config.trk_interval;
                    //}
                }

                // if (config.trk_gps && gps.speed.isValid() && gps.location.isValid() && gps.course.isValid() && (gps.hdop.hdop() < 10.0) && (gps.satellites.value() > 3))
                // if (config.trk_gps && gps.speed.isValid() && gps.location.isValid() && gps.course.isValid())
                if (config.trk_gps)
                {
                    SB_SPEED_OLD = SB_SPEED;
                    if (gps.satellites.value() > 3 && gps.hdop.hdop() < 10)
                    {
                        SB_SPEED = (unsigned char)gps.speed.kmph();
                        // if (gps.course.isUpdated())
                        if (gps.speed.kmph() > config.trk_lspeed)
                            SB_HEADING = (int16_t)gps.course.deg();
                    }
                    else
                    {
                        if (SB_SPEED > 0)
                            SB_SPEED--;
                    }
                    if (config.trk_smartbeacon) // SMART BEACON CAL
                    {
                        if (SB_SPEED < config.trk_lspeed && SB_SPEED_OLD > config.trk_lspeed) // Speed slow down to STOP
                        {                                                                     // STOPING
                            SB_SPEED_OLD = 0;
                            if (tx_counter > config.trk_mininterval)
                            {
                                EVENT_TX_POSITION = 7;
                                tx_interval = config.trk_slowinterval;
                            }
                        }
                        else
                        {
                            smartbeacon();
                        }
                    }
                    else if (tx_counter > tx_interval)
                    { // send gps location
                        if (gps.location.isValid() && gps.hdop.hdop() < 10)
                        {
                            EVENT_TX_POSITION = 8;
                            tx_interval = config.trk_interval;
                        }
                    }
                }
            }

            if (EVENT_TX_POSITION > 0)
            {
                String rawData;
                String cmn = "";
                Sleep_Activate &= ~ACTIVATE_TRACKER;
                StandByTick = millis() + (5000);
                if (config.trk_tlm_interval > 0)
                {
                    trkTlmInvCount++;
                    if (trkTlmInvCount >= config.trk_tlm_interval)
                    {
                        trkTlmInvCount = 0;
                        if (config.trk_tlm_sensor[0] | config.trk_tlm_sensor[1] | config.trk_tlm_sensor[2] | config.trk_tlm_sensor[3] | config.trk_tlm_sensor[4])
                        {
                            char tlm_result[100];
                            char tlm_data[200];
                            size_t tlm_sz;
                            if ((TLM_SEQ % 100) == 0)
                            {
                                char rawInfo[100];
                                char name[10];
                                sprintf(rawInfo, "PARM.");
                                int i, c = 0;
                                for (i = 0; i < 5; i++)
                                {
                                    if (config.trk_tlm_sensor[i] == 0)
                                    {
                                        c++;
                                        continue;
                                    }
                                    else
                                    {
                                        if (i > 0)
                                            strcat(rawInfo, ",");
                                        sprintf(name, "%s", config.trk_tlm_PARM[i]);
                                        strcat(rawInfo, name);
                                    }
                                }
                                for (int n = c + 8; n > 0; n--)
                                {
                                    strcat(rawInfo, ",");
                                }
                                sendTelemetry_trk(rawInfo);
                                memset(rawInfo, 0, sizeof(rawInfo));
                                sprintf(rawInfo, "UNIT.");
                                c = 0;
                                for (i = 0; i < 5; i++)
                                {
                                    if (config.trk_tlm_sensor[i] == 0)
                                    {
                                        c++;
                                        continue;
                                    }
                                    else
                                    {
                                        if (i > 0)
                                            strcat(rawInfo, ",");
                                        sprintf(name, "%s", config.trk_tlm_UNIT[i]);
                                        strcat(rawInfo, name);
                                    }
                                }
                                for (int n = c + 8; n > 0; n--)
                                {
                                    strcat(rawInfo, ",");
                                }
                                sendTelemetry_trk(rawInfo);
                                memset(rawInfo, 0, sizeof(rawInfo));
                                sprintf(rawInfo, "EQNS.");
                                c = 0;
                                for (i = 0; i < 5; i++)
                                {
                                    if (config.trk_tlm_sensor[i] == 0)
                                    {
                                        c++;
                                        continue;
                                    }
                                    else
                                    {
                                        if (i > 0)
                                            strcat(rawInfo, ",");
                                        if (fmod(config.trk_tlm_EQNS[i][0], 1) == 0)
                                            sprintf(name, "%0.f", config.trk_tlm_EQNS[i][0]);
                                        else
                                            sprintf(name, "%.3f", config.trk_tlm_EQNS[i][0]);
                                        strcat(rawInfo, name);
                                        if (fmod(config.trk_tlm_EQNS[i][1], 1) == 0)
                                            sprintf(name, ",%0.f", config.trk_tlm_EQNS[i][1]);
                                        else
                                            sprintf(name, ",%.3f", config.trk_tlm_EQNS[i][1]);
                                        strcat(rawInfo, name);
                                        if (fmod(config.trk_tlm_EQNS[i][2], 1) == 0)
                                            sprintf(name, ",%0.f", config.trk_tlm_EQNS[i][2]);
                                        else
                                            sprintf(name, ",%.3f", config.trk_tlm_EQNS[i][2]);
                                        strcat(rawInfo, name);
                                    }
                                }
                                for (int n = c; n > 0; n--)
                                {
                                    strcat(rawInfo, ",");
                                    sprintf(name, "0");
                                    strcat(rawInfo, name);
                                    sprintf(name, ",1");
                                    strcat(rawInfo, name);
                                    sprintf(name, ",0");
                                    strcat(rawInfo, name);
                                }
                                // strcat(rawInfo, ",");
                                sendTelemetry_trk(rawInfo);
                            }

                            if (++TLM_SEQ > 8279)
                                TLM_SEQ = 0;
                            memset(tlm_data, 0, 200);
                            memset(tlm_result, 0, 100);
                            int n = 0;
                            sprintf(tlm_data, "%i", TLM_SEQ);
                            for (int s = 0; s < 5; s++)
                            {
                                if (config.trk_tlm_sensor[s] == 0)
                                {
                                    continue;
                                    // strcat(tlm_data, "0");
                                }
                                else
                                {
                                    strcat(tlm_data, ",");
                                    int sen_idx = config.trk_tlm_sensor[s] - 1;
                                    double data = 0;
                                    if (sen[sen_idx].visable)
                                        data = sen[sen_idx].sample;
                                    double precision = pow(10.0f, (double)config.trk_tlm_precision[s]);
                                    int val = (int)((data + config.trk_tlm_offset[s]) * precision);
                                    // log_d("s:%d Data:%.2f /tPresion:%.5f /tOffset:%.5f/t Val:%d",s,sen[sen_idx].sample,precision,config.trk_tlm_offset[s],val);
                                    if (val > 8280)
                                        val = 8280;
                                    if (val < 0)
                                        val = 0;
                                    char strVal[10];
                                    sprintf(strVal, "%i", val);
                                    strcat(tlm_data, strVal);
                                }
                            }
                            // log_d("TLM_DATA:%s",tlm_data);
                            //  sprintf(tlm_data, "%i,%i,%i,%i", TLM_SEQ, (int)(VBat * 100), int(TempNTC * 100), gps.satellites.value());
                            telemetry_base91(tlm_data, tlm_result, tlm_sz);
                            cmn = String(tlm_result);
                        }
                    }
                }

                if (config.trk_rssi)
                {
                    cmn += " ?RSSI";
                }

                if (config.trk_gps) // TRACKER by GPS
                {
                    rawData = trk_gps_postion(cmn);
                    if (config.log & LOG_TRACKER)
                    {
                        logTracker(gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.course.deg());
                    }
                }
                else // TRACKER by FIX position
                {
                    rawData = trk_fix_position(cmn);
                    if (config.log & LOG_TRACKER)
                    {
                        logTracker(config.trk_lat, config.trk_lon, 0, 0);
                    }
                }

                log_d("TRACKER RAW: %s\n", rawData.c_str());
                log_d("TRACKER EVENT_TX_POSITION=%d\t INTERVAL=%d\n", EVENT_TX_POSITION, tx_interval);
                tx_counter = 0;
                EVENT_TX_POSITION = 0;
                last_heading = SB_HEADING;
#if defined OLED || defined ST7735_160x80
                if (config.trk_gps)
                {
                    // if (gps.location.isValid() && (gps.hdop.hdop() < 10.0))
                    sprintf(sts, "POSITION GPS\nSPD %dkPh/%d\nINTERVAL %ds", SB_SPEED, SB_HEADING, tx_interval);
                    // else
                    //     sprintf(sts, "POSITION GPS\nGPS INVALID\nINTERVAL %ds", tx_interval);
                }
                else
                {
                    sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);
                }
                char name[12];
                if (strlen(config.trk_item) > 3)
                {
                    sprintf(name, "%s", config.trk_item);
                }
                else
                {
                    if (config.trk_ssid > 0)
                        sprintf(name, "%s-%d", config.trk_mycall, config.trk_ssid);
                    else
                        sprintf(name, "%s", config.trk_mycall);
                }
#endif
                uint8_t SendMode = 0;
                if (config.trk_loc2rf)
                    SendMode |= RF_CHANNEL;
                if (config.trk_loc2inet)
                    SendMode |= INET_CHANNEL;
                pkgTxPush(rawData.c_str(), rawData.length(), 0, SendMode);

                //                 if (config.trk_loc2rf)
                //                 { // TRACKER SEND TO RF
                //                     char *rawP = (char *)calloc(rawData.length(), sizeof(char));
                //                     memcpy(rawP, rawData.c_str(), rawData.length());
                //                     // rawData.toCharArray(rawP, rawData.length());
                //                     pkgTxPush(rawP, rawData.length(), 0);

#if defined OLED || defined ST7735_160x80
                if(config.oled_enable) pushTxDisp(TXCH_RF, name, sts);
#endif
                //                     free(rawP);
                //                 }
                //                 if (config.trk_loc2inet)
                //                 { // TRACKER SEND TO APRS-IS
                //                     if (aprsClient.connected())
                //                     {
                //                         aprsClient.println(rawData); // Send packet to Inet
                // #if defined OLED || defined ST7735_160x80
                //                         // pushTxDisp(TXCH_TCP, "TX TRACKER", sts);
                // #endif
                //                     }
                //                 }
                rawData.clear();
                cmn.clear();
            }
        }

        // LOAD DATA incomming
        newIGatePkg = false;
        newDigiPkg = false;
        type = 0;
        // if (PacketBuffer.getCount() > 0)
        if (Ax25NewRxFrames())
        {
            if (Ax25ReadNextRxFrame(&buf, &size, &peak, &valley, &signalLevel, &fixed, &mV))
            {
                String tnc2 = "";
                // นำข้อมูลแพ็จเกจจาก TNC ออกจากคิว
                ax25_decode(buf, size, mV, &incomingPacket);
                status.rxCount++;
                if (packet2Raw(tnc2, incomingPacket) > 0)
                {
                    log_d("Peak:%d Valley:%d Signal:%d mV:%d", peak, valley, signalLevel, mV);
                    log_d("RX TNC2: %s", tnc2.c_str());
                    type = pkgType((const char *)incomingPacket.info);
                    newIGatePkg = true;
                    newDigiPkg = true;
                    if (config.ext_tnc_enable)
                    {
                        if (config.ext_tnc_channel > 0 && config.ext_tnc_channel < 5)
                        {
                            if (config.ext_tnc_mode == 1)
                            {
                                // KISS MODE
                                uint8_t pkg[500];
                                int sz = kiss_wrapper(pkg, buf, size);
                                if (config.ext_tnc_channel == 1)
                                {
                                    Serial0.write(pkg, sz);
                                }
                                else if (config.ext_tnc_channel == 2)
                                {
                                    Serial1.write(pkg, sz);
                                }
                                #ifdef ARDUINO_USB_MODE
                                else if (config.ext_tnc_channel == 4)
                                {
                                    Serial.write(pkg, sz);
                                }
                                #endif
                                #if SOC_UART_NUM > 2 
                                else if (config.ext_tnc_channel == 3)
                                {
                                    Serial2.write(pkg, sz);
                                }
                                #endif
                            }
                            else if (config.ext_tnc_mode == 2)
                            {
                                // TNC2
                                if (config.ext_tnc_channel == 1)
                                {
                                    Serial0.println(tnc2);
                                }
                                else if (config.ext_tnc_channel == 2)
                                {
                                    Serial1.println(tnc2);
                                }
                                #ifdef ARDUINO_USB_MODE
                                else if (config.ext_tnc_channel == 4)
                                {
                                    Serial.println(tnc2);
                                }
                                #endif
                                #if SOC_UART_NUM > 2 
                                else if (config.ext_tnc_channel == 3)
                                {
                                    Serial2.println(tnc2);
                                }
                                #endif
                            }
                        }
                    }
#ifdef BLUETOOTH
                    if (config.bt_master)
                    { // Output TNC2RAW to BT Serial
                      // SerialBT.println(tnc2);
                        if (config.bt_mode == 1)
                        {
                            char *rawP = (char *)malloc(tnc2.length());
                            memcpy(rawP, tnc2.c_str(), tnc2.length());
#if defined(CONFIG_IDF_TARGET_ESP32)
                            SerialBT.write((uint8_t *)rawP, tnc2.length());
#else
                            if (NuSerial.isConnected())
                            {
                                NuSerial.write((uint8_t *)rawP, tnc2.length());
                            }
#endif
                            free(rawP);
                        }
                        else if (config.bt_mode == 2)
                        { // KISS
                            uint8_t pkg[500];
                            int sz = kiss_wrapper(pkg);
#if defined(CONFIG_IDF_TARGET_ESP32)
                            SerialBT.write(pkg, sz);
#else
                            if (NuSerial.isConnected())
                            {
                                NuSerial.write(pkg, sz);
                            }
#endif
                        }
                    }
#endif

#ifdef MQTT
                if (config.en_mqtt && clientMQTT.connected() && (config.mqtt_topic_flag & MQTT_TOPIC_TNC))
                {
                    log_d("Publish MQTT Topic: %s Payload: %s", config.mqtt_topic, tnc2.c_str());
                    clientMQTT.publish(config.mqtt_topic, tnc2.c_str());
                }
#endif
                    // SerialBT.println(tnc2);
                    // uint16_t type = pkgType((char *)incomingPacket.info);
                    if (!(type & FILTER_THIRDPARTY))
                    {
                        char call[11];
                        if (incomingPacket.src.ssid > 0)
                            sprintf(call, "%s-%d", incomingPacket.src.call, incomingPacket.src.ssid);
                        else
                            sprintf(call, "%s", incomingPacket.src.call);

                        char *rawP = (char *)calloc(tnc2.length() + 1, sizeof(char));
                        if (rawP)
                        {
                            memset(rawP, 0, tnc2.length() + 1);
                            tnc2.toCharArray(rawP, tnc2.length(), 0);
                            // memcpy(rawP, tnc2.c_str(), tnc2.length());
                            int idx = pkgListUpdate(call, rawP, type, 0, incomingPacket.mVrms);

#if defined OLED || defined ST7735_160x80
                            if ((config.oled_enable) && (idx > -1))
                            {

                                if (config.rx_display && config.dispRF && (type & config.dispFilter))
                                {
                                    dispBuffer.push(tnc2.c_str());
                                    log_d("RF_putQueueDisp:[pkgList_idx=%d,Type=%d RAW:%s] %s\n", idx, type, call, tnc2.c_str());
                                }
                            }
#endif
                            handle_ws(rawP, tnc2.length(), incomingPacket.mVrms);
                            free(rawP);
                        }
                    }
                    lastPkg = true;
                    // handle_ws(tnc2, incomingPacket.mVrms);
                    //   ESP_BT.println(tnc2);
                    status.allCount++;

                    tnc2.clear();
                }
            }
        }

        // IGate Process
        if (config.igate_en)
        {
            if (config.igate_sts_interval > 10)
            {
                if (millis() > igateSTSInterval)
                {
                    igateSTSInterval = millis() + (config.igate_sts_interval * 1000);
                    igate_status(config.igate_status);
                }
            }
            // IGATE Position
            if (config.igate_bcn)
            {
                if (millis() > iGatetickInterval)
                {

                    String rawData = "";
                    if (config.igate_gps)
                    { // IGATE Send GPS position
                        if (gps.location.isValid())
                        {
                            rawData = igate_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), "");
                            if (config.log & LOG_IGATE)
                            {
                                logIGate(gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.course.deg());
                            }
                        }
                    }
                    else
                    { // IGATE Send fix position
                        rawData = igate_position(config.igate_lat, config.igate_lon, config.igate_alt, "");
                        if (config.log & LOG_TRACKER)
                        {
                            logIGate(config.igate_lat, config.igate_lon, 0, 0);
                        }
                    }
                    if (rawData != "")
                    {
                        iGatetickInterval = millis() + (config.igate_interval * 1000);
                        Sleep_Activate &= ~ACTIVATE_IGATE;
                        StandByTick = millis() + (5000);
                        if (config.igate_tlm_interval > 0)
                        {
                            igateTlmInvCount++;
                            if (igateTlmInvCount >= config.igate_tlm_interval)
                            {
                                igateTlmInvCount = 0;
                                if (config.igate_tlm_sensor[0] | config.igate_tlm_sensor[1] | config.igate_tlm_sensor[2] | config.igate_tlm_sensor[3] | config.igate_tlm_sensor[4])
                                {
                                    char tlm_result[100];
                                    char tlm_data[200];
                                    size_t tlm_sz;
                                    if ((IGATE_TLM_SEQ % 100) == 0)
                                    {
                                        char rawInfo[100];
                                        char name[10];
                                        sprintf(rawInfo, "PARM.");
                                        int i, c = 0;
                                        for (i = 0; i < 5; i++)
                                        {
                                            if (config.igate_tlm_sensor[i] == 0)
                                            {
                                                c++;
                                                continue;
                                            }
                                            else
                                            {
                                                if (i > 0)
                                                    strcat(rawInfo, ",");
                                                sprintf(name, "%s", config.igate_tlm_PARM[i]);
                                                strcat(rawInfo, name);
                                            }
                                        }
                                        for (int n = c + 8; n > 0; n--)
                                        {
                                            strcat(rawInfo, ",");
                                        }
                                        sendTelemetry_igate(rawInfo);
                                        memset(rawInfo, 0, sizeof(rawInfo));
                                        sprintf(rawInfo, "UNIT.");
                                        c = 0;
                                        for (i = 0; i < 5; i++)
                                        {
                                            if (config.igate_tlm_sensor[i] == 0)
                                            {
                                                c++;
                                                continue;
                                            }
                                            else
                                            {
                                                if (i > 0)
                                                    strcat(rawInfo, ",");
                                                sprintf(name, "%s", config.igate_tlm_UNIT[i]);
                                                strcat(rawInfo, name);
                                            }
                                        }
                                        for (int n = c + 8; n > 0; n--)
                                        {
                                            strcat(rawInfo, ",");
                                        }
                                        sendTelemetry_igate(rawInfo);
                                        memset(rawInfo, 0, sizeof(rawInfo));
                                        sprintf(rawInfo, "EQNS.");
                                        c = 0;
                                        for (i = 0; i < 5; i++)
                                        {
                                            if (config.igate_tlm_sensor[i] == 0)
                                            {
                                                c++;
                                                continue;
                                            }
                                            else
                                            {
                                                if (i > 0)
                                                    strcat(rawInfo, ",");
                                                if (fmod(config.igate_tlm_EQNS[i][0], 1) == 0)
                                                    sprintf(name, "%0.f", config.igate_tlm_EQNS[i][0]);
                                                else
                                                    sprintf(name, "%.3f", config.igate_tlm_EQNS[i][0]);
                                                strcat(rawInfo, name);
                                                if (fmod(config.igate_tlm_EQNS[i][1], 1) == 0)
                                                    sprintf(name, ",%0.f", config.igate_tlm_EQNS[i][1]);
                                                else
                                                    sprintf(name, ",%.3f", config.igate_tlm_EQNS[i][1]);
                                                strcat(rawInfo, name);
                                                if (fmod(config.igate_tlm_EQNS[i][2], 1) == 0)
                                                    sprintf(name, ",%0.f", config.igate_tlm_EQNS[i][2]);
                                                else
                                                    sprintf(name, ",%.3f", config.igate_tlm_EQNS[i][2]);
                                                strcat(rawInfo, name);
                                            }
                                        }
                                        for (int n = c; n > 0; n--)
                                        {
                                            strcat(rawInfo, ",");
                                            sprintf(name, "0");
                                            strcat(rawInfo, name);
                                            sprintf(name, ",1");
                                            strcat(rawInfo, name);
                                            sprintf(name, ",0");
                                            strcat(rawInfo, name);
                                        }
                                        // strcat(rawInfo, ",");
                                        sendTelemetry_igate(rawInfo);
                                    }

                                    if (++IGATE_TLM_SEQ > 8279)
                                        IGATE_TLM_SEQ = 0;
                                    memset(tlm_data, 0, 200);
                                    memset(tlm_result, 0, 100);
                                    int n = 0;
                                    sprintf(tlm_data, "%i", IGATE_TLM_SEQ);
                                    for (int s = 0; s < 5; s++)
                                    {
                                        if (config.igate_tlm_sensor[s] == 0)
                                        {
                                            continue;
                                            // strcat(tlm_data, "0");
                                        }
                                        else
                                        {
                                            strcat(tlm_data, ",");
                                            int sen_idx = config.igate_tlm_sensor[s] - 1;
                                            double data = 0;
                                            if (sen[sen_idx].visable)
                                                data = sen[sen_idx].sample;
                                            double precision = pow(10.0f, (double)config.igate_tlm_precision[s]);
                                            int val = (int)((data + config.igate_tlm_offset[s]) * precision);
                                            // log_d("s:%d Data:%.2f /tPresion:%.5f /tOffset:%.5f/t Val:%d",s,sen[sen_idx].sample,precision,config.trk_tlm_offset[s],val);
                                            if (val > 8280)
                                                val = 8280;
                                            if (val < 0)
                                                val = 0;
                                            char strVal[10];
                                            sprintf(strVal, "%i", val);
                                            strcat(tlm_data, strVal);
                                        }
                                    }
                                    // log_d("TLM_DATA:%s",tlm_data);
                                    //  sprintf(tlm_data, "%i,%i,%i,%i", TLM_SEQ, (int)(VBat * 100), int(TempNTC * 100), gps.satellites.value());
                                    telemetry_base91(tlm_data, tlm_result, tlm_sz);
                                    rawData += String(tlm_result);
                                }
                            }
                        }
                        if (strlen(config.igate_comment) > 0)
                        {
                            rawData += String(config.igate_comment);
                        }

                        log_d("IGATE_POSITION: %s", rawData.c_str());

                        if (config.igate_gps)
                            sprintf(sts, "POSITION GPS\nINTERVAL %ds", tx_interval);
                        else
                            sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);

                        uint8_t SendMode = 0;
                        if (config.igate_loc2rf)
                            SendMode |= RF_CHANNEL;
                        if (config.igate_loc2inet)
                            SendMode |= INET_CHANNEL;
                        pkgTxPush(rawData.c_str(), rawData.length(), 0, SendMode);
//                         if (config.igate_loc2rf)
//                         { // IGATE SEND POSITION TO RF
//                             char *rawP = (char *)calloc(rawData.length(), sizeof(char));
//                             // rawData.toCharArray(rawP, rawData.length());
//                             memcpy(rawP, rawData.c_str(), rawData.length());
//                             pkgTxPush(rawP, rawData.length(), 0);
#if defined OLED || defined ST7735_160x80
                       if(config.oled_enable) pushTxDisp(TXCH_RF, "TX IGATE", sts);
#endif
                        //                             free(rawP);
                        //                         }
                        //                         if (config.igate_loc2inet)
                        //                         { // IGATE SEND TO APRS-IS
                        //                             if (aprsClient.connected())
                        //                             {
                        //                                 status.txCount++;
                        //                                 aprsClient.println(rawData); // Send packet to Inet
                        // #if defined OLED || defined ST7735_160x80
                        //                                 pushTxDisp(TXCH_TCP, "TX IGATE", sts);
                        // #endif
                        //                             }
                        //                         }
                    }
                }
            }
            // IGATE send to inet
            if (newIGatePkg)
            {
                newIGatePkg = false;
                // if (config.rf2inet && aprsClient.connected())
                if (config.rf2inet)
                {
                    int ret = 0;
                    // uint16_t type = pkgType((const char *)&incomingPacket.info[0]);
                    //  IGate Filter RF->INET
                    if ((type & config.rf2inetFilter))
                        ret = igateProcess(incomingPacket);
                    if (ret == 0)
                    {
                        status.dropCount++;
                        igateTLM.DROP++;
                    }
                    else
                    {
                        status.rf2inet++;
                        igateTLM.RF2INET++;
                        igateTLM.TX++;
                    }
                }
            }
        }

        // Digi Repeater Process
        if (config.digi_en || (config.digi_auto && (aprsClient.connected() == false)))
        {
            if (config.digi_sts_interval > 10)
            {
                if (millis() > digiSTSInterval)
                {
                    digiSTSInterval = millis() + (config.digi_sts_interval * 1000);
                    digi_status(config.digi_status);
                }
            }
            // DIGI Position
            if (config.digi_bcn)
            {
                if (millis() > DiGiInterval)
                {

                    String rawData;
                    if (config.digi_gps)
                    { // DIGI Send GPS position
                        if (gps.location.isValid())
                        {
                            rawData = digi_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), "");
                            if (config.log & LOG_DIGI)
                            {
                                logDigi(gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.course.deg());
                            }
                        }
                    }
                    else
                    { // DIGI Send fix position
                        rawData = digi_position(config.digi_lat, config.digi_lon, config.digi_alt, "");
                        if (config.log & LOG_DIGI)
                        {
                            logDigi(config.digi_lat, config.digi_lon, 0, 0);
                        }
                    }
                    if (rawData != "")
                    {
                        DiGiInterval = millis() + (config.digi_interval * 1000);
                        Sleep_Activate &= ~ACTIVATE_DIGI;
                        StandByTick = millis() + (5000);
                        if (config.digi_tlm_interval > 0)
                        {
                            digiTlmInvCount++;
                            if (digiTlmInvCount >= config.digi_tlm_interval)
                            {
                                digiTlmInvCount = 0;

                                if (config.digi_tlm_sensor[0] | config.digi_tlm_sensor[1] | config.digi_tlm_sensor[2] | config.digi_tlm_sensor[3] | config.digi_tlm_sensor[4])
                                {
                                    char tlm_result[100];
                                    char tlm_data[200];
                                    size_t tlm_sz;
                                    if ((DIGI_TLM_SEQ % 100) == 0)
                                    {
                                        char rawInfo[100];
                                        char name[10];
                                        sprintf(rawInfo, "PARM.");
                                        int i, c = 0;
                                        for (i = 0; i < 5; i++)
                                        {
                                            if (config.digi_tlm_sensor[i] == 0)
                                            {
                                                c++;
                                                continue;
                                            }
                                            else
                                            {
                                                if (i > 0)
                                                    strcat(rawInfo, ",");
                                                sprintf(name, "%s", config.digi_tlm_PARM[i]);
                                                strcat(rawInfo, name);
                                            }
                                        }
                                        for (int n = c + 8; n > 0; n--)
                                        {
                                            strcat(rawInfo, ",");
                                        }
                                        sendTelemetry_digi(rawInfo);
                                        memset(rawInfo, 0, sizeof(rawInfo));
                                        sprintf(rawInfo, "UNIT.");
                                        c = 0;
                                        for (i = 0; i < 5; i++)
                                        {
                                            if (config.digi_tlm_sensor[i] == 0)
                                            {
                                                c++;
                                                continue;
                                            }
                                            else
                                            {
                                                if (i > 0)
                                                    strcat(rawInfo, ",");
                                                sprintf(name, "%s", config.digi_tlm_UNIT[i]);
                                                strcat(rawInfo, name);
                                            }
                                        }
                                        for (int n = c + 8; n > 0; n--)
                                        {
                                            strcat(rawInfo, ",");
                                        }
                                        sendTelemetry_digi(rawInfo);
                                        memset(rawInfo, 0, sizeof(rawInfo));
                                        sprintf(rawInfo, "EQNS.");
                                        c = 0;
                                        for (i = 0; i < 5; i++)
                                        {
                                            if (config.digi_tlm_sensor[i] == 0)
                                            {
                                                c++;
                                                continue;
                                            }
                                            else
                                            {
                                                if (i > 0)
                                                    strcat(rawInfo, ",");
                                                if (fmod(config.digi_tlm_EQNS[i][0], 1) == 0)
                                                    sprintf(name, "%0.f", config.digi_tlm_EQNS[i][0]);
                                                else
                                                    sprintf(name, "%.3f", config.digi_tlm_EQNS[i][0]);
                                                strcat(rawInfo, name);
                                                if (fmod(config.digi_tlm_EQNS[i][1], 1) == 0)
                                                    sprintf(name, ",%0.f", config.digi_tlm_EQNS[i][1]);
                                                else
                                                    sprintf(name, ",%.3f", config.digi_tlm_EQNS[i][1]);
                                                strcat(rawInfo, name);
                                                if (fmod(config.digi_tlm_EQNS[i][2], 1) == 0)
                                                    sprintf(name, ",%0.f", config.digi_tlm_EQNS[i][2]);
                                                else
                                                    sprintf(name, ",%.3f", config.digi_tlm_EQNS[i][2]);
                                                strcat(rawInfo, name);
                                            }
                                        }
                                        for (int n = c; n > 0; n--)
                                        {
                                            strcat(rawInfo, ",");
                                            sprintf(name, "0");
                                            strcat(rawInfo, name);
                                            sprintf(name, ",1");
                                            strcat(rawInfo, name);
                                            sprintf(name, ",0");
                                            strcat(rawInfo, name);
                                        }
                                        // strcat(rawInfo, ",");
                                        sendTelemetry_digi(rawInfo);
                                    }

                                    if (++DIGI_TLM_SEQ > 8279)
                                        DIGI_TLM_SEQ = 0;
                                    memset(tlm_data, 0, 200);
                                    memset(tlm_result, 0, 100);
                                    int n = 0;
                                    sprintf(tlm_data, "%i", DIGI_TLM_SEQ);
                                    for (int s = 0; s < 5; s++)
                                    {
                                        if (config.digi_tlm_sensor[s] == 0)
                                        {
                                            continue;
                                            // strcat(tlm_data, "0");
                                        }
                                        else
                                        {
                                            strcat(tlm_data, ",");
                                            int sen_idx = config.digi_tlm_sensor[s] - 1;
                                            double data = 0;
                                            if (sen[sen_idx].visable)
                                                data = sen[sen_idx].sample;
                                            double precision = pow(10.0f, (double)config.digi_tlm_precision[s]);
                                            int val = (int)((data + config.digi_tlm_offset[s]) * precision);
                                            // log_d("s:%d Data:%.2f /tPresion:%.5f /tOffset:%.5f/t Val:%d",s,sen[sen_idx].sample,precision,config.trk_tlm_offset[s],val);
                                            if (val > 8280)
                                                val = 8280;
                                            if (val < 0)
                                                val = 0;
                                            char strVal[10];
                                            sprintf(strVal, "%i", val);
                                            strcat(tlm_data, strVal);
                                        }
                                    }
                                    // log_d("TLM_DATA:%s",tlm_data);
                                    //  sprintf(tlm_data, "%i,%i,%i,%i", TLM_SEQ, (int)(VBat * 100), int(TempNTC * 100), gps.satellites.value());
                                    telemetry_base91(tlm_data, tlm_result, tlm_sz);
                                    rawData += String(tlm_result);
                                }
                            }
                        }
                        if (strlen(config.digi_comment) > 0)
                        {
                            rawData += String(config.digi_comment);
                        }

                        log_d("DIGI_POSITION: %s", rawData.c_str());

                        if (config.digi_gps)
                            sprintf(sts, "POSITION GPS\nINTERVAL %ds", tx_interval);
                        else
                            sprintf(sts, "POSITION FIX\nINTERVAL %ds", tx_interval);

                        uint8_t SendMode = 0;
                        if (config.digi_loc2rf)
                            SendMode |= RF_CHANNEL;
                        if (config.digi_loc2inet)
                            SendMode |= INET_CHANNEL;
                        pkgTxPush(rawData.c_str(), rawData.length(), 0, SendMode);
//                         if (config.digi_loc2rf)
//                         { // DIGI SEND POSITION TO RF
//                             char *rawP = (char *)calloc(rawData.length(), sizeof(char));
//                             // rawData.toCharArray(rawP, rawData.length());
//                             memcpy(rawP, rawData.c_str(), rawData.length());
//                             pkgTxPush(rawP, rawData.length(), 0);
#if defined OLED || defined ST7735_160x80
                        if(config.oled_enable) pushTxDisp(TXCH_RF, "TX DIGI POS", sts);
#endif
                        //                             free(rawP);
                        //                         }
                        //                         if (config.digi_loc2inet)
                        //                         { // DIGI SEND TO APRS-IS
                        //                             if (aprsClient.connected())
                        //                             {
                        //                                 status.txCount++;
                        //                                 aprsClient.println(rawData); // Send packet to Inet
                        // #if defined OLED || defined ST7735_160x80
                        //                                 pushTxDisp(TXCH_TCP, "TX DIGI POS", sts);
                        // #endif
                        //                             }
                        //                         }
                    }
                    rawData.clear();
                }
            }

            // Repeater packet
            if (newDigiPkg)
            {
                newDigiPkg = false;
                // uint16_t type = pkgType((const char *)&incomingPacket.info[0]);
                Sleep_Activate &= ~ACTIVATE_DIGI;
                StandByTick = millis() + (config.pwr_stanby_delay * 1000);
                // Digi repeater filter
                if ((type & config.digiFilter))
                {
                    // Packet recheck
                    pkgTxDuplicate(incomingPacket); // Search duplicate in tx and drop packet for renew
                    int dlyFlag = digiProcess(incomingPacket);
                    if (dlyFlag > 0)
                    {
                        int digiDelay;
                        status.digiCount++;
                        if (dlyFlag == 1)
                        {
                            digiDelay = 0;
                        }
                        else
                        {
                            if (config.digi_delay == 0)
                            { // Auto mode
                              // if (digiCount > 20)
                              //   digiDelay = random(5000);
                              // else if (digiCount > 10)
                              //   digiDelay = random(3000);
                              // else if (digiCount > 0)
                              //   digiDelay = random(1500);
                              // else
                                digiDelay = random(100);
                            }
                            else
                            {
                                digiDelay = random(config.digi_delay);
                            }
                        }

                        String digiPkg;
                        packet2Raw(digiPkg, incomingPacket);
                        log_d("DIGI_REPEAT: %s", digiPkg.c_str());
                        log_d("DIGI delay=%d ms.", digiDelay);
                        // char *rawP = (char *)calloc(digiPkg.length()+1, sizeof(char));
                        //  digiPkg.toCharArray(rawP, digiPkg.length());
                        // memcpy(rawP, digiPkg.c_str(), digiPkg.length());
                        pkgTxPush(digiPkg.c_str(), digiPkg.length(), digiDelay, RF_CHANNEL);
                        digiPkg.clear();
                        // pkgTxPush(rawP, digiPkg.length(), digiDelay, RF_CHANNEL);
                        sprintf(sts, "--src call--\n%s\nDelay: %dms.", incomingPacket.src.call, digiDelay);
#if defined OLED || defined ST7735_160x80
                        if(config.oled_enable) pushTxDisp(TXCH_DIGI, "DIGI REPEAT", sts);
#endif
                        // free(rawP);
                    }
                }
            }
        }

        // Weather
        if (config.wx_en)
        {
            if (millis() > WxInterval)
            {

                String rawData = "";
                if (config.wx_gps)
                { // Wx Send GPS position
                    if (gps.location.isValid())
                    {
                        rawData = wx_report(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), "");
                        if (config.log & LOG_WX)
                        {
                            logWeather(gps.location.lat(), gps.location.lng(), gps.speed.kmph(), gps.course.deg());
                        }
                    }
                }
                else
                { // Wx Send fix position
                    rawData = wx_report(config.wx_lat, config.wx_lon, config.wx_alt, "");
                    if (config.log & LOG_WX)
                    {
                        logWeather(config.wx_lat, config.wx_lon, 0, 0);
                    }
                }
                if (rawData != "")
                {
                    WxInterval = millis() + (config.wx_interval * 1000);
                    Sleep_Activate &= ~ACTIVATE_WX;
                    StandByTick = millis() + (5000);
                    log_d("WX_REPORT: %s", rawData.c_str());
                    uint8_t SendMode = 0;
                    if (config.wx_2rf)
                        SendMode |= RF_CHANNEL;
                    if (config.wx_2inet)
                        SendMode |= INET_CHANNEL;
                    pkgTxPush(rawData.c_str(), rawData.length(), 0, SendMode);
//                     if (config.wx_2rf)
//                     { // WX SEND POSITION TO RF
//                         char *rawP = (char *)calloc(rawData.length(), sizeof(char));
//                         // rawData.toCharArray(rawP, rawData.length());
//                         memcpy(rawP, rawData.c_str(), rawData.length());
//                         pkgTxPush(rawP, rawData.length(), 0);
#ifdef OLED
                    sprintf(sts, "--src call--\n%s\nDelay: %dms.", config.wx_mycall, (config.wx_interval * 1000));
                    if(config.oled_enable) pushTxDisp(TXCH_RF, "WX REPORT", sts);
#endif
                    //                         free(rawP);
                    //                     }
                    //                     if (config.wx_2inet)
                    //                     { // WX SEND TO APRS-IS
                    //                         if (aprsClient.connected())
                    //                         {
                    //                             status.txCount++;
                    //                             aprsClient.println(rawData); // Send packet to Inet
                    // #ifdef OLED
                    //                             // pushTxDisp(TXCH_TCP, "WX REPORT", sts);
                    // #endif
                    //                         }
                    //                     }
                }
                else
                {
                    WxInterval = millis() + (10 * 1000);
                }
#ifdef MQTT
                if (config.en_mqtt && (config.mqtt_topic_flag & MQTT_TOPIC_WX) && clientMQTT.connected())
                {
                    char payload[500];
                    char topic[100];
                    if(strlen(config.wx_object)<3)
                        sprintf(topic,"/%s/WEATHER/sample",config.wx_mycall);
                    else
                        sprintf(topic,"/%s/WEATHER/sample",config.wx_object);
                    getWxJson(&payload[0],false);
                    log_d("Publish MQTT Topic: %s Payload: %s", config.mqtt_topic, payload);
                    clientMQTT.publish(config.mqtt_topic, payload);
                }
#endif                
            }
#ifdef MQTT            
            if (millis() > WxIntervalAvg)
            {
                WxIntervalAvg = millis() + (600 * 1000);
                if (config.en_mqtt && (config.mqtt_topic_flag & MQTT_TOPIC_WX) && clientMQTT.connected())
                {
                    char payload[500];
                    char topic[100];
                    if(strlen(config.wx_object)<3)
                        sprintf(topic,"/%s/WEATHER/average",config.wx_mycall);
                    else
                        sprintf(topic,"/%s/WEATHER/average",config.wx_object);
                    getWxJson(&payload[0],true);
                    log_d("Publish MQTT Topic: %s Payload: %s", topic, payload);
                    clientMQTT.publish(topic, payload);
                }
            }
#endif               
        }

        if (config.tlm0_en)
        {
            if (systemTLM.ParmTimeout < millis())
            {
                systemTLM.ParmTimeout = millis() + (config.tlm0_info_interval * 1000);
                char rawInfo[100];
                char name[10];
                sprintf(rawInfo, "PARM.");
                for (int i = 0; i < 13; i++)
                {
                    if (i > 0)
                        strcat(rawInfo, ",");
                    sprintf(name, "%s", config.tlm0_PARM[i]);
                    strcat(rawInfo, name);
                }
                sendTelemetry_0(rawInfo, true);
                memset(rawInfo, 0, sizeof(rawInfo));
                sprintf(rawInfo, "UNIT.");
                for (int i = 0; i < 13; i++)
                {
                    if (i > 0)
                        strcat(rawInfo, ",");
                    sprintf(name, "%s", config.tlm0_UNIT[i]);
                    strcat(rawInfo, name);
                }
                sendTelemetry_0(rawInfo, true);
                memset(rawInfo, 0, sizeof(rawInfo));
                sprintf(rawInfo, "EQNS.");
                for (int i = 0; i < 5; i++)
                {
                    if (i > 0)
                        strcat(rawInfo, ",");
                    if (fmod(config.tlm0_EQNS[i][0], 1) == 0)
                        sprintf(name, "%0.f", config.tlm0_EQNS[i][0]);
                    else
                        sprintf(name, "%.3f", config.tlm0_EQNS[i][0]);
                    strcat(rawInfo, name);
                    if (fmod(config.tlm0_EQNS[i][1], 1) == 0)
                        sprintf(name, ",%0.f", config.tlm0_EQNS[i][1]);
                    else
                        sprintf(name, ",%.3f", config.tlm0_EQNS[i][1]);
                    strcat(rawInfo, name);
                    if (fmod(config.tlm0_EQNS[i][2], 1) == 0)
                        sprintf(name, ",%0.f", config.tlm0_EQNS[i][2]);
                    else
                        sprintf(name, ",%.3f", config.tlm0_EQNS[i][2]);
                    strcat(rawInfo, name);
                }
                sendTelemetry_0(rawInfo, true);
                memset(rawInfo, 0, sizeof(rawInfo));
                sprintf(rawInfo, "BITS.");
                uint8_t b = 1;
                for (int i = 0; i < 8; i++)
                {
                    if (config.tlm0_BITS_Active & b)
                    {
                        strcat(rawInfo, "1");
                    }
                    else
                    {
                        strcat(rawInfo, "0");
                    }
                    b <<= 1;
                }
                strcat(rawInfo, ",");
                strcat(rawInfo, config.tlm0_comment);
                sendTelemetry_0(rawInfo, true);
            }

            if (systemTLM.TeleTimeout < millis())
            {
                systemTLM.TeleTimeout = millis() + (config.tlm0_data_interval * 1000);
                char rawTlm[100];
                if (systemTLM.Sequence > 999)
                    systemTLM.Sequence = 0;
                else
                    systemTLM.Sequence++;
                getTelemetry_0();
                sprintf(rawTlm, "T#%03d,%03d,%03d,%03d,%03d,%03d,", systemTLM.Sequence, systemTLM.A1, systemTLM.A2, systemTLM.A3, systemTLM.A4, systemTLM.A5);
                uint8_t b = 1;
                for (int i = 0; i < 8; i++)
                {
                    if (!((systemTLM.BITS & b) ^ (config.tlm0_BITS_Active & b)))
                    {
                        strcat(rawTlm, "1");
                    }
                    else
                    {
                        strcat(rawTlm, "0");
                    }
                    b <<= 1;
                }
                sendTelemetry_0(rawTlm, false);
            }
        }
    }
}

void taskAPRSPoll(void *pvParameters)
{
    afskSetModem(config.modem_type, config.audio_lpf, config.tx_timeslot, config.preamble * 100, config.fx25_mode);
    afskSetSQL(config.rf_sql_gpio, config.rf_sql_active);
    afskSetPTT(config.rf_ptt_gpio, config.rf_ptt_active);
    afskSetPWR(config.rf_pwr_gpio, config.rf_pwr_active);
    
    // afskSetDCOffset(config.adc_dc_offset);
    afskSetADCAtten(config.adc_atten);

#ifdef STRIP_PIN
    AFSK_init(config.adc_gpio, config.dac_gpio, config.rf_ptt_gpio, config.rf_sql_gpio, config.rf_pwr_gpio, -1, -1, STRIP_PIN, config.rf_ptt_active, config.rf_sql_active, config.rf_pwr_active);
#else
    AFSK_init(config.adc_gpio, config.dac_gpio, config.rf_ptt_gpio, config.rf_sql_gpio, config.rf_pwr_gpio, LED_TX_PIN, LED_RX_PIN, -1, config.rf_ptt_active, config.rf_sql_active, config.rf_pwr_active);
#endif
    setPtt(false);
    log_d("APRS Polling Task Start on Core %d.", xPortGetCoreID());
    for (;;)
    {
        if (config.modem_type == 3)
            vTaskDelay(1 / portTICK_PERIOD_MS);
        else
            vTaskDelay(3 / portTICK_PERIOD_MS);

        if (AFSKInitAct == true)
        {
            AFSK_Poll(false, LOW);
        }
    }
}

int mqttRetry = 0;
long wifiTTL = 0;

// WiFi connect timeout per AP. Increase when connecting takes longer.
const uint32_t connectTimeoutMs = 60000;
uint8_t APStationNum = 0;
unsigned long waitISRetry = 0;
unsigned long lastHeardTimeout = 0;

pppType pppStatus;

IPAddress ap_ip(192, 168, 4, 1);
IPAddress ap_mask(255, 255, 255, 0);
IPAddress ap_leaseStart(192, 168, 4, 2);
IPAddress ap_dns(8, 8, 4, 4);

uint8_t wifiStatus = WL_DISCONNECTED;
bool vpnConnected = false;
bool wifiDisconnecting = false;
uint16_t wifiDisCount=0;
unsigned long vpnTimeout=0;

void onEvent(arduino_event_id_t event, arduino_event_info_t info)
{
    switch (event)
    {
    #ifdef PPPOS
    case ARDUINO_EVENT_PPP_START:
        log_d("PPP Started");
        break;
    case ARDUINO_EVENT_PPP_CONNECTED:
        log_d("PPP Connected");
        if(!WiFi.isConnected()){
            if(config.vpn){
                log_d("Setup Wireguard VPN by PPP!");
                vpnConnected = false;
                vpnTimeout=millis()+3000;
            }
        
#ifdef MQTT
        if (config.en_mqtt)
        {
            if (!clientMQTT.connected())
            {
                clientMQTT.disconnect();
                clientMQTT.setServer(config.mqtt_host, config.mqtt_port);
                clientMQTT.setCallback(mqtt_callback);
            }
            mqtt_reconnect();
        }
#endif
        }
        break;
    case ARDUINO_EVENT_PPP_GOT_IP:
        pppStatus.ip = info.got_ip.ip_info.ip.addr;
        pppStatus.gateway = info.got_ip.ip_info.gw.addr;
        pppStatus.netmask = info.got_ip.ip_info.netmask.addr;
        log_d("PPP Got IP");
        log_d("PPP IP: %s", IPAddress(pppStatus.ip).toString().c_str());
        log_d("PPP Gateway: %s", IPAddress(pppStatus.gateway).toString().c_str());
        log_d("PPP Subnet: %s", IPAddress(pppStatus.netmask).toString().c_str());
        WiFi.AP.enableNAPT(config.ppp_napt);
        break;
    case ARDUINO_EVENT_PPP_LOST_IP:
        log_d("PPP Lost IP");
        WiFi.AP.enableNAPT(false);
        pppTimeout = millis() + (60 * 1000);
        break;
    case ARDUINO_EVENT_PPP_DISCONNECTED:
        log_d("PPP Disconnected");
        WiFi.AP.enableNAPT(false);
        pppTimeout = millis() + (60 * 1000);
        break;
    case ARDUINO_EVENT_PPP_STOP:
        log_d("PPP Stopped");
        break;
    #endif

    case ARDUINO_EVENT_WIFI_AP_START:
        log_d("AP Started");
        Serial.println(WiFi.AP);
        break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
        log_d("AP STA Connected");
        break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
        log_d("AP STA Disconnected");
        break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
        log_d("AP STA IP Assigned: ");
        log_d("%s", IPAddress(info.wifi_ap_staipassigned.ip.addr).toString().c_str());
        break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
        log_d("AP Probe Request Received");
        break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
        log_d("AP Stopped");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        wifiDisCount=0;
        wifiDisconnecting = false;
// String info = "WiFi Infor:";
// info += "\nSSID: " + WiFi.SSID();
// info += "\nRSSI: " + String(WiFi.RSSI()) + "dBm";
// info += "\nIP: " + WiFi.localIP().toString();
// info += "\nGW IP: " + WiFi.gatewayIP().toString();
// info += "\nMAC: " + WiFi.macAddress();
// lv_label_set_text(ui_txtSystemInfo, info.c_str());
        if(config.vpn){
            log_d("Setup Wireguard VPN by WiFi!");
            vpnConnected = false;
            vpnTimeout=millis()+3000;
        }
#ifdef MQTT
        if (config.en_mqtt)
        {
            if (!clientMQTT.connected())
            {
                clientMQTT.disconnect();
                clientMQTT.setServer(config.mqtt_host, config.mqtt_port);
                clientMQTT.setCallback(mqtt_callback);
            }
            mqtt_reconnect();
        }
#endif        
        log_d("WiFi Connected");
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        log_d("WiFi Disconnected");
        wifiDisCount++;        
        // if(wifiDisCount%5==0){
        //     log_d("WiFi Disconnected count=%d",wifiDisCount);
        //     WiFi.disconnect(true,true);            
        //     wifiMulti.run(); // Move to next AP             
        //     WiFi.reconnect();
        // }
        if(wifiDisconnecting==false && wifiDisCount>10){
            wifiDisCount=0;
            wifiDisconnecting = true; 
            pingTimeout = millis() + 10000;
            #ifdef MQTT
        if (config.en_mqtt)
        {
            if (!clientMQTT.connected())
            {
                clientMQTT.disconnect();
                clientMQTT.setServer(config.mqtt_host, config.mqtt_port);
                clientMQTT.setCallback(mqtt_callback);
            }
            mqtt_reconnect();
        }
#endif                           
        #ifdef PPPOS
        if(config.vpn ){            
            //if(PPP.connected()){
                vpnConnected = false;  
                vpnTimeout=millis()+1000;              
            //}
        }
        #endif
        }
        break;
    default:
        break;
    }
}

long int pppTimeout = 0;

#ifdef PPPOS

void PPPOS_Start()
{
    if (config.ppp_enable)
    {        
        pppTimeout = millis() + (600 * 1000);
        log_d("Starting the modem. It might take a while!");
        PPP.end();
        pinMode(config.ppp_rst_gpio, OUTPUT);
        digitalWrite(config.ppp_rst_gpio, config.ppp_rst_active);
        delay(config.ppp_rst_delay);
        digitalWrite(config.ppp_rst_gpio, !config.ppp_rst_active);
        // pinMode(39, INPUT_PULLUP); // Set GPIO39 as input with pull-up resistor
        //  digitalWrite(39, HIGH); // Set GPIO39 to high to enable pull-up resistor
        delay(1000); // Wait for the modem to reset
        
        PPP.setApn(config.ppp_apn);     // Set the APN for the modem
        PPP.setResetPin(config.ppp_rst_gpio, config.ppp_rst_active, config.ppp_rst_delay);
        PPP.setPins(config.ppp_tx_gpio, config.ppp_rx_gpio);
        log_d("PPP Modem Init...");
        log_d("Using UART %d as PPP interface", config.ppp_serial);
        if(config.ppp_serial>1) return;
        if (PPP.begin(PPP_MODEM_MODEL, config.ppp_serial, 115200))
        {
            log_d("PPP started successfully");
        }
        else
        {
            log_d("PPP failed to start");
            sprintf(pppStatus.manufacturer, "Not found modem");
            sprintf(pppStatus.model, "N/A");
            return;
        }

        // Configure the modem
        if (config.ppp_gnss)
        {
            Serial.println(PPP.cmd("AT+QGPSCFG=\"outport\",\"uartdebug\"", 10000)); // กำหนดให้ NMEA output ผ่านพอร์ต UART Debug
            Serial.println(PPP.cmd("AT+QGPSCFG=\"nmeasrc\",0", 10000));             // ปิดการดึง NMEA sentence ผ่าน AT+QGPSGNMEA
            Serial.println(PPP.cmd("AT+QGPSCFG=\"gpsnmeatype\",63", 10000));        // เปิดใช้งาน NMEA sentence ทั้งหมด (GGA, RMC, GSV, GSA, VTG, GLL)
            Serial.println(PPP.cmd("AT+QGPSCFG=\"gnssconfig\",1", 10000));          // เลือกใช้ระบบดาวเทียม GPS + BeiDou
            Serial.println(PPP.cmd("AT+QGPSCFG=\"autogps\",0", 10000));             // ปิดใช้งาน GNSS อัตโนมัติเมื่อโมดูลเริ่มทำงาน
            Serial.println(PPP.cmd("AT+QGPSCFG=\"apflash\",1", 10000));             // เปิดใช้งาน AP-Flash เพื่อการเริ่มต้นที่รวดเร็ว
            Serial.println(PPP.cmd("AT+QGPS=1", 10000));
        }
        
        PPP.cmd("AT+CTZU=1", 1000); // Enable automatic time zone update
        // PPP.setUser(ppp_user);
        // PPP.setPass(ppp_pass);
        // PPP.setApn(config.ppp_apn);
        // PPP.setPin(config.ppp_pin);
        // PPP.setResetPin(config.ppp_rst_gpio, config.ppp_rst_active, config.ppp_rst_delay);
        // PPP.setPins(config.ppp_tx_gpio, config.ppp_rx_gpio, config.ppp_rts_gpio, config.ppp_cts_gpio, PPP_MODEM_FC);

        PPP.setBaudrate(config.ppp_serial_baudrate); // Set the baud rate for the modem
        sprintf(pppStatus.manufacturer, "%s", PPP.cmd("AT+CGMI", 10000).c_str());
        sprintf(pppStatus.model, "%s", PPP.moduleName().c_str());
        sprintf(pppStatus.imei, "%s", PPP.IMEI().c_str());
        log_d("Manufacturer: %s", pppStatus.manufacturer);
        log_d("Model: %s", pppStatus.model);
        log_d("IMEI: %s", pppStatus.imei);

        bool attached = PPP.attached();
        if (!attached)
        {
            int i = 0;
            unsigned int s = millis();
            log_d("Waiting to connect to network");
            while (!attached && ((++i) < 30))
            {
                log_d(".");
                delay(100);
                attached = PPP.attached();
            }
            log_d("%ds", (millis() - s) / 1000.0);
            attached = PPP.attached();
        }

        log_d("Attached: %d", attached);
        log_d("State: %d", PPP.radioState());
        if (attached)
        {
            sprintf(pppStatus.oper, "%s", PPP.operatorName().c_str());
            sprintf(pppStatus.imsi, "%s", PPP.IMSI().c_str());
            log_d("IMSI: %s", pppStatus.imsi);
            pppStatus.rssi = PPP.RSSI();
            log_d("Operator: %s", pppStatus.oper);
            log_d("RSSI: %d dBm", pppStatus.rssi);
            PPP.started(); // Start the PPP connection            
            //   if(PPP.sms("0984958488","Test SMS from ESP32")){
            //       log_d("SMS sent successfully!");
            //   } else {
            //       log_d("Failed to send SMS.");
            //   }
            int ber = PPP.BER();
            if (ber > 0)
            {
                log_d("BER: %i", ber);
                log_d("NetMode: %i", PPP.networkMode());
            }

            log_d("Switching to data mode...");
            PPP.mode(ESP_MODEM_MODE_DATA); // Data and Command mixed mode
            if (!PPP.waitStatusBits(ESP_NETIF_CONNECTED_BIT, 5000))
            {
                log_d("Failed to connect to internet!");
                PPP.mode(ESP_MODEM_MODE_COMMAND); // Switch back to command mode
            }
            else
            {
                log_d("Connected to internet!");
            }
        }
        else
        {
            log_d("Failed to connect to network!");
        }
    }
}
#endif

void taskNetwork(void *pvParameters)
{
    int c = 0;
    // char raw[500];
    log_d("Task Network has been start");
    // manualWiFi = true;

    // Listen for modem events
    Network.onEvent(onEvent);

    // WiFi.onEvent(Wifi_connected,SYSTEM_EVENT_STA_CONNECTED);
    // WiFi.onEvent(Get_IPAddress, SYSTEM_EVENT_STA_GOT_IP);
    // WiFi.onEvent(Wifi_disconnected, SYSTEM_EVENT_STA_DISCONNECTED);

    if (config.wifi_mode == WIFI_STA_FIX)
    { /**< WiFi station mode */
        WiFi.mode(WIFI_MODE_STA);
        WiFi.setTxPower((wifi_power_t)config.wifi_power);
    }
    else if (config.wifi_mode == WIFI_AP_FIX)
    { /**< WiFi soft-AP mode */
        WiFi.mode(WIFI_MODE_AP);
        WiFi.setTxPower((wifi_power_t)config.wifi_power);
    }
    else if (config.wifi_mode == WIFI_AP_STA_FIX)
    { /**< WiFi station + soft-AP mode */
        WiFi.mode(WIFI_MODE_APSTA);
        WiFi.setTxPower((wifi_power_t)config.wifi_power);
    }
    else
    {
        WiFi.mode(WIFI_MODE_NULL);
    }

    if (config.wifi_mode & WIFI_STA_FIX)
    {
        for (int i = 0; i < 5; i++)
        {
            if (config.wifi_sta[i].enable)
            {
                wifiMulti.addAP(config.wifi_sta[i].wifi_ssid, config.wifi_sta[i].wifi_pass);
            }
        }
        WiFi.setHostname("ESP32APRS_Audio");
        if (wifiMulti.run() == WL_CONNECTED)
        {
            NTP_Timeout = millis() + 2000;
        }
    }

    if (config.wifi_mode & WIFI_AP_FIX)
    {
        // manualWiFi = true;
        log_d("Access point running. IP address: ");
        log_d("%s", WiFi.softAPIP().toString().c_str());
        // Start the Access Point
        WiFi.AP.begin();
        WiFi.AP.config(ap_ip, ap_ip, ap_mask, ap_leaseStart, ap_dns);
        WiFi.AP.create(config.wifi_ap_ssid, config.wifi_ap_pass);
        if (!WiFi.AP.waitStatusBits(ESP_NETIF_STARTED_BIT, 1000))
        {
            Serial.println("Failed to start AP!");
            // return;
        }
    }

    if (wifiMulti.run() == WL_CONNECTED)
    {
        log_d("Wi-Fi CONNECTED!");
        log_d("IP address: %s", WiFi.localIP().toString().c_str());
        NTP_Timeout = millis() + 5000;
    }

    pingTimeout = millis() + 10000;
    unsigned long timeNetworkOld = millis();
    timeNetwork = 0;
    if (config.wifi_mode & WIFI_AP_STA_FIX)
        webService();

    // wireguard_ctx_t ctx = {0};
    esp_err_t err;
    #ifdef PPPOS
    PPPOS_Start();    // Start PPP connection if enabled
    #endif

    #ifdef BLUETOOTH
    bluetooth_init(); // Initialize Bluetooth if enabled
    #endif

    for (;;)
    {
        unsigned long now = millis();
        timeNetwork = now - timeNetworkOld;
        timeNetworkOld = now;
        // wdtNetworkTimer = millis();
        // serviceHandle();
        //esp_task_wdt_reset();
        timerNetwork = micros() - timerNetwork_old;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timerNetwork_old = micros();

        #ifdef PPPOS
        if (config.ppp_enable)
        {
            if (!PPP.connected())
            {
                if (millis() > pppTimeout)
                {
                    log_d("PPP connection timeout!");
                    PPPOS_Start(); // Restart PPP connection
                    pppTimeout = millis() + (600 * 1000);
                }
            }
        }


        if (WiFi.isConnected() == true || WiFi.softAPgetStationNum() > 0 || PPP.connected())
        #else
        if (WiFi.isConnected() == true || WiFi.softAPgetStationNum() > 0)
        #endif
        {
            if (lastHeard_Flag)
            {
                if (millis() > lastHeardTimeout)
                {
                    lastHeard_Flag = false;
                    lastHeardTimeout = millis() + 1000;
                    event_lastHeard(false);                    
                }
            }
        }

        if (config.wifi_mode & WIFI_AP_FIX)
        {
            APStationNum = WiFi.softAPgetStationNum();
            if (APStationNum > 0)
            {
                // config.pwr_sleep_activate |= ACTIVATE_WIFI;
                if (WiFi.isConnected() == false)
                {
                    vTaskDelay(9 / portTICK_PERIOD_MS);
                    continue;
                }
            }
        }

        if (wifiMulti.run(connectTimeoutMs) == WL_CONNECTED)
        {
            // config.pwr_sleep_activate |= ACTIVATE_WIFI;
            if (millis() > NTP_Timeout)
            {
                NTP_Timeout = millis() + 86400000;
                // setSyncProvider(getNtpTime);
                log_d("Contacting Time Server\n");
                configTime(3600 * config.timeZone, 0, config.ntp_host);
                // vTaskDelay(500 / portTICK_PERIOD_MS);
                struct tm tmstruct;
                if (getLocalTime(&tmstruct, 5000))
                {
                    time_t systemTime;
                    time(&systemTime);
                    setTime(systemTime);
                    if (systemUptime == 0)
                    {
                        systemUptime = time(NULL);
                    }
                    pingTimeout = millis() + 2000;
                    if (config.vpn)
                    {
                        if (!wireguard_active())
                        {
                            log_d("Setup Wiregurad VPN!");
                            wireguard_setup();
                        }
                        // //if(ctx.netif==NULL){
                        //     log_d("wireguard_setup");
                        //     err = wireguard_setup(&ctx);
                        //     if (err != ESP_OK) {
                        //         log_d("wireguard_setup: %s", esp_err_to_name(err));
                        //     }
                        // //}
                        // log_d("VPN Connecting.");
                        // //if(esp_wireguardif_peer_is_up(&ctx) != ESP_OK){

                        //     err = esp_wireguard_connect(&ctx);
                        //     if (err != ESP_OK) {
                        //         log_d("esp_wireguard_connect: %s", esp_err_to_name(err));
                        //     }else{
                        //         int to=0;
                        //         while (esp_wireguardif_peer_is_up(&ctx) != ESP_OK) {
                        //             vTaskDelay(1000 / portTICK_PERIOD_MS);
                        //             if(++to>5) break;
                        //         }
                        //         log_d("VPN Peer is up");
                        //         esp_wireguard_set_default(&ctx);
                        //     }
                    }
                }
                else
                {
                    NTP_Timeout = millis() + 60000;
                }
            }

            if (config.igate_en)
            {
                if (aprsClient.connected() == false)
                {
                    if (millis() > waitISRetry)
                    {
                        waitISRetry = millis() + 30000; // Retry connect 30Sec
                        if (APRSConnect())
                        {
                            if (config.igate_bcn)
                            {
                                iGatetickInterval = millis() + 10000; // send position after 10sec
                            }
                        }
                    }
                }
                else
                {
                    if (aprsClient.available())
                    {
                        pingTimeout = millis() + 300000;                // Reset ping timout
                        String line = aprsClient.readStringUntil('\n'); // อ่านค่าที่ Server ตอบหลับมาทีละบรรทัด
                        status.isCount++;
                        int start_val = line.indexOf(">", 0); // หาตำแหน่งแรกของ >
                        if (start_val > 3)
                        {
                            String src_call = line.substring(0, start_val);
                            String msg_call = "::" + src_call;

                            status.allCount++;
                            status.rxCount++;
                            igateTLM.RX++;

                            log_d("INET: %s\n", line.c_str());
                            start_val = line.indexOf(":", 10); // Search of info in ax25
                            if (start_val > 5)
                            {
                                String info = line.substring(start_val + 1);
                                size_t rawSize = line.length();
                                char *raw = (char *)calloc(rawSize + 1, sizeof(char));
                                if (raw)
                                {
                                    memset(raw, 0, rawSize + 1);
                                    memcpy(raw, info.c_str(), info.length());

                                    uint16_t type = pkgType(&raw[0]);
                                    int start_dstssid = line.indexOf("-", 1); // get SSID -
                                    if (start_dstssid < 0)
                                        start_dstssid = line.indexOf(" ", 1); // get ssid space
                                    char ssid = 0;
                                    if (start_dstssid > 0)
                                        ssid = line.charAt(start_dstssid + 1);

                                    if (ssid > 47 && ssid < 58)
                                    {
                                        size_t len = src_call.length();
                                        char call[15];
                                        memset(call, 0, sizeof(call));
                                        if (len > 15)
                                            len = 15;
                                        memcpy(call, src_call.c_str(), len);
                                        call[14] = 0;
                                        memset(raw, 0, rawSize + 1);
                                        memcpy(raw, line.c_str(), line.length());
                                        if (type & config.dispFilter)
                                        {
                                            int idx = pkgListUpdate(call, raw, type, 1, 0);
                                            // int cnt = 0;
                                            // if (idx > -1)
                                            // {
                                            // // Put queue affter filter for display popup
                                            // if (config.rx_display && config.dispINET)
                                            // {
                                            //     cnt = pushTNC2Raw(idx);
                                            //     log_d("INET_putQueueDisp:[pkgList_idx=%d/queue=%d,Type=%d] %s\n", idx, cnt, type, call);
                                            // }
                                            // }

#if defined OLED || defined ST7735_160x80
                                            if (idx > -1)
                                            {
                                                // Put queue affter filter for display popup
                                                if (config.rx_display && config.dispINET && (type & config.dispFilter))
                                                {
                                                    dispBuffer.push(line.c_str());
                                                    log_d("INET_putQueueDisp:[pkgList_idx=%d/queue=%d,Type=%d] %s\n", idx, dispBuffer.getCount(), type, call);
                                                }
                                            }
#endif
                                        }
                                        // INET2RF affter filter
                                        if (config.inet2rf)
                                        {
                                            if (type & config.inet2rfFilter)
                                            {
                                                String tnc2Raw = "";
                                                char *strtmp = (char *)calloc(350, sizeof(char));
                                                if (strtmp)
                                                {
                                                    if (config.aprs_ssid == 0)
                                                        sprintf(strtmp, "%s>APE32A", config.aprs_mycall);
                                                    else
                                                        sprintf(strtmp, "%s-%d>APE32A", config.aprs_mycall, config.aprs_ssid);
                                                    tnc2Raw = String(strtmp);
                                                    tnc2Raw += ",RFONLY"; // fix path to rf only not send loop to inet
                                                    tnc2Raw += ":}";      // 3rd-party frame
                                                    tnc2Raw += line;
                                                    pkgTxPush(tnc2Raw.c_str(), tnc2Raw.length(), 0, RF_CHANNEL);
                                                    char sts[50];
                                                    sprintf(sts, "--SRC CALL--\n%s\n", src_call.c_str());
#if defined OLED || defined ST7735_160x80
                                                    if(config.oled_enable) pushTxDisp(TXCH_3PTY, "TX INET->RF", sts);
#endif
                                                    status.inet2rf++;
                                                    igateTLM.INET2RF++;
                                                    log_d("INET2RF: %s\n", line);
                                                    free(strtmp);
                                                }
                                                tnc2Raw.clear();
                                            }
                                        }
                                    }
                                    free(raw);
                                }
                                info.clear();
                            }
                            src_call.clear();
                            msg_call.clear();
                        }
                        line.clear();
                    }
                }
            }

#ifdef MQTT
            if (config.en_mqtt)
            {
                if (!clientMQTT.connected())
                {
                    mqtt_reconnect();
                }
                else
                {
                    clientMQTT.loop();
                }
            }
#endif

            if (millis() > pingTimeout)
            {
                pingTimeout = millis() + 60000;
                log_d("Ping GW to %s\n", WiFi.gatewayIP().toString().c_str());
                if (ping_start(WiFi.gatewayIP(), 2, 0, 0, 5) == true)
                {
                    log_d("GW Success!!\n");
                }
                else
                {
                    log_d("GW Fail!\n");
                    WiFi.disconnect();
                    WiFi.persistent(false);
                    WiFi.mode(WIFI_OFF); // Switch WiFi off

                    wifiTTL = 0;
                    delay(3000);
                    if (config.wifi_mode == WIFI_STA_FIX)
                    { /**< WiFi station mode */
                        WiFi.mode(WIFI_MODE_STA);
                        WiFi.setTxPower((wifi_power_t)config.wifi_power);
                    }
                    else if (config.wifi_mode == WIFI_AP_FIX)
                    { /**< WiFi soft-AP mode */
                        WiFi.mode(WIFI_MODE_AP);
                        WiFi.setTxPower((wifi_power_t)config.wifi_power);
                    }
                    else if (config.wifi_mode == WIFI_AP_STA_FIX)
                    { /**< WiFi station + soft-AP mode */
                        WiFi.mode(WIFI_MODE_APSTA);
                        WiFi.setTxPower((wifi_power_t)config.wifi_power);
                    }
                    else
                    {
                        WiFi.mode(WIFI_MODE_NULL);
                    }
                    WiFi.reconnect();
                    wifiMulti.run(5000);
                }
                if (config.vpn)
                {
                    if (!wireguard_active())
                    {
                        log_d("Reconnection Wiregurad VPN!");
                        wireguard_remove();
                        delay(1000);
                        wireguard_setup();
                    }
                    //             IPAddress vpnIP;
                    //             vpnIP.fromString(String(config.wg_gw_address));
                    //             log_d("Ping VPN to %s", vpnIP.toString().c_str());
                    //             if (ping_start(vpnIP, 2, 0, 0, 10) == true)
                    //             {
                    //                 log_d("VPN Ping Success!!");
                    //             }
                    //             else
                    //             {
                    //                 log_d("VPN Ping Fail!");
                    //                 wireguard_remove();
                    //                 delay(3000);
                    //                 wireguard_setup();
                    //                 //esp_wireguard_disconnect(&ctx);
                    //                 log_d("VPN Connecting.");
                    // // err = esp_wireguard_connect(&ctx);
                    // // if (err != ESP_OK) {
                    // //     log_d("esp_wireguard_connect: %s", esp_err_to_name(err));
                    // // }
                    // // while (esp_wireguardif_peer_is_up(&ctx) != ESP_OK) {
                    // //     vTaskDelay(1000 / portTICK_PERIOD_MS);
                    // // }
                    // // log_d("VPN Peer is up");
                    // // esp_wireguard_set_default(&ctx);

                    //             }
                }
            }
        }
    } // for loop
}

#ifdef OLED
// Routine
void line_angle(signed int startx, signed int starty, unsigned int length, unsigned int angle, unsigned int color)
{
    display.drawLine(startx, starty, (startx + length * cosf(angle * 0.017453292519)), (starty + length * sinf(angle * 0.017453292519)), color);
}

int xSpiGlcdSelFontHeight = 8;
int xSpiGlcdSelFontWidth = 5;

void compass_label(signed int startx, signed int starty, unsigned int length, double angle, unsigned int color)
{
    double angleNew;
    // ushort Color[2];
    uint8_t x_N, y_N, x_S, y_S;
    int x[4], y[4], i;
    int xOffset, yOffset;
    yOffset = (xSpiGlcdSelFontHeight / 2);
    xOffset = (xSpiGlcdSelFontWidth / 2);
    // GLCD_WindowMax();
    angle += 270.0F;
    angleNew = angle;
    for (i = 0; i < 4; i++)
    {
        if (angleNew > 360.0F)
            angleNew -= 360.0F;
        x[i] = startx + (length * cosf(angleNew * 0.017453292519));
        y[i] = starty + (length * sinf(angleNew * 0.017453292519));
        x[i] -= xOffset;
        y[i] -= yOffset;
        angleNew += 90.0F;
    }
    angleNew = angle + 45.0F;
    for (i = 0; i < 4; i++)
    {
        if (angleNew > 360.0F)
            angleNew -= 360.0F;
        x_S = startx + ((length - 3) * cosf(angleNew * 0.017453292519));
        y_S = starty + ((length - 3) * sinf(angleNew * 0.017453292519));
        x_N = startx + ((length + 3) * cosf(angleNew * 0.017453292519));
        y_N = starty + ((length + 3) * sinf(angleNew * 0.017453292519));
        angleNew += 90.0F;
        display.drawLine(x_S, y_S, x_N, y_N, color);
    }
    display.drawCircle(startx, starty, length, color);
    display.setFont();
    display.drawChar((uint8_t)x[0], (uint8_t)y[0], 'N', WHITE, BLACK, 1);
    display.drawChar((uint8_t)x[1], (uint8_t)y[1], 'E', WHITE, BLACK, 1);
    display.drawChar((uint8_t)x[2], (uint8_t)y[2], 'S', WHITE, BLACK, 1);
    display.drawChar((uint8_t)x[3], (uint8_t)y[3], 'W', WHITE, BLACK, 1);
}

void compass_arrow(signed int startx, signed int starty, unsigned int length, double angle, unsigned int color)
{
    double angle1, angle2;
    int xdst, ydst, x1sta, y1sta, x2sta, y2sta;
    int length2 = length / 2;
    angle += 270.0F;
    if (angle > 360.0F)
        angle -= 360.0F;
    xdst = startx + length * cosf(angle * 0.017453292519);
    ydst = starty + length * sinf(angle * 0.017453292519);
    angle1 = angle + 135.0F;
    if (angle1 > 360.0F)
        angle1 -= 360.0F;
    angle2 = angle + 225.0F;
    if (angle2 > 360.0F)
        angle2 -= 360.0F;
    x1sta = startx + length2 * cosf(angle1 * 0.017453292519);
    y1sta = starty + length2 * sinf(angle1 * 0.017453292519);
    x2sta = startx + length2 * cosf(angle2 * 0.017453292519);
    y2sta = starty + length2 * sinf(angle2 * 0.017453292519);
    display.drawLine(startx, starty, xdst, ydst, color);
    display.drawLine(xdst, ydst, x1sta, y1sta, color);
    display.drawLine(x1sta, y1sta, startx, starty, color);
    display.drawLine(startx, starty, x2sta, y2sta, color);
    display.drawLine(x2sta, y2sta, xdst, ydst, color);
}

unsigned long disp_delay = 0;
void dispTxWindow(txDisp txs)
{
    if (config.tx_display == false)
        return;

#ifdef OLED
    display.clearDisplay();

    disp_delay = config.dispDelay * 1000;
    timeHalfSec = millis() + disp_delay;

#ifdef SSD1306_72x40
    display.fillRect(0, 0, 72, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(0, 1);
    // display.print("");

    txs.name[sizeof(txs.name) - 1] = 0;
    if (strlen(txs.name))
    {
        display.print(txs.name);
    }

    display.setTextColor(WHITE);
    display.setFont(&FreeSerifItalic9pt7b);

    if (txs.tx_ch == TXCH_TCP)
    {
        display.setCursor(10, 30);
        display.print("TCP");
    }
    else if (txs.tx_ch == TXCH_RF)
    {
        display.setCursor(15, 30);
        display.print("RF");
    }
    else if (txs.tx_ch == TXCH_DIGI)
    {
        display.setCursor(10, 30);
        display.print("RPT");
    }
    else if (txs.tx_ch == TXCH_3PTY)
    {
        display.setCursor(0, 30);
        display.print("IP>RF");
    }

    display.setFont();
    display.setTextColor(WHITE);

// char *pch;
// int y = 30;
// pch = strtok(txs.info, "\n");
// while (pch != NULL)
// {
//     display.setCursor(50, y);
//     display.printf("%s", pch);
//     pch = strtok(NULL, "\n");
//     y += 9;
// }
#else
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(0, 14);
    txs.name[sizeof(txs.name) - 1] = 0;
    if (strlen(txs.name))
    {
        display.printf("%s", txs.name);
    }

    display.setFont(&FreeSerifItalic9pt7b);

    if (txs.tx_ch == TXCH_TCP)
    {
        display.setCursor(5, 42);
        display.print("TCP");
        display.setCursor(15, 57);
        display.print("IP");
    }
    else if (txs.tx_ch == TXCH_RF)
    {
        display.setCursor(3, 42);
        display.print("SEND");
        display.setCursor(15, 57);
        display.print("RF");
    }
    else if (txs.tx_ch == TXCH_DIGI)
    {
        display.setCursor(3, 42);
        display.print("RPT");
        display.setCursor(15, 57);
        display.print("RF");
    }
    else if (txs.tx_ch == TXCH_3PTY)
    {
        display.setCursor(3, 42);
        display.print("FWD");
        display.setCursor(15, 57);
        display.print("RF");
    }

    display.setFont();
    display.setTextColor(WHITE);
    // display.setCursor(115, 0);
    // display.print("TX");

    display.drawRoundRect(0, 16, 128, 48, 5, WHITE);
    display.fillRoundRect(1, 17, 126, 10, 2, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(40, 18);
    display.print("TX STATUS");

    display.setTextColor(WHITE);
    // display.setCursor(50, 30);

    char *pch;
    int y = 30;
    pch = strtok(txs.info, "\n");
    while (pch != NULL)
    {
        display.setCursor(50, y);
        display.printf("%s", pch);
        pch = strtok(NULL, "\n");
        y += 9;
    }
#endif
    display.display();
#elif defined(ST7735_160x80)
    display.fillScreen(ST77XX_BLACK);
    display.setTextColor(ST77XX_BLUE);
    ledcWrite(ST7735_LED_K_Pin, config.disp_brightness);
    disp_delay = config.dispDelay * 1000;
    timeHalfSec = millis() + disp_delay;

    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(0, 14);
    txs.name[sizeof(txs.name) - 1] = 0;
    if (strlen(txs.name))
    {
        display.print(txs.name);
    }

    display.setTextColor(ST77XX_RED);
    display.setFont(&FreeSerifItalic9pt7b);

    if (txs.tx_ch == TXCH_TCP)
    {
        display.setCursor(5, 52);
        display.print("TCP");
        display.setCursor(15, 67);
        display.print("IP");
    }
    else if (txs.tx_ch == TXCH_RF)
    {
        display.setCursor(3, 52);
        display.print("SEND");
        display.setCursor(15, 67);
        display.print("RF");
    }
    else if (txs.tx_ch == TXCH_DIGI)
    {
        display.setCursor(3, 52);
        display.print("RPT");
        display.setCursor(15, 67);
        display.print("RF");
    }
    else if (txs.tx_ch == TXCH_3PTY)
    {
        display.setCursor(3, 52);
        display.print("FWD");
        display.setCursor(15, 67);
        display.print("RF");
    }

    display.setFont();
    display.setTextColor(WHITE);
    // display.setCursor(115, 0);
    // display.print("TX");

    display.drawRoundRect(0, 18, 160, 62, 5, WHITE);
    display.fillRoundRect(1, 19, 158, 10, 2, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(50, 22);
    display.print("TX STATUS");

    display.setTextColor(WHITE);
    // display.setCursor(50, 30);

    char *pch;
    int y = 35;
    pch = strtok(txs.info, "\n");
    while (pch != NULL)
    {
        display.setCursor(60, y);
        display.printf("%s", pch);
        pch = strtok(NULL, "\n");
        y += 10;
    }
    display.display();
#endif
}

const char *directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
bool dispPush = 0;

void dispWindow(String line, uint8_t mode, bool filter)
{
    if (config.rx_display == false)
        return;

    struct pbuf_t aprs;
    bool Monitor = false;
    char text[300];
    unsigned char x = 0;
    char itemname[10];
    int start_val = line.indexOf(":}", 10);
    if (start_val > 0)
    {
        String new_info = line.substring(start_val + 2);
        start_val = new_info.indexOf(">", 0);
        if (start_val > 3 && start_val < 12)
            line = new_info;
    }
    start_val = line.indexOf(">", 0); // หาตำแหน่งแรกของ >
    if (start_val > 3 && start_val < 12)
    {
        // powerWakeup();
        String src_call = line.substring(0, start_val);
        memset(&aprs, 0, sizeof(pbuf_t));
        aprs.buf_len = 300;
        aprs.packet_len = line.length();
        memcpy(aprs.data, line.c_str(), line.length());
        // line.toCharArray(&aprs.data[0], aprs.packet_len);
        int start_info = line.indexOf(":", 0);
        int end_ssid = line.indexOf(",", start_val);
        int start_dst = line.indexOf(">", 2);
        int start_dstssid = line.indexOf("-", start_dst);
        if ((end_ssid < 0) || (end_ssid > start_info))
            end_ssid = start_info;
        if ((start_dstssid > start_dst) && (start_dstssid < start_dst + 10))
        {
            aprs.dstcall_end_or_ssid = &aprs.data[start_dstssid];
        }
        else
        {
            aprs.dstcall_end_or_ssid = &aprs.data[end_ssid];
        }
        aprs.info_start = &aprs.data[start_info + 1];
        aprs.dstname = &aprs.data[start_dst + 1];
        aprs.dstname_len = end_ssid - start_dst;
        aprs.dstcall_end = &aprs.data[end_ssid];
        aprs.srccall_end = &aprs.data[start_dst];

        // aprsParse.parse_aprs(&aprs);
        if (aprsParse.parse_aprs(&aprs))
        {
            if (filter == true)
            {
                if ((config.dispFilter & FILTER_STATUS) && (aprs.packettype & T_STATUS))
                {
                    Monitor = true;
                }
                else if ((config.dispFilter & FILTER_MESSAGE) && (aprs.packettype & T_MESSAGE))
                {
                    Monitor = true;
                }
                else if ((config.dispFilter & FILTER_TELEMETRY) && (aprs.packettype & T_TELEMETRY))
                {
                    Monitor = true;
                }
                else if ((config.dispFilter & FILTER_WX) && ((aprs.packettype & T_WX) || (aprs.packettype & T_WAVE)))
                {
                    Monitor = true;
                }

                if ((config.dispFilter & FILTER_POSITION) && (aprs.packettype & T_POSITION))
                {
                    double lat, lon;
                    if (gps.location.isValid())
                    {
                        lat = gps.location.lat();
                        lon = gps.location.lng();
                    }
                    else
                    {
                        lat = config.igate_lat;
                        lon = config.igate_lon;
                    }
                    double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                    if (config.filterDistant == 0)
                    {
                        Monitor = true;
                    }
                    else
                    {
                        if (dist < config.filterDistant)
                            Monitor = true;
                        else
                            Monitor = false;
                    }
                }

                if ((config.dispFilter & FILTER_POSITION) && (aprs.packettype & T_POSITION))
                {
                    if (aprs.flags & F_CSRSPD)
                    {
                        double lat, lon;
                        if (gps.location.isValid())
                        {
                            lat = gps.location.lat();
                            lon = gps.location.lng();
                        }
                        else
                        {
                            lat = config.igate_lat;
                            lon = config.igate_lon;
                        }
                        double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                        if (config.filterDistant == 0)
                        {
                            Monitor = true;
                        }
                        else
                        {
                            if (dist < config.filterDistant)
                                Monitor = true;
                            else
                                Monitor = false;
                        }
                    }
                }

                if ((config.dispFilter & FILTER_POSITION) && (aprs.packettype & T_POSITION))
                {
                    if (aprs.flags & F_CSRSPD)
                    {
                        if (aprs.speed > 0)
                        {
                            double lat, lon;
                            if (gps.location.isValid())
                            {
                                lat = gps.location.lat();
                                lon = gps.location.lng();
                            }
                            else
                            {
                                lat = config.igate_lat;
                                lon = config.igate_lon;
                            }
                            double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                            if (config.filterDistant == 0)
                            {
                                Monitor = true;
                            }
                            else
                            {
                                if (dist < config.filterDistant)
                                    Monitor = true;
                                else
                                    Monitor = false;
                            }
                        }
                    }
                }
            }
            else
            {
                Monitor = true;
            }
        }
        else
        {
            log_d("APRS PARSE FAIL!");
            return;
        }

        if (Monitor)
        {

#ifdef OLED
#ifdef SH1106
            display.SH1106_command(0xE4);
#else
            display.ssd1306_command(0xE4);
#endif
            delay(10);
            display.clearDisplay();

            if (dispPush)
            {
                disp_delay = 600 * 1000;
                display.drawRoundRect(0, 0, 128, 16, 3, WHITE);
            }
            else
            {
                disp_delay = config.dispDelay * 1000;
            }
            timeHalfSec = millis() + disp_delay;
            // display.fillRect(0, 0, 128, 16, WHITE);
            const uint8_t *ptrSymbol;
            uint8_t symIdx = aprs.symbol[1] - 0x21;
            if (symIdx > 95)
                symIdx = 0;
            if (aprs.symbol[0] == '/')
            {
                ptrSymbol = &Icon_TableA[symIdx][0];
            }
            else if (aprs.symbol[0] == '\\')
            {
                ptrSymbol = &Icon_TableB[symIdx][0];
            }
            else
            {
                if (aprs.symbol[0] < 'A' || aprs.symbol[0] > 'Z')
                {
                    aprs.symbol[0] = 'N';
                    aprs.symbol[1] = '&';
                    symIdx = 5; // &
                }
                ptrSymbol = &Icon_TableB[symIdx][0];
            }
            display.drawYBitmap(0, 0, ptrSymbol, 16, 16, WHITE);
            if (!(aprs.symbol[0] == '/' || aprs.symbol[0] == '\\'))
            {
                display.drawChar(5, 4, aprs.symbol[0], BLACK, WHITE, 1);
                display.drawChar(6, 5, aprs.symbol[0], BLACK, WHITE, 1);
            }
#ifdef SSD1306_72x40
            display.setFont();
            display.setTextColor(BLACK);
            display.print(src_call);

            // if (aprs.srcname_len > 0)
            // {
            //     memset(&itemname, 0, sizeof(itemname));
            //     memcpy(&itemname, aprs.srcname, aprs.srcname_len);
            //     // Serial.println(itemname);
            //     display.setCursor(16, 0);
            //     display.print(itemname);
            //     display.setCursor(16, 7);
            //     display.print(src_call);
            // }
            // else
            // {
            //     display.setCursor(16, 4);
            //     display.print(src_call);
            // }

            display.setTextColor(WHITE);

            if (aprs.packettype & T_TELEMETRY)
            {
                bool show = false;
                int idx = tlmList_Find((char *)src_call.c_str());
                if (idx < 0)
                {
                    idx = tlmListOld();
                    if (idx > -1)
                        memset(&Telemetry[idx], 0, sizeof(Telemetry_struct));
                }
                if (idx > -1)
                {
                    Telemetry[idx].time = now();
                    strcpy(Telemetry[idx].callsign, (char *)src_call.c_str());

                    // for (int i = 0; i < 3; i++) Telemetry[idx].UNIT[i][5] = 0;
                    if (aprs.flags & F_UNIT)
                    {
                        memcpy(Telemetry[idx].UNIT, aprs.tlm_unit.val, sizeof(Telemetry[idx].UNIT));
                    }
                    else if (aprs.flags & F_PARM)
                    {
                        memcpy(Telemetry[idx].PARM, aprs.tlm_parm.val, sizeof(Telemetry[idx].PARM));
                    }
                    else if (aprs.flags & F_EQNS)
                    {
                        for (int i = 0; i < 15; i++)
                            Telemetry[idx].EQNS[i] = aprs.tlm_eqns.val[i];
                    }
                    else if (aprs.flags & F_BITS)
                    {
                        Telemetry[idx].BITS_FLAG = aprs.telemetry.bitsFlag;
                    }
                    else if (aprs.flags & F_TLM)
                    {
                        for (int i = 0; i < 5; i++)
                            Telemetry[idx].VAL[i] = aprs.telemetry.val[i];
                        Telemetry[idx].BITS = aprs.telemetry.bits;
                        show = true;
                    }

                    for (int i = 0; i < 4; i++)
                    { // Cut length
                        if (strstr(Telemetry[idx].PARM[i], "RxTraffic") != 0)
                            sprintf(Telemetry[idx].PARM[i], "RX");
                        if (strstr(Telemetry[idx].PARM[i], "TxTraffic") != 0)
                            sprintf(Telemetry[idx].PARM[i], "TX");
                        if (strstr(Telemetry[idx].PARM[i], "RxDrop") != 0)
                            sprintf(Telemetry[idx].PARM[i], "DROP");
                        Telemetry[idx].PARM[i][6] = 0;
                        Telemetry[idx].UNIT[i][3] = 0;
                        for (int a = 0; a < 3; a++)
                        {
                            if (Telemetry[idx].UNIT[i][a] == '/')
                                Telemetry[idx].UNIT[i][a] = 0;
                        }
                    }

                    for (int i = 0; i < 5; i++)
                    {
                        if (Telemetry[idx].PARM[i][0] == 0)
                        {
                            sprintf(Telemetry[idx].PARM[i], "CH%d", i + 1);
                        }
                    }
                }
                if (show || filter == false)
                {
                    display.fillRect(0, 0, 72, 9, WHITE);
                    display.setTextColor(BLACK);
                    display.setCursor(10, 1);
                    display.print("TELEMETRY");
                    display.setFont();
                    display.setTextColor(WHITE);
                    display.setCursor(0, 12);
                    // display.print(Telemetry[idx].PARM[0]);
                    // display.print(":");

                    if (fmod(Telemetry[idx].VAL[0], 1) == 0)
                        display.print(Telemetry[idx].VAL[0], 0);
                    else
                        display.print(Telemetry[idx].VAL[0], 1);
                    display.print(Telemetry[idx].UNIT[0]);
                    display.setCursor(20, 12);
                    // display.print(Telemetry[idx].PARM[1]);
                    // display.print(":");
                    if (fmod(Telemetry[idx].VAL[1], 1) == 0)
                        display.print(Telemetry[idx].VAL[1], 0);
                    else
                        display.print(Telemetry[idx].VAL[1], 1);
                    display.print(Telemetry[idx].UNIT[1]);
                    display.setCursor(0, 22);
                    // display.print(Telemetry[idx].PARM[2]);
                    // display.print(":");
                    if (fmod(Telemetry[idx].VAL[2], 1) == 0)
                        display.print(Telemetry[idx].VAL[2], 0);
                    else
                        display.print(Telemetry[idx].VAL[2], 1);
                    display.print(Telemetry[idx].UNIT[2]);
                    display.setCursor(21, 22);
                    // display.print(Telemetry[idx].PARM[3]);
                    // display.print(":");
                    if (fmod(Telemetry[idx].VAL[3], 1) == 0)
                        display.print(Telemetry[idx].VAL[3], 0);
                    else
                        display.print(Telemetry[idx].VAL[3], 1);
                    display.print(Telemetry[idx].UNIT[3]);
                    display.setCursor(0, 32);
                    // display.print(Telemetry[idx].PARM[4]);
                    // display.print(":");
                    display.print(Telemetry[idx].VAL[4], 1);
                    display.print(Telemetry[idx].UNIT[4]);

                    // display.setCursor(4, 55);
                    // display.print("BIT");
                    // uint8_t bit = Telemetry[idx].BITS;
                    // for (int i = 0; i < 8; i++)
                    // {
                    //     if (bit & 0x80)
                    //     {
                    //         display.fillCircle(30 + (i * 12), 58, 3, WHITE);
                    //     }
                    //     else
                    //     {
                    //         display.drawCircle(30 + (i * 12), 58, 3, WHITE);
                    //     }
                    //     bit <<= 1;
                    // }
                    display.display();
                }
                return;
            }
            else if (aprs.packettype & T_STATUS)
            {
                display.fillRect(0, 0, 72, 9, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(10, 1);
                display.print("STATUS");

                display.setFont();
                display.setCursor(0, 10);
                // memset(&text[0], 0, sizeof(text));
                // memcpy(&text[0], aprs.comment, aprs.comment_len);
                display.setTextColor(WHITE);
                display.print(aprs.comment);
                display.display();
                return;
            }
            else if (aprs.packettype & T_QUERY)
            {
                display.fillRect(0, 0, 72, 9, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(10, 1);
                display.print("?QUERY?");
                // memset(&text[0], 0, sizeof(text));
                // memcpy(&text[0], aprs.comment, aprs.comment_len);
                display.setFont();
                display.setTextColor(WHITE);
                display.setCursor(2, 10);
                display.print(aprs.comment);
                display.display();
                return;
            }
            else if (aprs.packettype & T_MESSAGE)
            {
                if (aprs.msg.is_ack == 1)
                {
                }
                else if (aprs.msg.is_rej == 1)
                {
                }
                else
                {
                    display.fillRect(0, 0, 72, 9, WHITE);
                    display.setTextColor(BLACK);
                    display.setCursor(10, 1);
                    display.print("MESSAGE");
                    display.setCursor(0, 10);
                    display.print("{");
                    // char txtID[7];
                    // memset(txtID, 0, sizeof(txtID));
                    // strncpy(&txtID[0], aprs.msg.msgid, aprs.msg.msgid_len);
                    // int msgid = atoi(txtID);
                    //  display.print(msgid, DEC);
                    // display.printf("%s", txtID);
                    // display.print("}");
                    //  memset(&text[0], 0, sizeof(text));
                    //  memcpy(&text[0], aprs.comment, aprs.comment_len);
                    display.setFont();
                    display.setTextColor(WHITE);
                    // display.setCursor(2, 10);
                    // display.print("To: ");
                    // memset(text, 0, sizeof(text));
                    // strncpy(&text[0], aprs.dstname, aprs.dstname_len);
                    // display.print(text);
                    // String mycall = "";
                    // if (config.aprs_ssid > 0)
                    //     mycall = String(config.aprs_mycall) + String("-") + String(config.aprs_ssid, DEC);
                    // else
                    //     mycall = String(config.aprs_mycall);
                    // if (strcmp(mycall.c_str(), text) == 0)
                    // {
                    //     display.setCursor(2, 54);
                    //     display.print("ACK:");
                    //     display.println(msgid);
                    //     String rawData = sendIsAckMsg(src_call, txtID);
                    //     log_d("IGATE_MSG: %s", rawData.c_str());
                    //     //if (config.igate_loc2rf)
                    //     { // IGATE SEND POSITION TO RF
                    //         char *rawP = (char *)malloc(rawData.length());
                    //         memcpy(rawP, rawData.c_str(), rawData.length());
                    //         pkgTxPush(rawP, rawData.length(), 0);
                    //         free(rawP);
                    //     }
                    //     // if (config.igate_loc2inet)
                    //     // { // IGATE SEND TO APRS-IS
                    //     //     if (aprsClient.connected())
                    //     //     {
                    //     //         aprsClient.println(rawData); // Send packet to Inet
                    //     //     }
                    //     // }
                    // }
                    memset(text, 0, sizeof(text));
                    strncpy(&text[0], aprs.msg.body, aprs.msg.body_len);
                    display.setCursor(2, 10);
                    display.print("Msg: ");
                    display.println(text);
                    display.display();
                }
                return;
            }
            display.setFont();
            display.drawFastHLine(0, 16, 70, WHITE);
            // display.drawFastVLine(48, 16, 48, WHITE);
            display.setCursor(16, 4);
            if (aprs.srcname_len > 0)
            {
                memset(&itemname, 0, sizeof(itemname));
                memcpy(&itemname, aprs.srcname, aprs.srcname_len);
                // Serial.println(itemname);
                display.print(itemname);
            }
            else
            {
                display.print(src_call);
            }

            x = 8;

            // if (aprs.srcname_len > 0)
            // {
            //     x += 9;
            //     display.fillRoundRect(0, 8, 70, 32, 0, WHITE);
            //     display.setTextColor(BLACK);
            //     display.setCursor(0, x);
            //     display.print("By " + src_call);
            //     display.setTextColor(WHITE);
            // }

            if (aprs.packettype & T_WAVE)
            {
                // Serial.println("WX Display");
                if (aprs.wave_report.flags & O_TEMP)
                {
                    display.setCursor(0, x += 10);
                    display.drawYBitmap(51, x, &Temperature_Symbol[0], 5, 8, WHITE);
                    display.printf("%.2fC", aprs.wave_report.Temp);
                }
                if (aprs.wave_report.flags & O_HS)
                {
                    // display.setCursor(102, x);
                    display.setCursor(0, x += 9);
                    display.printf("Hs:");
                    display.printf("%0.1f M", aprs.wave_report.Hs / 100);
                }
                if (aprs.wave_report.flags & O_TZ)
                {
                    display.setCursor(0, x += 9);
                    display.printf("Tz: ");
                    display.printf("%0.1f S", aprs.wave_report.Tz);
                }
                // if (aprs.wave_report.flags & O_TC)
                // {
                //     display.setCursor(58, x += 9);
                //     display.printf("Tc: ");
                //     display.printf("%0.1fS.", aprs.wave_report.Tc);
                // }
                if (aprs.wave_report.flags & O_BAT)
                {
                    display.setCursor(0, x += 9);
                    display.printf("BAT: ");
                    display.printf("%0.2fV", aprs.wave_report.Bat);
                }
            }
            if (aprs.packettype & T_WX)
            {
                // Serial.println("WX Display");
                if (aprs.wx_report.flags & W_TEMP)
                {
                    display.setCursor(8, x += 10);
                    display.drawYBitmap(0, x, &Temperature_Symbol[0], 5, 8, WHITE);
                    display.printf("%.1fC", aprs.wx_report.temp);
                }
                if (aprs.wx_report.flags & W_HUM)
                {
                    display.setCursor(28, x);
                    display.drawYBitmap(20, x, &Humidity_Symbol[0], 5, 8, WHITE);
                    display.printf("%0.1f%%", aprs.wx_report.humidity);
                }
                if (aprs.wx_report.flags & W_BAR)
                {
                    display.setCursor(28, x += 9);
                    display.drawYBitmap(20, x, &Pressure_Symbol[0], 5, 8, WHITE);
                    display.printf("%.1fhPa", aprs.wx_report.pressure);
                }
                if (aprs.wx_report.flags & W_R24H)
                {
                    // if (aprs.wx_report.rain_1h > 0) {
                    display.setCursor(28, x += 9);
                    display.drawYBitmap(20, x, &Rain_Symbol[0], 5, 8, WHITE);
                    display.printf("%.1fmm.", aprs.wx_report.rain_24h);
                    //}
                }
                if (aprs.wx_report.flags & W_PAR)
                {
                    // if (aprs.wx_report.luminosity > 10) {
                    display.setCursor(21, x += 9);
                    display.printf("%c", 0x0f);
                    display.setCursor(28, x);
                    display.printf("%dW/m", aprs.wx_report.luminosity);
                    if (aprs.wx_report.flags & W_UV)
                    {
                        display.printf(" UV%d", aprs.wx_report.uv);
                    }
                    //}
                }
                if (aprs.wx_report.flags & W_WS)
                {
                    display.setCursor(28, x += 9);
                    display.drawYBitmap(21, x, &Wind_Symbol[0], 5, 8, WHITE);
                    // int dirIdx=map(aprs.wx_report.wind_dir, -180, 180, 0, 8); ((angle+22)/45)%8]
                    int dirIdx = ((aprs.wx_report.wind_dir + 22) / 45) % 8;
                    if (dirIdx > 8)
                        dirIdx = 8;
                    display.printf("%.1fkPh(%s)", aprs.wx_report.wind_speed, directions[dirIdx]);
                }
                // Serial.printf("%.1fkPh(%d)", aprs.wx_report.wind_speed, aprs.wx_report.wind_dir);
                if (aprs.flags & F_HASPOS)
                {
                    // Serial.println("POS Display");
                    double lat, lon;
                    if (gps.location.isValid())
                    {
                        lat = gps.location.lat();
                        lon = gps.location.lng();
                    }
                    else
                    {
                        lat = config.igate_lat;
                        lon = config.igate_lon;
                    }
                    // double dtmp = aprsParse.direction(lon, lat, aprs.lng, aprs.lat);
                    // double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                    //  if (config.h_up == true)
                    //  {
                    //      // double course = gps.course.deg();
                    //      double course = SB_HEADING;
                    //      if (dtmp >= course)
                    //      {
                    //          dtmp -= course;
                    //      }
                    //      else
                    //      {
                    //          double diff = dtmp - course;
                    //          dtmp = diff + 360.0F;
                    //      }
                    //      compass_label(25, 37, 15, course, WHITE);
                    //      display.setCursor(0, 17);
                    //      display.printf("H");
                    //  }
                    //  else
                    //  {
                    //      compass_label(25, 37, 15, 0.0F, WHITE);
                    //  }
                    //  compass_label(25, 37, 15, 0.0F, WHITE);
                    //  compass_arrow(25, 37, 12, dtmp, WHITE);
                    //  display.drawFastHLine(1, 63, 45, WHITE);
                    //  display.drawFastVLine(1, 58, 5, WHITE);
                    //  display.drawFastVLine(46, 58, 5, WHITE);
                    //  display.setCursor(4, 55);
                    //  if (dist > 999)
                    //      display.printf("%.fKm", dist);
                    //  else
                    //      display.printf("%.1fKm", dist);
                }
                // else
                // {
                //     display.setCursor(0, 30);
                //     display.printf("NO\nPOSITION");
                // }
            }
            else if (aprs.flags & F_HASPOS)
            {
                // display.setCursor(50, x += 10);
                // display.printf("LAT %.5f\n", aprs.lat);
                // display.setCursor(51, x+=9);
                // display.printf("LNG %.4f\n", aprs.lng);
                String str;
                int l = 0;
                display.setCursor(0, x += 10);
                display.print("LAT:");
                str = String(aprs.lat, 5);
                l = str.length() * 6;
                display.setCursor(70 - l, x);
                display.print(str);

                display.setCursor(0, x += 9);
                display.print("LON:");
                str = String(aprs.lng, 5);
                l = str.length() * 6;
                display.setCursor(70 - l, x);
                display.print(str);

                double lat, lon;
                if (gps.location.isValid())
                {
                    lat = gps.location.lat();
                    lon = gps.location.lng();
                }
                else
                {
                    lat = config.igate_lat;
                    lon = config.igate_lon;
                }
                double dtmp = aprsParse.direction(lon, lat, aprs.lng, aprs.lat);
                double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                // if (config.h_up == true)
                // {
                //     // double course = gps.course.deg();
                //     double course = SB_HEADING;
                //     if (dtmp >= course)
                //     {
                //         dtmp -= course;
                //     }
                //     else
                //     {
                //         double diff = dtmp - course;
                //         dtmp = diff + 360.0F;
                //     }
                //     compass_label(25, 37, 15, course, WHITE);
                //     display.setCursor(0, 17);
                //     display.printf("H");
                // }
                // else
                // {
                //     compass_label(25, 37, 15, 0.0F, WHITE);
                // }
                // compass_arrow(25, 37, 12, dtmp, WHITE);
                // display.drawFastHLine(1, 55, 45, WHITE);
                // display.drawFastVLine(1, 55, 5, WHITE);
                // display.drawFastVLine(46, 55, 5, WHITE);
                // display.setCursor(0, 57);
                // if (dist > 999)
                //     display.printf("%.fKm", dist);
                // else
                //     display.printf("%.1fKm", dist);

                display.setCursor(0, x += 9);
                display.print("DX: ");
                str = String(dist, 5) + "Km";
                l = str.length() * 6;
                display.setCursor(70 - l, x);
                display.print(str);
                if (aprs.flags & F_CSRSPD)
                {
                    display.setCursor(0, x += 9);
                    // display.printf("SPD %d/", aprs.course);
                    // display.setCursor(50, x += 9);
                    display.printf("SPD %.1fkPh\n", aprs.speed);
                    int dirIdx = ((aprs.course + 22) / 45) % 8;
                    if (dirIdx > 8)
                        dirIdx = 8;
                    display.setCursor(0, x += 9);
                    display.printf("CSD %d(%s)", aprs.course, directions[dirIdx]);
                }
                if (aprs.flags & F_ALT)
                {
                    display.setCursor(0, x += 9);
                    display.printf("ALT %.1fM\n", aprs.altitude);
                }
                if (aprs.flags & F_PHG)
                {
                    int power, height, gain;
                    unsigned char tmp;
                    power = (int)aprs.phg[0] - 0x30;
                    power *= power;
                    height = (int)aprs.phg[1] - 0x30;
                    height = 10 << (height + 1);
                    height = height / 3.2808;
                    gain = (int)aprs.phg[2] - 0x30;
                    display.setCursor(0, x += 9);
                    display.printf("PHG %dM.\n", height);
                    display.setCursor(0, x += 9);
                    display.printf("PWR %dWatt\n", power);
                    display.setCursor(0, x += 9);
                    display.printf("ANT %ddBi\n", gain);
                }
                if (aprs.flags & F_RNG)
                {
                    display.setCursor(0, x += 9);
                    display.printf("RNG %dKm\n", aprs.radio_range);
                }
                /*if (aprs.comment_len > 0) {
                    display.setCursor(0, 56);
                    display.print(aprs.comment);
                }*/
            }
#else
            display.setCursor(20, 7);
            display.setTextSize(1);
            display.setFont(&FreeSansBold9pt7b);

            if (aprs.srcname_len > 0)
            {
                memset(&itemname, 0, sizeof(itemname));
                memcpy(&itemname, aprs.srcname, aprs.srcname_len);
                // Serial.println(itemname);
                display.print(itemname);
            }
            else
            {
                display.print(src_call);
            }

            display.setFont();
            display.setTextColor(WHITE);
            // if (selTab < 10)
            //     display.setCursor(121, 0);
            // else
            //     display.setCursor(115, 0);
            // display.print(selTab);

            if (mode == 1)
            {
                display.drawRoundRect(0, 16, 128, 48, 5, WHITE);
                display.fillRoundRect(1, 17, 126, 10, 2, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(40, 18);
                display.print("TNC2 RAW");

                display.setFont();
                display.setCursor(2, 30);
                display.setTextColor(WHITE);
                display.print(line);
                display.display();
                return;
            }

            if (aprs.packettype & T_TELEMETRY)
            {
                bool show = false;
                int idx = tlmList_Find((char *)src_call.c_str());
                if (idx < 0)
                {
                    idx = tlmListOld();
                    if (idx > -1)
                        memset(&Telemetry[idx], 0, sizeof(Telemetry_struct));
                }
                if (idx > -1)
                {
                    Telemetry[idx].time = now();
                    strcpy(Telemetry[idx].callsign, (char *)src_call.c_str());

                    // for (int i = 0; i < 3; i++) Telemetry[idx].UNIT[i][5] = 0;
                    if (aprs.flags & F_UNIT)
                    {
                        memcpy(Telemetry[idx].UNIT, aprs.tlm_unit.val, sizeof(Telemetry[idx].UNIT));
                    }
                    else if (aprs.flags & F_PARM)
                    {
                        memcpy(Telemetry[idx].PARM, aprs.tlm_parm.val, sizeof(Telemetry[idx].PARM));
                    }
                    else if (aprs.flags & F_EQNS)
                    {
                        for (int i = 0; i < 15; i++)
                            Telemetry[idx].EQNS[i] = aprs.tlm_eqns.val[i];
                    }
                    else if (aprs.flags & F_BITS)
                    {
                        Telemetry[idx].BITS_FLAG = aprs.telemetry.bitsFlag;
                    }
                    else if (aprs.flags & F_TLM)
                    {
                        for (int i = 0; i < 5; i++)
                            Telemetry[idx].VAL[i] = aprs.telemetry.val[i];
                        Telemetry[idx].BITS = aprs.telemetry.bits;
                        show = true;
                    }

                    for (int i = 0; i < 4; i++)
                    { // Cut length
                        if (strstr(Telemetry[idx].PARM[i], "RxTraffic") != 0)
                            sprintf(Telemetry[idx].PARM[i], "RX");
                        if (strstr(Telemetry[idx].PARM[i], "TxTraffic") != 0)
                            sprintf(Telemetry[idx].PARM[i], "TX");
                        if (strstr(Telemetry[idx].PARM[i], "RxDrop") != 0)
                            sprintf(Telemetry[idx].PARM[i], "DROP");
                        Telemetry[idx].PARM[i][6] = 0;
                        Telemetry[idx].UNIT[i][3] = 0;
                        for (int a = 0; a < 3; a++)
                        {
                            if (Telemetry[idx].UNIT[i][a] == '/')
                                Telemetry[idx].UNIT[i][a] = 0;
                        }
                    }

                    for (int i = 0; i < 5; i++)
                    {
                        if (Telemetry[idx].PARM[i][0] == 0)
                        {
                            sprintf(Telemetry[idx].PARM[i], "CH%d", i + 1);
                        }
                    }
                }
                if (show || filter == false)
                {
                    display.drawRoundRect(0, 16, 128, 48, 5, WHITE);
                    display.fillRoundRect(1, 17, 126, 10, 2, WHITE);
                    display.setTextColor(BLACK);
                    display.setCursor(40, 18);
                    display.print("TELEMETRY");
                    display.setFont();
                    display.setTextColor(WHITE);
                    display.setCursor(2, 28);
                    display.print(Telemetry[idx].PARM[0]);
                    display.print(":");

                    if (fmod(Telemetry[idx].VAL[0], 1) == 0)
                        display.print(Telemetry[idx].VAL[0], 0);
                    else
                        display.print(Telemetry[idx].VAL[0], 1);
                    display.print(Telemetry[idx].UNIT[0]);
                    display.setCursor(65, 28);
                    display.print(Telemetry[idx].PARM[1]);
                    display.print(":");
                    if (fmod(Telemetry[idx].VAL[1], 1) == 0)
                        display.print(Telemetry[idx].VAL[1], 0);
                    else
                        display.print(Telemetry[idx].VAL[1], 1);
                    display.print(Telemetry[idx].UNIT[1]);
                    display.setCursor(2, 37);
                    display.print(Telemetry[idx].PARM[2]);
                    display.print(":");
                    if (fmod(Telemetry[idx].VAL[2], 1) == 0)
                        display.print(Telemetry[idx].VAL[2], 0);
                    else
                        display.print(Telemetry[idx].VAL[2], 1);
                    display.print(Telemetry[idx].UNIT[2]);
                    display.setCursor(65, 37);
                    display.print(Telemetry[idx].PARM[3]);
                    display.print(":");
                    if (fmod(Telemetry[idx].VAL[3], 1) == 0)
                        display.print(Telemetry[idx].VAL[3], 0);
                    else
                        display.print(Telemetry[idx].VAL[3], 1);
                    display.print(Telemetry[idx].UNIT[3]);
                    display.setCursor(2, 46);
                    display.print(Telemetry[idx].PARM[4]);
                    display.print(":");
                    display.print(Telemetry[idx].VAL[4], 1);
                    display.print(Telemetry[idx].UNIT[4]);

                    display.setCursor(4, 55);
                    display.print("BIT");
                    uint8_t bit = Telemetry[idx].BITS;
                    for (int i = 0; i < 8; i++)
                    {
                        if (bit & 0x80)
                        {
                            display.fillCircle(30 + (i * 12), 58, 3, WHITE);
                        }
                        else
                        {
                            display.drawCircle(30 + (i * 12), 58, 3, WHITE);
                        }
                        bit <<= 1;
                    }
                    display.display();
                }
                return;
            }
            else if (aprs.packettype & T_STATUS)
            {
                display.drawRoundRect(0, 16, 128, 48, 5, WHITE);
                display.fillRoundRect(1, 17, 126, 10, 2, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(48, 18);
                display.print("STATUS");

                display.setFont();
                display.setCursor(2, 30);
                // memset(&text[0], 0, sizeof(text));
                // memcpy(&text[0], aprs.comment, aprs.comment_len);
                display.setTextColor(WHITE);
                display.print(aprs.comment);
                display.display();
                return;
            }
            else if (aprs.packettype & T_QUERY)
            {
                display.drawRoundRect(0, 16, 128, 48, 5, WHITE);
                display.fillRoundRect(1, 17, 126, 10, 2, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(48, 18);
                display.print("?QUERY?");
                // memset(&text[0], 0, sizeof(text));
                // memcpy(&text[0], aprs.comment, aprs.comment_len);
                display.setFont();
                display.setTextColor(WHITE);
                display.setCursor(2, 30);
                display.print(aprs.comment);
                display.display();
                return;
            }
            else if (aprs.packettype & T_MESSAGE)
            {
                if (aprs.msg.is_ack == 1)
                {
                }
                else if (aprs.msg.is_rej == 1)
                {
                }
                else
                {
                    display.drawRoundRect(0, 16, 128, 48, 5, WHITE);
                    display.fillRoundRect(1, 17, 126, 10, 2, WHITE);
                    display.setTextColor(BLACK);
                    display.setCursor(48, 18);
                    display.print("MESSAGE");
                    display.setCursor(100, 18);
                    display.print("{");
                    char txtID[7];
                    memset(txtID, 0, sizeof(txtID));
                    strncpy(&txtID[0], aprs.msg.msgid, aprs.msg.msgid_len);
                    int msgid = atoi(txtID);
                    // display.print(msgid, DEC);
                    display.printf("%s", txtID);
                    display.print("}");
                    // memset(&text[0], 0, sizeof(text));
                    // memcpy(&text[0], aprs.comment, aprs.comment_len);
                    display.setFont();
                    display.setTextColor(WHITE);
                    display.setCursor(2, 30);
                    display.print("To: ");
                    memset(text, 0, sizeof(text));
                    strncpy(&text[0], aprs.dstname, aprs.dstname_len);
                    display.print(text);
                    String mycall = "";
                    if (config.aprs_ssid > 0)
                        mycall = String(config.aprs_mycall) + String("-") + String(config.aprs_ssid, DEC);
                    else
                        mycall = String(config.aprs_mycall);
                    // if (strcmp(mycall.c_str(), text) == 0)
                    // {
                    //     display.setCursor(2, 54);
                    //     display.print("ACK:");
                    //     display.println(msgid);
                    //     String rawData = sendIsAckMsg(src_call, txtID);
                    //     log_d("IGATE_MSG: %s", rawData.c_str());
                    //     //if (config.igate_loc2rf)
                    //     { // IGATE SEND POSITION TO RF
                    //         char *rawP = (char *)malloc(rawData.length());
                    //         memcpy(rawP, rawData.c_str(), rawData.length());
                    //         pkgTxPush(rawP, rawData.length(), 0);
                    //         free(rawP);
                    //     }
                    //     // if (config.igate_loc2inet)
                    //     // { // IGATE SEND TO APRS-IS
                    //     //     if (aprsClient.connected())
                    //     //     {
                    //     //         aprsClient.println(rawData); // Send packet to Inet
                    //     //     }
                    //     // }
                    // }
                    memset(text, 0, sizeof(text));
                    strncpy(&text[0], aprs.msg.body, aprs.msg.body_len);
                    display.setCursor(2, 40);
                    display.print("Msg: ");
                    display.println(text);
                    display.display();
                }
                return;
            }
            display.setFont();
            display.drawFastHLine(0, 16, 128, WHITE);
            display.drawFastVLine(48, 16, 48, WHITE);
            x = 8;

            if (aprs.srcname_len > 0)
            {
                x += 9;
                display.fillRoundRect(51, 16, 77, 9, 2, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(53, x);
                display.print("By " + src_call);
                display.setTextColor(WHITE);
            }

            if (aprs.packettype & T_WAVE)
            {
                // Serial.println("WX Display");
                if (aprs.wave_report.flags & O_TEMP)
                {
                    display.setCursor(58, x += 10);
                    display.drawYBitmap(51, x, &Temperature_Symbol[0], 5, 8, WHITE);
                    display.printf("%.2fC", aprs.wave_report.Temp);
                }
                if (aprs.wave_report.flags & O_HS)
                {
                    // display.setCursor(102, x);
                    display.setCursor(58, x += 9);
                    display.printf("Hs:");
                    display.printf("%0.1f M", aprs.wave_report.Hs / 100);
                }
                if (aprs.wave_report.flags & O_TZ)
                {
                    display.setCursor(58, x += 9);
                    display.printf("Tz: ");
                    display.printf("%0.1f S", aprs.wave_report.Tz);
                }
                // if (aprs.wave_report.flags & O_TC)
                // {
                //     display.setCursor(58, x += 9);
                //     display.printf("Tc: ");
                //     display.printf("%0.1fS.", aprs.wave_report.Tc);
                // }
                if (aprs.wave_report.flags & O_BAT)
                {
                    display.setCursor(58, x += 9);
                    display.printf("BAT: ");
                    display.printf("%0.2fV", aprs.wave_report.Bat);
                }
            }
            if (aprs.packettype & T_WX)
            {
                // Serial.println("WX Display");
                if (aprs.wx_report.flags & W_TEMP)
                {
                    display.setCursor(58, x += 10);
                    display.drawYBitmap(51, x, &Temperature_Symbol[0], 5, 8, WHITE);
                    display.printf("%.1fC", aprs.wx_report.temp);
                }
                if (aprs.wx_report.flags & W_HUM)
                {
                    display.setCursor(102, x);
                    display.drawYBitmap(95, x, &Humidity_Symbol[0], 5, 8, WHITE);
                    display.printf("%0.1f%%", aprs.wx_report.humidity);
                }
                if (aprs.wx_report.flags & W_BAR)
                {
                    display.setCursor(58, x += 9);
                    display.drawYBitmap(51, x, &Pressure_Symbol[0], 5, 8, WHITE);
                    display.printf("%.1fhPa", aprs.wx_report.pressure);
                }
                if (aprs.wx_report.flags & W_R24H)
                {
                    // if (aprs.wx_report.rain_1h > 0) {
                    display.setCursor(58, x += 9);
                    display.drawYBitmap(51, x, &Rain_Symbol[0], 5, 8, WHITE);
                    display.printf("%.1fmm.", aprs.wx_report.rain_24h);
                    //}
                }
                if (aprs.wx_report.flags & W_PAR)
                {
                    // if (aprs.wx_report.luminosity > 10) {
                    display.setCursor(51, x += 9);
                    display.printf("%c", 0x0f);
                    display.setCursor(58, x);
                    display.printf("%dW/m", aprs.wx_report.luminosity);
                    if (aprs.wx_report.flags & W_UV)
                    {
                        display.printf(" UV%d", aprs.wx_report.uv);
                    }
                    //}
                }
                if (aprs.wx_report.flags & W_WS)
                {
                    display.setCursor(58, x += 9);
                    display.drawYBitmap(51, x, &Wind_Symbol[0], 5, 8, WHITE);
                    // int dirIdx=map(aprs.wx_report.wind_dir, -180, 180, 0, 8); ((angle+22)/45)%8]
                    int dirIdx = ((aprs.wx_report.wind_dir + 22) / 45) % 8;
                    if (dirIdx > 8)
                        dirIdx = 8;
                    display.printf("%.1fkPh(%s)", aprs.wx_report.wind_speed, directions[dirIdx]);
                }
                // Serial.printf("%.1fkPh(%d)", aprs.wx_report.wind_speed, aprs.wx_report.wind_dir);
                if (aprs.flags & F_HASPOS)
                {
                    // Serial.println("POS Display");
                    double lat, lon;
                    if (gps.location.isValid())
                    {
                        lat = gps.location.lat();
                        lon = gps.location.lng();
                    }
                    else
                    {
                        lat = config.igate_lat;
                        lon = config.igate_lon;
                    }
                    double dtmp = aprsParse.direction(lon, lat, aprs.lng, aprs.lat);
                    double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                    if (config.h_up == true)
                    {
                        // double course = gps.course.deg();
                        double course = SB_HEADING;
                        if (dtmp >= course)
                        {
                            dtmp -= course;
                        }
                        else
                        {
                            double diff = dtmp - course;
                            dtmp = diff + 360.0F;
                        }
                        compass_label(25, 37, 15, course, WHITE);
                        display.setCursor(0, 17);
                        display.printf("H");
                    }
                    else
                    {
                        compass_label(25, 37, 15, 0.0F, WHITE);
                    }
                    // compass_label(25, 37, 15, 0.0F, WHITE);
                    compass_arrow(25, 37, 12, dtmp, WHITE);
                    display.drawFastHLine(1, 63, 45, WHITE);
                    display.drawFastVLine(1, 58, 5, WHITE);
                    display.drawFastVLine(46, 58, 5, WHITE);
                    display.setCursor(4, 55);
                    if (dist > 999)
                        display.printf("%.fKm", dist);
                    else
                        display.printf("%.1fKm", dist);
                }
                else
                {
                    display.setCursor(20, 30);
                    display.printf("NO\nPOSITION");
                }
            }
            else if (aprs.flags & F_HASPOS)
            {
                // display.setCursor(50, x += 10);
                // display.printf("LAT %.5f\n", aprs.lat);
                // display.setCursor(51, x+=9);
                // display.printf("LNG %.4f\n", aprs.lng);
                String str;
                int l = 0;
                display.setCursor(50, x += 10);
                display.print("LAT:");
                str = String(aprs.lat, 5);
                l = str.length() * 6;
                display.setCursor(128 - l, x);
                display.print(str);

                display.setCursor(50, x += 9);
                display.print("LON:");
                str = String(aprs.lng, 5);
                l = str.length() * 6;
                display.setCursor(128 - l, x);
                display.print(str);

                double lat, lon;
                if (gps.location.isValid())
                {
                    lat = gps.location.lat();
                    lon = gps.location.lng();
                }
                else
                {
                    lat = config.igate_lat;
                    lon = config.igate_lon;
                }
                double dtmp = aprsParse.direction(lon, lat, aprs.lng, aprs.lat);
                double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                if (config.h_up == true)
                {
                    // double course = gps.course.deg();
                    double course = SB_HEADING;
                    if (dtmp >= course)
                    {
                        dtmp -= course;
                    }
                    else
                    {
                        double diff = dtmp - course;
                        dtmp = diff + 360.0F;
                    }
                    compass_label(25, 37, 15, course, WHITE);
                    display.setCursor(0, 17);
                    display.printf("H");
                }
                else
                {
                    compass_label(25, 37, 15, 0.0F, WHITE);
                }
                compass_arrow(25, 37, 12, dtmp, WHITE);
                display.drawFastHLine(1, 55, 45, WHITE);
                display.drawFastVLine(1, 55, 5, WHITE);
                display.drawFastVLine(46, 55, 5, WHITE);
                display.setCursor(4, 57);
                if (dist > 999)
                    display.printf("%.fKm", dist);
                else
                    display.printf("%.1fKm", dist);
                if (aprs.flags & F_CSRSPD)
                {
                    display.setCursor(51, x += 9);
                    // display.printf("SPD %d/", aprs.course);
                    // display.setCursor(50, x += 9);
                    display.printf("SPD %.1fkPh\n", aprs.speed);
                    int dirIdx = ((aprs.course + 22) / 45) % 8;
                    if (dirIdx > 8)
                        dirIdx = 8;
                    display.setCursor(51, x += 9);
                    display.printf("CSD %d(%s)", aprs.course, directions[dirIdx]);
                }
                if (aprs.flags & F_ALT)
                {
                    display.setCursor(51, x += 9);
                    display.printf("ALT %.1fM\n", aprs.altitude);
                }
                if (aprs.flags & F_PHG)
                {
                    int power, height, gain;
                    unsigned char tmp;
                    power = (int)aprs.phg[0] - 0x30;
                    power *= power;
                    height = (int)aprs.phg[1] - 0x30;
                    height = 10 << (height + 1);
                    height = height / 3.2808;
                    gain = (int)aprs.phg[2] - 0x30;
                    display.setCursor(51, x += 9);
                    display.printf("PHG %dM.\n", height);
                    display.setCursor(51, x += 9);
                    display.printf("PWR %dWatt\n", power);
                    display.setCursor(51, x += 9);
                    display.printf("ANT %ddBi\n", gain);
                }
                if (aprs.flags & F_RNG)
                {
                    display.setCursor(51, x += 9);
                    display.printf("RNG %dKm\n", aprs.radio_range);
                }
                /*if (aprs.comment_len > 0) {
                    display.setCursor(0, 56);
                    display.print(aprs.comment);
                }*/
            }
#endif
            display.display();
#elif defined(ST7735_160x80)
            display.fillScreen(ST77XX_BLACK);
            ledcWrite(ST7735_LED_K_Pin, config.disp_brightness);
            if (dispPush)
            {
                disp_delay = 600 * 1000;
                display.drawRoundRect(0, 0, 160, 16, 3, WHITE);
            }
            else
            {
                disp_delay = config.dispDelay * 1000;
            }
            timeHalfSec = millis() + disp_delay;

            const uint8_t *ptrSymbol;
            uint8_t symIdx = aprs.symbol[1] - 0x21;
            if (symIdx > 95)
                symIdx = 0;
            if (aprs.symbol[0] == '/')
            {
                ptrSymbol = &Icon_TableA[symIdx][0];
            }
            else if (aprs.symbol[0] == '\\')
            {
                ptrSymbol = &Icon_TableB[symIdx][0];
            }
            else
            {
                if (aprs.symbol[0] < 'A' || aprs.symbol[0] > 'Z')
                {
                    aprs.symbol[0] = 'N';
                    aprs.symbol[1] = '&';
                    symIdx = 5; // &
                }
                ptrSymbol = &Icon_TableB[symIdx][0];
            }
            display.drawYBitmap(0, 0, ptrSymbol, 16, 16, ST77XX_BLUE);
            if (!(aprs.symbol[0] == '/' || aprs.symbol[0] == '\\'))
            {
                display.drawChar(5, 4, aprs.symbol[0], BLACK, WHITE, 1);
                display.drawChar(6, 5, aprs.symbol[0], BLACK, WHITE, 1);
            }
            display.setCursor(20, 7);
            display.setTextSize(1);
            display.setTextColor(ST77XX_RED);
            display.setFont(&FreeSansBold9pt7b);

            if (aprs.srcname_len > 0)
            {
                memset(&itemname, 0, sizeof(itemname));
                memcpy(&itemname, aprs.srcname, aprs.srcname_len);
                // Serial.println(itemname);
                display.print(itemname);
            }
            else
            {
                display.print(src_call);
            }

            display.setFont();
            display.setTextColor(ST77XX_CYAN);

            if (mode == 1)
            {
                display.drawRoundRect(0, 16, 160, 64, 5, WHITE);
                display.fillRoundRect(1, 17, 160, 10, 2, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(40, 18);
                display.print("TNC2 RAW");

                display.setFont();
                display.setCursor(2, 30);
                display.setTextColor(WHITE);
                display.print(line);
                display.display();
                return;
            }

            if (aprs.packettype & T_TELEMETRY)
            {
                bool show = false;
                int idx = tlmList_Find((char *)src_call.c_str());
                if (idx < 0)
                {
                    idx = tlmListOld();
                    if (idx > -1)
                        memset(&Telemetry[idx], 0, sizeof(Telemetry_struct));
                }
                if (idx > -1)
                {
                    Telemetry[idx].time = now();
                    strcpy(Telemetry[idx].callsign, (char *)src_call.c_str());

                    // for (int i = 0; i < 3; i++) Telemetry[idx].UNIT[i][5] = 0;
                    if (aprs.flags & F_UNIT)
                    {
                        memcpy(Telemetry[idx].UNIT, aprs.tlm_unit.val, sizeof(Telemetry[idx].UNIT));
                    }
                    else if (aprs.flags & F_PARM)
                    {
                        memcpy(Telemetry[idx].PARM, aprs.tlm_parm.val, sizeof(Telemetry[idx].PARM));
                    }
                    else if (aprs.flags & F_EQNS)
                    {
                        for (int i = 0; i < 15; i++)
                            Telemetry[idx].EQNS[i] = aprs.tlm_eqns.val[i];
                    }
                    else if (aprs.flags & F_BITS)
                    {
                        Telemetry[idx].BITS_FLAG = aprs.telemetry.bitsFlag;
                    }
                    else if (aprs.flags & F_TLM)
                    {
                        for (int i = 0; i < 5; i++)
                            Telemetry[idx].VAL[i] = aprs.telemetry.val[i];
                        Telemetry[idx].BITS = aprs.telemetry.bits;
                        show = true;
                    }

                    for (int i = 0; i < 4; i++)
                    { // Cut length
                        if (strstr(Telemetry[idx].PARM[i], "RxTraffic") != 0)
                            sprintf(Telemetry[idx].PARM[i], "RX");
                        if (strstr(Telemetry[idx].PARM[i], "TxTraffic") != 0)
                            sprintf(Telemetry[idx].PARM[i], "TX");
                        if (strstr(Telemetry[idx].PARM[i], "RxDrop") != 0)
                            sprintf(Telemetry[idx].PARM[i], "DROP");
                        Telemetry[idx].PARM[i][6] = 0;
                        Telemetry[idx].UNIT[i][3] = 0;
                        for (int a = 0; a < 3; a++)
                        {
                            if (Telemetry[idx].UNIT[i][a] == '/')
                                Telemetry[idx].UNIT[i][a] = 0;
                        }
                    }

                    for (int i = 0; i < 5; i++)
                    {
                        if (Telemetry[idx].PARM[i][0] == 0)
                        {
                            sprintf(Telemetry[idx].PARM[i], "CH%d", i + 1);
                        }
                    }
                }
                if (show || filter == false)
                {
                    display.drawRoundRect(0, 16, 160, 64, 5, WHITE);
                    display.fillRoundRect(1, 17, 158, 10, 2, WHITE);
                    display.setTextColor(BLACK);
                    display.setCursor(50, 18);
                    display.print("TELEMETRY");
                    display.setFont();
                    display.setTextColor(WHITE);
                    display.setCursor(2, 28);
                    display.print(Telemetry[idx].PARM[0]);
                    display.print(":");

                    if (fmod(Telemetry[idx].VAL[0], 1) == 0)
                        display.print(Telemetry[idx].VAL[0], 0);
                    else
                        display.print(Telemetry[idx].VAL[0], 1);
                    display.print(Telemetry[idx].UNIT[0]);
                    display.setCursor(81, 28);
                    display.print(Telemetry[idx].PARM[1]);
                    display.print(":");
                    if (fmod(Telemetry[idx].VAL[1], 1) == 0)
                        display.print(Telemetry[idx].VAL[1], 0);
                    else
                        display.print(Telemetry[idx].VAL[1], 1);
                    display.print(Telemetry[idx].UNIT[1]);
                    display.setCursor(2, 37);
                    display.print(Telemetry[idx].PARM[2]);
                    display.print(":");
                    if (fmod(Telemetry[idx].VAL[2], 1) == 0)
                        display.print(Telemetry[idx].VAL[2], 0);
                    else
                        display.print(Telemetry[idx].VAL[2], 1);
                    display.print(Telemetry[idx].UNIT[2]);
                    display.setCursor(81, 37);
                    display.print(Telemetry[idx].PARM[3]);
                    display.print(":");
                    if (fmod(Telemetry[idx].VAL[3], 1) == 0)
                        display.print(Telemetry[idx].VAL[3], 0);
                    else
                        display.print(Telemetry[idx].VAL[3], 1);
                    display.print(Telemetry[idx].UNIT[3]);
                    display.setCursor(2, 46);
                    display.print(Telemetry[idx].PARM[4]);
                    display.print(":");
                    display.print(Telemetry[idx].VAL[4], 1);
                    display.print(Telemetry[idx].UNIT[4]);

                    display.setCursor(4, 55);
                    display.print("BIT");
                    uint8_t bit = Telemetry[idx].BITS;
                    for (int i = 0; i < 8; i++)
                    {
                        if (bit & 0x80)
                        {
                            display.fillCircle(30 + (i * 12), 58, 3, WHITE);
                        }
                        else
                        {
                            display.drawCircle(30 + (i * 12), 58, 3, WHITE);
                        }
                        bit <<= 1;
                    }
                    display.display();
                }
                return;
            }
            else if (aprs.packettype & T_STATUS)
            {
                display.drawRoundRect(0, 16, 160, 64, 5, WHITE);
                display.fillRoundRect(1, 17, 158, 10, 2, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(58, 18);
                display.print("STATUS");

                display.setFont();
                display.setCursor(2, 30);
                // memset(&text[0], 0, sizeof(text));
                // memcpy(&text[0], aprs.comment, aprs.comment_len);
                display.setTextColor(WHITE);
                display.print(aprs.comment);
                display.display();
                return;
            }
            else if (aprs.packettype & T_QUERY)
            {
                display.drawRoundRect(0, 16, 160, 64, 5, WHITE);
                display.fillRoundRect(1, 17, 158, 10, 2, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(58, 18);
                display.print("?QUERY?");
                // memset(&text[0], 0, sizeof(text));
                // memcpy(&text[0], aprs.comment, aprs.comment_len);
                display.setFont();
                display.setTextColor(WHITE);
                display.setCursor(2, 30);
                display.print(aprs.comment);
                display.display();
                return;
            }
            else if (aprs.packettype & T_MESSAGE)
            {
                if (aprs.msg.is_ack == 1)
                {
                }
                else if (aprs.msg.is_rej == 1)
                {
                }
                else
                {
                    display.drawRoundRect(0, 16, 160, 64, 5, WHITE);
                    display.fillRoundRect(1, 17, 158, 10, 2, WHITE);
                    display.setTextColor(BLACK);
                    display.setCursor(58, 18);
                    display.print("MESSAGE");
                    display.setCursor(100, 18);
                    display.print("{");
                    char txtID[7];
                    memset(txtID, 0, sizeof(txtID));
                    strncpy(&txtID[0], aprs.msg.msgid, aprs.msg.msgid_len);
                    int msgid = atoi(txtID);
                    // display.print(msgid, DEC);
                    display.print(txtID);

                    display.print("}");
                    // memset(&text[0], 0, sizeof(text));
                    // memcpy(&text[0], aprs.comment, aprs.comment_len);
                    display.setFont();
                    display.setTextColor(WHITE);
                    display.setCursor(2, 30);
                    display.print("To: ");
                    memset(text, 0, sizeof(text));
                    strncpy(&text[0], aprs.dstname, aprs.dstname_len);
                    display.print(text);
                    String mycall = "";
                    if (config.aprs_ssid > 0)
                        mycall = String(config.aprs_mycall) + String("-") + String(config.aprs_ssid, DEC);
                    else
                        mycall = String(config.aprs_mycall);
                    memset(text, 0, sizeof(text));
                    strncpy(&text[0], aprs.msg.body, aprs.msg.body_len);
                    display.setCursor(2, 40);
                    display.print("Msg: ");
                    display.println(text);
                    display.display();
                }
                return;
            }
            display.setFont();
            display.drawFastHLine(0, 16, 160, WHITE);
            display.drawFastVLine(53, 16, 64, WHITE);
            x = 8;

            if (aprs.srcname_len > 0)
            {
                x += 9;
                display.fillRoundRect(61, 16, 99, 9, 2, WHITE);
                display.setTextColor(BLACK);
                display.setCursor(53, x);
                display.print("By " + src_call);
                display.setTextColor(WHITE);
            }

            if (aprs.packettype & T_WAVE)
            {
                // Serial.println("WX Display");
                if (aprs.wave_report.flags & O_TEMP)
                {
                    display.setCursor(68, x += 10);
                    display.drawYBitmap(61, x, &Temperature_Symbol[0], 5, 8, WHITE);
                    // display.printf("%.2fC", aprs.wave_report.Temp);
                    display.print(aprs.wave_report.Temp, 2);
                    display.print("C");
                }
                if (aprs.wave_report.flags & O_HS)
                {
                    // display.setCursor(102, x);
                    display.setCursor(68, x += 9);
                    display.print("Hs: ");
                    // display.printf("%0.1f M", aprs.wave_report.Hs / 100);
                    display.print(aprs.wave_report.Hs / 100, 1);
                    display.print("M");
                }
                if (aprs.wave_report.flags & O_TZ)
                {
                    display.setCursor(68, x += 9);
                    display.print("Tz: ");
                    // display.printf("%0.1f S", aprs.wave_report.Tz);
                    display.print(aprs.wave_report.Tz, 1);
                    display.print("S");
                }
                // if (aprs.wave_report.flags & O_TC)
                // {
                //     display.setCursor(58, x += 9);
                //     display.printf("Tc: ");
                //     display.printf("%0.1fS.", aprs.wave_report.Tc);
                // }
                if (aprs.wave_report.flags & O_BAT)
                {
                    display.setCursor(68, x += 9);
                    display.print("BAT: ");
                    // display.printf("%0.2fV", aprs.wave_report.Bat);
                    display.print(aprs.wave_report.Bat, 2);
                    display.print("V");
                }
            }

            if (aprs.packettype & T_WX)
            {
                display.setFont();
                display.setTextColor(WHITE);
                // Serial.println("WX Display");
                if (aprs.wx_report.flags & W_TEMP)
                {
                    display.setCursor(68, x += 10);
                    display.drawYBitmap(61, x, &Temperature_Symbol[0], 5, 8, WHITE);
                    // display.printf("%.1fC", aprs.wx_report.temp);
                    display.print(aprs.wx_report.temp, 2);
                    display.print("C");
                }
                if (aprs.wx_report.flags & W_HUM)
                {
                    display.setCursor(122, x);
                    display.drawYBitmap(115, x, &Humidity_Symbol[0], 5, 8, WHITE);
                    display.printf("%0.1f%%", aprs.wx_report.humidity);
                }
                if (aprs.wx_report.flags & W_BAR)
                {
                    display.setCursor(68, x += 9);
                    display.drawYBitmap(61, x, &Pressure_Symbol[0], 5, 8, WHITE);
                    // display.printf("%.1fhPa", aprs.wx_report.pressure);
                    display.print(aprs.wx_report.pressure, 1);
                    display.print("hPa");
                }
                if (aprs.wx_report.flags & W_R24H)
                {
                    // if (aprs.wx_report.rain_1h > 0) {
                    display.setCursor(68, x += 9);
                    display.drawYBitmap(61, x, &Rain_Symbol[0], 5, 8, WHITE);
                    // display.printf("%.1fmm.", aprs.wx_report.rain_24h);
                    display.print(aprs.wx_report.rain_24h, 1);
                    display.print("mm");
                    //}
                }
                if (aprs.wx_report.flags & W_PAR)
                {
                    // if (aprs.wx_report.luminosity > 10) {
                    display.setCursor(61, x += 9);
                    display.printf("%c", 0x0f);
                    display.setCursor(68, x);
                    // display.printf("%dW/m", aprs.wx_report.luminosity);
                    display.print(aprs.wx_report.luminosity);
                    display.print("W/m");
                    if (aprs.wx_report.flags & W_UV)
                    {
                        // display.printf(" UV%d", aprs.wx_report.uv);
                        display.print("UV:");
                        display.print(aprs.wx_report.uv);
                    }
                    //}
                }
                if (aprs.wx_report.flags & W_WS)
                {
                    display.setCursor(68, x += 9);
                    display.drawYBitmap(61, x, &Wind_Symbol[0], 5, 8, WHITE);
                    // int dirIdx=map(aprs.wx_report.wind_dir, -180, 180, 0, 8); ((angle+22)/45)%8]
                    int dirIdx = ((aprs.wx_report.wind_dir + 22) / 45) % 8;
                    if (dirIdx > 8)
                        dirIdx = 8;
                    // display.printf("%.1fkPh(%s)", aprs.wx_report.wind_speed, directions[dirIdx]);
                    display.print(aprs.wx_report.wind_speed, 1);
                    display.print("kPh(");
                    display.print(directions[dirIdx]);
                    display.print(")");
                }
                // Serial.printf("%.1fkPh(%d)", aprs.wx_report.wind_speed, aprs.wx_report.wind_dir);
                if (aprs.flags & F_HASPOS)
                {
                    // Serial.println("POS Display");
                    double lat, lon;
                    if (gps.location.isValid())
                    {
                        lat = gps.location.lat();
                        lon = gps.location.lng();
                    }
                    else
                    {
                        lat = config.igate_lat;
                        lon = config.igate_lon;
                    }
                    double dtmp = aprsParse.direction(lon, lat, aprs.lng, aprs.lat);
                    double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                    if (config.h_up == true)
                    {
                        // double course = gps.course.deg();
                        double course = SB_HEADING;
                        if (dtmp >= course)
                        {
                            dtmp -= course;
                        }
                        else
                        {
                            double diff = dtmp - course;
                            dtmp = diff + 360.0F;
                        }
                        compass_label(25, 37, 20, course, ST77XX_GREEN);
                        display.setCursor(0, 17);
                        display.print("H");
                    }
                    else
                    {
                        compass_label(25, 37, 20, 0.0F, ST77XX_GREEN);
                    }
                    // compass_label(25, 37, 15, 0.0F, WHITE);
                    compass_arrow(25, 37, 17, dtmp, ST77XX_GREEN);
                    display.drawFastHLine(1, 73, 55, WHITE);
                    display.drawFastVLine(1, 68, 5, WHITE);
                    display.drawFastVLine(56, 68, 5, WHITE);
                    display.setCursor(4, 65);
                    // if (dist > 999)
                    //     //display.printf("%.fKm", dist);
                    //     display.print(dist,0);
                    // else
                    display.print(dist, 1);
                    display.print("Km");
                }
                else
                {
                    display.setCursor(20, 30);
                    display.print("NO\nPOSITION");
                }
            }
            else if (aprs.flags & F_HASPOS)
            {
                // display.setCursor(50, x += 10);
                // display.printf("LAT %.5f\n", aprs.lat);
                // display.setCursor(51, x+=9);
                // display.printf("LNG %.4f\n", aprs.lng);
                String str;
                int l = 0;
                display.setCursor(60, x += 10);
                display.print("LAT:");
                str = String(aprs.lat, 5);
                // l = str.length() * 6;
                // display.setCursor(160 - l-2, x);
                display.setCursor(85, x);
                display.print(str);

                display.setCursor(60, x += 9);
                display.print("LON:");
                str = String(aprs.lng, 5);
                // l = str.length() * 6;
                // display.setCursor(160 - l-2, x);
                display.setCursor(85, x);
                display.print(str);

                double lat, lon;
                if (gps.location.isValid())
                {
                    lat = gps.location.lat();
                    lon = gps.location.lng();
                }
                else
                {
                    lat = config.igate_lat;
                    lon = config.igate_lon;
                }
                double dtmp = aprsParse.direction(lon, lat, aprs.lng, aprs.lat);
                double dist = aprsParse.distance(lon, lat, aprs.lng, aprs.lat);
                if (config.h_up == true)
                {
                    // double course = gps.course.deg();
                    double course = SB_HEADING;
                    if (dtmp >= course)
                    {
                        dtmp -= course;
                    }
                    else
                    {
                        double diff = dtmp - course;
                        dtmp = diff + 360.0F;
                    }
                    compass_label(25, 37, 20, course, ST77XX_GREEN);
                    display.setCursor(0, 17);
                    display.print("H");
                }
                else
                {
                    compass_label(25, 43, 20, 0.0F, ST77XX_GREEN);
                }
                compass_arrow(25, 43, 17, dtmp, ST77XX_GREEN);
                // display.drawFastHLine(1, 65, 55, WHITE);
                // display.drawFastVLine(1, 65, 5, WHITE);
                // display.drawFastVLine(56, 65, 5, WHITE);
                display.setTextColor(ST77XX_ORANGE);
                display.setCursor(0, 70);
                // if (dist > 99)
                //     display.printf("DX:%dKm", (int)dist);
                // else
                //     display.printf("DX:%.1fKm", dist);
                display.print("DX:");
                display.print(dist, 1);
                display.print("Km");
                display.setTextColor(WHITE);
                if (aprs.flags & F_CSRSPD)
                {
                    display.setCursor(61, x += 9);
                    // display.printf("SPD %d/", aprs.course);
                    // display.setCursor(50, x += 9);
                    // display.printf("SPD %.1fkPh\n", aprs.speed);
                    display.print("SPD: ");
                    display.print(aprs.speed, 1);
                    display.print("Kph");
                    int dirIdx = ((aprs.course + 22) / 45) % 8;
                    if (dirIdx > 8)
                        dirIdx = 8;
                    display.setCursor(61, x += 9);
                    // display.printf("CSD %d(%s)", aprs.course, directions[dirIdx]);
                    display.print("CSD: ");
                    display.print(aprs.course);
                    display.print("(");
                    display.print(directions[dirIdx]);
                    display.print(")");
                }
                if (aprs.flags & F_ALT)
                {
                    display.setCursor(61, x += 9);
                    // display.printf("ALT %.1fM\n", aprs.altitude);
                    display.print("ALT: ");
                    display.print(aprs.altitude, 1);
                    display.print("M");
                }
                if (aprs.flags & F_PHG)
                {
                    int power, height, gain;
                    unsigned char tmp;
                    power = (int)aprs.phg[0] - 0x30;
                    power *= power;
                    height = (int)aprs.phg[1] - 0x30;
                    height = 10 << (height + 1);
                    height = height / 3.2808;
                    gain = (int)aprs.phg[2] - 0x30;
                    display.setCursor(61, x += 9);
                    // display.printf("PHG %dM.\n", height);
                    display.print("PHG: ");
                    display.print(height);
                    display.print("M");
                    display.setCursor(61, x += 9);
                    // display.printf("PWR %dWatt\n", power);
                    display.print("PWR: ");
                    display.print(power);
                    display.print("Watt");
                    display.setCursor(61, x += 9);
                    // display.printf("ANT %ddBi\n", gain);
                    display.print("ANT: ");
                    display.print(gain);
                    display.print("dBi");
                }
                if (aprs.flags & F_RNG)
                {
                    display.setCursor(61, x += 9);
                    // display.printf("RNG %dKm\n", aprs.radio_range);
                    display.print("RNG: ");
                    display.print(aprs.radio_range);
                    display.print("Km");
                }
                /*if (aprs.comment_len > 0) {
                    display.setCursor(0, 56);
                    display.print(aprs.comment);
                }*/
            }
            display.display();
#endif
        }
    }
}

void statisticsDisp()
{

    // uint8 wifi = 0, i;
    int x;
    String str;
    // display.fillRect(0, 16, 128, 10, WHITE);
    // display.drawLine(0, 16, 0, 63, WHITE);
    // display.drawLine(127, 16, 127, 63, WHITE);
    // display.drawLine(0, 63, 127, 63, WHITE);
    // display.fillRect(1, 25, 126, 38, BLACK);
    // display.setTextColor(BLACK);
    // display.setCursor(30, 17);
    // display.print("STATISTICS");
    // display.setCursor(108, 17);
    // display.print("1/5");
    // display.setTextColor(WHITE);
#ifdef OLED
#ifdef SSD1306_72x40
    display.clearDisplay();
    display.fillRect(0, 0, 72, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(10, 1);
    display.print("STATISTIC");
    display.setTextColor(WHITE);
    display.setCursor(0, 12);
    display.print("A:");
    str = String(status.allCount, DEC);
    x = str.length() * 6;
    display.setCursor(71 - x, 12);
    display.print(str);
    display.setCursor(0, 22);
    display.print("G:");
    str = String(status.rf2inet, DEC);
    x = str.length() * 6;
    display.setCursor(71 - x, 22);
    display.print(str);
    display.setCursor(0, 32);
    display.print("E:");
    str = String(status.errorCount + status.dropCount, DEC);
    x = str.length() * 6;
    display.setCursor(71 - x, 32);
    display.print(str);
    display.display();
#else
    display.fillRect(0, 0, 128, 15, WHITE);
    display.drawRect(0, 16, 128, 48, WHITE);
    display.fillRect(1, 17, 126, 46, BLACK);

    display.setCursor(5, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("STATISTIC");
    display.setFont();
    display.setCursor(119, 7);
    display.print("1");
    display.setTextColor(WHITE);

    display.setCursor(3, 18);
    display.print("ALL DATA");
    str = String(status.allCount, DEC);
    x = str.length() * 6;
    display.setCursor(126 - x, 18);
    display.print(str);

    display.setCursor(3, 26);
    display.print("DIGI RPT");
    str = String(status.digiCount, DEC);
    x = str.length() * 6;
    display.setCursor(126 - x, 26);
    display.print(str);

    display.setCursor(3, 35);
    display.print("RF->INET");
    str = String(status.rf2inet, DEC);
    x = str.length() * 6;
    display.setCursor(126 - x, 35);
    display.print(str);

    display.setCursor(3, 44);
    display.print("INET->RF");
    str = String(status.inet2rf, DEC);
    x = str.length() * 6;
    display.setCursor(126 - x, 44);
    display.print(str);

    display.setCursor(3, 53);
    display.print("ERROR/DROP");
    str = String(status.errorCount + status.dropCount, DEC);
    x = str.length() * 6;
    display.setCursor(126 - x, 53);
    display.print(str);
    display.display();
#endif
#elif defined(ST7735_160x80)
    display.fillRect(0, 0, 160, 15, WHITE);
    // display.drawRect(0, 16, 160, 64, WHITE);
    display.fillRect(0, 15, 160, 64, BLACK);

    display.setCursor(20, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("STATISTIC");
    display.setFont();
    display.setCursor(150, 7);
    display.print("1");
    display.setTextColor(WHITE);

    display.setCursor(3, 18);
    display.print("ALL DATA");
    str = String(status.allCount, DEC);
    x = str.length() * 6;
    display.setCursor(158 - x, 18);
    display.print(str);

    display.setCursor(3, 28);
    display.print("DIGI RPT");
    str = String(status.digiCount, DEC);
    x = str.length() * 6;
    display.setCursor(158 - x, 28);
    display.print(str);

    display.setCursor(3, 38);
    display.print("RF->INET");
    str = String(status.rf2inet, DEC);
    x = str.length() * 6;
    display.setCursor(158 - x, 38);
    display.print(str);

    display.setCursor(3, 48);
    display.print("INET->RF");
    str = String(status.inet2rf, DEC);
    x = str.length() * 6;
    display.setCursor(158 - x, 48);
    display.print(str);

    display.setCursor(3, 58);
    display.print("ERROR/DROP");
    str = String(status.errorCount + status.dropCount, DEC);
    x = str.length() * 6;
    display.setCursor(158 - x, 58);
    display.print(str);

    display.setCursor(3, 68);
    display.print("TX/RX");
    str = String(status.txCount) + "/" + String(status.rxCount);
    x = str.length() * 6;
    display.setCursor(158 - x, 68);
    display.print(str);
    display.display();
#endif
}

void pkgLastDisp()
{

    uint8_t k = 0;
    int i;
    // char list[4];
    int x, y;
    String str;
    // String times;
    // pkgListType *ptr[100];
#ifdef OLED
#ifdef SSD1306_72x40
    display.clearDisplay();
    display.fillRect(0, 0, 72, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(15, 1);
    display.print("STATION");
    display.setTextColor(WHITE);
    sort(pkgList, PKGLISTSIZE);
    k = 0;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[i].time > 0)
        {
            y = 12 + (k * 9);
            display.setCursor(0, y);
            pkgList[i].calsign[10] = 0;
            display.setTextColor(WHITE);
            display.setCursor(0, y);
            display.printf("%d:%s", i + 1, pkgList[i].calsign);
            k++;
            if (k >= 3)
                break;
        }
    }
#else
    display.fillRect(0, 0, 128, 15, WHITE);
    display.drawRect(0, 16, 128, 48, WHITE);
    display.fillRect(1, 17, 126, 46, BLACK);

    display.setCursor(15, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("STATION");
    display.setFont();
    display.setCursor(119, 7);
    display.print("2");
    display.setTextColor(WHITE);

    // display.fillRect(0, 16, 128, 10, WHITE);
    // display.drawLine(0, 16, 0, 63, WHITE);
    // display.drawLine(127, 16, 127, 63, WHITE);
    // display.drawLine(0, 63, 127, 63, WHITE);
    // display.fillRect(1, 25, 126, 38, BLACK);
    // display.setTextColor(BLACK);
    // display.setCursor(27, 17);
    // display.print("LAST STATIONS");
    // display.setCursor(108, 17);
    // display.print("2/5");
    // display.setTextColor(WHITE);

    sort(pkgList, PKGLISTSIZE);
    k = 0;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[i].time > 0)
        {
            y = 18 + (k * 9);
            // display.drawBitmap(3, y, &SYMBOL[0][0], 11, 6, WHITE);
            display.fillRoundRect(2, y, 7, 8, 2, WHITE);
            display.setCursor(3, y);
            pkgList[i].calsign[10] = 0;
            display.setTextColor(BLACK);
            switch (pkgList[i].type)
            {
            case PKG_OBJECT:
                display.print("O");
                break;
            case PKG_ITEM:
                display.print("I");
                break;
            case PKG_MESSAGE:
                display.print("M");
                break;
            case PKG_WX:
                display.print("W");
                break;
            case PKG_TELEMETRY:
                display.print("T");
                break;
            case PKG_QUERY:
                display.print("Q");
                break;
            case PKG_STATUS:
                display.print("S");
                break;
            default:
                display.print("*");
                break;
            }
            display.setTextColor(WHITE);
            display.setCursor(10, y);
            display.print(pkgList[i].calsign);
            display.setCursor(126 - 48, y);
            // display.printf("%02d:%02d:%02d", hour(pkgList[i].time), minute(pkgList[i].time), second(pkgList[i].time));

            // time_t tm = pkgList[i].time;
            struct tm tmstruct;
            localtime_r(&pkgList[i].time, &tmstruct);
            String str = String(tmstruct.tm_hour, DEC) + ":" + String(tmstruct.tm_min, DEC) + ":" + String(tmstruct.tm_sec, DEC);
            display.print(str);
            // str = String(hour(pkgList[i].time),DEC) + ":" + String(minute(pkgList[i].time), DEC) + ":" + String(second(pkgList[i].time), DEC);
            ////str = String(pkgList[pkgLast_array[i]].time, DEC);
            // x = str.length() * 6;
            // display.setCursor(126 - x, y);
            // display.print(str);
            k++;
            if (k >= 5)
                break;
        }
    }
#endif
    display.display();
#elif defined(ST7735_160x80)
    display.fillRect(0, 0, 160, 15, WHITE);
    // display.drawRect(0, 16, 160, 64, WHITE);
    display.fillRect(0, 15, 160, 64, BLACK);

    display.setCursor(30, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("STATION");
    display.setFont();
    display.setCursor(150, 7);
    display.print("2");
    display.setTextColor(WHITE);

    sort(pkgList, PKGLISTSIZE);
    k = 0;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[i].time > 0)
        {
            y = 18 + (k * 10);
            // display.drawBitmap(3, y, &SYMBOL[0][0], 11, 6, WHITE);
            display.fillRoundRect(2, y, 7, 8, 2, WHITE);
            display.setCursor(3, y);
            pkgList[i].calsign[10] = 0;
            display.setTextColor(BLACK);
            switch (pkgList[i].type)
            {
            case PKG_OBJECT:
                display.print("O");
                break;
            case PKG_ITEM:
                display.print("I");
                break;
            case PKG_MESSAGE:
                display.print("M");
                break;
            case PKG_WX:
                display.print("W");
                break;
            case PKG_TELEMETRY:
                display.print("T");
                break;
            case PKG_QUERY:
                display.print("Q");
                break;
            case PKG_STATUS:
                display.print("S");
                break;
            default:
                display.print("*");
                break;
            }
            display.setTextColor(WHITE);
            display.setCursor(12, y);
            display.print(pkgList[i].calsign);
            display.setCursor(158 - 48, y);

            struct tm tmstruct;
            localtime_r(&pkgList[i].time, &tmstruct);
            String str = String(tmstruct.tm_hour, DEC) + ":" + String(tmstruct.tm_min, DEC) + ":" + String(tmstruct.tm_sec, DEC);
            display.print(str);
            k++;
            if (k >= 6)
                break;
        }
    }
    display.display();
#endif
}

void pkgCountDisp()
{

    // uint8 wifi = 0, k = 0, l;
    uint k = 0;
    int i;
    // char list[4];
    int x, y;
    String str;
    // String times;
    // pkgListType *ptr[100];
#ifdef OLED
#ifdef SSD1306_72x40
    display.clearDisplay();
    display.fillRect(0, 0, 72, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(15, 1);
    display.print("TOP PKG");
    display.setTextColor(WHITE);
    sortPkgDesc(pkgList, PKGLISTSIZE);
    k = 0;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[i].time > 0)
        {
            y = 12 + (k * 9);
            display.setCursor(0, y);
            pkgList[i].calsign[10] = 0;
            display.setTextColor(WHITE);
            display.setCursor(0, y);
            display.printf("%d:%s", i + 1, pkgList[i].calsign);
            k++;
            if (k >= 3)
                break;
        }
    }
#else
    display.fillRect(0, 0, 128, 15, WHITE);
    display.drawRect(0, 16, 128, 48, WHITE);
    display.fillRect(1, 17, 126, 46, BLACK);

    display.setCursor(20, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("TOP PKG");
    display.setFont();
    display.setCursor(119, 7);
    display.print("3");
    display.setTextColor(WHITE);

    // display.setCursor(3, 18);

    // display.fillRect(0, 16, 128, 10, WHITE);
    // display.drawLine(0, 16, 0, 63, WHITE);
    // display.drawLine(127, 16, 127, 63, WHITE);
    // display.drawLine(0, 63, 127, 63, WHITE);
    // display.fillRect(1, 25, 126, 38, BLACK);
    // display.setTextColor(BLACK);
    // display.setCursor(30, 17);
    // display.print("TOP PACKAGE");
    // display.setCursor(108, 17);
    // display.print("3/5");
    // display.setTextColor(WHITE);

    sortPkgDesc(pkgList, PKGLISTSIZE);
    k = 0;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[i].time > 0)
        {
            y = 18 + (k * 9);
            // display.drawBitmapV(2, y-1, &SYMBOL[pkgList[i].symbol][0], 11, 8, WHITE);
            pkgList[i].calsign[10] = 0;
            display.fillRoundRect(2, y, 7, 8, 2, WHITE);
            display.setCursor(3, y);
            pkgList[i].calsign[10] = 0;
            display.setTextColor(BLACK);
            switch (pkgList[i].type)
            {
            case PKG_OBJECT:
                display.print("O");
                break;
            case PKG_ITEM:
                display.print("I");
                break;
            case PKG_MESSAGE:
                display.print("M");
                break;
            case PKG_WX:
                display.print("W");
                break;
            case PKG_TELEMETRY:
                display.print("T");
                break;
            case PKG_QUERY:
                display.print("Q");
                break;
            case PKG_STATUS:
                display.print("S");
                break;
            default:
                display.print("*");
                break;
            }
            display.setTextColor(WHITE);
            display.setCursor(10, y);
            display.print(pkgList[i].calsign);
            str = String(pkgList[i].pkg, DEC);
            x = str.length() * 6;
            display.setCursor(126 - x, y);
            display.print(str);
            k++;
            if (k >= 5)
                break;
        }
    }
#endif
    display.display();
#elif defined(ST7735_160x80)
    display.fillRect(0, 0, 160, 15, WHITE);
    // display.drawRect(0, 16, 160, 64, WHITE);
    display.fillRect(0, 15, 160, 64, BLACK);

    display.setCursor(35, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("TOP PKG");
    display.setFont();
    display.setCursor(150, 7);
    display.print("3");
    display.setTextColor(WHITE);

    sortPkgDesc(pkgList, PKGLISTSIZE);
    k = 0;
    for (i = 0; i < PKGLISTSIZE; i++)
    {
        if (pkgList[i].time > 0)
        {
            y = 18 + (k * 10);
            // display.drawBitmapV(2, y-1, &SYMBOL[pkgList[i].symbol][0], 11, 8, WHITE);
            pkgList[i].calsign[10] = 0;
            display.fillRoundRect(2, y, 7, 8, 2, WHITE);
            display.setCursor(3, y);
            pkgList[i].calsign[10] = 0;
            display.setTextColor(BLACK);
            switch (pkgList[i].type)
            {
            case PKG_OBJECT:
                display.print("O");
                break;
            case PKG_ITEM:
                display.print("I");
                break;
            case PKG_MESSAGE:
                display.print("M");
                break;
            case PKG_WX:
                display.print("W");
                break;
            case PKG_TELEMETRY:
                display.print("T");
                break;
            case PKG_QUERY:
                display.print("Q");
                break;
            case PKG_STATUS:
                display.print("S");
                break;
            default:
                display.print("*");
                break;
            }
            display.setTextColor(WHITE);
            display.setCursor(10, y);
            display.print(pkgList[i].calsign);
            str = String(pkgList[i].pkg, DEC);
            x = str.length() * 6;
            display.setCursor(158 - x, y);
            display.print(str);
            k++;
            if (k >= 6)
                break;
        }
    }
    display.display();
#endif
}

void systemDisp()
{

    // uint8 wifi = 0, k = 0, l;
    // char i;
    // char list[4];
    int x;
    String str;
    time_t upTime = now() - systemUptime;
    // String times;
    // pkgListType *ptr[100];
#ifdef OLED
#ifdef SSD1306_72x40
    display.clearDisplay();
    display.fillRect(0, 0, 72, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(18, 1);
    display.print("SYSTEM");
    display.setTextColor(WHITE);
    display.setCursor(0, 12);
    display.print("UP:");
    str = String(day(upTime) - 1, DEC) + "D " + String(hour(upTime), DEC) + ":" + String(minute(upTime), DEC) + ":" + String(second(upTime), DEC);
    x = str.length() * 6;
    display.setCursor(71 - x, 12);
    display.print(str);

    display.setCursor(0, 22);
    display.print("FW:");
    str = "V" + String(VERSION) + String(VERSION_BUILD);
    x = str.length() * 6;
    display.setCursor(71 - x, 22);
    display.print(str);

    display.setCursor(0, 32);
    display.print("RAM:");
    // str = String((float)ESP.getFreeHeap() / 1024, 1) + "/" + String((float)ESP.getHeapSize() / 1024, 1) + "KB";
    str = String((float)ESP.getFreeHeap() / 1024, 1) + "KB";
    x = str.length() * 6;
    display.setCursor(71 - x, 32);
    display.print(str);
#else
    display.fillRect(0, 0, 128, 15, WHITE);
    display.drawRect(0, 16, 128, 48, WHITE);
    display.fillRect(1, 17, 126, 46, BLACK);

    display.setCursor(20, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("SYSTEM");
    display.setFont();
    display.setCursor(119, 7);
    display.print("4");
    display.setTextColor(WHITE);

    // display.setCursor(3, 18);
    // // display.print("HMEM:");
    // display.print("MAC");
    // // str = String(ESP.getFreeHeap(), DEC)+"Byte";
    // str = String(WiFi.macAddress());
    // x = str.length() * 6;
    // display.setCursor(126 - x, 18);
    // display.print(str);

    // display.setCursor(3, 26);
    // display.print("IP:");
    // str = String(WiFi.localIP().toString());
    // x = str.length() * 6;
    // display.setCursor(126 - x, 26);
    // display.print(str);

    display.setCursor(3, 18);
    display.print("UPTIME:");
    str = String(day(upTime) - 1, DEC) + "D " + String(hour(upTime), DEC) + ":" + String(minute(upTime), DEC) + ":" + String(second(upTime), DEC);
    x = str.length() * 6;
    display.setCursor(126 - x, 18);
    display.print(str);

    display.setCursor(3, 26);
    display.print("Firmware:");
    str = "V" + String(VERSION) + String(VERSION_BUILD);
    x = str.length() * 6;
    display.setCursor(126 - x, 26);
    display.print(str);

    display.setCursor(3, 35);
    display.print("RAM:");
    str = String((float)ESP.getFreeHeap() / 1024, 1) + "/" + String((float)ESP.getHeapSize() / 1024, 1) + "KB";
    x = str.length() * 6;
    display.setCursor(126 - x, 35);
    display.print(str);

    // display.setCursor(3, 44);
    // display.print("PSRAM:");
    // str = String((float)ESP.getFreePsram() / 1000, 1) + "/" + String((float)ESP.getPsramSize() / 1000, 1) +"KB";
    // x = str.length() * 6;
    // display.setCursor(126 - x, 44);
    // display.print(str);
    display.setCursor(3, 44);
    display.print("FILE:");
    str = String((float)LITTLEFS.usedBytes() / 1024, 1) + "/" + String((float)LITTLEFS.totalBytes() / 1024, 1) + "KB";
    x = str.length() * 6;
    display.setCursor(126 - x, 44);
    display.print(str);
#if defined(TTGO_T_Beam_S3_SUPREME_V3) || defined(TTGO_T_Beam_V1_2)
    display.setCursor(3, 53);
    display.print("BAT:");
    str = String((float)PMU.getBattVoltage() / 1000, 2) + "V" + "(" + String(PMU.getBatteryPercent()) + "%)";
    x = str.length() * 6;
    display.setCursor(126 - x, 53);
    display.print(str);
#endif
#endif
    display.display();
#elif defined(ST7735_160x80)
    display.fillRect(0, 0, 160, 15, WHITE);
    // display.drawRect(0, 16, 160, 64, WHITE);
    display.fillRect(0, 15, 160, 64, BLACK);

    display.setCursor(35, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("SYSTEM");
    display.setFont();
    display.setCursor(150, 7);
    display.print("4");
    display.setTextColor(WHITE);

    display.setCursor(3, 18);
    display.print("UPTIME:");
    str = String(day(upTime) - 1, DEC) + "D " + String(hour(upTime), DEC) + ":" + String(minute(upTime), DEC) + ":" + String(second(upTime), DEC);
    x = str.length() * 6;
    display.setCursor(158 - x, 18);
    display.print(str);

    display.setCursor(3, 28);
    display.print("Firmware:");
    str = "V" + String(VERSION) + String(VERSION_BUILD);
    x = str.length() * 6;
    display.setCursor(158 - x, 28);
    display.print(str);

    display.setCursor(3, 38);
    display.print("RAM:");
    str = String((float)ESP.getFreeHeap() / 1024, 1) + "/" + String((float)ESP.getHeapSize() / 1024, 1) + "KB";
    x = str.length() * 6;
    display.setCursor(158 - x, 38);
    display.print(str);

    display.setCursor(3, 48);
    display.print("PSRAM:");
    str = String((float)ESP.getFreePsram() / 1024, 1) + "/" + String((float)ESP.getPsramSize() / 1024, 1) + "KB";
    x = str.length() * 6;
    display.setCursor(158 - x, 48);
    display.print(str);

    display.setCursor(3, 58);
    display.print("FILE:");
    str = String((float)LITTLEFS.usedBytes() / 1024, 1) + "/" + String((float)LITTLEFS.totalBytes() / 1024, 1) + "KB";
    x = str.length() * 6;
    display.setCursor(158 - x, 58);
    display.print(str);
#if defined(TTGO_T_Beam_S3_SUPREME_V3) || defined(TTGO_T_Beam_V1_2)
    display.setCursor(3, 68);
    display.print("BAT:");
    str = String((float)PMU.getBattVoltage() / 1024, 2) + "V" + "(" + String(PMU.getBatteryPercent()) + "%)";
    x = str.length() * 6;
    display.setCursor(126 - x, 68);
    display.print(str);
#endif
    display.display();
#endif
}

void wifiDisp()
{

    // uint8 wifi = 0, k = 0, l;
    // char i;
    // char list[4];
    int x;
    String str;
    time_t upTime = now() - systemUptime;
    // String times;
    // pkgListType *ptr[100];
#ifdef OLED
#ifdef SSD1306_72x40
    display.clearDisplay();
    // display.drawRect(0, 0, 72, 40, WHITE);
    display.fillRect(0, 0, 72, 9, WHITE);
    // display.fillRect(1, 17, 126, 46, BLACK);
    display.setTextColor(BLACK);
    display.setCursor(25, 1);
    display.print("WiFi");
    display.setTextColor(WHITE);
    display.setCursor(6, 10);
    display.print("IP Address");
    str = String(WiFi.localIP().toString());
    // x = str.length() * 6;
    // display.setCursor(126 - x, 44);
    display.setCursor(0, 20);
    display.print(str);
    display.display();
#else
    display.fillRect(0, 0, 128, 15, WHITE);
    display.drawRect(0, 16, 128, 48, WHITE);
    display.fillRect(1, 17, 126, 46, BLACK);

    display.setCursor(20, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("WiFi Info");
    display.setFont();
    display.setCursor(119, 7);
    display.print("6");
    display.setTextColor(WHITE);

    display.setCursor(3, 18);
    display.print("MODE");
    str = "OFF";
    if (config.wifi_mode == WIFI_STA_FIX)
    {
        str = "STA";
    }
    else if (config.wifi_mode == WIFI_AP_FIX)
    {
        str = "AP";
    }
    else if (config.wifi_mode == WIFI_AP_STA_FIX)
    {
        str = "AP+STA";
    }
    x = str.length() * 6;
    display.setCursor(126 - x, 18);
    display.print(str);

    display.setCursor(3, 26);
    display.print("SSID");
    str = WiFi.SSID().substring(0, 15);
    x = str.length() * 6;
    display.setCursor(126 - x, 26);
    display.print(str);

    display.setCursor(3, 35);
    display.print("RSSI:");
    str = String(WiFi.RSSI()) + "dBm";
    x = str.length() * 6;
    display.setCursor(126 - x, 35);
    display.print(str);

    display.setCursor(3, 44);
    display.print("IP:");
    str = String(WiFi.localIP().toString());
    x = str.length() * 6;
    display.setCursor(126 - x, 44);
    display.print(str);

    display.setCursor(3, 53);
    display.print("MAC");
    str = String(WiFi.macAddress());
    x = str.length() * 6;
    display.setCursor(126 - x, 53);
    display.print(str);
    display.display();
#endif
#elif defined(ST7735_160x80)
    display.fillRect(0, 0, 160, 15, WHITE);
    // display.drawRect(0, 16, 160, 64, WHITE);
    display.fillRect(0, 15, 160, 64, BLACK);

    display.setCursor(35, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    if (manualWiFi)
        display.print("WiFi [ON]");
    else
        display.print("WiFi [OFF]");
    display.setFont();
    display.setCursor(150, 7);
    display.print("6");
    display.setTextColor(WHITE);

    display.setCursor(3, 18);
    display.print("MODE");
    str = "OFF";
    if (config.wifi_mode == WIFI_STA_FIX)
    {
        str = "STA";
    }
    else if (config.wifi_mode == WIFI_AP_FIX)
    {
        str = "AP";
    }
    else if (config.wifi_mode == WIFI_AP_STA_FIX)
    {
        str = "AP+STA";
    }
    x = str.length() * 6;
    display.setCursor(158 - x, 18);
    display.print(str);

    display.setCursor(3, 28);
    display.print("SSID");
    str = WiFi.SSID().substring(0, 15);
    x = str.length() * 6;
    display.setCursor(158 - x, 28);
    display.print(str);

    display.setCursor(3, 38);
    display.print("RSSI:");
    str = String(WiFi.RSSI()) + "dBm";
    x = str.length() * 6;
    display.setCursor(158 - x, 38);
    display.print(str);

    display.setCursor(3, 48);
    display.print("IP:");
    str = String(WiFi.localIP().toString());
    x = str.length() * 6;
    display.setCursor(158 - x, 48);
    display.print(str);

    display.setCursor(3, 58);
    display.print("AP_IP:");
    str = String(WiFi.softAPIP().toString());
    x = str.length() * 6;
    display.setCursor(158 - x, 58);
    display.print(str);

    display.setCursor(3, 68);
    display.print("MAC");
    str = String(WiFi.macAddress());
    x = str.length() * 6;
    display.setCursor(158 - x, 68);
    display.print(str);
    display.display();
#endif
}

void radioDisp()
{

    // uint8 wifi = 0, k = 0, l;
    // char i;
    // char list[4];
    int x;
    String str;
    time_t upTime = now() - systemUptime;
    // String times;
    // pkgListType *ptr[100];
#ifdef OLED
#ifdef SSD1306_72x40
    display.clearDisplay();
    display.fillRect(0, 0, 72, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(21, 1);
    display.print("RADIO");
    display.setTextColor(WHITE);
    // str = String(config.rf_freq, 3) + " MHz";
    // x = str.length() * 6;
    // display.setCursor(126 - x, 18);
    display.setCursor(0, 12);
    display.printf("%.3f Mhz", config.rf_freq);
    display.setCursor(0, 22);
    display.printf("BW: %.1fKhz", config.rf_bw);
    if (config.rf_power >= 0)
        str = "Pwr:+" + String(config.rf_power) + " dBm";
    else
        str = "Pwr:-" + String(config.rf_power) + " dBm";
    x = str.length() * 6;
    display.setCursor(0, 32);
    display.print(str);
#else
    display.fillRect(0, 0, 128, 15, WHITE);
    display.drawRect(0, 16, 128, 48, WHITE);
    display.fillRect(1, 17, 126, 46, BLACK);

    display.setCursor(10, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("RADIO Info");
    display.setFont();
    display.setCursor(119, 7);
    display.print("5");
    display.setTextColor(WHITE);

    display.setCursor(3, 18);
    display.print("FreqRX:");
    str = String(config.freq_rx, 3) + " MHz";
    x = str.length() * 6;
    display.setCursor(126 - x, 18);
    display.print(str);

    display.setCursor(3, 26);
    display.print("FreqTX:");
    str = String(config.freq_tx, 3) + " MHz";
    x = str.length() * 6;
    display.setCursor(126 - x, 26);
    display.print(str);

    display.setCursor(3, 35);
    display.print("Modem:");
    str = String(MODEM_TYPE[config.modem_type]);
    x = str.length() * 6;
    display.setCursor(126 - x, 35);
    display.print(str);

    display.setCursor(3, 44);
    display.print("FX.25");
    str = String(FX25_MODE[config.fx25_mode]);
    x = str.length() * 6;
    display.setCursor(126 - x, 44);
    display.print(str);

    display.setCursor(3, 53);
    display.print("Bandwidth:");
    if(!config.band)
        str = "12.5Khz";
    else
        str = "25.0Khz";
    x = str.length() * 6;
    display.setCursor(126 - x, 53);
    display.print(str);

    // display.setCursor(3, 53);
    // display.print("TX Power");
    // // str = "V" + String(VERSION);
    // if (config.rf_power >= 0)
    //     str = "+" + String(config.rf_power) + " dBm";
    // else
    //     str = "-" + String(config.rf_power) + " dBm";
    // x = str.length() * 6;
    // display.setCursor(126 - x, 53);
    // display.print(str);
#endif
    display.display();
#elif defined(ST7735_160x80)
    display.fillRect(0, 0, 160, 15, WHITE);
    // display.drawRect(0, 16, 160, 64, WHITE);
    display.fillRect(0, 15, 160, 64, BLACK);

    display.setCursor(35, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    if (config.rf_en)
        display.print("RADIO [ON]");
    else
        display.print("RADIO [OFF]");
    display.setFont();
    display.setCursor(150, 7);
    display.print("5");
    display.setTextColor(WHITE);

    display.setCursor(3, 18);
    display.print("Frequency");
    str = String(config.rf_freq, 3) + " MHz";
    x = str.length() * 6;
    display.setCursor(158 - x, 18);
    display.print(str);

    display.setCursor(3, 28);
    display.print("Bandwidth");
    str = String(config.rf_bw, 2) + " Khz";
    x = str.length() * 6;
    display.setCursor(158 - x, 28);
    display.print(str);

    display.setCursor(3, 38);
    display.print("Spread Factor");
    str = String(config.rf_sf);
    x = str.length() * 6;
    display.setCursor(158 - x, 38);
    display.print(str);

    display.setCursor(3, 48);
    display.print("Coding Rate");
    str = String(config.rf_cr);
    x = str.length() * 6;
    display.setCursor(158 - x, 48);
    display.print(str);

    display.setCursor(3, 58);
    display.print("TX Power");
    if (config.rf_power >= 0)
        str = "+" + String(config.rf_power) + " dBm";
    else
        str = "-" + String(config.rf_power) + " dBm";
    x = str.length() * 6;
    display.setCursor(158 - x, 58);
    display.print(str);
    display.display();
#endif
}

void sensorDisp()
{

    // uint8 wifi = 0, k = 0, l;
    uint k = 0;
    int i;
    // char list[4];
    int x, y;
    String str;
    // String times;
    // pkgListType *ptr[100];
#ifdef OLED
#ifdef SSD1306_72x40
    display.clearDisplay();
    display.fillRect(0, 0, 72, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(18, 1);
    display.print("SENSOR");
    display.setTextColor(WHITE);
    for (i = 0; i < 10; i++)
    {
        if (config.sensor[i].enable)
        {
            y = 12 + (k * 9);
            if (++k > 5)
                break;
            display.setTextColor(WHITE);
            display.setCursor(0, y);
            display.printf("%i:", i + 1);
            // display.print(String(config.sensor[i].parm).substring(0, 12));
            str = String(sen[i].sample, 2) + String(config.sensor[i].unit);
            x = str.length() * 6;
            display.setCursor(71 - x, y);
            display.print(str);
        }
    }
#else
    display.fillRect(0, 0, 128, 15, WHITE);
    display.drawRect(0, 16, 128, 48, WHITE);
    display.fillRect(1, 17, 126, 46, BLACK);

    display.setCursor(20, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("SENSOR");
    display.setFont();
    display.setCursor(119, 7);
    display.print("7");
    display.setTextColor(WHITE);

    for (i = 0; i < 10; i++)
    {
        if (config.sensor[i].enable)
        {
            y = 18 + (k * 9);
            if (++k > 5)
                break;
            display.setTextColor(WHITE);
            display.setCursor(2, y);
            display.print(String(config.sensor[i].parm).substring(0, 12));
            str = String(sen[i].sample, 2) + String(config.sensor[i].unit);
            x = str.length() * 6;
            display.setCursor(126 - x, y);
            display.print(str);
        }
    }
#endif
    display.display();
#elif defined(ST7735_160x80)
    display.fillRect(0, 0, 160, 15, WHITE);
    // display.drawRect(0, 16, 160, 64, WHITE);
    display.fillRect(0, 15, 160, 64, BLACK);

    display.setCursor(35, 7);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(BLACK);
    display.print("SENSOR");
    display.setFont();
    display.setCursor(150, 7);
    display.print("7");
    display.setTextColor(WHITE);

    for (i = 0; i < 10; i++)
    {
        if (config.sensor[i].enable)
        {
            y = 18 + (k * 9);
            if (++k > 6)
                break;
            display.setTextColor(WHITE);
            display.setCursor(2, y);
            display.print(String(config.sensor[i].parm).substring(0, 15));
            str = String(sen[i].sample, 2) + String(config.sensor[i].unit);
            x = str.length() * 6;
            display.setCursor(158 - x, y);
            display.print(str);
        }
    }
    display.display();
#endif
}

void gpsDisp()
{
    int x;
    String str;

#ifdef OLED
#ifdef SSD1306_72x40
    display.clearDisplay();
    display.fillRect(0, 0, 72, 9, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(24, 1);
    display.print("GNSS");
    display.setTextColor(WHITE);
    str = "LAT " + String(gps.location.lat(), 4);
    display.setCursor(0, 12);
    display.print(str);
    str = "LON " + String(gps.location.lng(), 4);
    display.setCursor(0, 22);
    display.print(str);
    str = "Alt " + String(gps.altitude.meters(), 0) + "m";
    display.setCursor(0, 32);
    display.print(str);

#else
    if (gps_mode == 0)
    {
        display.fillRect(0, 0, 128, 15, WHITE);
        display.drawRect(0, 16, 128, 48, WHITE);
        display.fillRect(1, 17, 126, 46, BLACK);

        display.setCursor(20, 7);
        display.setTextSize(1);
        display.setFont(&FreeSansBold9pt7b);
        display.setTextColor(BLACK);

        // display.fillRect(0, 16, 128, 10, WHITE);
        // display.drawLine(0, 16, 0, 63, WHITE);
        // display.drawLine(127, 16, 127, 63, WHITE);
        // display.drawLine(0, 63, 127, 63, WHITE);
        // display.fillRect(1, 25, 126, 38, BLACK);
        // display.setTextColor(BLACK);
        // display.setCursor(35, 17);
        display.print("GPS INFO");
        display.setCursor(119, 17);
        display.print("0");
        display.setTextColor(WHITE);
        display.setFont();

        display.setCursor(3, 18);
        display.print("LAT:");
        str = String(gps.location.lat(), 5);
        x = str.length() * 6;
        display.setCursor(80 - x, 18);
        display.print(str);

        display.setCursor(3, 26);
        display.print("LON:");
        str = String(gps.location.lng(), 5);
        x = str.length() * 6;
        display.setCursor(80 - x, 26);
        display.print(str);

        display.drawYBitmap(90, 19, &Icon_TableB[50][0], 16, 16, WHITE);
        display.setCursor(110, 25);
        display.print(gps.satellites.value());

        display.setCursor(3, 35);
        display.print("ALT:");
        str = String(gps.altitude.meters(), 0) + "m";
        x = str.length() * 6;
        display.setCursor(80 - x, 35);
        display.print(str);

        display.setCursor(3, 44);
        display.print("SPD:");
        str = String(gps.speed.kmph(), 1) + "kph";
        x = str.length() * 6;
        display.setCursor(80 - x, 44);
        display.print(str);

        display.setCursor(90, 44);
        display.print("CST:");
        str = String(gps.course.deg(), 0);
        x = str.length() * 6;
        display.setCursor(126 - x, 44);
        display.print(str);

        display.setCursor(3, 54);
        // display.print("TIME:");
        str = String(gps.date.day(), DEC) + "/" + String(gps.date.month(), DEC) + "/" + String(gps.date.year(), DEC);
        display.setCursor(3, 53);
        display.print(str);
        str = String(gps.time.hour(), DEC) + ":" + String(gps.time.minute(), DEC) + ":" + String(gps.time.second(), DEC) + "z";
        x = str.length() * 6;
        display.setCursor(126 - x, 53);
        display.print(str);
    }
    else
    {
        // display.clearDisplay();
        display.fillRect(0, 0, 128, 64, BLACK);
        display.drawYBitmap(90, 0, &Icon_TableB[50][0], 16, 16, WHITE);
        display.setCursor(107, 7);
        display.setTextSize(1);
        display.setFont(&FreeSansBold9pt7b);
        display.print(gps.satellites.value());

        struct tm tmstruct;
        char strTime[10];
        tmstruct.tm_year = 0;
        getLocalTime(&tmstruct, 100);
        sprintf(strTime, "%02d:%02d:%02d", tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
        display.setCursor(0, 14);
        display.print(strTime);

        // if (config.dim == 2)
        // { // Auto dim timeout
        //     if (millis() > (dimTimeout + 60000))
        //     {
        //         display.dim(true);
        //     }
        //     else
        //     {
        //         display.dim(false);
        //     }
        // }
        // else if (config.dim == 3)
        // { // Dim for time
        //     if (tmstruct.tm_hour > 5 && tmstruct.tm_hour < 19)
        //     {
        //         display.dim(false);
        //     }
        //     else
        //     {
        //         display.dim(true);
        //     }
        // }

        display.setFont(&FreeSerifItalic9pt7b);
        display.setCursor(80, 28);
        display.printf("km/h");

        display.setFont(&Seven_Segment24pt7b);
        display.setCursor(70, 63);
        display.print(SB_SPEED, DEC);

        compass_label(25, 42, 19, 0.0F, WHITE);
        compass_arrow(25, 42, 16, SB_HEADING, WHITE);
        display.drawLine(0, 16, 60, 16, WHITE);
        display.drawLine(60, 16, 70, 29, WHITE);
        display.drawLine(50, 16, 60, 29, WHITE);
        display.drawLine(60, 29, 127, 29, WHITE);
        display.setFont();
    }
#endif
    display.display();
#elif defined(ST7735_160x80)
    if (gps_mode == 0)
    {
        display.fillRect(0, 0, 160, 15, WHITE);
        // display.drawRect(0, 16, 160, 64, WHITE);
        display.fillRect(0, 15, 160, 64, BLACK);

        display.setCursor(60, 7);
        display.setTextSize(1);
        display.setFont(&FreeSansBold9pt7b);
        display.setTextColor(BLACK);
        display.print("GNSS");
        display.setFont();
        display.setCursor(150, 7);
        display.print("0");

        display.setTextColor(WHITE);
        display.setCursor(3, 18);
        display.print("LAT:");
        str = String(gps.location.lat(), 5);
        x = str.length() * 6;
        display.setCursor(90 - x, 18);
        display.print(str);

        display.setCursor(3, 28);
        display.print("LON:");
        str = String(gps.location.lng(), 5);
        x = str.length() * 6;
        display.setCursor(90 - x, 28);
        display.print(str);

        display.drawYBitmap(120, 20, &Icon_TableB[50][0], 16, 16, ST77XX_BLUE);
        display.setCursor(140, 27);
        display.print(gps.satellites.value());

        display.setCursor(3, 38);
        display.print("SPD:");
        str = String(gps.speed.kmph(), 1) + "kph";
        x = str.length() * 6;
        display.setCursor(90 - x, 38);
        display.print(str);

        display.setCursor(3, 48);
        display.print("ALT:");
        str = String(gps.altitude.meters(), 0) + "m";
        x = str.length() * 6;
        display.setCursor(90 - x, 48);
        display.print(str);

        display.setCursor(3, 58);
        display.print("HDOP:");
        str = String(gps.hdop.hdop(), 2);
        x = str.length() * 6;
        display.setCursor(90 - x, 58);
        display.print(str);

        display.setCursor(3, 68);
        // display.print("TIME:");
        str = String(gps.date.day(), DEC) + "/" + String(gps.date.month(), DEC) + "/" + String(gps.date.year(), DEC);
        display.setCursor(3, 68);
        display.print(str);
        str = String(gps.time.hour(), DEC) + ":" + String(gps.time.minute(), DEC) + ":" + String(gps.time.second(), DEC) + "z";
        x = str.length() * 6;
        display.setCursor(158 - x, 68);
        display.print(str);
    }
    else
    {
        // display.fillRect(0, 0, 160, 80, BLACK);
        display.fillScreen(ST77XX_BLACK);
        display.setTextColor(ST77XX_CYAN);
        display.drawYBitmap(120, 0, &Icon_TableB[50][0], 16, 16, ST77XX_BLUE);
        display.setCursor(140, 7);
        display.print(gps.satellites.value());
        display.setTextSize(1);
        display.setFont(&FreeSansBold9pt7b);

        struct tm tmstruct;
        char strTime[10];
        tmstruct.tm_year = 0;
        getLocalTime(&tmstruct, 100);
        sprintf(strTime, "%02d:%02d:%02d", tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);
        display.setCursor(0, 14);
        display.print(strTime);

        display.setFont(&FreeSerifItalic9pt7b);
        display.setCursor(90, 28);
        display.printf("km/h");

        display.setTextColor(ST77XX_RED);
        display.setFont(&Seven_Segment24pt7b);
        display.setCursor(70, 70);
        display.print(SB_SPEED, DEC);

        compass_label(30, 47, 24, 0.0F, ST77XX_GREEN);
        compass_arrow(30, 47, 20, SB_HEADING, ST77XX_GREEN);
        display.drawLine(0, 16, 70, 16, WHITE);
        display.drawLine(70, 16, 80, 29, WHITE);
        display.drawLine(60, 16, 70, 29, WHITE);
        display.drawLine(70, 29, 159, 29, WHITE);
        display.setFont();
    }
    display.display();
#endif
}

#endif