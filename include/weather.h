/*
 Name:		ESP32APRS T-TWR Plus
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#ifndef WEATHER_H
#define WEATHER_H

#include "main.h"
#include "sensor.h"
//#include <ModbusMaster.h>

//Format: c...s...g...t...r...p...P...h..b.....L...S...m...M...w...W....v...d...D...x....F....T....u..n...
//#define WX_SENSOR_NUM 23

#define WX_NONE 0
#define WX_WIND_DIR (1 << 0)       // c Wind Direction (in degrees)
#define WX_WIND_SPD (1 << 1)       // s Wind Speed (mhp)
#define WX_WIND_GUST (1 << 2)      // g Wind Gust	(peak wind speed in mph in the last 5 minutes)
#define WX_TEMP (1 << 3)           // t Temperature (Fahenheit)
#define WX_RAIN (1 << 4)           // r Rain collector (in hundredths of an inch) last hour
#define WX_RAIN24HR (1 << 5)       // p Rain collector (in hundredths of an inch) Last 24 Hr
#define WX_RAIN_GMT (1 << 6)       // P Rain collector (in hundredths of an inch) Sine Midnight (GMT+0)
#define WX_HUMIDITY (1 << 7)       // h Humidity
#define WX_BARO (1 << 8)           // b Barometric Pressure (in thenths of millibars/tenths of hPascal)
#define WX_LUMINOSITY (1 << 9)     // L,l Luminosity (in watts per square meter) L=0-999,l=1000-1999
#define WX_SNOW (1 << 10)          // S Snowfall (in inches) in the last 24 hours
#define WX_SOIL_TEMP (1 << 11)     // m Soil temperature (Fahenheit)
#define WX_SOIL_MOISTURE (1 << 12) // M Soil Moisture (0-40%VWC)
#define WX_WATER_TEMP (1 << 13)    // w Water (Fahenheit)
#define WX_WATER_TDS (1 << 14)     // W Water TDS(Total Dissolved Solids) 0-1000ppm
#define WX_WATER_LEVEL (1 << 15)   // v Water Level (in hundredths of an inch)
#define WX_PM25 (1 << 16)          // d Dust PM 2.5 0~1000μg/m³
#define WX_PM100 (1 << 17)         // D Dust PM 10 ,0~1000μg/m³
#define WX_CO2 (1 << 18)           // X,x Co2 (PPM) x=0-9999,X=10-99990(x10)
#define WX_CH2O (1<<19)             //F,f Formaldehyde(CH2O) F=0-9999,f,1000-2000 μg/m³
#define WX_TVOC (1<<20)             // T
#define WX_UV (1<<21)               //UV Index
#define WX_SOUND (1<<22)            //n noise of sound Level dB

typedef struct Weather_Struct
{
    unsigned long int timeStamp;
    uint16_t winddirection; // Wind Direction (in degrees)
    uint32_t visable;
    float windspeed;        // Wind Speed (kph)
    float windgust;         // Wind Gust	(peak wind speed in kph in the last 5 minutes)
    float temperature;      // Temperature (Celsius)
    float rain;             // Rain collector (in mm.) last hour
    float rain24hr;         // Rain collector (in mm.) Last 24 Hr
    float rainmidnight;     // Rain collector (in mm.) Sine Midnight (GMT+0)
    float humidity;         // Humidity 0-100%RH
    float barometric;       // Barometric Pressure (in thenths of millibars/tenths of hPascal)
    uint16_t solar;         // Solar radiation/Luminosity (in watts per square meter) L=0-999,l=1000-9999 
    uint16_t snow;          // Snowfall (in mm.) in the last 24 hours
    float soil_temp;        // Soil Temperature (Celsius)
    float soil_moisture; // Soil Moisture (0-40%VWC)
    float water_temp;       // Water Temperature (Celsius)
    uint16_t water_tds;     // Water TDS(Total Dissolved Solids) 0-1000ppm
    uint16_t water_level;   // Water Level (in mm.)
    uint16_t pm25;          // Ordure PM 2.5 (0~1000μg/m³)
    uint16_t pm100;         // Ordure PM 10 (0~1000μg/m³)
    uint32_t co2;           // Co2 (ppm)
    uint16_t ch2o;           // F,f Formaldehyde(CH2O) F=0-9999,f,1000-2000 μg/m³
    uint16_t tvoc;         // (0~2000μg/m³)
    uint8_t uv;         // UV index 1-11
    float sound;        // Sound Level dB
    float vbat;             // Battery Voltage (V)
    float vsolar;           // Solar cell Voltage (V)
    float ibat;             // Battery Current (A)
    uint8_t pbat;           // Battery Percent (0-100%)
} WeatherData;

extern WeatherData weather;
extern float mslAltitude;

int getRawWx(char *strData);
bool getCSV2Wx(String stream);
void getSensor(uint32_t type, float *val, int i);
void getSensor(uint32_t type, uint16_t *val, int i);
void getSensor(uint32_t type, uint32_t *val, int i);
// bool getM70xModbus(ModbusMaster &node);
// bool getM70xModbus(ModbusMaster &node, uint32_t sensor);

#endif