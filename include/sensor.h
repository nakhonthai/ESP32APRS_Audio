#include "main.h"

#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_NUMBER   10
#define SENSOR_NONE     0
//Module
// #define SENSOR_BAT      1
// #define SENSOR_NTC      2
// #define SENSOR_DS18B20  3
// #define SENSOR_BME280   4
// #define SENSOR_M701     5
// #define SENSOR_M702     6
//Node
#define SENSOR_CO2      1
#define SENSOR_CH2O      2
#define SENSOR_TVOC      3
#define SENSOR_PM25      4
#define SENSOR_PM100      5
#define SENSOR_TEMPERATURE      6
#define SENSOR_HUMIDITY      7
#define SENSOR_PRESSURE      8
#define SENSOR_WIND_SPD      9
#define SENSOR_WIND_DIR      10
#define SENSOR_RAIN      11
#define SENSOR_LUMINOSITY      12
#define SENSOR_SOIL_TEMPERATURE      13
#define SENSOR_SOIL_MOISTURE      14
#define SENSOR_WATER_TEMPERATURE      15
#define SENSOR_WATER_TDS      16
#define SENSOR_WATER_LEVEL      17
#define SENSOR_WATER_FLOW      18
#define SENSOR_VOLTAGE      19
#define SENSOR_CURRENT      20
#define SENSOR_POWER      21
#define SENSOR_ENERGY      22
#define SENSOR_FREQ      23
#define SENSOR_PF      24
#define SENSOR_SAT_NUM      25
#define SENSOR_SAT_HDOP      26
#define SENSOR_BAT_VOLTAGE    27
#define SENSOR_BAT_PERCENT  28


#define PORT_UART0  0
#define PORT_UART1  1
#define PORT_ADC    2
#define PORT_I2C0   3
#define PORT_I2C1   4
#define PORT_CNT_0  5
#define PORT_CNT_1  6
#define PORT_LOGIC  7
#define PORT_M701   8
#define PORT_M702   9
#define PORT_BME280_I2C0   10
#define PORT_BME280_I2C1   11
#define PORT_BMP280_I2C0   12
#define PORT_BMP280_I2C1   13
#define PORT_SI7021_I2C0   14
#define PORT_SI7021_I2C1   15
#define PORT_CCS811_I2C0   16
#define PORT_CCS811_I2C1   17
#define PORT_SAT_NUM   18
#define PORT_SAT_HDOP   19
#define PORT_SHT_I2C0   20
#define PORT_SHT_I2C1   21
#define PORT_BATTERY   22
#define PORT_PZEM   23
#define PORT_MODBUS_16 24
#define PORT_MODBUS_32 25
#define PORT_DS1820 26

#define SENSOR_PORT_NUM 27
const char SENSOR_PORT[SENSOR_PORT_NUM][15] = {"UART0_CSV", "UART1_CSV", "ADC", "I2C_0","I2C_1","COUNTER_0","COUNTER_1","LOGIC","M701_Modbus","M702_Modbus","BME280_I2C0","BME280_I2C1","BMP280_I2C0","BMP280_I2C1","SI7021_I2C0","SI7021_I2C1","CCS811_I2C0","CCS811_I2C1","GNSS_NUM","GNSS_HDOP","SHTxx_I2C0","SHTxx_I2C1","SYS_BAT","PZEM","MODBUS_16Bit","MODBUS_32Bit","DS1820_1Wire"};

#define SENSOR_NAME_NUM 29
const char SENSOR_NAME[29][20] = {"NONE", "CO2", "CH2O", "TVOC","PM2.5","PM10","TEMPERATURE","HUMIDITY","PRESSURE","WIND_SPD","WIND_DIR","RAIN","LUMINOSITY","SOIL_TEMP","SOIL_HUM","WATER_TEMP","WATER_TDS","WATER_LEVEL","WATER_FLOW","VOLTAGE","CURRENT","POWER","ENERGY","FREQ","PF","SAT_NUM","SAT_HDOP","BATTERY_VOLT","BATTERY_PERCENT"};


typedef struct SensorInfo_Struct
{
    bool enable;
    uint16_t type;          // Type of sensor
    uint8_t port;           // Port or Channel of Sensor
    uint16_t address;       // Address registor
    uint16_t samplerate;    // Sample rate Sec.
    uint16_t averagerate;   // Average rate Sec.
    float eqns[3];          // Equation av^2+bv+c
    char unit[10];           // Unit text of sensor
    char parm[15];          // Parameter/name text of sensor
} SensorInfo;

typedef struct SensorData_Struct
{
    bool visable;
    double sample;      //Value of sample
    double average;     //Value of average
    double sum;
    uint16_t counter;
    unsigned long int timeSample;
    unsigned long int timeAvg;
    unsigned long int timeTick;
} SensorData;

void dispSensor();
void dispSensor(int i);
void taskSensor(void *pvParameters);
void sensorInit(bool resetAll);
#endif