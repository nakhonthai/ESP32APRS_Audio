/*
 Name:		ESP32IGate
 Created:	4-15-2024 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#include "sensor.h"
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Si7021.h>
#include <Adafruit_CCS811.h>
#include <SHTSensor.h>
#include <ModbusMaster.h>
#include <TinyGPS++.h>
#include <soc/gpio_struct.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>

//#include "DS18B20.h" // Digital Thermometer, 12bit
//#include "OneWireHub.h"

extern TwoWire Wire1;
extern Configuration config;
extern WiFiClient aprsClient;
extern ModbusMaster modbus;
extern bool i2c_busy;
extern TinyGPSPlus gps;
extern double VBat;
extern bool VBat_Flag;
extern double TempNTC;
#ifdef TTGO_T_Beam_S3_SUPREME_V3
#include <XPowersLib.h>
extern XPowersAXP2101 PMU;
#endif

Adafruit_BME280 *bme=NULL;    // I2C
Adafruit_BMP280 *bmp280=NULL; // I2C
Adafruit_Si7021 *Si7021=NULL;
Adafruit_CCS811 *ccs=NULL;
SHTSensor *sht=NULL; // Supported sensors:SHTC1, SHTC3, SHTW1, SHTW2, SHT3x-DIS (I2C), SHT2x, SHT85, SHT3x-ARP, SHT4x
//OneWire *oneWire=NULL;
//DallasTemperature *ds1820;

// OneWireHub *oneWire=NULL;
// //auto hub = OneWireHub(pin_onewire);

// auto ds18b20 = DS18B20(DS18B20::family_code, 0x00, 0x00, 0xB2, 0x18, 0xDA,
//                        0x00); // DS18B20: 9-12bit, -55 -  +85 degC


SensorData sen[SENSOR_NUMBER];

RTC_DATA_ATTR unsigned long cnt0, cnt1;

// Series resistor value
#define SERIESRESISTOR 10000

// Nominal resistance at 25C
#define THERMISTORNOMINAL 10000

// Nominal temperature in degrees
#define TEMPERATURENOMINAL 25

// Beta coefficient
#define BCOEFFICIENT 3950

double getTempNTC(double Vout)
{
    double average, kelvin, resistance, celsius;
    int i;

    resistance = SERIESRESISTOR * Vout / (3300 - Vout);
    log_d("ADC=%0.fmV R=%0.1f", Vout, resistance);
    // Convert to resistanceresistance = 4095 / average - 1;resistance = SERIESRESISTOR/resistance;
    /*
     * Use Steinhart equation (simplified B parameter equation) to convert resistance to kelvin
     * B param eq: T = 1/( 1/To + 1/B * ln(R/Ro) )
     * T = Temperature in Kelvin
     * R = Resistance measured
     * Ro = Resistance at nominal temperature
     * B = Coefficent of the thermistor
     * To = Nominal temperature in kelvin
     */
    double R1 = 10000.0;  // voltage divider resistor value
    double Beta = 3950.0; // Beta value
    double To = 298.15;   // Temperature in Kelvin for 25 degree Celsius
    double Ro = 10000.0;  // Resistance of Thermistor at 25 degree Celsius

    double T = 1 / (1 / To + log(resistance / Ro) / Beta); // Temperature in Kelvin
    celsius = T - 273.15F;                                 // Celsius

    // kelvin = resistance / THERMISTORNOMINAL;               // R/Ro
    // kelvin = log(kelvin);                                  // ln(R/Ro)
    // kelvin = (1 / BCOEFFICIENT) * kelvin;                  // 1/B * ln(R/Ro)
    // kelvin = (1 / (TEMPERATURENOMINAL + 273.15)) + kelvin; // 1/To + 1/B * ln(R/Ro)
    // kelvin = 1 / kelvin;                                   // 1/( 1/To + 1/B * ln(R/Ro) )​

    // // Convert Kelvin to Celsius
    // celsius = kelvin - 273.15;

    // Send the value back to be displayed
    return celsius;
}

/* Count RPM Function - takes first timestamp and last timestamp,
number of pulses, and pulses per revolution */
int countRPM(int firstTime, int lastTime, int pulseTotal, int pulsePerRev)
{
    int timeDelta = (lastTime - firstTime); // lastTime - firstTime
    if (timeDelta <= 0)
    { // This means we've gotten something wrong
        return -1;
    }
    return ((60000 * (pulseTotal / pulsePerRev)) / timeDelta);
}

void dispSensor()
{
    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if (config.sensor[i].enable && sen[i].visable)
        {
            log_d("SENSOR#%d %s: %.1f %s\tAvg: %.1f %s", i+1, config.sensor[i].parm, sen[i].sample, config.sensor[i].unit, sen[i].average, config.sensor[i].unit);
            // switch (config.sensor[i].type)
            // {
            // case SENSOR_TEMPERATURE:
            //     log_d("Temperature: %.1f \tAvg: %.1f C", sen[i].sample, sen[i].average);
            //     break;
            // case SENSOR_HUMIDITY:
            //     log_d("Humidity: %.1f \tAvg: %.1f %%RH", sen[i].sample, sen[i].average);
            //     break;
            // case SENSOR_PM25:
            //     log_d("PM2.5: %d \tAvg: %.1f μg/m³", (int)sen[i].sample, sen[i].average);
            //     break;
            // case SENSOR_PM100:
            //     log_d("PM10: %d \tAvg: %.1fd μg/m³", (int)sen[i].sample, sen[i].average);
            //     break;
            // case SENSOR_CO2:
            //     log_d("Co2: %d \tAvg: %.1f ppm", (int)sen[i].sample, sen[i].average);
            //     break;
            // case SENSOR_CH2O:
            //     log_d("CH2O: %d \tAvg: %.1fd μg/m³", (int)sen[i].sample, sen[i].average);
            //     break;
            // case SENSOR_TVOC:
            //     log_d("TVOC: %d \tAvg: %.1fd μg/m³", (int)sen[i].sample, sen[i].average);
            //     break;
            // default:
            //     break;
            // }
        }
    }
}

void dispSensor(int i)
{
    if (config.sensor[i].enable && sen[i].visable)
    {
        log_d("SENSOR#%d %s: %.1f %s\tAvg: %.1f %s", i+1, config.sensor[i].parm, sen[i].sample, config.sensor[i].unit, sen[i].average, config.sensor[i].unit);
    }
}

bool sensorUpdateSum(int i, double val)
{
    double sample;
    if (i >= SENSOR_NUMBER)
        return false;
    sen[i].visable = true;
    sample = (config.sensor[i].eqns[0] * pow(val, 2)) + (config.sensor[i].eqns[1] * val) + config.sensor[i].eqns[2];
    sen[i].sample += sample;
    sen[i].sum += sample;
    sen[i].counter++;
    sen[i].timeSample = millis();
    sen[i].timeTick = millis() + ((unsigned long)config.sensor[i].samplerate * 1000);
    if ((millis() - sen[i].timeAvg) > (config.sensor[i].averagerate * 1000))
    {
        if (sen[i].counter > 0)
            sen[i].average += sen[i].sum / sen[i].counter;
        else
            sen[i].average += sen[i].sum;
        sen[i].sum = 0;
        sen[i].counter = 0;
        sen[i].timeAvg = millis();
    }
    return true;
}

bool sensorUpdate(int i, double val)
{
    if (i >= SENSOR_NUMBER)
        return false;
    sen[i].visable = true;
    sen[i].sample = val;
    sen[i].sample = (config.sensor[i].eqns[0] * pow(sen[i].sample, 2)) + (config.sensor[i].eqns[1] * sen[i].sample) + config.sensor[i].eqns[2];
    sen[i].sum += sen[i].sample;
    sen[i].counter++;
    sen[i].timeSample = millis();
    sen[i].timeTick = millis() + ((unsigned long)config.sensor[i].samplerate * 1000);
    if ((millis() - sen[i].timeAvg) > (config.sensor[i].averagerate * 1000))
    {
        if (sen[i].counter > 0)
            sen[i].average = sen[i].sum / sen[i].counter;
        else
            sen[i].average = sen[i].sum;
        sen[i].sum = 0;
        sen[i].counter = 0;
        sen[i].timeAvg = millis();
    }
    return true;
}

bool getBAT(uint8_t port)
{
    // analogReadResolution(12);
    // analogSetAttenuation(ADC_11db);
    // double val=(double)analogReadMilliVolts(port);
    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if (config.sensor[i].type == SENSOR_BAT_VOLTAGE)
        {
            VBat_Flag=true;
#ifdef TTGO_T_Beam_S3_SUPREME_V3
            VBat = (double)PMU.getBattVoltage() / 1000;
            sensorUpdate(i, VBat);
#elif defined(HELTEC_HTIT_TRACKER)
            analogReadResolution(12);
            analogSetAttenuation(ADC_11db);
            digitalWrite(2, HIGH);
            delay(10);
            VBat = (double)analogReadMilliVolts(1) / 204.08F;  //390k/100k Voltage divider
            sensorUpdate(i, VBat + 0.285F); //Add offset 0.285V
            digitalWrite(2, LOW);
#elif defined(HELTEC_V3_GPS)
            analogReadResolution(12);
            analogSetAttenuation(ADC_11db);
            digitalWrite(37, HIGH);
            delay(10);
            VBat = (double)analogReadMilliVolts(1) / 204.08F;  //390k/100k Voltage divider
            sensorUpdate(i, VBat + 0.285F); //Add offset 0.285V
            digitalWrite(37, LOW);
#elif defined(APRS_LORA_HT)
            VBat = (double)analogReadMilliVolts(3) / 595.24F;
            sensorUpdate(i, VBat);
#elif defined(BUOY)
            analogReadResolution(12);
            analogSetAttenuation(ADC_11db);
            VBat = (double)analogReadMilliVolts(0) / 595.24F;
            sensorUpdate(i, VBat);
#endif
        }
        else if (config.sensor[i].type == SENSOR_BAT_PERCENT)
        {
#ifdef TTGO_T_Beam_S3_SUPREME_V3
            sensorUpdate(i, (double)PMU.getBatteryPercent());
#endif
        }
    }
    return true;
}

bool getLOGIC(uint8_t port)
{

    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if(!config.sensor[i].enable) continue;
        if (config.sensor[i].port == port)
        {
            double val=0;
            if(digitalRead((uint8_t)config.sensor[i].address)) val=1;
            sensorUpdate(i, (double)val);
            cnt0 = 0;
            break;
        }
    }
    return true;
}

bool getDS1820(uint8_t port)
{    
    // oneWire->attach(ds18b20);
    // //if(ds1820==NULL) return false;
    // //ds1820->requestTemperatures();
    // ds18b20.setTemperature(int8_t(85));
    // for (int i = 0; i < SENSOR_NUMBER; i++)
    // {
    //     if(!config.sensor[i].enable) continue;
    //     if (config.sensor[i].port == port)
    //     {
    //         float val=0;
    //         //val=ds1820->getTempCByIndex(config.sensor[i].address);
    //         val=ds18b20.getTemperature();
    //         sensorUpdate(i, (double)val);
    //         break;
    //     }
    // }
    return true;
}

bool getCNT0(uint8_t port)
{

    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if(!config.sensor[i].enable) continue;
        if (config.sensor[i].type == SENSOR_RAIN && config.sensor[i].port == port)
        {
            sensorUpdateSum(i, (double)cnt0);
            cnt0 = 0;
            break;
        }
        else if (config.sensor[i].port == port)
        {
            sensorUpdate(i, (double)cnt0);
            cnt0 = 0;
            break;
        }
    }
    return true;
}

bool getCNT1(uint8_t port)
{

    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if(!config.sensor[i].enable) continue;
        if (config.sensor[i].type == SENSOR_RAIN && config.sensor[i].port == port)
        {
            sensorUpdateSum(i, (double)cnt1);
            cnt1 = 0;
            break;
        }
        else if (config.sensor[i].port == port)
        {
            sensorUpdate(i, (double)cnt1);
            cnt1 = 0;
            break;
        }
    }
    return true;
}

bool getADC(uint8_t pin)
{
    double val = 0;
    //set the resolution to 12 bits (0-4095)
    analogReadResolution(12);
    // Optional: Set different attenaution (default is ADC_11db)
    analogSetAttenuation(ADC_11db);
    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if(!config.sensor[i].enable) continue;     
        if (((config.sensor[i].type == SENSOR_VOLTAGE)||(config.sensor[i].type == SENSOR_BAT_VOLTAGE)) && (config.sensor[i].port == PORT_ADC))
        {
            val = (double)analogReadMilliVolts(config.sensor[i].address);
            sensorUpdate(i, (double)val); // mV /595.24F; 4.2V=>> *0.0016799946
        }
        else if (config.sensor[i].type == SENSOR_CURRENT && config.sensor[i].port == PORT_ADC)
        {
            val = (double)analogReadMilliVolts(config.sensor[i].address);
            sensorUpdate(i, (double)val); // mA
        }
        else if (config.sensor[i].type == SENSOR_POWER && config.sensor[i].port == PORT_ADC)
        {
            val = (double)analogReadMilliVolts(config.sensor[i].address);
            sensorUpdate(i, (double)val); // mW
        }
        else if (((config.sensor[i].type == SENSOR_TEMPERATURE)||(config.sensor[i].type == SENSOR_WATER_TEMPERATURE)) && (config.sensor[i].port == PORT_ADC))
        {
            val = (double)analogReadMilliVolts(config.sensor[i].address);
            TempNTC = (double)getTempNTC(val);
            sensorUpdate(i, TempNTC); // NTC 10K
        }
    }
    return true;
}

bool getBME_I2C(Adafruit_BME280 &node, uint8_t port)
{
    bool result;
    node.setSampling(Adafruit_BME280::MODE_FORCED,
                     Adafruit_BME280::SAMPLING_X2, // temperature
                     Adafruit_BME280::SAMPLING_X4, // pressure
                     Adafruit_BME280::SAMPLING_X2, // humidity
                     Adafruit_BME280::FILTER_X16);
    result = node.takeForcedMeasurement(); // has no effect in normal mode
    if (result)
    {
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == port)
            {
                sensorUpdate(i, node.readTemperature()); // Temperature
            }
            else if (config.sensor[i].type == SENSOR_HUMIDITY && config.sensor[i].port == port)
            {
                sensorUpdate(i, node.readHumidity()); // Humidity
            }
            else if (config.sensor[i].type == SENSOR_PRESSURE && config.sensor[i].port == port)
            {
                sensorUpdate(i, node.readPressure() / 100.0F); // Pressure
            }
            // else if (config.sensor[i].type == SENSOR_ALTITUDE && config.sensor[i].port == port)
            // {
            //     sensorUpdate(i, node.readAltitude(1013.25)); // Altutude
            // }
        }
        return true;
    }
    return false;
}
// bool getBME_I2C1(Adafruit_BME280 &node)
// {
//     bool result;
//     node.setSampling(Adafruit_BME280::MODE_FORCED,
//                      Adafruit_BME280::SAMPLING_X1, // temperature
//                      Adafruit_BME280::SAMPLING_X1, // pressure
//                      Adafruit_BME280::SAMPLING_X1, // humidity
//                      Adafruit_BME280::FILTER_OFF);
//     result = node.takeForcedMeasurement(); // has no effect in normal mode
//     if (result)
//     {
//         for (int i = 0; i < SENSOR_NUMBER; i++)
//         {
//             if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == PORT_BME280_I2C1)
//             {
//                 sensorUpdate(i, node.readTemperature()); // Temperature
//             }
//             else if (config.sensor[i].type == SENSOR_HUMIDITY && config.sensor[i].port == PORT_BME280_I2C1)
//             {
//                 sensorUpdate(i, node.readHumidity()); // Humidity
//             }
//             else if (config.sensor[i].type == SENSOR_PRESSURE && config.sensor[i].port == PORT_BME280_I2C1)
//             {
//                 sensorUpdate(i, node.readPressure() / 100.0F); // Pressure
//             }
//         }
//         return true;
//     }
//     return false;
// }

bool getBMP_I2C(Adafruit_BMP280 &node, uint8_t port)
{
    bool result;
    /* Default settings from datasheet. */
    node.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    result = node.takeForcedMeasurement();             // has no effect in normal mode
    if (result)
    {
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == port)
            {
                sensorUpdate(i, node.readTemperature()); // Temperature
            }
            // else if (config.sensor[i].type == SENSOR_ALTITUDE && config.sensor[i].port == port)
            // {
            //     sensorUpdate(i, node.readAltitude(1013.25)); // Altutude
            // }
            else if (config.sensor[i].type == SENSOR_PRESSURE && config.sensor[i].port == port)
            {
                sensorUpdate(i, node.readPressure() / 100.0F); // Pressure
            }
        }
        return true;
    }
    return false;
}

bool getSI7021_I2C(Adafruit_Si7021 &node, uint8_t port)
{
    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if(!config.sensor[i].enable) continue;
        if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == port)
        {
            sensorUpdate(i, node.readTemperature()); // Temperature
        }
        else if (config.sensor[i].type == SENSOR_HUMIDITY && config.sensor[i].port == port)
        {
            sensorUpdate(i, node.readHumidity()); // Humidity
        }
    }
    return true;
}

bool getSHT_I2C(SHTSensor &node, uint8_t port)
{
    node.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x
    if (node.readSample())
    {
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == port)
            {
                sensorUpdate(i, node.getTemperature()); // Temperature
            }
            else if (config.sensor[i].type == SENSOR_HUMIDITY && config.sensor[i].port == port)
            {
                sensorUpdate(i, node.getHumidity()); // Humidity
            }
        }
        return true;
    }
    return false;
}

bool getCCS_I2C(Adafruit_CCS811 &node, uint8_t port)
{
    if (!node.readData())
    {
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_CO2 && config.sensor[i].port == port)
            {
                sensorUpdate(i, (double)node.geteCO2()); // Co2
                log_d("CCS811 Co2 %d PPM",node.geteCO2());
            }
            else if (config.sensor[i].type == SENSOR_TVOC && config.sensor[i].port == port)
            {
                sensorUpdate(i, (double)node.getTVOC()); // TVOC
            }
        }
        return true;
    }
    return false;
}

bool getSAT()
{
    if (config.gnss_enable)
    {
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_SAT_NUM)
            {
                sensorUpdate(i, (double)gps.satellites.value()); // gps num
            }
            else if (config.sensor[i].type == SENSOR_SAT_HDOP)
            {
                sensorUpdate(i, (double)gps.hdop.hdop()); // HDOP
            }
        }
        return true;
    }
    return false;
}

bool getM701Modbus(ModbusMaster &node)
{
    uint8_t result;

    result = node.readHoldingRegisters(0x0002, 7);
    if (result == node.ku8MBSuccess)
    {
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_CO2 && config.sensor[i].port == PORT_M701)
            {
                sensorUpdate(i, node.getResponseBuffer(0)); // Co2
            }
            else if (config.sensor[i].type == SENSOR_CH2O && config.sensor[i].port == PORT_M701)
            {
                sensorUpdate(i, node.getResponseBuffer(1)); // CH2O
            }
            else if (config.sensor[i].type == SENSOR_TVOC && config.sensor[i].port == PORT_M701)
            {
                sensorUpdate(i, node.getResponseBuffer(2)); // TVOC
            }
            else if (config.sensor[i].type == SENSOR_PM25 && config.sensor[i].port == PORT_M701)
            {
                sensorUpdate(i, node.getResponseBuffer(3)); // PM 2.5
            }
            else if (config.sensor[i].type == SENSOR_PM100 && config.sensor[i].port == PORT_M701)
            {
                sensorUpdate(i, node.getResponseBuffer(4)); // PM 10.0
            }
            else if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == PORT_M701)
            {
                sensorUpdate(i, node.getResponseBuffer(5)); // Temperature
            }
            else if (config.sensor[i].type == SENSOR_HUMIDITY && config.sensor[i].port == PORT_M701)
            {
                sensorUpdate(i, node.getResponseBuffer(6)); // Humidity
            }
        }
        // co2 = (uint32_t)node.getResponseBuffer(0);                        // Co2 PPM
        // ch2o = node.getResponseBuffer(1);                       // ug
        // tvoc = node.getResponseBuffer(2);                       // ug
        // pm25 = node.getResponseBuffer(3);                       // PM2.5ug
        // pm100 = node.getResponseBuffer(4);                      // PM10 ug
        // &temperature = (double)node.getResponseBuffer(5) / 10.0f; // C
        // humidity = (double)node.getResponseBuffer(6) / 10.0f;    //%RH
        return true;
    }
    else
    {
        // for (int i = 0; i < SENSOR_NUMBER; i++)
        // {
        //     if(config.sensor[i].type == SENSOR_CO2 | )
        //     sen[i].visable = false;
        // }
    }
    return false;
}

bool getM702Modbus(ModbusMaster &node)
{
    uint8_t result;

    result = node.readHoldingRegisters(0x0006, 5);
    if (result == node.ku8MBSuccess)
    {
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_TVOC && config.sensor[i].port == PORT_M702)
            {
                sen[i].visable = true;
                sen[i].sample = (double)node.getResponseBuffer(0); // ug
                sen[i].sample = (config.sensor[i].eqns[0] * pow(sen[i].sample, 2)) + (config.sensor[i].eqns[1] * sen[i].sample) + config.sensor[i].eqns[2];
                sen[i].sum += sen[i].sample;
                sen[i].counter++;
                sen[i].timeSample = millis();
                sen[i].timeTick = millis() + ((unsigned long)config.sensor[i].samplerate * 1000);
                if ((millis() - sen[i].timeAvg) > ((unsigned long)config.sensor[i].averagerate * 1000))
                {
                    if (sen[i].counter > 0)
                        sen[i].average = sen[i].sum / sen[i].counter;
                    else
                        sen[i].average = sen[i].sum;
                    sen[i].sum = 0;
                    sen[i].counter = 0;
                    sen[i].timeAvg = millis();
                }
            }
            else if (config.sensor[i].type == SENSOR_PM25 && config.sensor[i].port == PORT_M702)
            {
                sen[i].visable = true;
                sen[i].sample = (double)node.getResponseBuffer(1); // PM2.5ug
                sen[i].sample = (config.sensor[i].eqns[0] * pow(sen[i].sample, 2)) + (config.sensor[i].eqns[1] * sen[i].sample) + config.sensor[i].eqns[2];
                sen[i].sum += sen[i].sample;
                sen[i].counter++;
                sen[i].timeSample = millis();
                sen[i].timeTick = millis() + ((unsigned long)config.sensor[i].samplerate * 1000);
                if ((millis() - sen[i].timeAvg) > ((unsigned long)config.sensor[i].averagerate * 1000))
                {
                    if (sen[i].counter > 0)
                        sen[i].average = sen[i].sum / sen[i].counter;
                    else
                        sen[i].average = sen[i].sum;
                    sen[i].sum = 0;
                    sen[i].counter = 0;
                    sen[i].timeAvg = millis();
                }
            }
            else if (config.sensor[i].type == SENSOR_PM100 && config.sensor[i].port == PORT_M702)
            {
                sen[i].visable = true;
                sen[i].sample = (double)node.getResponseBuffer(2); // PM10 ug
                sen[i].sample = (config.sensor[i].eqns[0] * pow(sen[i].sample, 2)) + (config.sensor[i].eqns[1] * sen[i].sample) + config.sensor[i].eqns[2];
                sen[i].sum += sen[i].sample;
                sen[i].counter++;
                sen[i].timeSample = millis();
                sen[i].timeTick = millis() + ((unsigned long)config.sensor[i].samplerate * 1000);
                if ((millis() - sen[i].timeAvg) > ((unsigned long)config.sensor[i].averagerate * 1000))
                {
                    if (sen[i].counter > 0)
                        sen[i].average = sen[i].sum / sen[i].counter;
                    else
                        sen[i].average = sen[i].sum;
                    sen[i].sum = 0;
                    sen[i].counter = 0;
                    sen[i].timeAvg = millis();
                }
            }
            else if (config.sensor[i].type == SENSOR_TEMPERATURE && config.sensor[i].port == PORT_M702)
            {
                sen[i].visable = true;
                sen[i].sample = (double)node.getResponseBuffer(3) / 10.0f; // C
                sen[i].sample = (config.sensor[i].eqns[0] * pow(sen[i].sample, 2)) + (config.sensor[i].eqns[1] * sen[i].sample) + config.sensor[i].eqns[2];
                sen[i].sum += sen[i].sample;
                sen[i].counter++;
                sen[i].timeSample = millis();
                sen[i].timeTick = millis() + ((unsigned long)config.sensor[i].samplerate * 1000);
                if ((millis() - sen[i].timeAvg) > ((unsigned long)config.sensor[i].averagerate * 1000))
                {
                    if (sen[i].counter > 0)
                        sen[i].average = sen[i].sum / sen[i].counter;
                    else
                        sen[i].average = sen[i].sum;
                    sen[i].sum = 0;
                    sen[i].counter = 0;
                    sen[i].timeAvg = millis();
                }
            }
            else if (config.sensor[i].type == SENSOR_HUMIDITY && config.sensor[i].port == PORT_M702)
            {
                sen[i].visable = true;
                sen[i].sample = (double)node.getResponseBuffer(4) / 10.0f; //%RH
                sen[i].sample = (config.sensor[i].eqns[0] * pow(sen[i].sample, 2)) + (config.sensor[i].eqns[1] * sen[i].sample) + config.sensor[i].eqns[2];
                sen[i].sum += sen[i].sample;
                sen[i].counter++;
                sen[i].timeSample = millis();
                sen[i].timeTick = millis() + ((unsigned long)config.sensor[i].samplerate * 1000);
                if ((millis() - sen[i].timeAvg) > ((unsigned long)config.sensor[i].averagerate * 1000))
                {
                    if (sen[i].counter > 0)
                        sen[i].average = sen[i].sum / sen[i].counter;
                    else
                        sen[i].average = sen[i].sum;
                    sen[i].sum = 0;
                    sen[i].counter = 0;
                    sen[i].timeAvg = millis();
                }
            }
        }
        // tvoc = node.getResponseBuffer(2);                       // ug
        // pm25 = node.getResponseBuffer(3);                       // PM2.5ug
        // pm100 = node.getResponseBuffer(4);                      // PM10 ug
        // &temperature = (double)node.getResponseBuffer(5) / 10.0f; // C
        // humidity = (double)node.getResponseBuffer(6) / 10.0f;    //%RH
        return true;
    }
    return false;
}

bool getPZEMModbus(ModbusMaster &node)
{
    uint8_t result;

    result = node.readHoldingRegisters(0x0000, 9);
    if (result == node.ku8MBSuccess)
    {
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if(!config.sensor[i].enable) continue;
            if (config.sensor[i].type == SENSOR_VOLTAGE && config.sensor[i].port == PORT_PZEM)
            {
                double val = (double)node.getResponseBuffer(0); // Voltage 0x0000
                val *= 0.1F; //to value 0.1V
                sensorUpdate(i, val);
            }
            else if (config.sensor[i].type == SENSOR_CURRENT && config.sensor[i].port == PORT_PZEM)
            {
                uint32_t val32 = (uint32_t)node.getResponseBuffer(2); // Current 0x0002 MSB
                val32 <<= 16;
                val32 |= (uint32_t)node.getResponseBuffer(1); // Current 0x000 LSB
                double val = (double)val32*0.001F; //to value 0.001A
                sensorUpdate(i, val);
            }
            else if (config.sensor[i].type == SENSOR_POWER && config.sensor[i].port == PORT_PZEM)
            {
                uint32_t val32 = (uint32_t)node.getResponseBuffer(4); // Power 0x0004 MSB
                val32 <<= 16;
                val32 |= (uint32_t)node.getResponseBuffer(3); // Power 0x0003 LSB
                double val = (double)val32*0.1F; //to value 0.1W
                sensorUpdate(i, val);
            }
            else if (config.sensor[i].type == SENSOR_ENERGY && config.sensor[i].port == PORT_PZEM)
            {
                uint32_t val32 = (uint32_t)node.getResponseBuffer(6); // Energy 0x0006 MSB
                val32 <<= 16;
                val32 |= (uint32_t)node.getResponseBuffer(5); // Energy 0x0005 LSB
                double val = (double)val32; //to value 1W
                sensorUpdate(i, val);
            }
            else if (config.sensor[i].type == SENSOR_FREQ && config.sensor[i].port == PORT_PZEM)
            {
                double val = (double)node.getResponseBuffer(7); // Frequency 0x0007
                val *= 0.1F; //to value 0.1Hz
                sensorUpdate(i, val);
            }
            else if (config.sensor[i].type == SENSOR_PF && config.sensor[i].port == PORT_PZEM)
            {
                double val = (double)node.getResponseBuffer(8); // Frequency 0x0008
                val *= 0.01F; //to value 0.01
                sensorUpdate(i, val);
            }            
        }
        return true;
    }
    return false;
}

bool getModbus16Bit(ModbusMaster &node, int i)
{
    uint8_t result;
    uint16_t addr = config.sensor[i].address % 1000;
log_d("Modbus Reg=%d",addr);
    result = node.readHoldingRegisters(addr, 1);
    if (result == node.ku8MBSuccess)
    {
        if (config.sensor[i].port == PORT_MODBUS_16)
        {
            double val = (double)node.getResponseBuffer(0); // 0x0000
            sensorUpdate(i, val);
            return true;
        }
    }
    return false;
}

bool getModbus32Bit(ModbusMaster &node, int i)
{
    uint8_t result;
    uint16_t addr = config.sensor[i].address % 1000;

    result = node.readHoldingRegisters(addr, 2);
    if (result == node.ku8MBSuccess)
    {
        if (config.sensor[i].port == PORT_MODBUS_32)
        {
            uint32_t val32 = (uint32_t)node.getResponseBuffer(0); // MSB
            val32 <<= 16;
            val32 |= (uint32_t)node.getResponseBuffer(1); // LSB
            double val = (double)val32;                   // to value double
            sensorUpdate(i, val);
            return true;
        }
    }
    return false;
}

bool getSensor(int cfgIdx)
{
    bool status;
    // log_d("Sensor Port %d", config.sensor[cfgIdx].port);
    uint8_t port = config.sensor[cfgIdx].port;
    switch (port)
    {
    case PORT_ADC:
        if(getADC(config.sensor[cfgIdx].address)){
            return true;
        }
        break;
    case PORT_M701:
        if (config.modbus_channel == 1)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial0);
        }
        else if (config.modbus_channel == 2)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial1);
        }
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C6)
        else if (config.modbus_channel == 3)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial2);
        }
#endif
        else
        {
            return false;
        }
        if (getM701Modbus(modbus))
            return true;
        break;
    case PORT_M702:
        if (config.modbus_channel == 1)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial0);
        }
        else if (config.modbus_channel == 2)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial1);
        }
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C6)
        else if (config.modbus_channel == 3)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial2);
        }
#endif
        else
        {
            return false;
        }
        if (getM702Modbus(modbus))
            return true;
        break;
    case PORT_PZEM:
        if (config.modbus_channel == 1)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial0);
        }
        else if (config.modbus_channel == 2)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial1);
        }
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C6)
        else if (config.modbus_channel == 3)
        {
            modbus.begin(config.sensor[cfgIdx].address, Serial2);
        }
#endif
        else
        {
            return false;
        }
        if (getPZEMModbus(modbus))
            return true;
        break;
    case PORT_MODBUS_16:
        if (config.modbus_channel == 1)
        {
            modbus.begin(config.sensor[cfgIdx].address/1000, Serial);
        }
        else if (config.modbus_channel == 2)
        {
            modbus.begin(config.sensor[cfgIdx].address/1000, Serial1);
        }
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C6)
        else if (config.modbus_channel == 3)
        {
            modbus.begin(config.sensor[cfgIdx].address/1000, Serial2);
        }
#endif
        else
        {
            return false;
        }
        if (getModbus16Bit(modbus,cfgIdx))
            return true;
        break;
    case PORT_MODBUS_32:
        if (config.modbus_channel == 1)
        {
            modbus.begin(config.sensor[cfgIdx].address/1000, Serial);
        }
        else if (config.modbus_channel == 2)
        {
            modbus.begin(config.sensor[cfgIdx].address/1000, Serial1);
        }
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C6)
        else if (config.modbus_channel == 3)
        {
            modbus.begin(config.sensor[cfgIdx].address/1000, Serial2);
        }
#endif
        else
        {
            return false;
        }
        if (getModbus32Bit(modbus,cfgIdx))
            return true;
        break;
    case PORT_BME280_I2C0:
        if (config.i2c_enable)
        {
            int i2c_timeout = 0;
            while (i2c_busy)
            {
                delay(10);
                if (++i2c_timeout > 50)
                    break;
            }
            i2c_busy = true;
            if (getBME_I2C(*bme, PORT_BME280_I2C0))
            {
                i2c_busy = false;
                return true;
            }
            i2c_busy = false;
        }
        break;
    case PORT_BME280_I2C1:
        if (config.i2c1_enable)
        {
            if (getBME_I2C(*bme, PORT_BME280_I2C1))
            {
                return true;
            }
        }
        break;
    case PORT_BMP280_I2C0:
        if (config.i2c_enable)
        {
            if (getBMP_I2C(*bmp280, port))
                return true;
        }
        break;
    case PORT_BMP280_I2C1:
        if (config.i2c1_enable)
        {
            if (getBMP_I2C(*bmp280, port))
                return true;
        }
        break;
    case PORT_SI7021_I2C0:
        if (config.i2c_enable)
        {
            if (getSI7021_I2C(*Si7021, port))
                return true;
        }
        break;
    case PORT_SI7021_I2C1:
        if (config.i2c1_enable)
        {
            if (getSI7021_I2C(*Si7021, port))
                return true;
        }
        break;
    case PORT_CCS811_I2C0:
        if (config.i2c_enable)
        {
            if (ccs->available())
            {
                //float temp = ccs->calculateTemperature();
                //log_d("CCS811 temperature %0.2fC", temp);
            //}
                if (getCCS_I2C(*ccs, port))
                    return true;
            }
            else
            {
                if (ccs->checkError())
                {
                    ccs->begin(config.sensor[cfgIdx].address, &Wire);
                    log_d("CCS811 Restart boot");
                }
            }
        }
        break;
    case PORT_CCS811_I2C1:
        if (config.i2c1_enable)
        {
            if (ccs->available())
            {
                if (getCCS_I2C(*ccs, port))
                {
                    return true;
                }
            }
        }
        break;
    case PORT_SAT_NUM:
        if (config.gnss_enable)
        {
            getSAT();
        }
        else
        {
            log_d("Not enable GNSS");
        }
        break;
    case PORT_SAT_HDOP:
        if (config.gnss_enable)
        {
            getSAT();
        }
        else
        {
            log_d("Not enable GNSS");
        }
        break;
    case PORT_SHT_I2C0:
        if (config.i2c_enable)
        {

            if (getSHT_I2C(*sht, port))
                return true;
        }
        break;
    case PORT_SHT_I2C1:
        if (config.i2c_enable)
        {
            if (getSHT_I2C(*sht, port))
                return true;
        }
        break;
    case PORT_BATTERY:
        getBAT(0);
        break;
    case PORT_CNT_0:
        getCNT0(port);
        break;
    case PORT_CNT_1:
        getCNT1(port);
        break;
    case PORT_LOGIC:
        getLOGIC(port);
        break; 
    case PORT_DS1820:
        if(config.onewire_enable){
            getDS1820(port);
        }
        break;       
    default:
        log_d("Sensor Not config");
        break;
    }

    return false;
}

// void taskCounter(void *pvParameters)
// {
//     for (;;)
//     {
//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }

// }

void pulse_ch0() // measure the quantity of square wave
{
    // waterFlow += 1.0 / 5880.0;
    cnt0++;
}

void pulse_ch1() // measure the quantity of square wave
{
    // waterFlow += 1.0 / 5880.0;
    cnt1++;
}

void sensorInit(bool resetAll)
{
    // Initialize with the begin sensor
    for (int i = 0; i < SENSOR_NUMBER; i++)
    {
        if(resetAll){
            sen[i].counter = 0;
            sen[i].sum = 0;
            sen[i].timeAvg = 0;
            sen[i].timeTick = 0;
        }
        sen[i].visable = false;
        sen[i].timeSample = 0;

        if (!config.sensor[i].enable) continue;

        uint8_t port = config.sensor[i].port;
        switch (port)
        {
        case PORT_ADC:
            analogReadResolution(12);
            analogSetAttenuation(ADC_11db);
            break;
        case PORT_CNT_0:
            pinMode(config.sensor[i].address, INPUT);
            attachInterrupt(config.sensor[i].address, pulse_ch0, RISING);
            break;
        case PORT_CNT_1:
            pinMode(config.sensor[i].address, INPUT);
            attachInterrupt(config.sensor[i].address, pulse_ch1, RISING);
            break;
        case PORT_BME280_I2C0:
            if (config.i2c_enable)
            {
                int i2c_timeout = 0;
                while (i2c_busy)
                {
                    delay(10);
                    if (++i2c_timeout > 50)
                        break;
                }
                i2c_busy = true;
                if (bme != NULL)
                    break;
                bme = new Adafruit_BME280();
                if (!bme->begin(config.sensor[i].address, &Wire)) // 0x76=118,0x77=119
                {
                    log_d("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
                    log_d("SensorID was: 0x%0X", bme->sensorID()); // log_d(bme.sensorID(),16);
                    log_d("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
                    log_d("ID of 0x56-0x58 represents a BMP 280,");
                    log_d("ID of 0x60 represents a BME 280.");
                    log_d("ID of 0x61 represents a BME 680.");
                }
                i2c_busy = false;
            }
            else
            {
                log_d("Not enable I2C0 port");
            }
            break;
        case PORT_BME280_I2C1:
            if (config.i2c1_enable)
            {
                if (bme != NULL)
                    break;
                bme = new Adafruit_BME280();
                if (!bme->begin(config.sensor[i].address, &Wire1)) // 0x76=118,0x77=119
                {
                    log_d("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
                    log_d("SensorID was: 0x%0X", bme->sensorID()); // log_d(bme.sensorID(),16);
                    log_d("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
                    log_d("ID of 0x56-0x58 represents a BMP 280,");
                    log_d("ID of 0x60 represents a BME 280.");
                    log_d("ID of 0x61 represents a BME 680.");
                }
            }
            else
            {
                log_d("Not enable I2C1 port");
            }
            break;
        case PORT_BMP280_I2C0:
            if (config.i2c_enable)
            {
                if (bmp280 != NULL)
                    break;
                bmp280 = new Adafruit_BMP280(&Wire);
                if (!bmp280->begin(config.sensor[i].address)) // 0x76=118,0x77=119
                {
                    log_d("Could not find a valid BMP280 sensor, check wiring, address, sensor ID!");
                    log_d("SensorID was: 0x%0X", bmp280->sensorID()); // log_d(bme.sensorID(),16);
                    log_d("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
                    log_d("ID of 0x56-0x58 represents a BMP 280,");
                    log_d("ID of 0x60 represents a BME 280.");
                    log_d("ID of 0x61 represents a BME 680.");
                }
            }
            else
            {
                log_d("Not enable I2C0 port");
            }
            break;
        case PORT_BMP280_I2C1:
            if (config.i2c1_enable)
            {
                if (bmp280 != NULL)
                    break;
                bmp280 = new Adafruit_BMP280(&Wire1);
                if (!bmp280->begin(config.sensor[i].address)) // 0x76=118,0x77=119
                {
                    log_d("Could not find a valid BMP280 sensor, check wiring, address, sensor ID!");
                    log_d("SensorID was: 0x%0X", bmp280->sensorID()); // log_d(bme.sensorID(),16);
                    log_d("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
                    log_d("ID of 0x56-0x58 represents a BMP 280,");
                    log_d("ID of 0x60 represents a BME 280.");
                    log_d("ID of 0x61 represents a BME 680.");
                }
            }
            else
            {
                log_d("Not enable I2C1 port");
            }
            break;
        case PORT_SI7021_I2C0:
            if (config.i2c_enable)
            {
                if (Si7021 != NULL)
                    break;
                Si7021 = new Adafruit_Si7021(&Wire);
                if (!Si7021->begin())
                {
                    log_d("Could not find a valid SI7021 sensor, check wiring, address, sensor ID!");
                }
            }
            else
            {
                log_d("Not enable I2C0 port");
            }
            break;
        case PORT_SI7021_I2C1:
            if (config.i2c1_enable)
            {
                if (Si7021 != NULL)
                    break;
                Si7021 = new Adafruit_Si7021(&Wire1);
                if (!Si7021->begin())
                {
                    log_d("Could not find a valid SI7021 sensor, check wiring, address, sensor ID!");
                }
            }
            else
            {
                log_d("Not enable I2C1 port");
            }
            break;
        case PORT_CCS811_I2C0:
            if (config.i2c_enable)
            {
                int i2c_timeout = 0;
                while (i2c_busy)
                {
                    delay(10);
                    if (++i2c_timeout > 50)
                        break;
                }
                i2c_busy = true;
                if (ccs != NULL)
                    break;
                ccs = new Adafruit_CCS811();
                if (ccs->begin(config.sensor[i].address, &Wire))
                {
                    ccs->setTempOffset(0);
                }
                else
                {
                    log_d("Could not find a valid CCS811 sensor, check wiring, address, sensor ID!");
                }
                i2c_busy = false;
            }
            else
            {
                log_d("Not enable I2C0 port");
            }
            break;
        case PORT_CCS811_I2C1:
            if (config.i2c1_enable)
            {
                if (ccs != NULL)
                    break;
                ccs = new Adafruit_CCS811();
                if (ccs->begin(config.sensor[i].address, &Wire1))
                {
                    ccs->setTempOffset(0);
                }
                else
                {
                    log_d("Could not find a valid CCS811 sensor, check wiring 1, address, sensor ID!");
                }
            }
            else
            {
                log_d("Not enable I2C1 port");
            }
            break;
        case PORT_SHT_I2C0:
            if (config.i2c_enable)
            {
                sht = new SHTSensor();
                if (!sht->init(Wire))
                {
                    log_d("Could not find a valid SHTxx sensor, check wiring, address, sensor ID!");
                }
            }
            else
            {
                log_d("Not enable I2C0 port");
            }
            break;
        case PORT_SHT_I2C1:
            if (config.i2c1_enable)
            {
                sht = new SHTSensor();
                if (!sht->init(Wire1))
                {
                    log_d("Could not find a valid SHTxx sensor, check wiring, address, sensor ID!");
                }
            }
            else
            {
                log_d("Not enable I2C1 port");
            }
            break;
        case PORT_DS1820:
            // if(oneWire!=NULL)
            // {
            //     //ds1820 = new DallasTemperature(oneWire);
            //     //ds1820->begin();                
            // }else{
            //     log_d("Not enable 1-Wire port");
            // }
            break;
        }
                    
    }
}

void taskSensor(void *pvParameters)
{
    bool cntEnable = false;
    log_d("Sensor Task Init");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
#ifdef HELTEC_HTIT_TRACKER
    pinMode(2, INPUT_PULLUP);
    pinMode(1, ANALOG);
    digitalWrite(2, HIGH);
#elif defined(HELTEC_V3_GPS)
    pinMode(37, INPUT_PULLUP);
    pinMode(1, ANALOG);
    digitalWrite(37, HIGH);
#elif defined(BUOY)
    pinMode(0, ANALOG);
#endif

    if (config.i2c_enable)
    {
        int i2c_timeout = 0;
        while (i2c_busy)
        {
            delay(10);
            if (++i2c_timeout > 50)
                break;
        }
        i2c_busy = true;
        Wire.begin(config.i2c_sda_pin, config.i2c_sck_pin, config.i2c_freq);
        // ccs = new Adafruit_CCS811();
        // ccs->begin(90, &Wire); // 0x5A=90
        i2c_busy = false;
    }
    if (config.i2c1_enable)
    {
        Wire1.begin(config.i2c1_sda_pin, config.i2c1_sck_pin, config.i2c1_freq);
    }

    if(config.onewire_enable)
    {
        //oneWire = new OneWireHub(config.onewire_gpio);
    }

    sensorInit(true);

    // if(cntEnable){
    //     TaskHandle_t counterTaskHandle;
    //     xTaskCreate(taskCounter, "Counter Task", 1024, NULL, 1, &counterTaskHandle);
    // }
    log_d("Sensor Initialize successfully");
    int dp=0;
    for (;;)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        unsigned long tick=millis();
        for (int i = 0; i < SENSOR_NUMBER; i++)
        {
            if (config.sensor[i].enable)
            {
                if (tick > sen[i].timeTick)
                {   
                    if(getSensor(i)){
                        sen[i].timeTick = millis() + ((unsigned long)config.sensor[i].samplerate * 1000);
                    }else{
                        sen[i].timeTick = millis() + (30 * 1000);
                    }
                    log_d("Request getSensor [%d] for %s timeTick=%d/%d", i, config.sensor[i].parm,tick,sen[i].timeTick);
                    //dispSensor(i);
                    //vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
        }
        if(++dp>100){
            dp=0;
            dispSensor();
        }
    }
}
