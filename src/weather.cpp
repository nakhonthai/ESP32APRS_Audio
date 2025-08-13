/*
 Name:		ESP32IGate
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/

#include "weather.h"
#include <WiFi.h>
#include <TinyGPSPlus.h>

extern Configuration config;
extern WiFiClient aprsClient;
extern SensorData sen[SENSOR_NUMBER];
extern TinyGPSPlus gps;

WeatherData weather;

float wgArray[5];
uint8_t wgIdx=0;

float mslAltitude=0;

bool weatherUpdate = false;
bool getCSV2Wx(String stream)
{
    bool ret = false;
    String weatherRaw;
    int st = 0;
    // stream = Serial.readString();
    //  Serial.println(stream);
    // delay(100);
    st = stream.indexOf("MQTT:");
    // Serial.printf("Find ModBus > %d",st);
    // Serial.println(stream);
    if (st >= 0)
    {
        // String data = stream.substring(st+5);
        weatherRaw = stream.substring(st + 5);
        weatherUpdate = true;
        // Serial.println("Weather DATA: " + weatherRaw);
    }
    // else
    // {
    //     st = stream.indexOf("CLK?");
    //     if (st >= 0)
    //     {
    //         struct tm br_time;
    //         getLocalTime(&br_time, 5000);
    //         char strtmp[30];
    //         Serial.printf(strtmp, "#CLK=%02d/%02d/%02d,%02d:%02d:%02d\r\n", br_time.tm_year - 2000, br_time.tm_mon, br_time.tm_mday, br_time.tm_hour, br_time.tm_min, br_time.tm_sec);
    //     }
    // }
    // CSV Format DATA:date,time,rain(mm),ws(kph),wd(deg),solar(w/m),barometric(hpa/10),temp(c),humidity(%RH),vbat(V),vsolar(V),ibat(A),pbat(W),windgust(kph)
    if (weatherUpdate)
    {
        // String value;
        float rainSample = getValue(weatherRaw, ',', 2).toFloat();
        weather.rain += rainSample;
        weather.rain24hr += rainSample;
        weather.windspeed = getValue(weatherRaw, ',', 3).toFloat();
        weather.winddirection = getValue(weatherRaw, ',', 4).toInt();
        weather.solar = getValue(weatherRaw, ',', 5).toInt();
        weather.barometric = getValue(weatherRaw, ',', 6).toFloat() * 10.0F;
        weather.temperature = getValue(weatherRaw, ',', 7).toFloat();
        weather.humidity = getValue(weatherRaw, ',', 8).toFloat();
        weather.windgust = getValue(weatherRaw, ',', 13).toFloat();
        weather.vbat = getValue(weatherRaw, ',', 9).toFloat();
        weather.vsolar = getValue(weatherRaw, ',', 10).toFloat();
        weather.ibat = getValue(weatherRaw, ',', 11).toFloat();
        weather.pbat = getValue(weatherRaw, ',', 12).toInt();

        weather.visable |= WX_RAIN;
        weather.visable |= WX_RAIN24HR;
        weather.visable |= WX_WIND_SPD;
        weather.visable |= WX_WIND_DIR;
        weather.visable |= WX_LUMINOSITY;
        weather.visable |= WX_BARO;
        weather.visable |= WX_WIND_GUST;
        weather.visable |= WX_TEMP;
        weather.visable |= WX_HUMIDITY;

        weatherUpdate = false;
        // wxTimeout = millis();
        char strData[300];
        size_t lng = getRawWx(strData);

        uint8_t SendMode = 0;
        if (config.wx_2rf) SendMode |= RF_CHANNEL;
        if (config.wx_2inet) SendMode |= INET_CHANNEL;
        pkgTxPush(strData, lng, 0,SendMode);
        // if (config.wx_2rf)
        // { // WX SEND POSITION TO RF
        //     pkgTxPush(strData, lng, 0);
        // }
        // if (config.wx_2inet)
        // { // WX SEND TO APRS-IS
        //     if (aprsClient.connected())
        //     {
        //         // status.txCount++;
        //         aprsClient.println(strData); // Send packet to Inet
        //     }
        // }
        // Serial.printf("APRS: %s\r\n",strData);
        return true;
    }
    return false;
    // ModbusSerial.flush();
}

void getSensor(uint32_t type, float *val, int i)
{
    int senIdx = config.wx_sensor_ch[i];
    //log_d("type:%d i:%d senIdx:%d en:%x vis:%x",type,i,senIdx,config.wx_sensor_enable[i],sen[senIdx].visable);
    if (config.wx_sensor_enable[i] && senIdx > 0)
    {
        senIdx -= 1;
        if (sen[senIdx].visable)
        {
            
            weather.visable |= type;
            if (!config.wx_sensor_avg[i] || (config.sensor[senIdx].averagerate <= config.sensor[senIdx].samplerate) || (sen[senIdx].timeAvg == 0))
                *val = sen[senIdx].sample;
            else
                *val = sen[senIdx].average;
        }
        else
        {
            weather.visable &= ~type;
        }
    }
    else
    {
        weather.visable &= ~type;
    }
}

void getSensor(uint32_t type, uint16_t *val, int i)
{
    int senIdx = config.wx_sensor_ch[i];
    if (config.wx_sensor_enable[i] && senIdx > 0)
    {
        senIdx -= 1;
        if (sen[senIdx].visable)
        {
            weather.visable |= type;
            if (!config.wx_sensor_avg[i] || (config.sensor[senIdx].averagerate <= config.sensor[senIdx].samplerate) || (sen[senIdx].timeAvg == 0))
                *val = sen[senIdx].sample;
            else
                *val = sen[senIdx].average;
        }
        else
        {
            weather.visable &= ~type;
        }
    }
    else
    {
        weather.visable &= ~type;
    }
}

void getSensor(uint32_t type, uint32_t *val, int i)
{
    int senIdx = config.wx_sensor_ch[i];
    if (config.wx_sensor_enable[i] && senIdx > 0)
    {
        senIdx -= 1;
        if (sen[senIdx].visable)
        {            
            weather.visable |= type;
            if (!config.wx_sensor_avg[i] || (config.sensor[senIdx].averagerate <= config.sensor[senIdx].samplerate) || (sen[senIdx].timeAvg == 0))
                *val = sen[senIdx].sample;
            else
                *val = sen[senIdx].average;
        }
        else
        {
            weather.visable &= ~type;
        }
    }
    else
    {
        weather.visable &= ~type;
    }
}

int getRawWx(char *strData)
{
    unsigned int i;
    char strtmp[300], obj[30];

    memset(&obj[0], 0, sizeof(obj));

    weather.visable = 0;

    uint32_t senType = 1;
    for (i = 0; i < WX_SENSOR_NUM; i++)
    {
        
        //log_d("Type:%d",senType);
        switch (senType)
        {
        case WX_WIND_DIR:
            getSensor(senType, &weather.winddirection, i);
            break;
        case WX_WIND_SPD:
            getSensor(senType, &weather.windspeed, i);
            break;
        case WX_WIND_GUST:
            float windspeed;
            getSensor(senType, &windspeed, i);
            if(wgIdx>=(sizeof(wgArray)/4)) wgIdx=0;
            wgArray[wgIdx++]=windspeed;
            weather.windgust=0.0F;
            for(int i;i<(sizeof(wgArray)/4);i++)
            {
                if(wgArray[i]>weather.windgust) weather.windgust=wgArray[i];
            }
            //getSensor(senType, &weather.windgust, i);
            break;
        case WX_TEMP:
            getSensor(senType, &weather.temperature, i);
            break;
        case WX_RAIN:
            getSensor(senType, &weather.rain, i);
            break;
        case WX_RAIN24HR:
            getSensor(senType, &weather.rain24hr, i);
            break;
        case WX_RAIN_GMT:
            getSensor(senType, &weather.rainmidnight, i);
            break;
        case WX_HUMIDITY:
            getSensor(senType, &weather.humidity, i);
            break;
        case WX_BARO:
            getSensor(senType, &weather.barometric, i);
            break;
        case WX_LUMINOSITY:
            getSensor(senType, &weather.solar, i);
            break;
        case WX_SNOW:
            getSensor(senType, &weather.snow, i);
            break;
        case WX_SOIL_TEMP:
            getSensor(senType, &weather.soil_temp, i);
            break;
        case WX_SOIL_MOISTURE:
            getSensor(senType, &weather.soil_moisture, i);
            break;
        case WX_WATER_TEMP:
            getSensor(senType, &weather.water_temp, i);
            break;
        case WX_WATER_TDS:
            getSensor(senType, &weather.water_tds, i);
            break;
        case WX_WATER_LEVEL:
            getSensor(senType, &weather.water_level, i);
            break;
        case WX_PM25:
            getSensor(senType, &weather.pm25, i);
            break;
        case WX_PM100:
            getSensor(senType, &weather.pm100, i);
            break;
        case WX_CO2:
            getSensor(senType, &weather.co2, i);
            break;
        case WX_CH2O:
            getSensor(senType, &weather.ch2o, i);
            break;
        case WX_TVOC:
            getSensor(senType, &weather.tvoc, i);
            break;
        case WX_UV:
            getSensor(senType, (uint16_t*)&weather.uv, i);
            break;
        case WX_SOUND:
            getSensor(senType, &weather.sound, i);
            break;
        }
        senType <<= 1;
    }

    config.wx_flage = 0xFFFFFFFF;

    if (config.wx_flage & WX_WIND_DIR)
    {
        if (weather.visable & WX_WIND_DIR & (weather.winddirection<361))
            sprintf(strtmp, "%03u/", weather.winddirection);
        else
            sprintf(strtmp, ".../");
    }
    else
    {
        sprintf(strtmp, ".../");
    }
    strcat(strData, strtmp);
    if (config.wx_flage & WX_WIND_SPD)
    {
        if (weather.visable & WX_WIND_SPD)
            sprintf(strtmp, "%03u", (unsigned int)round(weather.windspeed * 0.621));
        else
            sprintf(strtmp, "...");
    }
    else
    {
        sprintf(strtmp, "...");
    }
    strcat(strData, strtmp);

    if ((weather.visable & WX_WIND_SPD) && (weather.visable & WX_WIND_GUST))
    {
        sprintf(strtmp, "g%03u", (unsigned int)round(weather.windgust * 0.621));
    }
    else
    {
        sprintf(strtmp, "g...");
    }
    strcat(strData, strtmp);

    if (config.wx_flage & WX_TEMP)
    {
        if ((weather.visable & WX_TEMP) && (weather.temperature<537.0F))
        {
            unsigned int tempF = (unsigned int)round((weather.temperature * 9 / 5) + 32);
            sprintf(strtmp, "t%03u", tempF);
        }
        else
        {
            sprintf(strtmp, "t...");
        }
    }
    else
    {
        sprintf(strtmp, "t...");
    }
    strcat(strData, strtmp);

    unsigned int rain = (unsigned int)round((weather.rain * 100.0F) / 25.6F);
    unsigned int rain24 = (unsigned int)round((weather.rain24hr * 100.0F) / 25.6F);
    unsigned int rainGMT = (unsigned int)round((weather.rainmidnight * 100.0F) / 25.6F);
    time_t now;
    time(&now);
    struct tm *info = gmtime(&now);
    if (info->tm_min == 0)
    {
        rain = 0;
        weather.rain = 0;
    }
    if (info->tm_hour == 0 && info->tm_min == 0)
    {
        rain24 = 0;
        weather.rain24hr = 0;
    }

    if (config.wx_flage & WX_RAIN)
    {
        if (weather.visable & WX_RAIN)
            sprintf(strtmp, "r%03u", rain);
        else
            sprintf(strtmp, "r...");
    }
    else
    {
        sprintf(strtmp, "r...");
    }
    strcat(strData, strtmp);
    if (config.wx_flage & WX_RAIN24HR)
    {
        if (weather.visable & WX_RAIN24HR)
            sprintf(strtmp, "p%03u", rain24);
        else
            sprintf(strtmp, "p...");
    }
    else
    {
        sprintf(strtmp, "p...");
    }
    strcat(strData, strtmp);

    if (config.wx_flage & WX_RAIN_GMT)
    {
        if (weather.visable & WX_RAIN_GMT)
            sprintf(strtmp, "P%03u", rainGMT);
        else
            sprintf(strtmp, "P...");
    }
    else
    {
        sprintf(strtmp, "P...");
    }
    strcat(strData, strtmp);

    if (config.wx_flage & WX_HUMIDITY)
    {
        if ((weather.visable & WX_HUMIDITY) && (weather.humidity<100.0F))
            sprintf(strtmp, "h%02u", (unsigned int)round(weather.humidity));
        else
            sprintf(strtmp, "h..");
    }
    else
    {
        sprintf(strtmp, "h..");
    }
    strcat(strData, strtmp);

    if (config.wx_flage & WX_BARO)
    {
        if (weather.visable & WX_BARO){
            //* @param pressure - The current pressure in Pascals (Pa).
            //* @param altitude - The altitude in meters (m) above sea level.
            // Calculate the MSL pressure using the barometric formula
            float pressure = weather.barometric * 100.0;
            float mslPressure = pressure / pow(1 - (mslAltitude / 44330.0), 5.255);
            sprintf(strtmp, "b%05u", (unsigned int)(mslPressure/10.0));
        }else{
            sprintf(strtmp, "b.....");
        }
    }
    else
    {
        sprintf(strtmp, "b.....");
    }
    strcat(strData, strtmp);

    if (config.wx_flage & WX_LUMINOSITY)
    {
        if (weather.visable & WX_LUMINOSITY)
        {
            if (weather.solar < 1000)
                sprintf(strtmp, "L%03u", (unsigned int)weather.solar);
            else
                sprintf(strtmp, "l%03u", (unsigned int)weather.solar - 1000);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_SNOW)
    {
        if (weather.visable & WX_SNOW)
        {
            sprintf(strtmp, "S%03u", (unsigned int)(weather.snow));
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_SOIL_TEMP)
    {
        if (weather.visable & WX_SOIL_TEMP)
        {
            sprintf(strtmp, "m%03u", (int)round((weather.soil_temp * 9 / 5) + 32));
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_SOIL_MOISTURE)
    {
        if (weather.visable & WX_SOIL_MOISTURE)
        {
            sprintf(strtmp, "M%03u", (unsigned int)round(weather.soil_moisture * 10));
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_WATER_TEMP)
    {
        if (weather.visable & WX_WATER_TEMP)
        {
            sprintf(strtmp, "w%03u", (int)round((weather.water_temp * 9 / 5) + 32));
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_WATER_TDS)
    {
        if (weather.visable & WX_WATER_TDS)
        {
            sprintf(strtmp, "W%03u", (int)weather.water_tds);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_WATER_LEVEL)
    {
        if (weather.visable & WX_WATER_LEVEL)
        {
            sprintf(strtmp, "v%03u", (int)weather.water_level);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_PM25)
    {

        if (weather.visable & WX_PM25)
        {
            sprintf(strtmp, "d%03u", (int)weather.pm25);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_PM100)
    {
        if (weather.visable & WX_PM100)
        {
            sprintf(strtmp, "D%03u", (int)weather.pm100);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_CO2)
    {
        if (weather.visable & WX_CO2)
        {
            if (weather.co2 < 10000)
                sprintf(strtmp, "x%04u", (unsigned int)weather.co2);
            else
                sprintf(strtmp, "X%04u", (unsigned int)weather.co2/10);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_CH2O)
    {
        if (weather.visable & WX_CH2O)
        {
            sprintf(strtmp, "F%04u", (unsigned int)weather.ch2o);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_TVOC)
    {
        if (weather.visable & WX_TVOC)
        {
            sprintf(strtmp, "o%04u", (unsigned int)weather.tvoc);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_UV)
    {
        if (weather.visable & WX_UV)
        {
            sprintf(strtmp, "u%02u", (unsigned int)weather.uv);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_SOUND)
    {
        if (weather.visable & WX_SOUND)
        {
            sprintf(strtmp, "n%03u", (unsigned int)round(weather.sound));
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_TEMP)
    {
        if (weather.visable & WX_TEMP)
        {
            double tmp=(weather.temperature * 9.0F / 5.0F) + 32.0F;
            tmp *= 50.0F;
            unsigned int tempF = (unsigned int)round(tmp);
            if(tempF < 0){
                sprintf(strtmp, "T-%03u", tempF);
            }else{
                sprintf(strtmp, "T%04u", tempF);
            }
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_HUMIDITY)
    {
        if (weather.visable & WX_HUMIDITY){
            sprintf(strtmp, "H%03u", (unsigned int)round(weather.humidity*10.0F));
            strcat(strData, strtmp);
        }
    }

    // sprintf(strtmp, " BAT:%0.2fV/%dmA", weather.vbat, (int)(weather.ibat * 1000));
    // strcat(strData, strtmp);
    //  sprintf(strtmp,"/%0.1fmA",weather.ibat*1000);
    //  strcat(strData,strtmp);
    //  i06L...
    //  i04S...
    //  i03I...

    i = strlen(strData);
    return i;
    // c...s...g...t...r...p...P...h..b.....L...S..m...M...w...W....v...d...D...x....n...T....H...
}

int getWxJson(char *strData)
{
    unsigned int i;
    char strtmp[300], obj[30];

    memset(&obj[0], 0, sizeof(obj));

    weather.visable = 0;

    uint32_t senType = 1;
    for (i = 0; i < WX_SENSOR_NUM; i++)
    {
        
        //log_d("Type:%d",senType);
        switch (senType)
        {
        case WX_WIND_DIR:
            getSensor(senType, &weather.winddirection, i);
            break;
        case WX_WIND_SPD:
            getSensor(senType, &weather.windspeed, i);
            break;
        case WX_WIND_GUST:
            getSensor(senType, &weather.windgust, i);
            break;
        case WX_TEMP:
            getSensor(senType, &weather.temperature, i);
            break;
        case WX_RAIN:
            getSensor(senType, &weather.rain, i);
            break;
        case WX_RAIN24HR:
            getSensor(senType, &weather.rain24hr, i);
            break;
        case WX_RAIN_GMT:
            getSensor(senType, &weather.rainmidnight, i);
            break;
        case WX_HUMIDITY:
            getSensor(senType, &weather.humidity, i);
            break;
        case WX_BARO:
            getSensor(senType, &weather.barometric, i);
            break;
        case WX_LUMINOSITY:
            getSensor(senType, &weather.solar, i);
            break;
        case WX_SNOW:
            getSensor(senType, &weather.snow, i);
            break;
        case WX_SOIL_TEMP:
            getSensor(senType, &weather.soil_temp, i);
            break;
        case WX_SOIL_MOISTURE:
            getSensor(senType, &weather.soil_moisture, i);
            break;
        case WX_WATER_TEMP:
            getSensor(senType, &weather.water_temp, i);
            break;
        case WX_WATER_TDS:
            getSensor(senType, &weather.water_tds, i);
            break;
        case WX_WATER_LEVEL:
            getSensor(senType, &weather.water_level, i);
            break;
        case WX_PM25:
            getSensor(senType, &weather.pm25, i);
            break;
        case WX_PM100:
            getSensor(senType, &weather.pm100, i);
            break;
        case WX_CO2:
            getSensor(senType, &weather.co2, i);
            break;
        case WX_CH2O:
            getSensor(senType, &weather.ch2o, i);
            break;
        case WX_TVOC:
            getSensor(senType, &weather.tvoc, i);
            break;
        case WX_UV:
            getSensor(senType, (uint16_t*)&weather.uv, i);
            break;
        case WX_SOUND:
            getSensor(senType, &weather.sound, i);
            break;
        case WX_VBAT:
            getSensor(senType, &weather.vbat, i);
            break;
        case WX_IBAT:
            getSensor(senType, &weather.ibat, i);
            break;
        case WX_VSOLAR:
            getSensor(senType, &weather.vsolar, i);
            break;
        }
        senType <<= 1;
    }

    config.wx_flage = 0xFFFFFFFF;

    time_t timeStamp;
    time(&timeStamp);
    sprintf(strData,"{\"timeStamp\":\"%i\"",timeStamp);

    if (config.wx_flage & WX_WIND_DIR)
    {
        if (weather.visable & WX_WIND_DIR){
            sprintf(strtmp, ",\"windCourse\":\"%d\"", weather.winddirection);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_WIND_SPD)
    {
        if (weather.visable & WX_WIND_SPD){
            sprintf(strtmp, ",\"windSpeed\":\"%.1f\"", weather.windspeed * 0.621);
            strcat(strData, strtmp);
        }
    }

    if ((weather.visable & WX_WIND_SPD) && (weather.visable & WX_WIND_GUST))
    {
        sprintf(strtmp, ",\"windGust\":\"%.1f\"", weather.windgust * 0.621);
        strcat(strData, strtmp);
    }

    if (config.wx_flage & WX_TEMP)
    {
        if (weather.visable & WX_TEMP)
        {
            //unsigned int tempF = (unsigned int)round((weather.temperature * 9 / 5) + 32);
            sprintf(strtmp, ",\"Temperature\":\"%.2f\"", weather.temperature);
            strcat(strData, strtmp);
        }
    }

    unsigned int rain = (unsigned int)round((weather.rain * 100.0F) / 25.6F);
    unsigned int rain24 = (unsigned int)round((weather.rain24hr * 100.0F) / 25.6F);
    unsigned int rainGMT = (unsigned int)round((weather.rainmidnight * 100.0F) / 25.6F);
    time_t now;
    time(&now);
    struct tm *info = gmtime(&now);
    if (info->tm_min == 0)
    {
        rain = 0;
        weather.rain = 0;
    }
    if (info->tm_hour == 0 && info->tm_min == 0)
    {
        rain24 = 0;
        weather.rain24hr = 0;
    }

    if (config.wx_flage & WX_RAIN)
    {
        if (weather.visable & WX_RAIN){
            sprintf(strtmp, ",\"Rain\":\"%d\"", weather.rain);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_RAIN24HR)
    {
        if (weather.visable & WX_RAIN24HR){
            sprintf(strtmp, ",\"Rain24h\":\"%.1f\"", weather.rain24hr);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_RAIN_GMT)
    {
        if (weather.visable & WX_RAIN_GMT){
            sprintf(strtmp, ",\"RainGMT\":\"%.1f\"", weather.rainmidnight );
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_HUMIDITY)
    {
        if (weather.visable & WX_HUMIDITY){
            sprintf(strtmp, ",\"Humidity\":\"%.1f\"",weather.humidity);
            strcat(strData, strtmp);
        }
    }
    

    if (config.wx_flage & WX_BARO)
    {
        if (weather.visable & WX_BARO){
            //* @param pressure - The current pressure in Pascals (Pa).
            //* @param altitude - The altitude in meters (m) above sea level.
            // Calculate the MSL pressure using the barometric formula
            float pressure = weather.barometric * 100.0;
            float mslPressure = pressure / pow(1 - (mslAltitude / 44330.0), 5.255);
            sprintf(strtmp, ",\"Barometric\":\"%.2f\"", mslPressure/100.0F);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_LUMINOSITY)
    {
        if (weather.visable & WX_LUMINOSITY)
        {
            sprintf(strtmp, ",\"solarRad\":\"%u\"", (unsigned int)weather.solar);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_VBAT)
    {
        if (weather.visable & WX_VBAT)
        {
            sprintf(strtmp, ",\"Voltage\":\"%.2f\"", weather.vbat);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_IBAT)
    {
        if (weather.visable & WX_IBAT)
        {
            sprintf(strtmp, ",\"Current\":\"%.3f\"", weather.ibat);
            strcat(strData, strtmp);
        }
    }

    if (config.wx_flage & WX_VSOLAR)
    {
        if (weather.visable & WX_VSOLAR)
        {
            sprintf(strtmp, ",\"VSolar\":\"%.2f\"", weather.vsolar);
            strcat(strData, strtmp);
        }
    }

    // if (config.wx_flage & WX_SNOW)
    // {
    //     if (weather.visable & WX_SNOW)
    //     {
    //         sprintf(strtmp, "S%03u", (unsigned int)(weather.snow));
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_SOIL_TEMP)
    // {
    //     if (weather.visable & WX_SOIL_TEMP)
    //     {
    //         sprintf(strtmp, "m%03u", (int)round((weather.soil_temp * 9 / 5) + 32));
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_SOIL_MOISTURE)
    // {
    //     if (weather.visable & WX_SOIL_MOISTURE)
    //     {
    //         sprintf(strtmp, "M%03u", (unsigned int)round(weather.soil_moisture * 10));
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_WATER_TEMP)
    // {
    //     if (weather.visable & WX_WATER_TEMP)
    //     {
    //         sprintf(strtmp, "w%03u", (int)round((weather.water_temp * 9 / 5) + 32));
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_WATER_TDS)
    // {
    //     if (weather.visable & WX_WATER_TDS)
    //     {
    //         sprintf(strtmp, "W%03u", (int)weather.water_tds);
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_WATER_LEVEL)
    // {
    //     if (weather.visable & WX_WATER_LEVEL)
    //     {
    //         sprintf(strtmp, "v%03u", (int)weather.water_level);
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_PM25)
    // {

    //     if (weather.visable & WX_PM25)
    //     {
    //         sprintf(strtmp, "d%03u", (int)weather.pm25);
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_PM100)
    // {
    //     if (weather.visable & WX_PM100)
    //     {
    //         sprintf(strtmp, "D%03u", (int)weather.pm100);
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_CO2)
    // {
    //     if (weather.visable & WX_CO2)
    //     {
    //         if (weather.co2 < 10000)
    //             sprintf(strtmp, "x%04u", (unsigned int)weather.co2);
    //         else
    //             sprintf(strtmp, "X%04u", (unsigned int)weather.co2/10);
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_CH2O)
    // {
    //     if (weather.visable & WX_CH2O)
    //     {
    //         sprintf(strtmp, "F%04u", (unsigned int)weather.ch2o);
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_TVOC)
    // {
    //     if (weather.visable & WX_TVOC)
    //     {
    //         sprintf(strtmp, "o%04u", (unsigned int)weather.tvoc);
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_UV)
    // {
    //     if (weather.visable & WX_UV)
    //     {
    //         sprintf(strtmp, "u%02u", (unsigned int)weather.uv);
    //         strcat(strData, strtmp);
    //     }
    // }

    // if (config.wx_flage & WX_SOUND)
    // {
    //     if (weather.visable & WX_SOUND)
    //     {
    //         sprintf(strtmp, "n%03u", (unsigned int)round(weather.sound));
    //         strcat(strData, strtmp);
    //     }
    // }

    strcat(strData,"}");
    i = strlen(strData);
    return i;
}