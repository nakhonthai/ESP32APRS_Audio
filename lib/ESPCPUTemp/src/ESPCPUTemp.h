#ifndef ESP_CPU_TEMP_H
#define ESP_CPU_TEMP_H

#include <Arduino.h>

#if defined(ESP_IDF_VERSION)
  #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || \
      defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || \
      defined(CONFIG_IDF_TARGET_ESP32H2)
    #if __has_include(<driver/temperature_sensor.h>)
      #include <driver/temperature_sensor.h>
      #define NEW_DRIVER_AVAILABLE
    #endif
  #else
    #if __has_include(<driver/temp_sensor.h>)
      #include <driver/temp_sensor.h>
      #if defined(CONFIG_TEMP_SENSOR_ENABLE)
        #define LEGACY_DRIVER_AVAILABLE
      #endif
    #endif
  #endif
#else
  #if __has_include(<driver/temp_sensor.h>)
    #include <driver/temp_sensor.h>
    #define LEGACY_DRIVER_AVAILABLE
  #endif
#endif

class ESPCPUTemp {
public:
    ESPCPUTemp();
    ~ESPCPUTemp();

    bool begin();

    bool tempAvailable() const;

    float getTemp();

private:
    String chip_model;
    bool sensor_available;

#ifdef NEW_DRIVER_AVAILABLE
    temperature_sensor_handle_t temp_sensor_handle;
#endif

    bool initTempSensorLegacy();

    bool initTempSensorNew();

    bool readTempLegacy(float &result);

    bool readTempNew(float &result);
};

#endif // ESP_CPU_TEMP_H
