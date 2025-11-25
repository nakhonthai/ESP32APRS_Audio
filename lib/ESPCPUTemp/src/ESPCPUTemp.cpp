#include "ESPCPUTemp.h"

ESPCPUTemp::ESPCPUTemp() : sensor_available(false) {
#ifdef NEW_DRIVER_AVAILABLE
    temp_sensor_handle = NULL;
#endif
}

ESPCPUTemp::~ESPCPUTemp() {
#ifdef NEW_DRIVER_AVAILABLE
    if (temp_sensor_handle != NULL) {
        temperature_sensor_disable(temp_sensor_handle);
        temperature_sensor_uninstall(temp_sensor_handle);
        temp_sensor_handle = NULL;
    }
#endif
}

bool ESPCPUTemp::initTempSensorLegacy() {
#ifdef LEGACY_DRIVER_AVAILABLE
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L2; // -10°C ~ 80°C
    esp_err_t ret = temp_sensor_set_config(temp_sensor);
    if (ret != ESP_OK) {
        Serial.printf("Failed to configure legacy temperature sensor (err: %d)\n", ret);
        return false;
    }
    ret = temp_sensor_start();
    if (ret != ESP_OK) {
        Serial.printf("Failed to start legacy temperature sensor (err: %d)\n", ret);
        return false;
    }
    Serial.println("Legacy temperature sensor initialized");
    return true;
#else
    Serial.println("Legacy driver not available in this Arduino core");
    return false;
#endif
}

bool ESPCPUTemp::initTempSensorNew() {
#ifdef NEW_DRIVER_AVAILABLE
    temperature_sensor_config_t temp_sensor = {
        .range_min = -10, // -10°C
        .range_max = 80,  // 80°C
    };
    esp_err_t ret = temperature_sensor_install(&temp_sensor, &temp_sensor_handle);
    if (ret != ESP_OK) {
        Serial.printf("Failed to install new temperature sensor (err: %d)\n", ret);
        return false;
    }
    ret = temperature_sensor_enable(temp_sensor_handle);
    if (ret != ESP_OK) {
        Serial.printf("Failed to enable new temperature sensor (err: %d)\n", ret);
        return false;
    }
    Serial.println("New temperature sensor initialized");
    return true;
#else
    Serial.println("New driver not available in this Arduino core");
    return false;
#endif
}

bool ESPCPUTemp::begin() {
    chip_model = ESP.getChipModel();
//    Serial.print("Detected chip model: ");
//    Serial.println(chip_model);

    sensor_available = false;

    // ESP32 variánsok (D0WD, D0WDQ6, V3, stb.): legacy driver
    if (chip_model.startsWith("ESP32") && chip_model != "ESP32-S2" && chip_model != "ESP32-S3" &&
        chip_model != "ESP32-C3" && chip_model != "ESP32-C6" && chip_model != "ESP32-H2") {
        #ifdef LEGACY_DRIVER_AVAILABLE
        sensor_available = initTempSensorLegacy();
        #else
        Serial.println("Legacy driver not available for ESP32 variants");
        #endif
    }
    // ESP32-S2, S3, C3, C6, H2: új driver
    else if (chip_model == "ESP32-S2" || chip_model == "ESP32-S3" || chip_model == "ESP32-C3" ||
             chip_model == "ESP32-C6" || chip_model == "ESP32-H2") {
        #ifdef NEW_DRIVER_AVAILABLE
        sensor_available = initTempSensorNew();
        #else
        Serial.println("New driver not available for ESP32-S2/S3/C3/C6/H2");
        #endif
    }
    else {
        Serial.println("Unsupported chip model for temperature sensor");
    }

    if (!sensor_available) {
        Serial.println("ESP CPU temperature sensor initialization failed");
    }
    return sensor_available;
}

bool ESPCPUTemp::tempAvailable() const {
    return sensor_available;
}

bool ESPCPUTemp::readTempLegacy(float &result) {
#ifdef LEGACY_DRIVER_AVAILABLE
    esp_err_t ret = temp_sensor_read_celsius(&result);
    if (ret != ESP_OK) {
        Serial.printf("Failed to read temperature (legacy, err: %d)\n", ret);
        return false;
    }
    return true;
#else
    Serial.println("Legacy driver not available");
    return false;
#endif
}

bool ESPCPUTemp::readTempNew(float &result) {
#ifdef NEW_DRIVER_AVAILABLE
    esp_err_t ret = temperature_sensor_get_celsius(temp_sensor_handle, &result);
    if (ret != ESP_OK) {
        Serial.printf("Failed to read temperature (new, err: %d)\n", ret);
        return false;
    }
    return true;
#else
    Serial.println("New driver not available");
    return false;
#endif
}

float ESPCPUTemp::getTemp() {
    if (!sensor_available) {
        Serial.println("Temperature sensor not available");
        return NAN;
    }

    float result = 0;
    bool success = false;

    if (chip_model == "ESP32-S2" || chip_model == "ESP32-S3" || chip_model == "ESP32-C3" ||
        chip_model == "ESP32-C6" || chip_model == "ESP32-H2") {
        success = readTempNew(result);
    } else if (chip_model.startsWith("ESP32")) {
        success = readTempLegacy(result);
    }

    return success ? result : NAN;
}
