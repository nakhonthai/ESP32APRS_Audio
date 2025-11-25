# ESPCPUTemp Arduino Library

![GitHub release (latest by date)](https://img.shields.io/github/v/release/PelicanHu/ESPCPUTemp?style=flat-square)
![GitHub license](https://img.shields.io/github/license/PelicanHu/ESPCPUTemp?style=flat-square)

The **ESPCPUTemp** library provides a simple and unified interface to read the internal CPU temperature sensor on Espressif ESP32-based microcontrollers. It supports both legacy and new temperature sensor drivers, ensuring compatibility across various ESP32 variants, including ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6, and ESP32-H2.

## Features
- **Unified Interface**: Seamlessly supports both legacy (ESP32) and new (ESP32-S2/S3/C3/C6/H2) temperature sensor drivers.
- **Automatic Chip Detection**: Automatically detects the ESP32 chip model and selects the appropriate driver.
- **Error Handling**: Robust error handling with detailed Serial output for debugging.
- **Lightweight**: Minimal memory footprint, optimized for Arduino environments.
- **Cross-Platform**: Compatible with multiple ESP32 variants using the Arduino core.

## Supported Devices
| Chip Model       | Driver Type |  Sensor Status   |
|------------------|-------------|-----------|
| ESP32-D0WD       | Legacy      | not found |
| ESP32-D0WDQ6     | Legacy      | not found |
| ESP32-D0WD-V3    | Legacy      | not found |
| ESP32-S2         | New         | working |
| ESP32-S3         | New         | working |
| ESP32-C3         | New         | working |
| ESP32-C6         | New         | working |
| ESP32-H2         | New         | working |

## Installation

### Using Arduino Library Manager
1. Open the Arduino IDE.
2. Go to **Sketch > Include Library > Manage Libraries**.
3. Search for `ESPCPUTemp`.
4. Click **Install** to download and install the library.

### Manual Installation
1. Download the latest release from the [GitHub Releases page](https://github.com/PelicanHu/ESPCPUTemp/releases).
2. Extract the `.zip` file.
3. Move the `ESPCPUTemp` folder to your Arduino libraries directory:
   - **Windows**: `C:\Users\<YourUsername>\Documents\Arduino\libraries`
   - **MacOS/Linux**: `~/Documents/Arduino/libraries`
4. Restart the Arduino IDE.

## Dependencies
- **Arduino core for ESP32**: Version 2.0.0 or higher (recommended: 3.2.0 or later).
  - Install via the Arduino Boards Manager by searching for `esp32` and selecting the Espressif Systems package.
- No additional external libraries are required.

## Usage

### Basic Example
The following example demonstrates how to initialize the temperature sensor and read the CPU temperature:

```cpp
#include <ESPCPUTemp.h>

ESPCPUTemp tempSensor;

void setup() {
  Serial.begin(115200);
  delay(100);

  if (tempSensor.begin()) {
    Serial.println("Temperature sensor initialized successfully");
  } else {
    Serial.println("Failed to initialize temperature sensor");
  }
}

void loop() {
  if (tempSensor.tempAvailable()) {
    float temp = tempSensor.getTemp();
    if (!isnan(temp)) {
      Serial.print("CPU Temperature: ");
      Serial.print(temp);
      Serial.println(" °C");
    } else {
      Serial.println("Failed to read temperature");
    }
  }
  delay(5000); // Read every 5 seconds
}
```

### Example Output

```
Temperature sensor initialized successfully
CPU Temperature: 45.2 °C
CPU Temperature: 46.1 °C
```

### API Reference

- **Constructor**: `ESPCPUTemp()`
  - Initializes the temperature sensor object.

- **Destructor**: `~ESPCPUTemp()`
  - Cleans up resources and disables the sensor.

- **begin()**: `bool begin()`
  - Initializes the temperature sensor based on the detected chip model.
  - Returns `true` if successful, `false` otherwise.

- **tempAvailable()**: `bool tempAvailable() const`
  - Checks if the temperature sensor is available.
  - Returns `true` if the sensor is initialized, `false` otherwise.

- **getTemp()**: `float getTemp()`
  - Reads the current CPU temperature in °C.
  - Returns the temperature as a `float` or `NAN` if the reading fails.

### Error Handling

The library provides detailed Serial output for debugging, including:

- Chip model detection.
- Driver availability (legacy or new).
- Initialization errors with ESP error codes.
- Read errors during temperature measurement.

## Configuration

The library uses preprocessor directives to enable/disable driver support:

- `NEW_DRIVER_AVAILABLE`: Defined for ESP32-S2, S3, C3, C6, and H2 chips when the new driver is available (requires `<driver/temperature_sensor.h>`).
- `LEGACY_DRIVER_AVAILABLE`: Defined for ESP32 variants when the legacy driver is available (requires `<driver/temp_sensor.h>`).

These directives are automatically handled by the Arduino core based on the selected board.

## Notes

- **Temperature Range**: The sensor is configured for -10°C to 80°C. Modify the `range_min` and `range_max` values in `initTempSensorNew()` or `dac_offset` in `initTempSensorLegacy()` if a different range is needed.
- **Accuracy**: The internal temperature sensor provides approximate values. For precise measurements, consider an external sensor.
- **Arduino Core Compatibility**: Ensure you are using a recent version of the ESP32 Arduino core (e.g., 3.2.0 or later) for full compatibility with newer chips like ESP32-C6 and H2.

## Troubleshooting

- **"Failed to initialize temperature sensor"**:
  - Check if the correct board is selected in the Arduino IDE (e.g., "ESP32 Dev Module" for ESP32, "ESP32C3 Dev Module" for ESP32-C3).
  - Ensure the Arduino core is up-to-date.
- **"Legacy/New driver not available"**:
  - Verify that the required driver headers (`<driver/temp_sensor.h>` or `<driver/temperature_sensor.h>`) are available in your Arduino core.
- **"Unsupported chip model"**:
  - The library only supports ESP32-based chips. Check the chip model using `ESP.getChipModel()`.

## Contributing

Contributions are welcome! To contribute:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature/your-feature`).
3. Make your changes and commit (`git commit -m "Add your feature"`).
4. Push to the branch (`git push origin feature/your-feature`).
5. Open a Pull Request.

Please ensure your code follows the existing style and includes appropriate documentation.

## License

This library is licensed under the [MIT License](LICENSE). See the [LICENSE](LICENSE) file for details.

## Contact

For questions or support, please open an issue on the [GitHub repository](https://github.com/PelicanHu/ESPCPUTemp/issues).

