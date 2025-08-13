# Nordic UART Service (NuS) and BLE serial communications (NimBLE stack)

Library for serial communications through Bluetooth Low Energy on ESP32-Arduino boards

In summary, this library provides:

- A BLE serial communications object that can be used as Arduino's [Serial](https://www.arduino.cc/reference/en/language/functions/communication/serial/).
- A BLE serial communications object that can handle incoming data in packets, eluding active waiting thanks to blocking semantics.
- A customizable and easy to use [AT command](https://www.twilio.com/docs/iot/supersim/introduction-to-modem-at-commands) processor based on NuS.
- A customizable [shell](https://en.wikipedia.org/wiki/Shell_(computing)) command processor based on NuS.
- A generic class to implement custom protocols for serial communications through BLE.

## Supported DevKit boards

Any DevKit supported by [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino).

> [!NOTE]
> Since version 3.3.0, *FreeRTOS* is no longer required.

## Installing and upgrading to a newer version

The Arduino IDE should list this library in all available versions,
but sometimes the *library indexer* **fails to catch updates**.
In this  case, download the ZIP file from the
[releases section](https://github.com/afpineda/NuS-NimBLE-Serial/releases)
or the `CODE` drop-down button found on this GitHub page (see above).
Then, import the ZIP file into the Arduino IDE or install manually.
For instructions, see the
[official guide](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries/).

## Introduction

Serial communications are already available through the old [Bluetooth classic](https://www.argenox.com/library/bluetooth-classic/introduction-to-bluetooth-classic/) specification (see [this tutorial](https://circuitdigest.com/microcontroller-projects/using-classic-bluetooth-in-esp32-and-toogle-an-led)), [Serial Port Profile (SPP)](https://www.bluetooth.com/specifications/specs/serial-port-profile-1-2/).
However, this is not the case with the [Bluetooth Low Energy (BLE) specification](https://en.wikipedia.org/wiki/Bluetooth_Low_Energy).
**No standard** protocol was defined for serial communications in BLE (see [this article](https://punchthrough.com/serial-over-ble/) for further information).

As bluetooth classic is being dropped in favor of BLE, an alternative is needed. [Nordic UART Service (NuS)](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth_services/services/nus.html) is a popular alternative, if not the *de facto* standard.
This library implements the Nordic UART service on the *NimBLE-Arduino* stack.

## Client-side application

You may need a generic terminal (PC or smartphone) application in order to communicate with your Arduino application through BLE. Such a generic application must support the Nordic UART Service. There are several free alternatives (known to me):

- Android:
  - [nRF connect for mobile](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp)
  - [Serial bluetooth terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal)
- iOS:
  - [nRF connect for mobile](https://apps.apple.com/es/app/nrf-connect-for-mobile/id1054362403)
- Multi-platform:
  - [NeutralNUS](https://github.com/KevinJohnMulligan/neutral-nus-terminal/releases)

> [!NOTE]
> In Android, you have to enable both bluetooth and geolocalization,
> otherwise, your device will not be discovered.

## How to use this library

Summary:

- The `NuSerial` object provides non-blocking serial communications through BLE, *Arduino's style*.
- The `NuPacket` object provides blocking serial communications through BLE.
- The `NuATCommands` object provides custom processing of AT commands through BLE.
- The `NuShellCommands` object provides custom processing of shell commands through BLE.
- Create your own object to provide a custom protocol based on serial communications through BLE, by deriving a new class from `NordicUARTService`.

The **basic rules** are:

- You must initialize the *NimBLE stack* **before** using this library.
  See [NimBLEDevice::init()](https://h2zero.github.io/NimBLE-Arduino/class_nim_b_l_e_device.html).

> [!TIP]
> Due to changes in *NimBLE-Arduino* version 2.1.0+
> you may need to manually add the device name to the advertised data:
>
> `NimBLEDevice::getAdvertising()->setName(DEVICE_NAME);`

- You must also call `<object>.start()` **after** all BLE initialization is complete.
- By default, just one object can use the Nordic UART Service.
  For example, this code **fails** at run time:

  ```c++
  void setup() {
    ...
    NuSerial.start();
    NuPacket.start(); // raises an exception (runtime_error)
  }
  ```

  Most client applications expect a single Nordic UART service in your device.
  However, at your own risk, you can start multiple objects by setting
  the static field `NordicUARTService::allowMultipleInstances` to `true` before
  calling `<object>.start()`.

- The Nordic UART Service can coexist with other GATT services in your application.
  This library does not require specific code for this.
  Just ignore the fact that *NuS-NimBLE-Serial* is there and
  register other services with *NimBLE-Arduino*.

- Since version 3.1.0, `<object>.isConnected()` and `<object>.connect()`
  refer to devices connected **and subscribed** to the NuS transmission characteristic.
  If you have other services,
  a client may be connected but not using the Nordic UART Service.
  In this case, `<object>.isConnected()` will return `false`
  but [NimBLEServer::getConnectedCount()](https://h2zero.github.io/NimBLE-Arduino/class_nim_b_l_e_server.html#a98ea12f57c10c0477b0c1c5efab23ee5)
  will return `1`.

- By default, this library will automatically advertise existing GATT services when no peer is connected.
  This includes the Nordic UART Service and other
  services you configured for advertising (if any).
  To change this behavior, call `<object>.start(false)` instead of `<object>.start()`
  and handle advertising on your own.
  To disable automatic advertising once NuS is started,
  call `NimBLEDevice::getServer()->advertiseOnDisconnect(false)` and
  remove the service UUID (constant `NORDIC_UART_SERVICE_UUID`) from the advertised data (if required).

- You can stop the service by calling `<object>.stop()`.
  However, **this is discouraged** as there are **side effects**:
  all peer connections will be closed,
  advertising needs to be restarted and
  there is no thread safety.
  Design your application in a way that *NuS* does not need to be stopped.

You may learn from the provided [examples](./examples/README.md).
Read the [API documentation](https://afpineda.github.io/NuS-NimBLE-Serial/) for more information.

### Non-blocking serial communications

```c++
#include "NuSerial.hpp"
```

In short, use the `NuSerial` object as you do with the Arduino's `Serial` object. For example:

```c++
void setup()
{
    ...
    NimBLEDevice::init("My device");
    ...
    NuSerial.begin(115200); // Note: parameter is ignored
}

void loop()
{
    if (NuSerial.available())
    {
        // read incoming data and do something
        ...
    } else {
        // other background processing
        ...
    }
}
```

Take into account:

- `NuSerial` inherits from Arduino's `Stream`, so you can use it with other libraries.
- As you should know, `read()` will immediately return if there is no data available.
  But, this is also the case when no peer device is connected.
  Use `NuSerial.isConnected()` to know the case (if you need to).
- `NuSerial.begin()` or `NuSerial.start()` must be called at least once before reading. Calling more than once have no effect.
- `NuSerial.end()` (as well as `NuSerial.disconnect()`) will terminate any peer connection.
  If you pretend to read again, it's not mandatory to call `NuSerial.begin()` (nor `NuSerial.start()`) again, but you can.
- As a bonus, `NuSerial.readBytes()` does not perform active waiting, unlike `Serial.readBytes()`.
- As you should know, `Stream` read methods are not thread-safe. Do not read from two different OS tasks.

### Blocking serial communications

```c++
#include "NuPacket.hpp"
```

Use the `NuPacket` object, based on blocking semantics. The advantages are:

- Efficiency in terms of CPU usage, since no active waiting is used.
- Performance, since incoming bytes are processed in packets, not one by one.
- Simplicity. Only two methods are strictly needed: `read()` and `write()`.
  You don't need to worry about data being available or not.
  However, you have to handle packet size.

For example:

```c++
void setup()
{
    ...
    NimBLEDevice::init("My device");
    ... // other initialization
    NuPacket.start(); // don't forget this!!
}

void loop()
{
    size_t size;
    const uint8_t *data = NuPacket.read(size); // "size" is an output parameter
    while (data)
    {
        // do something with data and size
        ...
        data = NuPacket.read(size);
    }
    // No peer connection at this point
}
```

Take into account:

- **Just one** OS task can work with `NuPacket` (others will get blocked).
- Data should be processed as soon as possible. Use other tasks and buffers/queues for time-consuming computation.
  While data is being processed, the peer will stay blocked, unable to send another packet.
- If you just pretend to read a known-sized burst of bytes, `NuSerial.readBytes()` do the job with the same benefits as `NuPacket`
  and there is no need to manage packet sizes. Call `NuSerial.setTimeout(ULONG_MAX)` previously to get the blocking semantics.

### Custom AT commands

```c++
#include "NuATCommands.hpp"
```

**This API is new to version 3.x**.
To keep **old** code working, use the following header instead:

```c++
#include "NuATCommandsLegacy2.hpp"
using namespace NuSLegacy2;
```

- Call `NuATCommands.allowLowerCase()` and/or `NuATCommands.stopOnFirstFailure()` to your convenience.
- Call `NuATCommands.on*()` to provide a command name and the callback to be executed if such a command is found.
  - `onExecute()`: commands with no suffix.
  - `onSet()`: commands with "=" suffix.
  - `onQuery()`: commands with "?" suffix.
  - `onTest()`: commands with "=?" suffix.
- Call `NuATCommands.onNotACommandLine()` to provide a callback to be executed if non-AT text is received.
- You may chain calls to "`on*()`" methods.
- Call `NuATCommands.start()`

Implementation is based in these sources:

- [Espressif's AT command set](https://docs.espressif.com/projects/esp-at/en/release-v2.2.0.0_esp8266/AT_Command_Set/index.html)
- [An Introduction to AT Commands](https://www.twilio.com/docs/iot/supersim/introduction-to-modem-at-commands)
- [GSM AT Commands Tutorial](https://microcontrollerslab.com/at-commands-tutorial/#Response_of_AT_commands)
- [General Syntax of Extended AT Commands](https://www.developershome.com/sms/atCommandsIntro2.asp)
- [ITU-T recommendation V.250](./doc/T-REC-V.250-200307.pdf)
- [AT command set for User Equipment (UE)](./doc/AT%20commands%20spec.docx)

The following implementation details may be relevant to you:

- ASCII, ANSI, and UTF8 character encodings are accepted,
  but note that AT commands are supposed to work in ASCII.
- Only "extended syntax" is allowed (all commands must have a prefix, either "+" or "&").
  This is non-standard behavior.
- In string parameters (between double quotes), the following rules apply:
  - Write `\\` to insert a single backslash character (`\`). This is standard behavior.
  - Write `\"` to insert a single double quotes character (`"`). This is standard behavior.
  - Write `\<hex>` to insert a non-printable character in the ASCII table,
    where `<hex>` is a **two-digit** hexadecimal number.
    This is standard behavior.
  - The escape character (`\`) is ignored in all other cases. For example, `\a` is the same as `a`.
    This is non-standard behavior.
  - Any non-printable character is allowed without escaping. This is non-standard behavior.
- In non-string parameters (without double quotes),
  a number is expected either in binary, decimal or hexadecimal format.
  No prefixes or suffixes are allowed to denote format. This is standard behavior.
- Text after the line terminator (carriage return), if any, will be parsed as another command line.
  This is non-standard behavior.
- Any text bigger than 256 bytes will be disregarded and handled as a
  syntax error in order to prevent denial of service attacks.
  However, you may disable or adjust this limit to your needs by calling
  `NuATCommands.maxCommandLineLength()`.

As a bonus, you may use class `NuATParser` to implement an AT command processor that takes data from other sources.

### Custom shell commands

```c++
#include "NuShellCommands.hpp"

void setup()
{
  NuShellCommands
    .on("cmd1", [](NuCommandLine_t &commandLine)
    {
      // Note: commandLine[0] == "cmd1"
      //       commandLine[1] is the first argument an so on
      ...
    }
    )
    .on("cmd2", [](NuCommandLine_t &commandLine)
    {
      ...
    }
    .onUnknown([](NuCommandLine_t &commandLine)
    {
      Serial.printf("ERROR: unknown command \"%s\"\n",commandLine[0].c_str());
    }
    )
    .onParseError([](NuCLIParsingResult_t result, size_t index)
    {
      if (result == CLI_PR_ILL_FORMED_STRING)
        Serial.printf("Syntax error at character index %d\n",index);
    }
    )
    .start();
}
```

- Call `NuShellCommands.caseSensitive()` to your convenience. By default, command names are not case-sensitive.
- Call `on()` to provide a command name and the callback to be executed if such a command is found.
- Call `onUnknown()` to provide a callback to be executed if the command line does not contain any command name.
- Call `onParseError()` to provide a callback to be executed in case of error.
- You can chain calls to "`on*`" methods.
- Call `NuShellCommands.start()`.
- Note that all callbacks will be executed at the NimBLE OS task, so make them thread-safe.

Command line syntax:

- Blank spaces, LF and CR characters are separators.
- Command arguments are separated by one or more consecutive separators. For example, the command line `cmd   arg1  arg2 arg3\n` is parsed as the command "cmd" with three arguments: "arg1", "arg2" and "arg3", being `\n` the LF character. `cmd arg1\narg2\n\narg3` would be parsed just the same. Usually, LF and CR characters are command line terminators, so don't worry about them.
- Unquoted arguments can not contain a separator, but can contain double quotes. For example: `this"is"valid`.
- Quoted arguments can contain a separator, but double quotes have to be escaped with another double quote.
  For example: `"this ""is"" valid"` is parsed to `this "is" valid` as a single argument.
- ASCII, ANSI and UTF-8 character encodings are supported. Client software must use the same character encoding as your application.

As a bonus, you may use class `NuCLIParser` to implement a shell that takes data from other sources.

### Custom serial communications protocol

```c++
#include "NuS.hpp"

class MyCustomSerialProtocol: public NordicUARTService {
    public:
        void onWrite(
          NimBLECharacteristic *pCharacteristic,
          NimBLEConnInfo &connInfo) override;
    ...
}
```

Derive a new class and override
[onWrite()](https://h2zero.github.io/NimBLE-Arduino/class_nim_b_l_e_characteristic_callbacks.html).
Then, use `pCharacteristic` to read incoming data. For example:

```c++
void MyCustomSerialProtocol::onWrite(
        NimBLECharacteristic *pCharacteristic,
        NimBLEConnInfo &connInfo)
{
    // Retrieve a pointer to received data and its size
    NimBLEAttValue val = pCharacteristic->getValue();
    const uint8_t *receivedData = val.data();
    size_t receivedDataSize = val.size();

    // Custom processing here
    ...
}
```

In the previous example, the data pointed by `*receivedData` will **not remain valid** after `onWrite()` has finished to execute. If you need that data for later use, you must make a copy of the data itself, not just the pointer. For that purpose, you may store a non-local copy of the `pCharacteristic->getValue()` object.

Since just one object can use the Nordic UART Service, you should also implement a
[singleton pattern](https://www.geeksforgeeks.org/implementation-of-singleton-class-in-cpp/) (not mandatory).

## Licensed work

[cyanhill/semaphore](https://github.com/cyanhill/semaphore) under MIT License.
