/**
 * @file NuS.hpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-18
 * @brief Nordic UART Service implementation on NimBLE stack
 *
 * @note NimBLE-Arduino library is required.
 *       https://github.com/h2zero/NimBLE-Arduino
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */
#ifndef __NUS_NIMBLE_HPP__
#define __NUS_NIMBLE_HPP__

#include <NimBLEServer.h>
#include <NimBLEService.h>
#include <NimBLECharacteristic.h>
#include <cstring>

#if __cplusplus < 202002L
#include "cyan_semaphore.h"
using namespace cyan;
#else
#include <semaphore>
using namespace std;
#endif

/**
 * @brief UUID for the Nordic UART Service
 *
 * @note You may need this to handle advertising on your own
 */
#define NORDIC_UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

/**
 * @brief Nordic UART Service (NuS) implementation using the NimBLE stack
 *
 * @note This is an abstract class.
 *       Override NimBLECharacteristicCallbacks::onWrite()
 *       to process incoming data. A singleton pattern is suggested.
 */
class NordicUARTService : protected NimBLECharacteristicCallbacks
{
public:
  /**
   * @brief When true, allow multiple instances of the Nordic UART Service
   *
   * @note False by default.
   *       When false, your application can not start two or more instances
   *       of the Nordic UART service.
   *       This is what most client applications expect.
   *
   */
  static bool allowMultipleInstances;

  /**
   * @brief Check if a peer is connected and subscribed to this service
   *
   * @return true When a connection is established and
   *              the peer is subscribed to the Nordic UART TX characteristic
   * @return false When no peer is connected or a peer is connected but not subscribed yet.
   *               Call `NimBLEServer::getConnectedCount()` to know the case.
   */
  bool isConnected();

  /**
   * @brief Get the count of clients subscribed
   *
   * @return uint8_t Number of clients subscribed to the service
   */
  size_t subscriberCount() { return _subscriberCount; };

  /**
   * @brief Wait for a peer connection or a timeout if set (blocking)
   *
   * @param[in] timeoutMillis Maximum time to wait (in milliseconds) or
   *                          zero to disable timeouts and wait forever
   *
   * @note It is not mandatory to call this method in order to read or write.
   *
   * @note Just one task can go beyond connect(), except in case of timeout,
   *       if more than one exists.
   *
   * @return true on peer connection and service subscription
   * @return false on timeout
   */
  bool connect(const unsigned int timeoutMillis = 0);

  /**
   * @brief Terminate all peer connections (if any),
   *        subscribed or not.
   *
   */
  void disconnect(void);

  /**
   * @brief Send bytes
   *
   * @param[in] data Pointer to bytes to be sent.
   * @param[in] size Count of bytes to be sent.
   * @return size_t @p size .
   */
  size_t write(const uint8_t *data, size_t size);

  /**
   * @brief Send a null-terminated string (ANSI encoded)
   *
   * @param[in] str Pointer to null-terminated string to be sent.
   * @param[in] includeNullTerminatingChar When true, the null terminating character is sent too.
   *            When false, such a character is not sent, so @p str should end with another
   *            termination token, like CR (Unix), LF (old MacOS) or CR+LF (Windows).
   *
   * @return size_t Count of bytes sent.
   */
  size_t send(const char *str, bool includeNullTerminatingChar = false);

  /**
   * @brief Send a string (any encoding)
   *
   * @param str String to send
   * @return size_t Count of bytes sent.
   */
  size_t print(std::string str)
  {
    return write((const uint8_t *)str.data(), str.length());
  };

  /**
   * @brief Send a formatted string (ANSI encoded)
   *
   * @note The null terminating character is sent too.
   *
   * @param[in] format String that follows the same specifications as format in printf()
   * @param[in] ... Depending on the format string, a sequence of additional arguments,
   *            each containing a value to replace a format specifier in the format string.
   *
   * @return size_t Count of bytes sent.
   */
  size_t printf(const char *format, ...);

  /**
   * @brief Start the Nordic UART Service
   *
   * @note NimBLEDevice::init() **must** be called in advance.
   * @note The service is unavailable if start() is not called.
   *       Do not call start() before BLE initialization is complete in your application.
   *
   * @param autoAdvertising True to automatically handle BLE advertising.
   *                        When false, you have to handle advertising on your own.
   *
   * @throws std::runtime_error if the UART service is already created or can not be created
   */
  void start(bool autoAdvertising = true);
  void start_uuid(const char *uuid,const char *uuid_tx,const char *uuid_rx,bool autoAdvertising = true);

  /**
   * @brief Stop and remove the Nordic UART Service
   *
   * @note Advertising will be temporarily disabled.
   *       If you handle advertising on your own,
   *       you should stop it first and then restart it with your own parameters.
   *       Otherwise, this function will restart advertising with default parameters.
   *
   * @warning This will terminate all open connections,
   *          including peers not using the Nordic UART Service.
   *          This funcion is discouraged.
   *          Design your application so that it is not necessary to stop the service.
   *
   * @warning This method is not thread-safe
   */
  void stop();

  /**
   * @brief Check if the Nordic UART Service is started
   *
   * @return true If started
   * @return false If not started
   */
  bool isStarted();

protected:
  virtual void onSubscribe(
      NimBLECharacteristic *pCharacteristic,
      NimBLEConnInfo &connInfo,
      uint16_t subValue) override;

protected:
  /**
   * @brief Event callback for client subscription to the TX characteristic
   *
   * @note Called before the semaphore is released.
   *
   * @param subscriberCount Number of subscribed clients
   */
  virtual void onSubscribe(size_t subscriberCount) {};

  /**
   * @brief Event callback for client unsubscription to the TX characteristic
   *
   * @param subscriberCount Number of subscribed clients
   */
  virtual void onUnsubscribe(size_t subscriberCount) {};

protected:
  NordicUARTService() {};
  virtual ~NordicUARTService() {};

private:
  NimBLEService *pNus = nullptr;
  NimBLECharacteristic *pTxCharacteristic = nullptr;
  binary_semaphore peerConnected{0};
  uint32_t _subscriberCount = 0;

  /**
   * @brief Create the NuS service in a new or existing GATT server
   *
   * @param advertise True to add the NuS UUID to the advertised data, false otherwise.
   *
   * @return NimBLEService internal instance of the NuS
   * @throws std::runtime_error if the UART service can not be created
   */

  void init(bool advertise);
  void init_uuid(const char *uuid, const char *uuid_tx, const char *uuid_rx, bool advertise);

  /**
   * @brief Remove the NuS service
   *
   * @note Advertising will be temporarily disabled
   *
   * @warning The service will not be removed until all open connections are closed.
   *          In the meantime the service will have it's visibility disabled.
   *
   * @warning This method is not thread-safe
   */
  void deinit();
};

#endif