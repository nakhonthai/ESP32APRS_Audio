/**
 * @file NuS.cpp
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

#include <NimBLEDevice.h>
#include <exception>
#include <stdexcept>
#include <vector>
#include <string.h>
#include <cstdio>
#include <chrono>
#include "NuS.hpp"

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

#define RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

//-----------------------------------------------------------------------------
// Initialization / Deinitialization
//-----------------------------------------------------------------------------

bool NordicUARTService::allowMultipleInstances = false;

void NordicUARTService::init(bool advertise)
{
   // Get the server instance or create one
   NimBLEServer *pServer = NimBLEDevice::getServer();
   if (pServer == nullptr)
      pServer = NimBLEDevice::createServer();
   if (pServer)
   {
      // Add the service UUID to the advertised data
      if (advertise)
         pServer->getAdvertising()->addServiceUUID(NORDIC_UART_SERVICE_UUID);

      // Check if there is another service instance
      if ((!pServer->getServiceByUUID(NORDIC_UART_SERVICE_UUID) || allowMultipleInstances))
      {
         // Create an instance of the service.
         // Note that a server can have many instances of the same service
         // if `allowMultipleInstances` is true.
         pNus = pServer->createService(NORDIC_UART_SERVICE_UUID);
         if (pNus)
         {
            // Create the transmission characteristic
            pTxCharacteristic = pNus->createCharacteristic(TX_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::NOTIFY);
            if (pTxCharacteristic)
            {
               pTxCharacteristic->setCallbacks(this); // uses onSubscribe

               // Create the receive characteristic
               NimBLECharacteristic *pRxCharacteristic =
                   pNus->createCharacteristic(RX_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::WRITE);
               if (pRxCharacteristic)
               {
                  pRxCharacteristic->setCallbacks(this); // uses onWrite
                  return;
               }
            }
         }
      }
   }
   // Unable to initialize the service
   throw std::runtime_error("Unable to create BLE server and/or Nordic UART Service");
}

void NordicUARTService::init_uuid(const char *uuid, const char *uuid_tx, const char *uuid_rx, bool advertise)
{
   // Get the server instance or create one
   NimBLEServer *pServer = NimBLEDevice::getServer();
   if (pServer == nullptr)
      pServer = NimBLEDevice::createServer();
   if (pServer)
   {
      // Add the service UUID to the advertised data
      if (advertise)
         pServer->getAdvertising()->addServiceUUID(uuid);

      // Check if there is another service instance
      if ((!pServer->getServiceByUUID(uuid) || allowMultipleInstances))
      {
         // Create an instance of the service.
         // Note that a server can have many instances of the same service
         // if `allowMultipleInstances` is true.
         pNus = pServer->createService(uuid);
         if (pNus)
         {
            // Create the transmission characteristic
            pTxCharacteristic = pNus->createCharacteristic(uuid_tx, NIMBLE_PROPERTY::NOTIFY);
            if (pTxCharacteristic)
            {
               pTxCharacteristic->setCallbacks(this); // uses onSubscribe

               // Create the receive characteristic
               NimBLECharacteristic *pRxCharacteristic =
                   pNus->createCharacteristic(uuid_rx, NIMBLE_PROPERTY::WRITE);
               if (pRxCharacteristic)
               {
                  pRxCharacteristic->setCallbacks(this); // uses onWrite
                  return;
               }
            }
         }
      }
   }
   // Unable to initialize the service
   throw std::runtime_error("Unable to create BLE server and/or Nordic UART Service");
}

void NordicUARTService::deinit()
{
   NimBLEServer *pServer = pNus->getServer();
   bool wasAdvertising = pServer->getAdvertising()->isAdvertising();
   pServer->removeService(pNus, true);
   // At this point, the pNus pointer is invalid
   pNus = nullptr;
   pTxCharacteristic = nullptr;
   _subscriberCount = 0;
   if (wasAdvertising)
      pServer->startAdvertising();
}

//-----------------------------------------------------------------------------
// Start/Stop service
//-----------------------------------------------------------------------------

void NordicUARTService::start(bool autoAdvertising)
{
   if (!pNus)
   {
      init(autoAdvertising);
      pNus->start();
      if (autoAdvertising)
      {
         pNus->getServer()->advertiseOnDisconnect(true);
         pNus->getServer()->startAdvertising();
      }
   }
}

void NordicUARTService::start_uuid(const char *uuid, const char *uuid_tx, const char *uuid_rx, bool autoAdvertising)
{
   if (!pNus)
   {
      init_uuid(uuid, uuid_tx, uuid_rx, autoAdvertising);
      pNus->start();
      if (autoAdvertising)
      {
         pNus->getServer()->advertiseOnDisconnect(true);
         pNus->getServer()->startAdvertising();
      }
   }
}

void NordicUARTService::stop()
{
   if (pNus)
   {
      disconnect();
      deinit();
   }
}

bool NordicUARTService::isStarted()
{
   return (pNus != nullptr);
}

//-----------------------------------------------------------------------------
// Connection
//-----------------------------------------------------------------------------

bool NordicUARTService::isConnected()
{
   return (_subscriberCount > 0);
}

bool NordicUARTService::connect(const unsigned int timeoutMillis)
{
   if (timeoutMillis == 0)
   {
      peerConnected.acquire();
      return true;
   }
   else
   {
      return peerConnected.try_acquire_for(std::chrono::milliseconds(timeoutMillis));
   }
}

void NordicUARTService::disconnect(void)
{
   NimBLEServer *pServer = NimBLEDevice::getServer();
   if (pServer)
   {
      std::vector<uint16_t> devices = pServer->getPeerDevices();
      for (uint16_t id : devices)
         pServer->disconnect(id);
   }
}

//-----------------------------------------------------------------------------
// TX events
//-----------------------------------------------------------------------------

void NordicUARTService::onSubscribe(
    NimBLECharacteristic *pCharacteristic,
    NimBLEConnInfo &connInfo,
    uint16_t subValue)
{
   // Note: for robustness, we assume this callback could be called
   // even if no subscription event exists.

   if (subValue == 0)
   {
      // unsubscribe
      if (_subscriberCount > 0)
      {
         _subscriberCount--;
         onUnsubscribe(_subscriberCount);
      }
   }
   else if (subValue < 4)
   {
      // subscribe
      _subscriberCount++;
      onSubscribe(_subscriberCount);
      peerConnected.release();
   }
   // else: Invalid subscription value, ignore
}

//-----------------------------------------------------------------------------
// Data transmission
//-----------------------------------------------------------------------------

size_t NordicUARTService::write(const uint8_t *data, size_t size)
{
   if (pTxCharacteristic)
   {
      pTxCharacteristic->notify(data, size);
      return size;
   }
   else
      return 0;
}

size_t NordicUARTService::send(const char *str, bool includeNullTerminatingChar)
{
   if (pTxCharacteristic)
   {
      size_t size = includeNullTerminatingChar ? strlen(str) + 1 : strlen(str);
      pTxCharacteristic->notify((uint8_t *)str, size);
      return size;
   }
   else
      return 0;
}

size_t NordicUARTService::printf(const char *format, ...)
{
   char dummy;
   va_list args;
   va_start(args, format);
   int requiredSize = vsnprintf(&dummy, 1, format, args);
   va_end(args);
   if (requiredSize == 0)
   {
      return write((uint8_t *)&dummy, 1);
   }
   else if (requiredSize > 0)
   {
      char *buffer = (char *)malloc(requiredSize + 1);
      if (buffer)
      {
         va_start(args, format);
         int result = vsnprintf(buffer, requiredSize + 1, format, args);
         va_end(args);
         if ((result >= 0) && (result <= requiredSize))
         {
            size_t writtenBytesCount = write((uint8_t *)buffer, result + 1);
            free(buffer);
            return writtenBytesCount;
         }
         free(buffer);
      }
   }
   return 0;
}