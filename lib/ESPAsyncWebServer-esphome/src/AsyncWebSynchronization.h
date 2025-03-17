#ifndef ASYNCWEBSYNCHRONIZATION_H_
#define ASYNCWEBSYNCHRONIZATION_H_

// Synchronisation is only available on ESP32, as the ESP8266 isn't using FreeRTOS by default

#include <ESPAsyncWebServer.h>

#if defined(ESP32) || (defined(LIBRETINY) && LT_HAS_FREERTOS)
//#include "esp_private/freertos_debug.h"

// This is the ESP32 version of the Sync Lock, using the FreeRTOS Semaphore
class AsyncWebLock
{
private:
  SemaphoreHandle_t _lock;
  mutable void *_lockedBy;

public:
  AsyncWebLock() {
    _lock = xSemaphoreCreateBinary();
    _lockedBy = NULL;
    xSemaphoreGive(_lock);
  }

  ~AsyncWebLock() {
    vSemaphoreDelete(_lock);
  }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
  bool lock() const {
  TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
  if (_lockedBy != currentTask) {
      xSemaphoreTake(_lock, portMAX_DELAY);
      _lockedBy = currentTask;
      return true;
  }
  return false;
}
#else
    bool lock() const {
    extern void *pxCurrentTCB;
    if (_lockedBy != pxCurrentTCB) {
      xSemaphoreTake(_lock, portMAX_DELAY);
      _lockedBy = pxCurrentTCB;
      return true;
    }
    return false;
  }
#endif

  void unlock() const {
    _lockedBy = NULL;
    xSemaphoreGive(_lock);
  }
};

#else

// This is the 8266 version of the Sync Lock which is currently unimplemented
class AsyncWebLock
{

public:
  AsyncWebLock() {
  }

  ~AsyncWebLock() {
  }

  bool lock() const {
    return false;
  }

  void unlock() const {
  }
};
#endif

class AsyncWebLockGuard
{
private:
  const AsyncWebLock *_lock;

public:
  AsyncWebLockGuard(const AsyncWebLock &l) {
    if (l.lock()) {
      _lock = &l;
    } else {
      _lock = NULL;
    }
  }

  ~AsyncWebLockGuard() {
    if (_lock) {
      _lock->unlock();
    }
  }
};

#endif // ASYNCWEBSYNCHRONIZATION_H_
