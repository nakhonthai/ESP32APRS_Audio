/**
 * @file NuPacket.cpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-18
 * @brief Communications stream based on the Nordic UART Service
 *        with blocking semantics
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#include <exception>
#include <stdexcept>
#include "NuPacket.hpp"

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

NordicUARTPacket &NuPacket = NordicUARTPacket::getInstance();

//-----------------------------------------------------------------------------
// Event callback
//-----------------------------------------------------------------------------

void NordicUARTPacket::onUnsubscribe(size_t subscriberCount)
{
    if (subscriberCount == 0)
    {
        // Awake task at read()
        availableByteCount = 0;
        incomingBuffer = nullptr;
        dataAvailable.release();
    }
};

//-----------------------------------------------------------------------------
// NordicUARTService implementation
//-----------------------------------------------------------------------------

void NordicUARTPacket::onWrite(
    NimBLECharacteristic *pCharacteristic,
    NimBLEConnInfo &connInfo)
{
    // Wait for previous data to get consumed
    dataConsumed.acquire();

    // Hold data until next read
    incomingPacket = pCharacteristic->getValue();
    incomingBuffer = incomingPacket.data();
    availableByteCount = incomingPacket.size();

    // signal available data
    dataAvailable.release();
}

//-----------------------------------------------------------------------------
// Reading
//-----------------------------------------------------------------------------

const uint8_t *NordicUARTPacket::read(size_t &size)
{
    dataConsumed.release();
    dataAvailable.acquire();
    size = availableByteCount;
    return incomingBuffer;
}
