/**
 * @file NuStream.cpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-24
 * @brief Communications stream based on the Nordic UART Service
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#include "NuStream.hpp"
#include <chrono>

//-----------------------------------------------------------------------------
// GATT server events
//-----------------------------------------------------------------------------

void NordicUARTStream::onUnsubscribe(size_t subscriberCount)
{
    if (subscriberCount == 0)
    {
        // Awake task at readBytes()
        disconnected = true;
        dataAvailable.release();
    }
};

//-----------------------------------------------------------------------------
// NordicUARTService implementation
//-----------------------------------------------------------------------------

void NordicUARTStream::onWrite(
    NimBLECharacteristic *pCharacteristic,
    NimBLEConnInfo &connInfo)
{
    // Wait for previous data to get consumed
    dataConsumed.acquire();

    // Hold data until next read
    incomingPacket = pCharacteristic->getValue();
    unreadByteCount = incomingPacket.size();
    disconnected = false;

    // signal available data
    dataAvailable.release();
}

//-----------------------------------------------------------------------------
// Reading with no active wait
//-----------------------------------------------------------------------------

size_t NordicUARTStream::readBytes(uint8_t *buffer, size_t size)
{
    size_t totalReadCount = 0;
    while (size > 0)
    {
        // copy previously available data, if any
        if (unreadByteCount > 0)
        {
            const uint8_t *incomingData = incomingPacket.data() + incomingPacket.size() - unreadByteCount;
            size_t readBytesCount = (unreadByteCount > size) ? size : unreadByteCount;
            memcpy(buffer, incomingData, readBytesCount);
            buffer = buffer + readBytesCount;
            unreadByteCount = unreadByteCount - readBytesCount;
            totalReadCount = totalReadCount + readBytesCount;
            size = size - readBytesCount;
        } // note: at this point (unreadByteCount == 0) || (size == 0)
        if (unreadByteCount == 0)
        {
            dataConsumed.release();
            // xSemaphoreGive(dataConsumed);
        }
        if (size > 0)
        {
            // wait for more data or timeout or disconnection
            bool waitResult = true;
            if (_timeout == ULONG_MAX)
                dataAvailable.acquire();
            else
                waitResult = dataAvailable.try_acquire_for(std::chrono::milliseconds(_timeout));
            if (!waitResult || disconnected)
                size = 0; // break;
            // Note: at this point, readBuffer and unreadByteCount were updated thanks to onWrite()
        }
    }
    return totalReadCount;
}

//-----------------------------------------------------------------------------
// Stream implementation
//-----------------------------------------------------------------------------

int NordicUARTStream::available()
{
    return unreadByteCount;
}

int NordicUARTStream::peek()
{
    if (unreadByteCount > 0)
    {
        const uint8_t *readBuffer = incomingPacket.data();
        size_t index = incomingPacket.size() - unreadByteCount;
        return readBuffer[index];
    }
    return -1;
}

int NordicUARTStream::read()
{
    if (unreadByteCount > 0)
    {
        const uint8_t *readBuffer = incomingPacket.data();
        size_t index = incomingPacket.size() - unreadByteCount;
        int result = readBuffer[index];
        unreadByteCount--;
        if (unreadByteCount == 0)
            dataConsumed.release();
        return result;
    }
    return -1;
}