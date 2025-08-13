/**
 * @file NuPacket.hpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-18
 * @brief Communications stream based on the Nordic UART Service
 *        with blocking semantics
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#ifndef __NUPACKET_HPP__
#define __NUPACKET_HPP__

#include "NuS.hpp"

/**
 * @brief Blocking serial communications through BLE and Nordic UART Service
 *
 * @note Unlike `Serial`, the semantics
 *       are those of blocking communications. This is more efficient in
 *       terms of CPU usage, since no active waiting is used, and a performance
 *       boost, since incoming bytes are processed in packets, not one bye one.
 *       However, a multi-tasking app design must be adopted.
 *
 */
class NordicUARTPacket : public NordicUARTService
{
public:
    // Singleton pattern

    NordicUARTPacket(const NordicUARTPacket &) = delete;
    void operator=(NordicUARTPacket const &) = delete;

    /**
     * @brief Get the instance of the BLE stream
     *
     * @note No need to use. Use `NuPacket` instead.
     *
     * @return NordicUARTPacket&
     */
    static NordicUARTPacket &getInstance()
    {
        static NordicUARTPacket instance;
        return instance;
    };

protected:
    // Overriden Methods
    virtual void onUnsubscribe(size_t subscriberCount) override;
    void onWrite(
        NimBLECharacteristic *pCharacteristic,
        NimBLEConnInfo &connInfo) override;

public:
    /**
     * @brief Wait for and get incoming data in packets (blocking)
     *
     * @note The calling task will get blocked until incoming data is
     *       available or the connection is lost. Just one task
     *       can go beyond read() if more than one exists.
     *
     * @note You should not perform any time-consuming task between calls.
     *       Use buffers/queues/etc for that. Follow this advice to increase
     *       app responsiveness.
     *
     * @param[out] size Count of incoming bytes,
     *                  or zero if the connection was lost. This is the size of
     *                  the data packet.
     * @return uint8_t* Pointer to incoming data, or `nullptr` if the connection
     *                  was lost.
     *                  Do not access more bytes than available as given in
     *                  @p size. Otherwise, a segmentation fault may occur.
     */
    const uint8_t *read(size_t &size);

private:
    binary_semaphore dataConsumed{1};
    binary_semaphore dataAvailable{0};
    NimBLEAttValue incomingPacket;
    size_t availableByteCount = 0;
    const uint8_t *incomingBuffer = nullptr;
    NordicUARTPacket() {};
    ~NordicUARTPacket() {};
};

/**
 * @brief Singleton instance of the NordicUARTPacket class
 *
 */
extern NordicUARTPacket &NuPacket;

#endif