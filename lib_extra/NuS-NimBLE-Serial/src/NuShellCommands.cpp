/**
 * @file NuShellCommands.cpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-27
 * @brief Shell command processor using the Nordic UART Service
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#include "NuShellCommands.hpp"

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

NuShellCommandProcessor &NuShellCommands = NuShellCommandProcessor::getInstance();

//-----------------------------------------------------------------------------
// NordicUARTService implementation
//-----------------------------------------------------------------------------

void NuShellCommandProcessor::onWrite(
    NimBLECharacteristic *pCharacteristic,
    NimBLEConnInfo &connInfo)
{
    // Incoming data
    NimBLEAttValue incomingPacket = pCharacteristic->getValue();

    // Parse and execute
    execute((const uint8_t *)incomingPacket.data(), incomingPacket.size());
}
