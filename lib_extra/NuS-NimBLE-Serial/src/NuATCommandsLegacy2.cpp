/**
 * @file NuATCommandsLegacy2.cpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-18
 * @brief AT command processor using the Nordic UART Service
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#include <exception>
#include <stdexcept>
#include "NuATCommandsLegacy2.hpp"

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

NuSLegacy2::NuATCommandProcessor &NuSLegacy2::NuATCommands =
    NuSLegacy2::NuATCommandProcessor::getInstance();

//-----------------------------------------------------------------------------
// NordicUARTService implementation
//-----------------------------------------------------------------------------

void NuSLegacy2::NuATCommandProcessor::onWrite(
    NimBLECharacteristic *pCharacteristic,
    NimBLEConnInfo &connInfo)
{
    // Incoming data
    NimBLEAttValue incomingPacket = pCharacteristic->getValue();
    // const char *in = pCharacteristic->getValue().c_str();
    const char *in = incomingPacket.c_str();
    // Serial.printf("onWrite(): %s\n");

    // Parse
    parseCommandLine(in);
}

//-----------------------------------------------------------------------------
// Callbacks
//-----------------------------------------------------------------------------

void NuSLegacy2::NuATCommandProcessor::setATCallbacks(NuATCommandCallbacks *pCallbacks)
{
    if (!isConnected())
        NuATCommandParser::setATCallbacks(pCallbacks);
    else
        throw std::runtime_error("Unable to set AT command callbacks while connected");
}

//-----------------------------------------------------------------------------
// Printing
//-----------------------------------------------------------------------------

void NuSLegacy2::NuATCommandProcessor::printATResponse(const char message[])
{
    send("\r\n");
    send(message);
    send("\r\n");
}
