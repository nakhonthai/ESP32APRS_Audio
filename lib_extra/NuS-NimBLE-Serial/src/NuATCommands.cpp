/**
 * @file NuATCommands.cpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2024-08-21
 * @brief AT command processor using the Nordic UART Service
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#include "NuATCommands.hpp"

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

NuATCommandProcessor &NuATCommands = NuATCommandProcessor::getInstance();

//-----------------------------------------------------------------------------
// NordicUARTService implementation
//-----------------------------------------------------------------------------

void NuATCommandProcessor::onWrite(
    NimBLECharacteristic *pCharacteristic,
    NimBLEConnInfo &connInfo)
{
    // Incoming data
    NimBLEAttValue incomingPacket = pCharacteristic->getValue();
    const char *in = incomingPacket.c_str();
    if ((uMaxCommandLineLength > 0) &&
        (incomingPacket.size() > uMaxCommandLineLength))
    {
        printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
        notifyError("", NuATSyntaxError_t::AT_ERR_TOO_LONG);
    }
    else
        execute((const uint8_t *)in, incomingPacket.size());
}

//-----------------------------------------------------------------------------
// Printing
//-----------------------------------------------------------------------------

void NuATCommandProcessor::printATResponse(std::string message)
{
    print("\r\n");
    print(message);
    print("\r\n");
}

//-----------------------------------------------------------------------------
// Other
//-----------------------------------------------------------------------------

uint32_t NuATCommandProcessor::maxCommandLineLength(uint32_t value)
{
    uint32_t result = uMaxCommandLineLength;
    uMaxCommandLineLength = value;
    return result;
}