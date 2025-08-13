/**
 * @file NuATCommands.hpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2024-08-21
 * @brief AT command processor using the Nordic UART Service
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#ifndef __NU_AT_COMMANDS_HPP__
#define __NU_AT_COMMANDS_HPP__
#ifndef __NU_AT_COMMANDS_LEGACY2_HPP__

#include "NuS.hpp"
#include "NuATParser.hpp"

/**
 * @brief Execute AT commands received thanks to the Nordic UART Service
 *
 */
class NuATCommandProcessor : public NordicUARTService, public NuATParser
{
public:
    // Singleton pattern

    NuATCommandProcessor(const NuATCommandProcessor &) = delete;
    void operator=(NuATCommandProcessor const &) = delete;

    /**
     * @brief Get the instance of the NuATCommandProcessor
     *
     * @note No need to use. Use `NuATCommands` instead.
     *
     * @return NuATCommandProcessor& Single instance
     */
    static NuATCommandProcessor &getInstance()
    {
        static NuATCommandProcessor instance;
        return instance;
    };

public:
    // Overriden Methods
    virtual void onWrite(
        NimBLECharacteristic *pCharacteristic,
        NimBLEConnInfo &connInfo) override;
    virtual void printATResponse(std::string message) override;

    // New methods

    /**
     * @brief Set a maximum command line length to prevent overflow
     *
     * @note If a command line exceeds this limit, it will be ignored
     *       and an error response will be sent.
     *
     * @note A 256 bytes limit is recommended.
     *
     * @param value Zero to disable this feature.
     *              Otherwise, a maximum line length in bytes.
     * @return uint32_t previous limit or zero if disabled.
     */
    uint32_t maxCommandLineLength(uint32_t value = 0);

private:
    uint32_t uMaxCommandLineLength = 256;
    NuATCommandProcessor() {};
};

/**
 * @brief Singleton instance of the NuATCommandProcessor class
 *
 */
extern NuATCommandProcessor &NuATCommands;

#else
#error NuATCommands.hpp is incompatible with NuATCommandsLegacy2.hpp
#endif

#endif
