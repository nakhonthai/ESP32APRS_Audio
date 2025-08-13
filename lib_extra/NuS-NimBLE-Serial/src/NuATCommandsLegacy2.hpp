/**
 * @file NuATCommandsLegacy2.hpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-18
 * @brief AT command processor using the Nordic UART Service
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */
#ifndef __NU_AT_COMMANDS_LEGACY2_HPP__
#define __NU_AT_COMMANDS_LEGACY2_HPP__

#ifndef __NU_AT_COMMANDS_HPP__

#include "NuS.hpp"
#include "NuATCommandParserLegacy2.hpp"

namespace NuSLegacy2
{
    /**
     * @brief Execute AT commands received thanks to the Nordic UART Service
     *
     */
    class NuATCommandProcessor : public NordicUARTService, public NuATCommandParser
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
         * @return NuATCommandProcessor&
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
        virtual void printATResponse(const char message[]) override;

        /**
         * @brief Set custom AT command processing callbacks
         *
         * @note This method should be called before start(). Any way,
         *       you are not allowed to set a new callbacks object while
         *       a peer is connected.
         *
         * @param pCallbacks A pointer to your own callbacks. Must
         *        remain valid forever (do not destroy).
         *
         * @throws std::runtime_error If called while a peer is connected.
         */
        void setATCallbacks(NuATCommandCallbacks *pCallbacks);

    private:
        NuATCommandProcessor() {};
    };

    /**
     * @brief Singleton instance of the NuATCommandProcessor class
     *
     */
    extern NuATCommandProcessor &NuATCommands;
}

#else
#error NuATCommands.hpp is incompatible with NuATCommandsLegacy2.hpp
#endif

#endif