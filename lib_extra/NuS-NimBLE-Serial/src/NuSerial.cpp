/**
 * @file NuSerial.cpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-24
 * @brief Communications stream based on the Nordic UART Service
 *        with non-blocking Arduino semantics
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#include "NuSerial.hpp"

//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------

NordicUARTSerial &NuSerial = NordicUARTSerial::getInstance();