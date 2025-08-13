/**
 * @file NuATParser.cpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2024-08-20
 * @brief Simple command line parser
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#include "NuATParser.hpp"
#include <HardwareSerial.h> // For testing
#include <algorithm>

//-----------------------------------------------------------------------------
// Set callbacks
//-----------------------------------------------------------------------------

NuATParser &NuATParser::onExecute(
    const std::string commandName,
    NuATCommandCallback_t callback)
{
    if (callback && (commandName.length() > 0))
    {
        vsOnExecuteCN.push_back(commandName);
        auto &newCmd = vsOnExecuteCN[vsOnExecuteCN.size() - 1];
        transform(newCmd.begin(), newCmd.end(), newCmd.begin(), ::toupper);
        vcbOnExecuteCallback.push_back(callback);
    }
    return *this;
}

//-----------------------------------------------------------------------------

NuATParser &NuATParser::onSet(
    const std::string commandName,
    NuATCommandCallback_t callback)
{
    if (callback && (commandName.length() > 0))
    {
        vsOnSetCN.push_back(commandName);
        auto &newCmd = vsOnSetCN[vsOnSetCN.size() - 1];
        transform(newCmd.begin(), newCmd.end(), newCmd.begin(), ::toupper);
        vcbOnSetCallback.push_back(callback);
    }
    return *this;
}

//-----------------------------------------------------------------------------

NuATParser &NuATParser::onQuery(
    const std::string commandName,
    NuATCommandCallback_t callback)
{
    if (callback && (commandName.length() > 0))
    {
        vsOnQueryCN.push_back(commandName);
        auto &newCmd = vsOnQueryCN[vsOnQueryCN.size() - 1];
        transform(newCmd.begin(), newCmd.end(), newCmd.begin(), ::toupper);
        vcbOnQueryCallback.push_back(callback);
    }
    return *this;
}

//-----------------------------------------------------------------------------

NuATParser &NuATParser::onTest(
    const std::string commandName,
    NuATCommandCallback_t callback)
{
    if (callback && (commandName.length() > 0))
    {
        vsOnTestCN.push_back(commandName);
        auto &newCmd = vsOnTestCN[vsOnTestCN.size() - 1];
        transform(newCmd.begin(), newCmd.end(), newCmd.begin(), ::toupper);
        vcbOnTestCallback.push_back(callback);
    }
    return *this;
}

//-----------------------------------------------------------------------------

NuATParser &NuATParser::onError(NuATErrorCallback_t callback)
{
    if (callback)
        cbErrorCallback = callback;
    return *this;
}

//-----------------------------------------------------------------------------

NuATParser &NuATParser::onNotACommandLine(
    NuATNotACommandLineCallback_t callback)
{
    if (callback)
        cbNoCommandsCallback = callback;
    return *this;
}

//-----------------------------------------------------------------------------
// Other
//-----------------------------------------------------------------------------

bool NuATParser::allowLowerCase(bool yesOrNo)
{
    bool result = bAllowLowerCase;
    bAllowLowerCase = yesOrNo;
    return result;
}

//-----------------------------------------------------------------------------

bool NuATParser::stopOnFirstFailure(bool yesOrNo)
{
    bool result = bStopOnFirstFailure;
    bStopOnFirstFailure = yesOrNo;
    return result;
}

//-----------------------------------------------------------------------------
// Auxiliary methods
//-----------------------------------------------------------------------------

void NuATParser::notifyError(
    const uint8_t *command,
    size_t size,
    NuATSyntaxError_t errorCode)
{
    std::string sCommandCopy((const char *)command, size);
    notifyError(sCommandCopy, errorCode);
}

//-----------------------------------------------------------------------------

void NuATParser::notifyError(
    std::string command,
    NuATSyntaxError_t errorCode)
{
    if (cbErrorCallback)
        cbErrorCallback(command, errorCode);
}

//-----------------------------------------------------------------------------

void NuATParser::printResultResponse(const NuATCommandResult_t response)
{
    switch (response)
    {
    case AT_RESULT_INVALID_PARAM:
        printATResponse("INVALID INPUT PARAMETERS");
        break;
    case AT_RESULT_ERROR:
        printATResponse("ERROR");
        break;
    case AT_RESULT_OK:
        printATResponse("OK");
        break;
    case AT_RESULT_SEND_OK:
        printATResponse("SEND OK");
        break;
    case AT_RESULT_SEND_FAIL:
        printATResponse("SEND FAIL");
        break;
    }
}

//-----------------------------------------------------------------------------

bool NuATParser::findCallback(
    std::string name,
    std::vector<std::string> nameList,
    std::vector<NuATCommandCallback_t> callbackList,
    NuATCommandCallback_t &callback)
{
    if (bAllowLowerCase)
        transform(name.begin(), name.end(), name.begin(), ::toupper);
    size_t index = 0;
    while (index < nameList.size())
    {
        if (nameList.at(index) == name)
        {
            callback = callbackList.at(index);
            return true;
        }
        index++;
    }
    return false;
}

//-----------------------------------------------------------------------------
// Parsing macros
//-----------------------------------------------------------------------------

void dec(size_t &size, size_t decrement)
{
    if (size > decrement)
        size -= decrement;
    else
        size = 0;
}

//-----------------------------------------------------------------------------

bool isATPreamble(const uint8_t *in, bool allowLowerCase)
{
    return ((in[0] == 'A') && (in[1] == 'T')) ||
           (allowLowerCase && (in[0] == 'a') && (in[1] == 't'));
}

//-----------------------------------------------------------------------------

size_t findSuffix(const uint8_t *in, size_t size)
{
    size_t index = 0;
    while ((index < size) && (in[index] != '?') && (in[index] != '='))
        index++;
    return index;
}

//-----------------------------------------------------------------------------

bool isAlphaString(const uint8_t *in, size_t size, bool allowLowerCase)
{
    size_t index = 0;
    while (index < size)
    {
        if (((in[index] >= 'A') && (in[index] <= 'Z')) ||
            (allowLowerCase && ((in[index] >= 'a') && (in[index] <= 'z'))))
            index++;
        else
            return false;
    }
    return true;
}

//-----------------------------------------------------------------------------

size_t findCommandSeparator(const uint8_t *in, size_t size)
{
    if (size == 0)
        return 0;
    if ((in[0] == ';') || (in[0] == '\n'))
        return 0;
    bool bDoubleQuotes = (in[0] == '\"');
    size_t index = 1;
    while (index < size)
    {
        if ((in[index] == '\"') && (in[index - 1] != '\\'))
            bDoubleQuotes = !bDoubleQuotes;
        else if (!bDoubleQuotes)
            if ((in[index] == ';') || (in[index] == '\n'))
                break;
        index++;
    }
    return index;
}

//-----------------------------------------------------------------------------

size_t findParamSeparator(const uint8_t *in, size_t size)
{
    size_t index = 0;
    if ((size > 0) && (in[0] == '\"'))
    {
        // text is a string between double quotes
        index++;
        // ignore all characters except non-escaped double quotes
        while ((index < size) && ((in[index] != '\"') || (in[index - 1] == '\\')))
            index++;
    }
    // Find comma separator
    while ((index < size) && (in[index] != ','))
        index++;
    return index;
}

//-----------------------------------------------------------------------------

bool isDigit(const uint8_t ch)
{
    return ((ch >= 'A') && (ch <= 'F')) ||
           ((ch >= 'a') && (ch <= 'f')) ||
           ((ch >= '0') && (ch <= '9'));
}

//-----------------------------------------------------------------------------

uint8_t parseHexByte(uint8_t high, uint8_t low)
{
    uint8_t a = 0;
    if ((high >= '1') && (high <= '9'))
        a = (high - '0');
    if ((high >= 'A') && (high <= 'F'))
        a = (high - 'A' + 11);
    if ((high >= 'a') && (high <= 'f'))
        a = (high - 'a' + 11);

    uint8_t b = 0;
    if ((low >= '1') && (low <= '9'))
        b = (low - '0');
    if ((low >= 'A') && (low <= 'F'))
        b = (low - 'A' + 11);
    if ((low >= 'a') && (low <= 'f'))
        b = (low - 'a' + 11);

    return (a * 16) + b;
}

//-----------------------------------------------------------------------------
// Execute
//-----------------------------------------------------------------------------

void NuATParser::execute(const uint8_t *commandLine, size_t size)
{
    bool bNotACommandLine = false;

    if ((size > 1) && isATPreamble(commandLine, bAllowLowerCase))
    {
        // skip preamble
        commandLine = commandLine + 2;
        dec(size, 2);
        if ((size >= 1) && (commandLine[0] == '\n'))
        {
            // This is a preamble with no commands at all.
            // Response is OK to signal that AT commands are accepted.
            printResultResponse(NuATCommandResult_t::AT_RESULT_OK);
            if (size > 1)
                // There is text after the terminator character
                // Maybe another command line
                execute(commandLine + 1, size - 1);
            return;
        }
        else if (size == 0)
        {
            // Command line is "AT" without a terminator character
            bNotACommandLine = true;
        }
    }
    else
        bNotACommandLine = true;

    if (bNotACommandLine)
    {
        // Not an AT command line
        doNotACommandLine(commandLine, size);
        return;
    }

    while (size > 0)
    {
        bool bNoError = true;
        // Determine length of next command
        size_t commandLength = findCommandSeparator(commandLine, size);

        // Parse and execute next command
        if (commandLength == 0)
        {
            // Empty command (";;" or ";\n")
            printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
            notifyError(
                commandLine,
                size,
                NuATSyntaxError_t::AT_ERR_EMPTY_COMMAND);
            bNoError = false;
        }
        else
            bNoError = executeSingleCommand(commandLine, commandLength);

        if (bStopOnFirstFailure && !bNoError)
            return;

        bool bCmdLineTerminatorFound = (commandLine[commandLength] == '\n');
        // move forward
        commandLength += 1; // jump over the ";" or "\n" separator
        commandLine += commandLength;
        dec(size, commandLength);

        // Handle command line terminator
        if (bCmdLineTerminatorFound && (size > 0))
        {
            // There is another text line after this one
            // Maybe another command line
            execute(commandLine, size);
            return;
        }
    }
}

//-----------------------------------------------------------------------------

bool NuATParser::executeSingleCommand(const uint8_t *in, size_t size)
{
    // Detect prefix.
    if ((in[0] == '&') || (in[0] == '+'))
    {
        uint8_t prefix = in[0];
        dec(size, 1);
        in++;

        // Prefix is valid, now detect suffix.
        // Text between a prefix and a suffix is a command name.
        // Text between a prefix and ";" is also a command name.
        size_t cmdNameLength = findSuffix(in, size);

        // Look for invalid command names
        // Note: if prefix is '&', just a single letter is allowed as command name
        if ((cmdNameLength == 0) || ((prefix == '&') && (cmdNameLength != 1)) || !isAlphaString(in, cmdNameLength, bAllowLowerCase))
        {
            printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
            notifyError(in, size, NuATSyntaxError_t::AT_ERR_INVALID_NAME);
            return false;
        }

        // Command with no suffix
        if (cmdNameLength >= size)
            doExecute(in, size);
        else
        {
            // Command with suffix
            if ((in[cmdNameLength] == '=') && ((cmdNameLength == (size - 1)) || (in[cmdNameLength + 1] != '?')))
            {
                // Suffix is "=" (but not "=?")
                // Parameters are allowed, but not mandatory
                std::string commandName((const char *)in, cmdNameLength);
                NuATCommandParameters_t params;

                // jump to the first parameter
                in += cmdNameLength + 1;
                dec(size, cmdNameLength + 1);
                while (size > 0)
                {
                    // Parse next parameter
                    size_t paramLength = findParamSeparator(in, size);
                    std::string aParameter;
                    if (!parseParameter(in, paramLength, aParameter))
                        // Syntax error. Do not execute.
                        return false;
                    params.push_back(aParameter);
                    // jump over the "," character and move to the next parameter
                    in += paramLength + 1;
                    dec(size, paramLength + 1);
                }
                doSet(commandName, params);
            }
            else
            {
                if ((in[cmdNameLength] == '=') && (cmdNameLength == (size - 2)))
                    // Suffix is "=?"
                    doTest(in, cmdNameLength);
                else if ((in[cmdNameLength] == '?') && (cmdNameLength == (size - 1)))
                    // Suffix is "?"
                    doQuery(in, cmdNameLength);
                else
                {
                    // Text found after the suffix
                    printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
                    notifyError(in, size, NuATSyntaxError_t::AT_ERR_PARAMETER_NOT_ALLOWED);
                    return false;
                }
            }
        }
    }
    else
    {
        printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
        notifyError(in, size, NuATSyntaxError_t::AT_ERR_INVALID_PREFIX);
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------

bool NuATParser::parseParameter(const uint8_t *in, size_t size, std::string &text)
{
    if (size == 0)
        return true;
    if (in[0] == '"')
    {
        // Parse string parameter
        if (in[size - 1] != '"')
        {
            printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
            notifyError(in, size, NuATSyntaxError_t::AT_ERR_ILL_FORMED_STRING);
            return false;
        }

        // Ignore leading and trailing double quotes
        in++;
        dec(size, 2);

        // Look for escape characters
        while (size > 0)
        {
            if ((in[0] == '\\') && (size > 1))
            {
                if ((size > 2) && isDigit(in[1]) && isDigit(in[2]))
                {
                    // ITU V.250, paragraph 5.4.2.2
                    uint8_t ch = parseHexByte(in[1], in[2]);
                    text.push_back(ch);
                    in += 2;
                    dec(size, 2);
                }
                else
                {
                    text.push_back(in[1]);
                    in++;
                    dec(size, 1);
                }
            }
            else if (in[0] == '\\')
            {
                // No character after escape character
                printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
                notifyError(in, size, NuATSyntaxError_t::AT_ERR_ILL_FORMED_STRING);
                return false;
            }
            else
                text.push_back(in[0]);
            in++;
            dec(size, 1);
        }
    }
    else
    {
        // Parse non-string parameter
        // According to ITU V.250, paragraph 5.4.2.1, only numeric constants
        // are allowed (binary, decimal and hexadecimal)
        size_t index = 0;
        while ((index < size) && isDigit(in[index]))
            index++;
        if (index < size)
        {
            // Invalid format
            printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
            notifyError(in, size, NuATSyntaxError_t::AT_ERR_ILL_FORMED_NUMBER);
            return false;
        }
        text.assign((const char *)in, size);
    }
    return true;
}

//-----------------------------------------------------------------------------
// Execute callbacks
//-----------------------------------------------------------------------------

void NuATParser::doExecute(const uint8_t *in, size_t size)
{
    std::string name((const char *)in, size);
    NuATCommandCallback_t callback;
    if (findCallback(name, vsOnExecuteCN, vcbOnExecuteCallback, callback))
    {
        NuATCommandParameters_t empty;
        NuATCommandResult_t result = callback(empty);
        printResultResponse(result);
    }
    else
    {
        printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
        notifyError(name, NuATSyntaxError_t::AT_ERR_NO_CALLBACK);
    }
}

//-----------------------------------------------------------------------------

void NuATParser::doQuery(const uint8_t *in, size_t size)
{
    std::string name((const char *)in, size);
    NuATCommandCallback_t callback;
    if (findCallback(name, vsOnQueryCN, vcbOnQueryCallback, callback))
    {
        NuATCommandParameters_t empty;
        NuATCommandResult_t result = callback(empty);
        printResultResponse(result);
    }
    else
    {
        printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
        notifyError(name, NuATSyntaxError_t::AT_ERR_NO_CALLBACK);
    }
}

//-----------------------------------------------------------------------------

void NuATParser::doTest(const uint8_t *in, size_t size)
{
    std::string name((const char *)in, size);
    NuATCommandCallback_t callback;
    if (findCallback(name, vsOnTestCN, vcbOnTestCallback, callback))
    {
        NuATCommandParameters_t empty;
        NuATCommandResult_t result = callback(empty);
        printResultResponse(result);
    }
    else
    {
        printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
        notifyError(name, NuATSyntaxError_t::AT_ERR_NO_CALLBACK);
    }
}

//-----------------------------------------------------------------------------

void NuATParser::doSet(std::string command, NuATCommandParameters_t &params)
{
    NuATCommandCallback_t callback;
    if (findCallback(command, vsOnSetCN, vcbOnSetCallback, callback))
    {
        NuATCommandResult_t result = callback(params);
        printResultResponse(result);
    }
    else
    {
        printResultResponse(NuATCommandResult_t::AT_RESULT_ERROR);
        notifyError(command, NuATSyntaxError_t::AT_ERR_NO_CALLBACK);
    }
}

//-----------------------------------------------------------------------------

void NuATParser::doNotACommandLine(const uint8_t *in, size_t size)
{
    if (cbNoCommandsCallback)
        cbNoCommandsCallback(in, size);
}

//-----------------------------------------------------------------------------
