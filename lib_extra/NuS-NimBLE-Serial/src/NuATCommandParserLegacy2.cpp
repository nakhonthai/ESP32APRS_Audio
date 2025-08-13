/**
 * @file NuATCommandParserLegacy2.cpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-24
 * @brief AT command parser
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#include <string.h>
#include "NuATCommandParserLegacy2.hpp"

//-----------------------------------------------------------------------------
// Parsing macros
//-----------------------------------------------------------------------------

inline bool isATPreamble(const char *in, bool allowLowerCase)
{
    return ((in[0] == 'A') && (in[1] == 'T')) || (allowLowerCase && (in[0] == 'a') && (in[1] == 't'));
}

inline bool isCommandEndToken(const char c)
{
    return (c == '\n') || (c == '\0') || (c == ';');
}

bool isAlphaString(const char *in)
{
    while (((in[0] >= 'A') && (in[0] <= 'Z')) || ((in[0] >= 'a') && (in[0] <= 'z')))
        in++;
    return (in[0] == '\0');
}

const char *followingCommand(const char *in, NuATCommandResult_t conditional = AT_RESULT_OK)
{
    if ((conditional < 0) || (in[0] == '\0') || (in[0] == '\n'))
        return nullptr;
    else if (in[0] == ';')
        return in + 1;
    else
        // should not enter here
        return nullptr;
}

const char *findSuffix(const char *in)
{
    while ((in[0] != '\0') && (in[0] != '\n') && (in[0] != ';') && (in[0] != '?') && (in[0] != '='))
        in++;
    return in;
}

//-----------------------------------------------------------------------------
// Parsing machinery
//-----------------------------------------------------------------------------

void NuATCommandParser::parseCommandLine(const char *in)
{
    lastParsingResult = AT_PR_NO_CALLBACKS;
    if (!pCmdCallbacks)
        // no callbacks: nothing to do here
        return;

    // Detect AT preamble
    if (isATPreamble(in, bLowerCasePreamble))
    {
        // skip preamble
        in = in + 2;
        if ((in[0] == '\n') || (in[0] == '\0'))
        {
            // This is a preamble with no commands at all.
            // Response is OK to signal that AT commands are accepted.
            printResultResponse(AT_RESULT_OK);
            lastParsingResult = AT_PR_NO_COMMANDS;
            return;
        }
    }
    else
    {
        // Not an AT command line
        lastParsingResult = AT_PR_NO_PREAMBLE;
        try
        {
            pCmdCallbacks->onNonATCommand(in);
        }
        catch (...)
        {
        };
        return;
    }

    // Parse all commands contained in incoming data
    int commandIndex = 0;
    do
    {
        // Serial.printf("parseCommandLine(): %s\n", in);
        lastParsingResult = AT_PR_OK; // may be changed later
        in = parseSingleCommand(in);
        try
        {
            pCmdCallbacks->onFinished(commandIndex++, lastParsingResult);
        }
        catch (...)
        {
        };
    } while (in);
}

const char *NuATCommandParser::parseSingleCommand(const char *in)
{
    // Detect prefix.
    // Note: if prefix is '&', just a single letter is allowed as command name
    // Serial.printf("parseSingleCommand(): %s\n", in);
    if ((in[0] == '&') || (in[0] == '+'))
    {
        // Prefix is valid, now detect suffix.
        // Text between a prefix and a suffix is a command name.
        // Text between a prefix and ";", "\n" or "\0" is also a command name.
        const char *suffix = findSuffix(in + 1);
        size_t cmdNameLength = suffix - (in + 1);
        if ((cmdNameLength > 0) && (cmdNameLength < bufferSize) && ((in[0] == '+') || (cmdNameLength == 1)))
        {
            // Serial.printf("parseSingleCommand(1): %s. Suffix: %s. Length: %d\n", in + 1, suffix, cmdNameLength);
            // store command name in "cmdName" as a null-terminated string
            // char cmdName[bufferSize];
            char *cmdName = (char *)malloc(cmdNameLength + 1);
            if (!cmdName)
            {
                lastParsingResult = AT_PR_NO_HEAP;
                printResultResponse(AT_RESULT_ERROR);
                return nullptr;
            }
            // memcpy(cmdName, in + 1, cmdNameLength);
            for (int i = 0; i < cmdNameLength; i++)
                cmdName[i] = *(in + 1 + i);
            cmdName[cmdNameLength] = '\0';
            // Serial.printf("parseSingleCommand(2): %s. Suffix: %s. Name: %s. Length: %d. pName %d. pIn+1:%d\n", in + 1, suffix, cmdName, cmdNameLength, cmdName, in + 1);

            if (isAlphaString(cmdName))
            {
                // check if command is supported
                int commandId;
                try
                {
                    commandId = pCmdCallbacks->getATCommandId(cmdName);
                }
                catch (...)
                {
                    commandId = -1;
                }
                if (commandId >= 0)
                {
                    // continue parsing
                    free(cmdName);
                    return parseAction(suffix, commandId);
                }
                else // this command is not supported
                    lastParsingResult = AT_PR_UNSUPPORTED_CMD;
            }
            else // command name contains non-alphabetic characters
                lastParsingResult = AT_PR_INVALID_CMD2;

            free(cmdName);
        }
        else // error: no command name, buffer overflow or command name has "&" prefix but more than one letter
            lastParsingResult = AT_PR_INVALID_CMD1;

    } // invalid prefix
    else
    {
        lastParsingResult = AT_PR_INVALID_PREFIX;
        // Serial.printf("Invalid prefix\n");
    }
    printResultResponse(AT_RESULT_ERROR);
    return nullptr;
}

const char *NuATCommandParser::parseAction(const char *in, int commandId)
{
    // Serial.printf("parseAction(): %s\n", in);
    //  Note: "in" points to a suffix or an end-of-command token
    if ((in[0] == '=') && (in[1] == '?'))
    {
        // This is a TEST command
        if (isCommandEndToken(in[2]))
        {
            NuATCommandResult_t result = AT_RESULT_OK;
            try
            {
                pCmdCallbacks->onTest(commandId);
            }
            catch (...)
            {
                result = AT_RESULT_ERROR;
            }
            printResultResponse(result);
            return followingCommand(in + 2, result);
        } // else syntax error
    }
    else if (in[0] == '?')
    {
        // This is a READ/QUERY command
        if (isCommandEndToken(in[1]))
        {
            NuATCommandResult_t response;
            try
            {
                response = pCmdCallbacks->onQuery(commandId);
            }
            catch (...)
            {
                response = AT_RESULT_ERROR;
            }
            printResultResponse(response);
            return followingCommand(in + 1, response);
        } // else syntax Error
    }
    else if (in[0] == '=')
    {
        // This is a SET/WRITE command
        return parseWriteParameters(in + 1, commandId);
    }
    else if (isCommandEndToken(in[0]))
    {
        // This is an EXECUTE Command
        NuATCommandResult_t response;
        try
        {
            response = pCmdCallbacks->onExecute(commandId);
        }
        catch (...)
        {
            response = AT_RESULT_ERROR;
        }
        printResultResponse(response);
        return followingCommand(in, response);
    } // else syntax error
    lastParsingResult = AT_PR_END_TOKEN_EXPECTED;
    printResultResponse(AT_RESULT_ERROR);
    return nullptr;
}

const char *NuATCommandParser::parseWriteParameters(const char *in, int commandId)
{
    // See https://docs.espressif.com/projects/esp-at/en/release-v2.2.0.0_esp8266/AT_Command_Set/index.html
    // about parameters' syntax.

    NuATCommandParameters_t paramList;
    // char buffer[bufferSize];
    char *buffer = (char *)malloc(bufferSize);
    if (!buffer)
    {
        lastParsingResult = AT_PR_NO_HEAP;
        printResultResponse(AT_RESULT_ERROR);
        return nullptr;
    }
    size_t l = 0;
    bool doubleQuotes = false;
    bool syntaxError = false;
    char *currentParam = buffer;

    // Parse, tokenize and copy parameters to buffer
    while (!isCommandEndToken(in[0]) && (l < bufferSize))
    {
        if (doubleQuotes)
        {
            if ((in[0] == '\"') && ((in[1] == ',') || isCommandEndToken(in[1])))
            {
                // Closing double quotes
                doubleQuotes = false;
                in++;
                continue;
            }
            else if (in[0] == '\"')
            {
                // there is more text after the closing double quotes
                syntaxError = true;
                break;
            }
            else if ((in[0] == '\\') && (in[1] != '\0'))
            {
                // Escaped character
                in++;
                buffer[l++] = in[0];
                in++;
                continue;
            }
        }
        else
        {
            if ((in[0] == '\"') && (currentParam == (buffer + l)))
            {
                // Opening double quotes
                doubleQuotes = true;
                in++;
                continue;
            }
            else if (in[0] == '\"')
            {
                // There is some text before the opening double quotes
                syntaxError = true;
                break;
            }
        }

        // copy char to buffer and tokenize
        if (in[0] == ',')
        {
            // Serial.println("param token");
            if (doubleQuotes)
            {
                // Missing closing double quotes
                syntaxError = true;
                break;
            }
            else
            {
                // End of this parameter
                buffer[l++] = '\0';
                paramList.push_back(currentParam);
                // Serial.printf("Prev param: %s\n", currentParam);
                currentParam = (buffer + l);
                in++;
            }
        }
        else
        {
            buffer[l++] = in[0];
            in++;
        }
    } // end-while

    // check for syntax errors or missing double quotes in last parameter
    if (syntaxError || doubleQuotes)
    {
        free(buffer);
        lastParsingResult = AT_PR_ILL_FORMED_STRING;
        printResultResponse(AT_RESULT_ERROR);
        return nullptr;
    }

    // check for buffer overflow
    if (l >= bufferSize)
    {
        free(buffer);
        lastParsingResult = AT_PR_SET_OVERFLOW;
        printResultResponse(AT_RESULT_ERROR);
        return nullptr;
    }

    // Add the last parameter
    buffer[l] = '\0';
    paramList.push_back(currentParam);
    // Serial.printf("Last param: %s\n", currentParam);

    // Invoke callback
    NuATCommandResult_t response;
    try
    {
        response = pCmdCallbacks->onSet(commandId, paramList);
    }
    catch (...)
    {
        response = AT_RESULT_ERROR;
    }
    free(buffer);
    printResultResponse(response);
    return followingCommand(in, response);
}

//-----------------------------------------------------------------------------
// Buffer size
//-----------------------------------------------------------------------------

void NuATCommandParser::setBufferSize(size_t size)
{
    if (size > 5)
    {
        bufferSize = size;
    }
    else
        // absolute minimum
        bufferSize = 5;
}

//-----------------------------------------------------------------------------
// Printing
//-----------------------------------------------------------------------------

void NuATCommandParser::printResultResponse(const NuATCommandResult_t response)
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