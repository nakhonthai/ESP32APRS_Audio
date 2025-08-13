/**
 * @file NuATCommandParserLegacy2.hpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-24
 * @brief AT command parser
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#ifndef __NU_AT_COMMAND_PARSER_LEGACY2_HPP__
#define __NU_AT_COMMAND_PARSER_LEGACY2_HPP__

#include <vector>

/**
 * @brief Pseudo-standardized result of AT command execution
 *
 * @note Negative means error and non-negative means success.
 *       In case of doubt, use either AT_RESULT_OK or AT_RESULT_ERROR.
 */
typedef enum
{
    /** Failure to send a command to a protocol stack */
    AT_RESULT_SEND_FAIL = -3,
    /** Command not executed due to invalid or missing parameter(s) */
    AT_RESULT_INVALID_PARAM = -2,
    /** Command executed with no success */
    AT_RESULT_ERROR = -1,
    /** Command executed with success */
    AT_RESULT_OK = 0,
    /** Command send successfully to a protocol stack but execution pending */
    AT_RESULT_SEND_OK = 1
} NuATCommandResult_t;

/**
 * @brief Parsing state of a received command
 *
 * @note Additional information about parsing for debug or logging purposes
 */
typedef enum
{
    /** No Parsing error */
    AT_PR_OK = 0,
    /** Callbacks not set */
    AT_PR_NO_CALLBACKS,
    /** Not an AT command line  */
    AT_PR_NO_PREAMBLE,
    /** AT preamble found but no commands */
    AT_PR_NO_COMMANDS,
    /** Prefix token was not found */
    AT_PR_INVALID_PREFIX,
    /** No command name,  buffer overflow or command name has "&" prefix but more than one letter */
    AT_PR_INVALID_CMD1,
    /** Command name contains non alphabetic characters */
    AT_PR_INVALID_CMD2,
    /** Command name valid, but not supported by this app */
    AT_PR_UNSUPPORTED_CMD,
    /** Command - end token was expected but not found */
    AT_PR_END_TOKEN_EXPECTED,
    /** Buffer overflow in a SET command(parameters too long) */
    AT_PR_SET_OVERFLOW,
    /** A string parameter is not properly enclosed between double quotes */
    AT_PR_ILL_FORMED_STRING,
    /** Unable to allocate buffer memory */
    AT_PR_NO_HEAP
} NuATParsingResult_t;

typedef std::vector<const char *> NuATCommandParameters_t;

/**
 * @brief Custom AT command processing for your application
 *
 * @note Derive a new class to implement your own AT commands
 */
class NuATCommandCallbacks
{
public:
    /**
     * @brief Custom processing of non-AT data
     *
     * @note Optional
     *
     * @param text Null-terminated incoming string,
     *             not matching an AT command line.
     */
    virtual void onNonATCommand(const char text[]){};

    /**
     * @brief Identify supported command names
     *
     * @note Override this method to inform which commands are
     *       supported or not. This is mandatory.
     *
     * @note AT commands should comprise uppercase characters, but this is up
     *       to you. You may return the same ID for lowercase
     *       command names. You may also return the same ID for aliases.
     *
     * @param commandName A null-terminated string containing a valid command name.
     *        This string does not contain any prefix (`&` or `+`), just the
     *        name. Length of command names is limited by buffer size.
     *        Will comprise alphabetic characters only, as required by the AT
     *        standard, so don't expect something like "PARAM1".
     *
     * @return int Any negative value if @p commandName is not a supported
     *         AT command. Any positive number as an **unique**
     *         identification (ID) of a supported command name.
     */
    virtual int getATCommandId(const char commandName[]) = 0;

    /**
     * @brief Execute a supported AT command (with no suffix)
     *
     * @param commandId Unique (non-negative) identification number as returned by getATCommandId()
     * @return NuATCommandResult_t Proper result of command execution
     */
    virtual NuATCommandResult_t onExecute(int commandId) { return AT_RESULT_ERROR; };

    /**
     * @brief Execute or set the value given in a supported AT command (with '=' suffix)
     *
     * @param commandId Unique (non-negative) identification number as returned by getATCommandId()
     *
     * @param parameters A sorted list of null-terminated strings, one for each parameter,
     *                   from left to right. Total length of all parameters
     *                   is limited by buffer size. There is at least one parameter, but any parameter
     *                   may be an empty string. AT string parameters are properly parsed before calling.
     *
     * @return NuATCommandResult_t Proper result of command execution
     */
    virtual NuATCommandResult_t onSet(int commandId, NuATCommandParameters_t &parameters) { return AT_RESULT_ERROR; };

    /**
     * @brief Print the value requested in a supported AT command (with '?' suffix)
     *
     * @note Use NuATCommands.printATResponse() to print the requested value.
     *
     * @param commandId Unique (non-negative) identification number as returned by getATCommandId()
     *
     * @return NuATCommandResult_t Proper result of command execution
     */
    virtual NuATCommandResult_t onQuery(int commandId) { return AT_RESULT_ERROR; };

    /**
     * @brief Print the syntax and parameters of a supported command (with '=?' suffix)
     *
     * @note Optional. Use NuATCommands.printATResponse() to print.
     *
     * @param commandId Unique (non-negative) identification number as returned by getATCommandId()
     */
    virtual void onTest(int commandId){};

    /**
     * @brief Get informed of the parsing result of each received command
     *
     * @note Optional. Use this method to store the details about the result of the last command, so you can
     *       implement another AT command to send that information. Always called after the command
     *       is parsed and, if no parsing errors were found, executed. Note that if a parsing error is found
     *       in a command line, the following commands in that command line are not parsed nor executed.
     *
     * @param index 0-based index of the command being informed as it was written in the command line,
     *              from left to right. For example, for the command line "AT&F;&G;&H", index
     *              1 refers to "&G".
     *
     * @param parsingResult Detailed result of command parsing
     */
    virtual void onFinished(int index, NuATParsingResult_t parsingResult){};
};

/**
 * @brief Parse and execute AT commands
 *
 */
class NuATCommandParser
{
public:
    /**
     * @brief Print a message properly formatted as an AT response
     *
     * @note Error and success messages are already managed by this class.
     *       Do not print those messages to avoid misunderstandings.
     *
     * @note An AT response is just a text starting with CR+LF and
     *       ending with CR+LF.
     *       You may use `NuATCommands.printf("\r\n%s\r\n",...)` instead.
     *
     * @param message Text to print.
     *                Must not contain the CR+LF sequence of characters.
     */
    virtual void printATResponse(const char message[]) = 0;

    /**
     * @brief Set custom AT command processing callbacks
     *
     * @note Not thread-safe.
     *
     * @param pCallbacks A pointer to your own callbacks. Must
     *        remain valid forever (do not destroy).
     *
     */
    void setATCallbacks(NuATCommandCallbacks *pCallbacks)
    {
        pCmdCallbacks = pCallbacks;
    };

    /**
     * @brief Size of the parsing buffer
     *
     * @note An error response will be printed if command names or
     *       command parameters exceed this size. Buffer is allocated
     *       in the heap.
     *
     * @note Default size is 42 bytes
     *
     * @param size Size in bytes
     */
    void setBufferSize(size_t size);

    /**
     * @brief Allow or disallow a lower case "AT" preamble
     *
     * @note By default, the "at" preamble (in lower case) is not allowed.
     *       Should be called before start().
     *
     * @param allowOrNot When true, "at" is a valid preamble for a command line.
     * When false, just "AT" is allowed as preamble.
     */
    void lowerCasePreamble(bool allowOrNot = true)
    {
        bLowerCasePreamble = allowOrNot;
    };

public:
    /**
     * @brief Check this attribute to know why parsing failed (or not)
     *        on the last received command
     *
     * @note Exposed for testing, mainly. Do not write.
     */
    NuATParsingResult_t lastParsingResult = AT_PR_OK;

private:
    NuATCommandCallbacks *pCmdCallbacks = nullptr;
    size_t bufferSize = 42;
    bool bLowerCasePreamble = false;

    const char *parseSingleCommand(const char *in);
    const char *parseAction(const char *in, int commandId);
    const char *parseWriteParameters(const char *in, int commandId);

protected:
    virtual void printResultResponse(const NuATCommandResult_t response);
    void parseCommandLine(const char *in);
};

#endif