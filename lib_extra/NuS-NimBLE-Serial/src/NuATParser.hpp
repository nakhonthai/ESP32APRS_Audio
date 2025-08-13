/**
 * @file NuATParser.hpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2024-08-20
 * @brief Simple command line parser
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#ifndef __NU_AT_PARSER_HPP__
#define __NU_AT_PARSER_HPP__

#include <vector>
#include <string>
#include <functional>
#include <cstring> // Needed for strlen()

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
 * @brief Parsing/execution errors in a single command
 *
 * @note Additional information for debug or logging purposes
 */
typedef enum
{
    /** No command */
    AT_ERR_EMPTY_COMMAND = 0,
    /** Prefix token was not found */
    AT_ERR_INVALID_PREFIX,
    /** No command name, or command name has "&" prefix but more than one letter,
     *  or contains non alphabetic characters, or not upper case */
    AT_ERR_INVALID_NAME,
    /** Text found after a "?" or "=?" suffix */
    AT_ERR_PARAMETER_NOT_ALLOWED,
    /** No syntax error, but the given command is not supported by this app */
    AT_ERR_NO_CALLBACK,
    /** A string parameter is not properly enclosed between double quotes */
    AT_ERR_ILL_FORMED_STRING,
    /** A numeric parameter (without double quotes) is not properly
     * formatted as binary, decimal or hexadecimal */
    AT_ERR_ILL_FORMED_NUMBER,
    /** Command line is too long (not used by class NuATParser) */
    AT_ERR_TOO_LONG,
    /** Unspecified error to be used by descendant classes
     * (not used by class NuATParser) */
    AT_ERR_UNSPECIFIED
} NuATSyntaxError_t;

/**
 * @brief AT command parameters
 *
 */
typedef std::vector<std::string> NuATCommandParameters_t;

/**
 * @brief Callback to execute for AT commands
 *
 * @param[in] params Parameters as a string vector.
 *                   Empty if there are no parameters or parameters are not allowed.
 */
typedef std::function<NuATCommandResult_t(NuATCommandParameters_t &)> NuATCommandCallback_t;

/**
 * @brief Callback to execute for parsing/execution errors
 *
 * @param[in] text The text causing an error
 * @param[in] errorCode Code of error
 */
typedef std::function<void(const std::string text, NuATSyntaxError_t errorCode)> NuATErrorCallback_t;

/**
 * @brief Callback to execute for non-AT commands
 *
 * @param[in] text Pointer to buffer containing text
 * @param[in] errorCode Size of the buffer
 */
typedef std::function<void(const uint8_t *text, size_t size)> NuATNotACommandLineCallback_t;

/**
 * @brief Parse and execute AT commands
 *
 */
class NuATParser
{
public:
    /**
     * @brief Allow or not AT preamble/command names in lower case
     *
     * @note The AT standard requires upper-case
     *
     * @param[in] yesOrNo True to allow, false to disallow.
     * @return true Previously, allowed.
     * @return false Previously, disallowed.
     */
    bool allowLowerCase(bool yesOrNo);

    /**
     * @brief Stop execution on failure of a single command, or not
     *
     * @note Applies to a single call to execute()
     *
     * @param yesOrNo True to stop parsing if a single command fails,
     *                false to continue the execution of the following commands.
     * @return true Previously, stop
     * @return false Previously, continue
     */
    bool stopOnFirstFailure(bool yesOrNo);

    /**
     * @brief Set a callback for a command with no suffix
     *
     * @note If you set two or more callbacks for the same command name,
     *       just the first one will be executed, so don't do that.
     *
     * @param[in] commandName Command name
     * @param[in] callback Function to execute if @p commandName is found
     *                     with no suffix
     *
     * @return NuATParser& This instance. Used to chain calls.
     */
    NuATParser &onExecute(const std::string commandName, NuATCommandCallback_t callback);

    /**
     * @brief Set a callback for a command with "=" suffix
     *
     * @note If you set two or more callbacks for the same command name,
     *       just the first one will be executed, so don't do that.
     *
     * @param[in] commandName Command name
     * @param[in] callback Function to execute if @p commandName is found
     *                     with "=" suffix
     *
     * @return NuATParser& This instance. Used to chain calls.
     */
    NuATParser &onSet(const std::string commandName, NuATCommandCallback_t callback);

    /**
     * @brief Set a callback for a command with "?" suffix
     *
     * @note If you set two or more callbacks for the same command name,
     *       just the first one will be executed, so don't do that.
     *
     * @param[in] commandName Command name
     * @param[in] callback Function to execute if @p commandName is found
     *                     with "?" suffix
     *
     * @return NuATParser& This instance. Used to chain calls.
     */
    NuATParser &onQuery(const std::string commandName, NuATCommandCallback_t callback);

    /**
     * @brief Set a callback for a command with "=?" suffix
     *
     * @note If you set two or more callbacks for the same command name,
     *       just the first one will be executed, so don't do that.
     *
     * @param[in] commandName Command name
     * @param[in] callback Function to execute if @p commandName is found
     *                     with "=?" suffix
     *
     * @return NuATParser& This instance. Used to chain calls.
     */
    NuATParser &onTest(const std::string commandName, NuATCommandCallback_t callback);

    /**
     * @brief Set a callback for command errors
     *
     * @param[in] callback Function to execute on command errors
     *
     * @return NuATParser& This instance. Used to chain calls.
     */
    NuATParser &onError(NuATErrorCallback_t callback);

    /**
     * @brief Set a callback for non-AT commands
     *
     * @param[in] callback Function to execute
     *
     * @return NuATParser& This instance. Used to chain calls
     */
    NuATParser &onNotACommandLine(NuATNotACommandLineCallback_t callback);

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
    virtual void printATResponse(std::string message) = 0;

    /**
     * @brief Execute the given AT command line
     *
     * @param commandLine Pointer to a buffer containing a command line
     * @param size Size in bytes of @p commandLine
     */
    void execute(const uint8_t *commandLine, size_t size);

    /**
     * @brief Execute the given AT command line
     *
     * @param commandLine String containing command line
     */
    void execute(std::string commandLine)
    {
        execute((const uint8_t *)commandLine.data(), commandLine.length());
    };

    /**
     * @brief Execute the given AT command line
     *
     * @param commandLine Null-terminated string containing a command line
     */
    void execute(const char *commandLine)
    {
        if (commandLine)
            execute((const uint8_t *)commandLine, strlen(commandLine));
    };

protected:
    virtual void notifyError(
        std::string command,
        NuATSyntaxError_t errorCode);

    virtual void doExecute(const uint8_t *in, size_t size);

    virtual void doQuery(const uint8_t *in, size_t size);

    virtual void doTest(const uint8_t *in, size_t size);

    virtual void doSet(std::string command, NuATCommandParameters_t &params);

    virtual void doNotACommandLine(const uint8_t *in, size_t size);

    void printResultResponse(const NuATCommandResult_t response);

private:
    bool bAllowLowerCase = false;
    bool bStopOnFirstFailure = false;
    std::vector<std::string> vsOnExecuteCN;
    std::vector<std::string> vsOnSetCN;
    std::vector<std::string> vsOnQueryCN;
    std::vector<std::string> vsOnTestCN;
    std::vector<NuATCommandCallback_t> vcbOnExecuteCallback;
    std::vector<NuATCommandCallback_t> vcbOnSetCallback;
    std::vector<NuATCommandCallback_t> vcbOnQueryCallback;
    std::vector<NuATCommandCallback_t> vcbOnTestCallback;
    NuATErrorCallback_t cbErrorCallback = nullptr;
    NuATNotACommandLineCallback_t cbNoCommandsCallback = nullptr;

    bool findCallback(
        std::string name,
        std::vector<std::string> nameList,
        std::vector<NuATCommandCallback_t> callbackList,
        NuATCommandCallback_t &callback);

    void notifyError(
        const uint8_t *command,
        size_t size,
        NuATSyntaxError_t errorCode);

    bool executeSingleCommand(const uint8_t *in, size_t size);

    bool parseParameter(const uint8_t *in, size_t size, std::string &text);
};

#endif