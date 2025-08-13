/**
 * @file NuCLIParser.hpp
 * @author Ángel Fernández Pineda. Madrid. Spain.
 * @date 2023-12-26
 * @brief Simple command line parser
 *
 * @copyright Creative Commons Attribution 4.0 International (CC BY 4.0)
 *
 */

#ifndef __NU_CLI_PARSER_HPP__
#define __NU_CLI_PARSER_HPP__

#include <vector>
#include <string>
#include <cstring> // Needed for strlen()
#include <functional>

/**
 * @brief Parsing state of a received command
 *
 * @note Additional information about parsing for debug or logging purposes
 */
typedef enum
{
    /** No Parsing error */
    CLI_PR_OK = 0,
    /** Callbacks not set */
    CLI_PR_NO_CALLBACKS,
    /** Command line is empty */
    CLI_PR_NO_COMMAND,
    /** A string parameter is not properly enclosed between double quotes */
    CLI_PR_ILL_FORMED_STRING

} NuCLIParsingResult_t;

/**
 * @brief Parsed strings in a command line, from left to right
 *
 * @note First item is always a command name, so this vector
 *       is never empty. However, it may contain empty strings
 *       typed as "".
 */
typedef std::vector<std::string> NuCommandLine_t;

/**
 * @brief Callback to execute for a parsed command line
 *
 * @param[in] commandLine Parsed command line.
 */
typedef std::function<void(NuCommandLine_t &)> NuCLICommandCallback_t;

/**
 * @brief Callback to execute in case of parsing errors
 *
 * @note This callback is never executed with parsing result CLI_PR_OK.
 *
 * @param[in] result Parsing result
 * @param[in] index Byte index where the parsing error was found (0-based).
 */
typedef void (*NuCLIParseErrorCallback_t)(NuCLIParsingResult_t, size_t);

/**
 * @brief Parse and execute simple commands
 *
 */
class NuCLIParser
{
public:
    /**
     * @brief Enable or disable case-sensitive command names
     *
     * @param[in] yesOrNo True for case-sensitive. False, otherwise.
     * @return true Previously, case-sensitive.
     * @return false Previously, case-insensitive.
     */
    bool caseSensitive(bool yesOrNo);

    /**
     * @brief Set a callback for a command name
     *
     * @note If you set two or more callbacks for the same command name,
     *       just the first one will be executed, so don't do that.
     *
     * @note Example:
     *       @code {.cpp}
     *       NuShellCommands
     *          .on("mycmd", [](NuCommandLine_t &commandLine)
     *           { ...do something...})
     *          .onUnknown([](NuCommandLine_t &commandLine)
     *           { ...do something else...});
     *
     *       @endcode
     *
     * @param[in] commandName Command name
     * @param[in] callback Function to execute if @p commandName is found
     *
     * @return NuCLIParser& This instance. Used to chain calls.
     */
    NuCLIParser &on(const std::string commandName, NuCLICommandCallback_t callback);

    /**
     * @brief Set a callback for unknown commands
     *
     * @param[in] callback Function to execute if the parsed command line contains
     *                     an unknown command name.
     *
     * @return NuCLIParser& This instance. Used to chain calls.
     */
    NuCLIParser &onUnknown(NuCLICommandCallback_t callback)
    {
        cbUnknown = callback;
        return *this;
    };

    /**
     * @brief Set a callback for parsing errors
     *
     * @param[in] callback Function to execute if some parsing error is found.
     * @return NuCLIParser& This instance. Used to chain calls.
     */
    NuCLIParser &onParseError(NuCLIParseErrorCallback_t callback)
    {
        cbParseError = callback;
        return *this;
    };

    /**
     * @brief Execute the given command line
     *
     * @param commandLine Pointer to a buffer containing a command line
     * @param size Size in bytes of @p commandLine
     */
    void execute(const uint8_t *commandLine, size_t size);

    /**
     * @brief Execute the given command line
     *
     * @param commandLine String containing command line
     */
    void execute(std::string commandLine)
    {
        execute((const uint8_t *)commandLine.data(), commandLine.length());
    };

    /**
     * @brief Execute the given command line
     *
     * @param commandLine Null-terminated string containing a command line
     */
    void execute(const char *commandLine)
    {
        if (commandLine)
            execute((const uint8_t *)commandLine, strlen(commandLine));
    };

protected:
    static NuCLIParsingResult_t parse(const uint8_t *in, size_t size, size_t &index, NuCommandLine_t &parsedCommandLine);
    static NuCLIParsingResult_t parseNext(const uint8_t *in, size_t size, size_t &index, NuCommandLine_t &parsedCommandLine);
    static void ignoreSeparator(const uint8_t *in, size_t size, size_t &index);
    static bool isSeparator(const uint8_t *in, size_t size, size_t index);

    /**
     * @brief Notify successfully parsed command line
     *
     * @note Current implementation executes the appropiate callback.
     *       Override for custom command processing if you don't like callbacks.
     *
     * @param[in] commandLine Parsed command line
     */
    virtual void onParsingSuccess(NuCommandLine_t &commandLine);

    /**
     * @brief Notify parsing error
     *
     * @note Current implementation executes onParseError() callback.
     *       Override for custom error processing.
     *
     * @param[in] result Parsing result
     * @param[in] index Byte index where the parsing error was found (0-based).
     */
    virtual void onParsingFailure(NuCLIParsingResult_t result, size_t index);

private:
    bool bCaseSensitive = false;
    NuCLIParseErrorCallback_t cbParseError = nullptr;
    NuCLICommandCallback_t cbUnknown = nullptr;
    std::vector<std::string> vsCommandName;
    std::vector<NuCLICommandCallback_t> vcbCommand;
};

#endif