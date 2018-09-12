#define BOOST_LOG_DYN_LINK 1  // needed for compiling
/****************************************************************************************
 *
 * File:
 * 		Logger.h
 *
 * Purpose:
 *		Provides functions for logging data to file and console.
 *      The logger starts and stops automatically with the program.
 *
 *
 * Developer Notes:
 *      Useful links:
 *        Boost library documentation: https://www.boost.org/
 *        Stack Overflow topic:
 *          https://stackoverflow.com/questions/20086754/how-to-use-boost-log-from-multiple-files-with-gloa/22068278#22068278
 *          https://stackoverflow.com/questions/29785243/c-how-to-set-a-severity-filter-on-a-boost-global-logger
 *
 *
 ***************************************************************************************/

#pragma once
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/trivial.hpp>

#define DEFAULT_LOG_NAME "sailing.log"
#define DEFAULT_LOG_NAME_WRSC "wrsc.log"
#define FILE_PATH "../logs/"
#define MAX_LOG_SIZE 256 * 2

// Narrow-char thread-safe logger, with default severity levels from boost library
typedef boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger_t;

// declares a global logger with a custom initialization declared in cpp file
BOOST_LOG_GLOBAL_LOGGER(global_logger, logger_t)

class Logger {
   public:
    static std::string m_filename;
    /////////////////////////////////////////////////////////////////////////////////////
    /// Initialises the singleton logger system, returns false if it is unable to
    /// generate a log file.
    ///
    /// @params logType 			The type of log message, if this parameter is not
    ///								provided then a default name is used.
    ///
    /////////////////////////////////////////////////////////////////////////////////////
    static bool init(const char* filename = 0);

    /// SHOULD ONLY BE USED FOR UNIT TESTS!
    static void DisableLogging();

    /////////////////////////////////////////////////////////////////////////////////////
    /// A globally accessable function to log messages to that works exactly like printf.
    ///
    /// @params message 			The log message.
    /// @params ...					A variable list, this allows printf like behaviour
    ///
    /////////////////////////////////////////////////////////////////////////////////////
    static void trace(std::string message, ...);
    static void debug(std::string message, ...);
    static void info(std::string message, ...);
    static void warning(std::string message, ...);
    static void error(std::string message, ...);
    static void fatal(std::string message, ...);


   private:
    static bool m_DisableLogging;
};
