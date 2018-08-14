 /****************************************************************************************
 *
 * File:
 * 		Logger.cpp
 *
 * Purpose:
 *		Provides functions for logging data to file and console.
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

#include <boost/log/expressions.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/move/utility_core.hpp>
#include "SysClock.h"
#include "Logger.h"

namespace logging  = boost::log;
namespace src      = boost::log::sources;
namespace keywords = boost::log::keywords;
namespace attrs    = boost::log::attributes;
namespace expr     = boost::log::expressions;

bool Logger::m_DisableLogging = false;
std::string Logger::m_filename = DEFAULT_LOG_NAME;

bool Logger::init(const char* filename) {
	if(filename==0) {
		m_filename = DEFAULT_LOG_NAME;
	} else {
	    m_filename = filename;
	}
	return true;
}

//Defines a global logger initialization routine
BOOST_LOG_GLOBAL_LOGGER_INIT(global_logger, logger_t)
{
    logger_t lg; // Defined in SafeLogger.h

    logging::core::get()->add_global_attribute
        (
        "TimeStamp",
        attrs::utc_clock()
        );

    logging::add_common_attributes();

    char fileName[256];
    snprintf(fileName, 256, "../logs/%s%s-%s", FILE_PATH, SysClock::timeStampStr().c_str(), Logger::m_filename.c_str());

    // Log to file
    logging::add_file_log(
        keywords::file_name = fileName, 
        keywords::auto_flush = true, // logs entries are written immediately                                        
        //keywords::rotation_size = 10 * 1024 * 1024,                   // rotate when 10MB file size                
        //keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0), //example for rotating at a given time, midnight in this case
        //keywords::format = "[%TimeStamp%]: %Message%"  //One way to format

        // Output example:  "000112	[2018-06-25_05:39:33.621094] : <info>    Info  message"
        keywords::format =
	    (
	        expr::stream
	        << std::setw(6) << std::setfill('0') // set size 6 for LineID number / fill with zeros on left side
	        << expr::attr< unsigned int >("LineID")
	        << "\t"
	        << expr::format_date_time<boost::posix_time::ptime>("TimeStamp","[%Y-%m-%d %H:%M:%S.%f]")
	        << "\t: <" << expr::attr<logging::trivial::severity_level>("Severity")
	        << "> \t" << expr::smessage
	    )                                
    );

    // Log to console
    logging::add_console_log(
        std::cout,
        // Output example:  "000112	[05:39:33.621094] : <info>    Info  message"
        keywords::format = 
        (
                expr::stream 
                << expr::format_date_time<boost::posix_time::ptime>("TimeStamp","[%Y-%m-%d %H:%M:%S.%f]")
		        << "\t: <" << expr::attr<logging::trivial::severity_level>("Severity")
		        << "> \t" << expr::smessage
        )
    );

    // Keep only "info" and higher severity logs
    logging::core::get()->set_filter
    (
        expr::attr< logging::trivial::severity_level >("Severity") >= logging::trivial::info
    );

    return lg;
}

void Logger::trace(std::string message, ...) {
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

	BOOST_LOG_SEV(global_logger::get(), logging::trivial::trace) << logBuffer;
}

void Logger::debug(std::string message, ...) {
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

	BOOST_LOG_SEV(global_logger::get(), logging::trivial::debug) << logBuffer;
}

void Logger::info(std::string message, ...) {
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

	BOOST_LOG_SEV(global_logger::get(), logging::trivial::info) << logBuffer;
}

void Logger::warning(std::string message, ...) {
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

	BOOST_LOG_SEV(global_logger::get(), logging::trivial::warning) << logBuffer;
}

void Logger::error(std::string message, ...) {
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

	BOOST_LOG_SEV(global_logger::get(), logging::trivial::error) << logBuffer;
}

void Logger::fatal(std::string message, ...) {
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

	BOOST_LOG_SEV(global_logger::get(), logging::trivial::fatal) << logBuffer;
}

void Logger::DisableLogging()
{
	m_DisableLogging = true;
}
