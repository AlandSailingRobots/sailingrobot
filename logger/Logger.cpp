/*
 * Logger.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: sailbot
 */

#define BOOST_LOG_DYN_LINK

#include "Logger.h"
#include <iostream>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/support/date_time.hpp>

#include <ctime>

#define GET_UNIX_TIME() static_cast<long int>(std::time(0))
#define MAX_LOG_SIZE	256

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace keywords = boost::log::keywords;

Logger* Logger::m_instance = NULL;
bool Logger::m_GPSTimeSet = false;


Logger::Logger() 
	:m_LogFilePath(), m_LastClockStamp(0), m_LastTimeStamp(0)
{
	m_logger = global_logger::get();
}

Logger::~Logger() 
{
	if(m_LogFile != NULL)
	{
		m_LogFile->close();
	}

	if(m_LogFileWRSC != NULL)
	{
		m_LogFileWRSC->close();
	}
}

/*bool Logger::init(std::string name) {
	if (name.compare("") == 0 ) {
		std::cout << "error in name"<<std::endl;
		return false;
	}
	logging::add_file_log(
		keywords::auto_flush = true,
		keywords::file_name = name + "_%N.log",
	    keywords::format = (
	        expr::stream
	        << expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%d-%m-%Y %H:%M:%S.%f ")
	        << ": <" << logging::trivial::severity
	        << "> " << expr::smessage
	    )
	);
	logging::add_common_attributes();

	return true;
}*/

void Logger::info(std::string message) {
	BOOST_LOG_SEV(m_logger, logging::trivial::info) << message ;
}
void Logger::error(std::string message) {
	BOOST_LOG_SEV(m_logger, logging::trivial::error) << message;
}

bool Logger::init(const char* filename)
{
	if(m_instance != NULL)
	{
		m_instance = new Logger();

		// Set the timestamp to the system's unix time.
		m_instance->m_LastClockStamp = GET_UNIX_TIME();

		if(not m_instance->createLogFiles(filename))
		{
			std::cout << "[" <<m_instance->getTimeStamp() << "] Failed to create log files\n";
			delete m_instance;
			m_instance = NULL;
			return false;
		}
	}
	logging::add_file_log(
		keywords::auto_flush = true,
		keywords::file_name = "name_N.log",
	    keywords::format = (
	        expr::stream
	        << expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%d-%m-%Y %H:%M:%S.%f ")
	        << ": <" << logging::trivial::severity
	        << "> " << expr::smessage
	    )
	);
	logging::add_common_attributes();

	// Assumes the logger is already setup and going
	return true;
}

void Logger::setTime(unsigned long seconds)
{
	if(not m_GPSTimeSet && m_instance != NULL)
	{
		m_instance->m_LastTimeStamp = seconds;
		m_instance->m_LastClockStamp = GET_UNIX_TIME();
		m_GPSTimeSet = true;
	}
}

void Logger::log(LogType logType, std::string message, ...)
{
	va_list args;
	char logBuffer[MAX_LOG_SIZE];

	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

    std::string logTypeText;

    switch(logType)
    {
    	case LogType::INFO:
    		logTypeText = "<INFO>";
    		break;
 		case LogType::WARNING:
 			logTypeText = "<WARNING>";
 			break;
 		case LogType::ERROR:
 			logTypeText = "<ERROR>";
 			break;
 		default:
 			logTypeText = "";
    }

    char buff[256];

    snprintf(buff, 256, "[%s] %s %s\n", m_instance->getTimeStamp().c_str(), logTypeText.c_str(), logBuffer);

    printf("%s", buff);

    if(m_instance != NULL && m_instance->m_LogFile != NULL)
    {
    	*m_instance->m_LogFile << buff;
    }
}

void Logger::log(std::string message, ...)
{
	va_list args;
	char logBuffer[MAX_LOG_SIZE];

	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

	std::cout << m_instance->getTimeStamp() << logBuffer << "\n";

	//m_LogFile << m_instance->getTimeStamp() << logBuffer << "\n";

	//TODO - Jordan: Log to file.
}

void Logger::logWRSC(const GPSModel* const gps)
{

}

bool Logger::createLogFiles(const char* filename)
{
	std::string filePath;

	if(filename == 0)
	{
		filePath = DEFAULT_LOG_NAME;
	}
	else
	{
		filePath = filename;
	}

	m_LogFile = new std::ofstream(filePath, std::ios::out | std::ios::trunc);

	#ifdef ENABLE_WRSC_LOGGING
		m_LogFileWRSC = new std::ofstream(DEFAULT_LOG_NAME_WRSC, std::ios::out | std::ios::trunc);

		if(m_LogFile->is_open() && m_LogFileWRSC->is_open())
		{
			return true;
		}
		else
		{
			delete m_LogFile;
			delete m_LogFileWRSC;
		}
	#else
		if(m_LogFile->is_open())
		{
			return true;
		}
		else
		{
			delete m_LogFile;
		}
	#endif

	return false;
}

std::string Logger::getTimeStamp()
{
	unsigned long seconds = 0;
	char buff[90];

	if(m_GPSTimeSet)
	{
		seconds = m_LastTimeStamp + (GET_UNIX_TIME() - m_LastClockStamp);
		m_LastTimeStamp = seconds;
	}
	else
	{
		seconds = GET_UNIX_TIME();
	}

	time_t unix_time = (time_t)seconds;

	strftime(buff, sizeof(buff), "%F %T", gmtime(&unix_time));

	return std::string(buff);
}

std::string Logger::getTimeStampWRSC()
{
	unsigned long seconds = 0;
	char buff[9];

	if(m_GPSTimeSet)
	{
		seconds = m_LastTimeStamp + (GET_UNIX_TIME() - m_LastClockStamp);
		m_LastTimeStamp = seconds;
	}
	else
	{
		seconds = GET_UNIX_TIME();
	}

	time_t unix_time = (time_t)seconds;

	strftime(buff, sizeof(buff), "%H%M%S%d", gmtime(&unix_time));

	return std::string(buff);
}