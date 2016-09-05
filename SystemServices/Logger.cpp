 /****************************************************************************************
 *
 * File:
 * 		Logger.cpp
 *
 * Purpose:
 *		Provides functions for logging data to file and console. Support for the WRSC2016
 *		format is also included, see Notes.
 *
 *
 * Developer Notes:
 *		WRSC2016 Logging:
 *			The WRSC 2016 logging format is as follows:
 *
 *				“hhmmssdd	Lat*10^7		Lon*10^7”
 *
 *			The GPS coordinates need to be in the format of degress in decimals, e.g:
 *			60.3456. When WRSC logging is enabled a separate log file is generated
 *			containing only this data and is located alongside the program.
 *
 ***************************************************************************************/


#include "Logger.h"
#include <iostream>
#include "../utility/SysClock.h"


#define MAX_LOG_SIZE	256*2
#define MAX_MSG_BUFFER 100

std::string 				Logger::m_LogFilePath;
std::ofstream 				Logger::m_LogFile;
std::vector<std::string> 	Logger::m_LogBuffer;
#ifndef _WIN32
std::mutex 					Logger::m_Mutex;
#endif
bool Logger::m_DisableLogging = false;

#define ENABLE_WRSC_LOGGING
#ifdef ENABLE_WRSC_LOGGING
static std::ofstream 			m_LogFileWRSC;
#endif

#define LOG_FOLDER				"logFolder/"


bool Logger::init(const char* filename)
{
	if(m_DisableLogging) { return true; }

	if(not createLogFiles(filename))
	{
		std::cout << "[" << SysClock::timeStampStr().c_str() << "] Failed to create log files\n";
		return false;
	}

	// Assumes the logger is already setup and going
	return true;
}

void Logger::DisableLogging()
{
	m_DisableLogging = true;
}

void Logger::shutdown()
{
	if(m_DisableLogging) { return; }

	if(m_LogFile.is_open())
	{
		m_LogFile.close();
	}

	#ifdef ENABLE_WRSC_LOGGING
	if(m_LogFileWRSC.is_open())
	{
		m_LogFileWRSC.close();
	}
	#endif
}

void Logger::log(std::string message)
{
	if(m_DisableLogging) { return; }

	#ifndef _WIN32
	m_Mutex.lock();
	#endif
	
	if(m_LogFile.is_open())
	{
		m_LogFile << message.c_str();
		m_LogFile.flush();
	}
	else
	{
		if(m_LogBuffer.size() < MAX_MSG_BUFFER)
		{
			m_LogBuffer.push_back(message);
		}
		else
		{
			//printf(" === NO ROOM IN BUFFER FOR MORE MESSAGES ===\n");
		}
	}
	#ifndef _WIN32
	m_Mutex.unlock();
	#endif
	}

void Logger::info(std::string message, ...)
{
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

    char buff[256];
		int millis = SysClock::millis();

		if (millis<10)
      snprintf(buff, 256, "[%s:00%d] <info>\t %s\n", SysClock::timeStampStr().c_str(),millis, logBuffer);
	  else if(millis<100)
      snprintf(buff, 256, "[%s:0%d] <info>\t %s\n", SysClock::timeStampStr().c_str(),millis, logBuffer);
		else
      snprintf(buff, 256, "[%s:%d] <info>\t %s\n", SysClock::timeStampStr().c_str(),millis, logBuffer);

    printf("%s", buff);
    log(buff);
}

void Logger::error(std::string message, ...)
{
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

    char buff[256];

    snprintf(buff, 256, "[%s:%d] <error>\t %s\n", SysClock::timeStampStr().c_str(), SysClock::millis(), logBuffer);

    printf("%s", buff);
    log(buff);
}

void Logger::warning(std::string message, ...)
{
	va_list args;

	if(m_DisableLogging) { return; }

	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

    char buff[256];

    snprintf(buff, 256, "[%s:%d] <warning>\t %s\n", SysClock::timeStampStr().c_str(), SysClock::millis(), logBuffer);

    printf("%s", buff);
    log(buff);
}

void Logger::logWRSC(double latitude, double longitude)
{
	if(m_DisableLogging) { return; }

	#ifdef ENABLE_WRSC_LOGGING
	// Flag so that we only issue one error about the log file not existing
	static bool errorMsgUsge = false;

	#ifndef _WIN32
	m_Mutex.lock();
	#endif
	if(m_LogFileWRSC.is_open())
	{
		char logBuffer[MAX_LOG_SIZE];
		snprintf(logBuffer, MAX_LOG_SIZE, "%s%d, %d, %d\n", SysClock::hh_mm_ss().c_str(), SysClock::day(),
															(int)(latitude*10000000),
															(int)(longitude*10000000));
		m_LogFileWRSC << logBuffer;
		m_LogFileWRSC.flush();
	}
	else if(not errorMsgUsge)
	{
		Logger::error("WRSC log file is not open! Cannot log WRSC messages");
		errorMsgUsge = true;
	}
	#ifndef _WIN32
	m_Mutex.unlock();
	#endif
	#endif
}

bool Logger::createLogFiles(const char* filename)
{
	if(m_DisableLogging) { return true; }

	char fileName[256];

	if(filename == 0)
	{
		snprintf(fileName, 256, "%s%s-%s", LOG_FOLDER, SysClock::hh_mm_ss().c_str(), DEFAULT_LOG_NAME);
	}
	else
	{
		snprintf(fileName, 256, "%s%s-%s", LOG_FOLDER, SysClock::hh_mm_ss().c_str(), filename);
	}

	m_LogFile.open(fileName, std::ios::out | std::ios::trunc);

	#ifdef ENABLE_WRSC_LOGGING
		char wrscFileName[256];
		snprintf(wrscFileName, 256, "%s%s-%s", LOG_FOLDER, SysClock::hh_mm_ss().c_str(), DEFAULT_LOG_NAME_WRSC);
		m_LogFileWRSC.open(wrscFileName, std::ios::out | std::ios::trunc);

		if(m_LogFile.is_open() && m_LogFileWRSC.is_open())
		{
			Logger::info("Log files %s, %s have been created", fileName, wrscFileName);
			writeBufferedLogs();
			return true;
		}
		else
		{
			Logger::info("Failed to create Log files %s, %s", fileName, wrscFileName);
		}
	#else
		if(m_LogFile.is_open())
		{
			Logger::info("Log file %s has been created", fileName);
			writeBufferedLogs();
			return true;
		}
		else
		{
			Logger::info("Failed to create Log file %s", fileName);
		}
	#endif

	return false;
}

void Logger::writeBufferedLogs()
{
	if(m_DisableLogging) { return; }

	for(std::string log : m_LogBuffer)
	{
		m_LogFile << log.c_str();
	}
	m_LogFile.flush();
}
