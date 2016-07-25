/*
 * Logger.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: sailbot
 */

#include "Logger.h"
#include <iostream>
#include "utility/SysClock.h"

#define MAX_LOG_SIZE	256
#define MAX_MSG_BUFFER 100

Logger* Logger::m_instance = NULL;


Logger::Logger() 
	:m_LogFilePath()
{
	
}

Logger::~Logger() 
{
	if(m_LogFile != NULL)
	{
		m_LogFile->close();
	}

	#ifdef ENABLE_WRSC_LOGGING
	if(m_LogFileWRSC != NULL)
	{
		m_LogFileWRSC->close();
	}
	#endif
}

bool Logger::init(const char* filename)
{
	if(m_instance == NULL)
	{
		m_instance = new Logger();

		if(not m_instance->createLogFiles(filename))
		{
			std::cout << "[" << SysClock::timeStampStr().c_str() << "] Failed to create log files\n";
			delete m_instance;
			m_instance = NULL;
			return false;
		}
	}

	// Assumes the logger is already setup and going
	return true;
}

void Logger::shutdown()
{
	if(m_instance != NULL)
	{
		delete m_instance;
	}
}

void Logger::log(std::string message)
{
	m_Mutex.lock();
	if(m_LogFile->is_open())
	{
		(*m_LogFile) << message.c_str();
		m_LogFile->flush();
	}
	else
	{
		if(m_LogBuffer.size() < MAX_MSG_BUFFER)
		{
			m_LogBuffer.push_back(message);
		}
		else
		{
			printf(" === NO ROOM IN BUFFER FOR MORE MESSAGES ===\n");
		}
	}
	m_Mutex.unlock();
}

void Logger::info(std::string message, ...)
{
	va_list args;
	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

    char buff[256];

    snprintf(buff, 256, "[%s] <info>\t %s\n", SysClock::timeStampStr().c_str(), logBuffer);

    printf("%s", buff);

    if(m_instance != NULL)
    {
    	m_instance->log(buff);
    }
}

void Logger::error(std::string message, ...)
{
	va_list args;
	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

    char buff[256];

    snprintf(buff, 256, "[%s] <error>\t %s\n", SysClock::timeStampStr().c_str(), logBuffer);

    printf("%s", buff);
    if(m_instance != NULL)
    {
    	m_instance->log(buff);
    }
}

void Logger::warning(std::string message, ...)
{
	va_list args;
	char logBuffer[MAX_LOG_SIZE];
	// Put together the formatted string
	va_start(args, message);
    vsnprintf(logBuffer, MAX_LOG_SIZE, message.c_str(), args);
    va_end(args);

    char buff[256];

    snprintf(buff, 256, "[%s] <warning>\t %s\n", SysClock::timeStampStr().c_str(), logBuffer);

    printf("%s", buff);
    if(m_instance != NULL)
    {
    	m_instance->log(buff);
    };
}

void Logger::logWRSC(const GPSModel* const gps)
{
	#ifdef ENABLE_WRSC_LOGGING
	// Flag so that we only issue one error about the log file not existing
	static bool errorMsgUsge = false;

	m_Mutex.lock();
	if(m_instance != NULL)
	{
		if(m_instance->m_LogFileWRSC->is_open())
		{
			char logBuffer[MAX_LOG_SIZE];
			snprintf(logBuffer, MAX_LOG_SIZE, "%s%d, %d, %d\n", SysClock::hh_mm_ss().c_str(), SysClock::day(),
																(int)(gps->positionModel.latitude*10000000), 
																(int)(gps->positionModel.longitude*10000000));
			*(m_instance->m_LogFileWRSC) << logBuffer;
			m_instance->m_LogFileWRSC->flush();
		}
	}
	else if(not errorMsgUsge)
	{
		Logger::error("WRSC log file is not open! Cannot log WRSC messages");
		errorMsgUsge = true;
	}
	m_Mutex.unlock();
	#endif
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
			Logger::info("Log files %s, %s have been created", filePath.c_str(), DEFAULT_LOG_NAME_WRSC);
			writeBufferedLogs();
			return true;
		}
		else
		{
			Logger::info("Failed to create Log files %s, %s", filePath.c_str(), DEFAULT_LOG_NAME_WRSC);
			delete m_LogFile;
			delete m_LogFileWRSC;
		}
	#else
		if(m_LogFile->is_open())
		{
			Logger::info("Log file %s has been created", filePath.c_str());
			writeBufferedLogs();
			return true;
		}
		else
		{
			Logger::info("Failed to create Log file %s", filePath.c_str());
			delete m_LogFile;
		}
	#endif

	return false;
}

void Logger::writeBufferedLogs()
{
	for(std::string log : m_LogBuffer)
	{
		*m_LogFile << log.c_str();
	} 
	m_LogFile->flush();
}
