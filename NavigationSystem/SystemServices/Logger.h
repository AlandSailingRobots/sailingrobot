 /****************************************************************************************
 *
 * File:
 * 		Logger.h
 *
 * Purpose:
 *		Provides functions for logging data to file and console. Support for the WRSC2016
 *		format is also included, see Notes.
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


#pragma once


#include <string>
#include <vector>
#include <cstdarg>
#ifndef _WIN32
#include <mutex>
#endif
#include <iostream>
#include <fstream>
#include "sys/stat.h"

#define DEFAULT_LOG_NAME			"sailing-log.log"
#define DEFAULT_LOG_NAME_WRSC		"wrsc-log.log"
#define FILE_PATH 						"../logs/"
// Uncomment for a WRSC2016 position log file
//#define ENABLE_WRSC_LOGGING


class Logger {
public:
	/////////////////////////////////////////////////////////////////////////////////////
	/// Initialises the singleton logger system, returns false if it is unable to
	/// generate a log file.
	///
	/// @params logType 			The type of log message, if this paramter is not
	///								provided then a default name is used.
	///
	/////////////////////////////////////////////////////////////////////////////////////
	static bool init(const char* filename = 0);

	/// SHOULD ONLY BE USED FOR UNIT TESTS!
	static void DisableLogging();

	static void shutdown();

	/////////////////////////////////////////////////////////////////////////////////////
	/// A globally accessable function to log messages to that works exactly like printf.
	///
	/// @params message 			The log message.
	/// @params ...					A variable list, this allows printf like behaviour
	///
	/////////////////////////////////////////////////////////////////////////////////////
	static void info(std::string message, ...);
	static void error(std::string message, ...);
	static void warning(std::string message, ...);

	static void logWRSC(double latitude, double longitude);

private:
	static void log(std::string message);

	static bool createLogFiles(const char* filename = 0);

	static void writeBufferedLogs();

	static std::string 				m_LogFilePath;
	static std::ofstream 			m_LogFile;
	static std::vector<std::string> m_LogBuffer;
	#ifndef _WIN32
	static std::mutex 				m_Mutex;
	#endif
	static bool						m_DisableLogging;

	#ifdef ENABLE_WRSC_LOGGING
	static std::ofstream 			m_LogFileWRSC;
	#endif
};
