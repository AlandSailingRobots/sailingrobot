 /****************************************************************************************
 *
 * File:
 * 		Logger.h
 *
 * Purpose:
 *		Provides functions for logging data to file and console. Support for the WRSC2016
 *		format is also included, see Notes. The Logger is a singleton class that can be 
 *		accessed using a number of static functions.
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

#ifndef LOGGER_LOGGER_H_
#define LOGGER_LOGGER_H_

#include <string>
#include <vector>
#include <cstdarg>
#include <mutex>
#include <iostream>
#include <fstream>
#include "models/GPSModel.h"

#define DEFAULT_LOG_NAME			"./sailing-log.log"
#define DEFAULT_LOG_NAME_WRSC		"./wrsc-log.log"

// Uncomment for a WRSC2016 position log file
//#define ENABLE_WRSC_LOGGING

// Provide a error string
#define CLASS_ERROR(...) Logger::log(LogType::ERROR, "%s::%d %s", __PRETTY_FUNCTION__, __LINE__, ##__VA_ARGS__)

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
 	/// Updates the current time.
 	///
 	/// @params seconds 			The unix time in seconds.
 	///
 	/////////////////////////////////////////////////////////////////////////////////////
	static void setTime(unsigned long seconds);

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

 	static void logWRSC(const GPSModel* const gps);

private:
	Logger();
	~Logger();

	void log(std::string message);

	bool createLogFiles(const char* filename = 0);

	void writeBufferedLogs();

	/////////////////////////////////////////////////////////////////////////////////////
 	/// Returns a time stamp in the format YYYY-MM-DD HH:MM:SS.
 	///
 	/// @returns 					A time stamp string.
 	///
 	/////////////////////////////////////////////////////////////////////////////////////
	std::string getTimeStamp();

	/////////////////////////////////////////////////////////////////////////////////////
 	/// Returns a time stamp in the format HHMMSSDD
 	///
 	/// @returns 					A time stamp string.
 	///
 	/////////////////////////////////////////////////////////////////////////////////////
	std::string getTimeStampWRSC();

	static Logger* m_instance;
	static bool m_GPSTimeSet;
	static bool m_DisableLogging;

	std::string m_LogFilePath;
	unsigned long m_LastClockStamp;
	unsigned long m_LastTimeStamp;
	std::ofstream* m_LogFile;
	std::vector<std::string> m_LogBuffer;
	std::mutex m_Mutex;

	#ifdef ENABLE_WRSC_LOGGING
	std::ofstream* m_LogFileWRSC;
	#endif
};

#endif /* LOGGER_LOGGER_H_ */
