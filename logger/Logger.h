/*
 * Logger.h
 *
 *  Created on: Apr 29, 2015
 *      Author: sailbot
 */

#ifndef LOGGER_LOGGER_H_
#define LOGGER_LOGGER_H_

#include <string>
#include <boost/log/trivial.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

BOOST_LOG_INLINE_GLOBAL_LOGGER_DEFAULT(global_logger,
		boost::log::sources::severity_logger_mt<
			boost::log::trivial::severity_level>)

enum class LogType {
 	INFO,
 	WARNING,
 	ERROR
};

#define CLASS_ERROR(...) Logger::log(LogType::ERROR, "%s::%d %s", __PRETTY_FUNCTION__, __LINE__, ##__VA_ARGS__)

class Logger {
public:
	Logger();
	/**
	 * set the name of the file you want to log to
	 * return 1 one success
	 */
	bool init(std::string name);
	/**
	 * log info messages
	 */
	void info(std::string message);
	/**
	 * log error messages
	 */
	void error(std::string message);

	/////////////////////////////////////////////////////////////////////////////////////
 	/// A globally accessable function to log messages to that works exactly like printf.
 	///
 	/// @params logType 			The type of log message
 	/// @params message 			The log message.
 	/// @params ...					A variable list, this allows printf like behaviour
 	/////////////////////////////////////////////////////////////////////////////////////
 	static void log(LogType logType, std::string message, ...);

	virtual ~Logger();

private:
	boost::log::sources::severity_logger_mt< 
		boost::log::trivial::severity_level > m_logger;
};

#endif /* LOGGER_LOGGER_H_ */
