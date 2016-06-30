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
	virtual ~Logger();

private:
	boost::log::sources::severity_logger_mt< 
		boost::log::trivial::severity_level > m_logger;
};

#endif /* LOGGER_LOGGER_H_ */
