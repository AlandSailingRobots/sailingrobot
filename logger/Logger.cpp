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


namespace logging = boost::log;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace keywords = boost::log::keywords;


Logger::Logger() {
	m_logger = global_logger::get();
}

Logger::~Logger() {
	// TODO Auto-generated destructor stub
}

bool Logger::init(std::string name) {
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
}

void Logger::info(std::string message) {
	BOOST_LOG_SEV(m_logger, logging::trivial::info) << message ;
}
void Logger::error(std::string message) {
	BOOST_LOG_SEV(m_logger, logging::trivial::error) << message;
}

































