/****************************************************************************************
 *
 * File:
 * 		DBLogger.cpp
 *
 * Purpose:
 *		Logs dataLogs to the database in a efficient manor and offloads the work to a
 *		worker thread.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#include "DBLogger.h"


DBLogger::DBLogger(unsigned int logBufferSize, DBHandler& dbHandler)
	:m_dbHandler(dbHandler), m_bufferSize(logBufferSize)
{
	m_logBufferFront = new std::vector<LogItem>();
	m_logBufferFront->reserve(logBufferSize);

	m_logBufferBack = new std::vector<LogItem>();
	m_logBufferBack->reserve(logBufferSize);
	m_working = false;
}

DBLogger::~DBLogger()
{
	m_working.store(false);

	// Wait for the mutex to be unlocked.
	{
		std::lock_guard<std::mutex> lk(m_mutex);
	}
	// instruct the worker thread to work
	m_cv.notify_one();

	//m_thread->join();

	//delete m_thread;
}

void DBLogger::startWorkerThread()
{
	m_working.store(true);
	m_thread = new std::thread(workerThread, this);
}

void DBLogger::log(LogItem& item)
{
	m_logBufferFront->push_back(item);

	// Kick off the worker thread
	if(m_logBufferFront->size() >= m_bufferSize)
	{
		std::vector<LogItem>* tmp = m_logBufferBack;
		std::cout << "/* DBLOGGER Queue charge en logitem */" << '\n';
		m_logBufferBack = m_logBufferFront;
		m_logBufferFront = tmp;

		// Wait for the mutex to be unlocked.
		{
			std::lock_guard<std::mutex> lk(m_mutex);
		}
		// instruct the worker thread to work
		m_cv.notify_one();
	}
}

template<typename FloatOrDouble>
FloatOrDouble DBLogger::setValue(FloatOrDouble value) //Function to check if value is NaN before setting the value
{
	if(value != value) //If value is NaN
	{
		return -1;
	}

	return value;
}

void DBLogger::workerThread(DBLogger* ptr)
{
	while(ptr->m_working.load() == true)
	{
		std::unique_lock<std::mutex> lk(ptr->m_mutex);
		ptr->m_cv.wait(lk);
		if(ptr->m_logBufferBack->size() > 0)
		{
			std::cout << "/* DBLOGGER Envoie de la queue charge en item */" << '\n';
			ptr->m_dbHandler.insertDataLogs(*ptr->m_logBufferBack);
			ptr->m_logBufferBack->clear();
		}
	}
}
