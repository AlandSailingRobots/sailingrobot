/**************************************************************************
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
    : m_dbHandler(dbHandler), m_bufferSize(logBufferSize) {
/*    m_logBufferFront = new std::vector<LogItem>();
    m_logBufferFront->reserve(logBufferSize);

    m_logBufferBack = new std::vector<LogItem>();
    m_logBufferBack->reserve(logBufferSize);*/
	m_runFlag = true;
    m_logFifoIn = new std::queue<LogItem>();
}

DBLogger::~DBLogger() {
    // .store(false);

    // Wait for the mutex to be unlocked.
    { std::lock_guard<std::mutex> lk(m_logFifoMutex); }

    // instruct the worker thread to work
    m_signal.notify_all();

	if (m_workerThread->joinable()) m_workerThread->join();
    delete m_workerThread;
}

void DBLogger::log(LogItem& item) {
/*	m_logBufferFront->push_back(item);

	// Kick off the worker thread
	if (m_logBufferFront->size() >= m_bufferSize) {
		std::vector<LogItem>* tmp = m_logBufferBack;
		m_logBufferBack = m_logBufferFront;
		m_logBufferFront = tmp;

		// Wait for the mutex to be unlocked.
		{ std::lock_guard<std::mutex> lk(m_logFifoMutex); }
		// instruct the worker thread to work
		m_cv.notify_one();
	}*/
	std::unique_lock<std::mutex> lock(m_logFifoMutex);
	m_logFifoIn->push(item);  // Maybe emplace? Check std::move one level up
	m_hasWork = true;
	m_signal.notify_one();
}
/*
template <typename FloatOrDouble>
FloatOrDouble DBLogger::setValue(
    FloatOrDouble value)  // Function to check if value is NaN before setting the value
{
    if (value != value)  // If value is NaN
    {
        return -1;
    }

    return value;
}*/

void DBLogger::startWorkerThread() {
    // m_working.store(true);
    m_workerThread = new std::thread(workerThread, this);
}

void DBLogger::workerThread(DBLogger* ptr) {
	std::queue<LogItem> logFifoOut;
	while (ptr->m_runFlag) {
		{
			std::unique_lock<std::mutex> lock(ptr->m_logFifoMutex);
			while ((*ptr->m_logFifoIn).empty()) { // avoid spurious wake-ups
				ptr->m_signal.wait(lock);
			}
		}
		while (!(*ptr->m_logFifoIn).empty()) {
			std::unique_lock<std::mutex> lock(ptr->m_logFifoMutex);
			logFifoOut.emplace(std::move((*ptr->m_logFifoIn).front())); // emplace should move but here we are
			(*ptr->m_logFifoIn).pop();
			lock.unlock();
		}
		while (!logFifoOut.empty()) {   // an if-clause would do fine but DBHandler might fail
			ptr->m_dbHandler.insertDataLogs(logFifoOut);
		}
	}
/*    while (ptr->m_working.load() == true) {
        std::unique_lock<std::mutex> locker(ptr->m_logFifoMutex);
        ptr->m_cv.wait(locker);
        if (ptr->m_logBufferBack->size() > 0) {
            ptr->m_dbHandler.insertDataLogs(*ptr->m_logBufferBack);
            ptr->m_logBufferBack->clear();
        }
    }*/
}
