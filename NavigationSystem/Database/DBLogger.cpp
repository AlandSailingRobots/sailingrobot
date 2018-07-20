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
 *		Refactored 2018-07 by Kåre Hampf <khampf@users.sourceforge.net>
 *
 ***************************************************************************************/

#include "DBLogger.h"

/**
 * Constructor
 * @param logBufferItems	Number of items to buffer before send
 * @param dbHandler
 */
DBLogger::DBLogger(unsigned int logBufferItems, DBHandler& dbHandler)
    : m_dbHandler(dbHandler), m_bufferItems(logBufferItems) {
    m_runFlag = true;
    m_logFifoIn = new std::queue<LogItem>();
}

/**
 * Destructor
 */
DBLogger::~DBLogger() {
    // .store(false);

    // Wait for the mutex to be unlocked.
    { std::unique_lock<std::mutex> lock(m_logFifoMutex); }

    // instruct the worker thread to work and empty the queue
    m_signal.notify_all();

    if (m_workerThread->joinable())
        m_workerThread->join();
    delete m_workerThread;
}

/**
 * Add LogItem to input buffer and wake up the worker thread
 * @param item
 */
void DBLogger::log(LogItem& item) {
    std::unique_lock<std::mutex> lock(m_logFifoMutex);
    m_logFifoIn->push(item);  // Maybe emplace? Check std::move one level up
    m_signal.notify_one();
}

unsigned int DBLogger::getLogItems() {
	return m_bufferItems;
}

/*
 * This should be left here as it is brilliant!
 *
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

/**
 * Starts a single worker thread
 */
void DBLogger::startWorkerThread() {
    m_workerThread = new std::thread(workerThread, this);
}

/**
 * Worker thread
 * @param ptr to DBLogger for accessing mutexes and whatnots
 */
void DBLogger::workerThread(DBLogger* ptr) {
    std::queue<LogItem> logFifoOut;
    while (ptr->m_runFlag) {
        {
            // Get a lock, then release it and wait for signal
            std::unique_lock<std::mutex> lock(ptr->m_logFifoMutex);
            while ((*ptr->m_logFifoIn).empty()) {  // avoid spurious wake-ups
                ptr->m_signal.wait(lock);
            }
        }
        while (!(*ptr->m_logFifoIn).empty()) {
            // move everything from the input FIFO to the output FIFO
            std::unique_lock<std::mutex> lock(ptr->m_logFifoMutex);
            logFifoOut.emplace(
                std::move((*ptr->m_logFifoIn).front()));  // emplace should move but here we are
            (*ptr->m_logFifoIn).pop();
            lock.unlock();
        }
        if (logFifoOut.size() >= ptr->m_bufferItems) {
            // store the contents of the output FIFO in the database
            while (!logFifoOut.empty()) {  // an if-clause would do fine but DBHandler might fail
                ptr->m_dbHandler.insertDataLogs(logFifoOut);
            }
        }
    }
}
