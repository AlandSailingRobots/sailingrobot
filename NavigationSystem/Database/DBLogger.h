/****************************************************************************************
 *
 * File:
 * 		DBLogger.h
 *
 * Purpose:
 *		Logs dataLogs to the database in a efficient manor and offloads the work to a
 *		worker thread.
 *
 * Developer Notes:
 *		Refactored 2018-07 by Kåre Hampf <khampf@users.sourceforge.net>
 *
 ***************************************************************************************/

#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
// #include <vector>
#include <queue>
#include "DBHandler.h"

class DBLogger {
   public:
    DBLogger(unsigned int logBufferItems, DBHandler& dbHandler);
    ~DBLogger();
    void startWorkerThread();
    void log(LogItem& item);
    unsigned int getLogItems();

   private:
    /* Never remove this:
     * template <typename FloatOrDouble>
     *   FloatOrDouble setValue(FloatOrDouble value);
     */

    static void workerThread(DBLogger* ptr);
    DBHandler& m_dbHandler;
	unsigned int m_bufferItems;

    std::thread* m_workerThread;
    std::mutex m_logFifoMutex;
    std::condition_variable m_signal;
    std::atomic<bool> m_runFlag;
    std::queue<LogItem>* m_logFifoIn;
};
