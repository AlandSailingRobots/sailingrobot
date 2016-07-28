/****************************************************************************************
 *
 * File:
 * 		DBLogger.h
 *
 * Purpose:
 *		Logs datalogs to the database in a efficient manor and offloads the work to a
 *		worker thread.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#pragma once


#include "DBHandler.h"
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>


class DBLogger {
public:
	DBLogger(unsigned int LogBufferSize, DBHandler& dbHandler);
	~DBLogger();

	void startWorkerThread();

	void log(VesselStateMsg* msg, double rudder, double sail, int sailServoPosition, int rudderServoPosition,
			double courseCalculationDistanceToWaypoint, double courseCalculationBearingToWaypoint,
			double courseCalculationCourseToSteer, bool courseCalculationTack, bool courseCalculationGoingStarboard,
			int waypointId, double trueWindDirectionCalc, bool routeStarted);

	unsigned int bufferSize() { return m_bufferSize; }
private:

	static void workerThread(DBLogger* ptr);

	std::thread* 			m_thread;
	std::mutex				m_mutex;
	std::condition_variable m_cv;
	DBHandler& 				m_dbHandler;
	unsigned int 			m_bufferSize;
	std::vector<LogItem>* 	m_logBufferFront;
	std::vector<LogItem>* 	m_logBufferBack;
};
