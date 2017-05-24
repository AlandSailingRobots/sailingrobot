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
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>


class DBLogger {
public:

	struct Item {
		double 	rudder;
		double 	sail;
		int 	sailServoPosition;
		int 	rudderServoPosition;
		double 	distanceToWaypoint;
		double 	bearingToWaypoint;
		double 	courseToSteer;
		bool 	tack;
		bool 	goingStarboard;
		int 	waypointId;
		double 	twd;
		bool 	routeStarted;
		std::string timestamp_str;
	};

	DBLogger(unsigned int LogBufferSize, DBHandler& dbHandler);
	~DBLogger();

	void startWorkerThread();

	void log(double rudder, double sail, int sailServoPosition, int rudderServoPosition,
			double courseCalculationDistanceToWaypoint, double courseCalculationBearingToWaypoint,
			double courseCalculationCourseToSteer, bool courseCalculationTack, bool courseCalculationGoingStarboard,
			int waypointId, double trueWindDirectionCalc, bool routeStarted,std::string timestamp_str);

	unsigned int bufferSize() { return m_bufferSize; }
private:

	template<typename FloatOrDouble>
	FloatOrDouble setValue(FloatOrDouble value);

	static void workerThread(DBLogger* ptr);

	std::thread* 			m_thread;
	static bool				m_working;
	std::mutex				m_mutex;
	std::condition_variable m_cv;
	DBHandler& 				m_dbHandler;
	unsigned int 			m_bufferSize;
	std::vector<LogItem>* 	m_logBufferFront;
	std::vector<LogItem>* 	m_logBufferBack;
};
