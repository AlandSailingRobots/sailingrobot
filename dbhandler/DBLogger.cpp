/****************************************************************************************
 *
 * File:
 * 		DBLogger.cpp
 *
 * Purpose:
 *		Logs datalogs to the database in a efficient manor and offloads the work to a
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
	m_logBuffer.reserve(logBufferSize);
}

DBLogger::~DBLogger()
{
	delete m_thread;
}

void DBLogger::startWorkerThread()
{
	m_thread = new std::thread(workerThread, this);
}

void DBLogger::log(VesselStateMsg* msg, double rudder, double sail, int sailServoPosition, int rudderServoPosition,
		double distanceToWaypoint, double bearingToWaypoint, double courseToSteer, bool tack, bool goingStarboard,
		int waypointId, double twd, bool routeStarted)
{
	LogItem item;

	item.m_compassHeading = msg->compassHeading();
	item.m_compassPitch = msg->compassPitch();
	item.m_compassRoll = msg->compassRoll();
	item.m_gpsHasFix = msg->gpsHasFix();
	item.m_gpsOnline = msg->gpsOnline();
	item.m_gpsLat = msg->latitude();
	item.m_gpsLon = msg->longitude();
	item.m_gpsUnixTime = msg->unixTime();
	item.m_gpsSpeed = msg->speed();
	item.m_gpsHeading = msg->gpsHeading();
	item.m_gpsSatellite = msg->gpsSatellite();
	item.m_windDir = msg->windDir();
	item.m_windSpeed = msg->windSpeed();
	item.m_windTemp = msg->windTemp();
	item.m_arduinoPressure = msg->arduinoPressure();
	item.m_arduinoRudder = msg->arduinoPressure();
	item.m_arduinoSheet = msg->arduinoSheet();
	item.m_arduinoBattery = msg->arduinoBattery();
	item.m_rudder = rudder;
	item.m_sail = sail;
	item.m_sailServoPosition = sailServoPosition;
	item.m_rudderServoPosition = rudderServoPosition;
	item.m_distanceToWaypoint = distanceToWaypoint;
	item.m_bearingToWaypoint = bearingToWaypoint;
	item.m_courseToSteer = courseToSteer;
	item.m_tack = tack;
	item.m_goingStarboard = goingStarboard;
	item.m_waypointId = waypointId;
	item.m_twd = twd;
	item.m_routeStarted = routeStarted;

	m_logBuffer.push_back(item);

	// Kick off the worker thread
	if(m_logBuffer.size() == m_bufferSize)
	{
		// Wait for the mutex to be unlocked.
		{
			std::lock_guard<std::mutex> lk(m_mutex);
		}
		// instruct the worker thread to work
		m_cv.notify_one();
	}
}

void DBLogger::workerThread(DBLogger* ptr)
{
	while(true)
	{
		std::unique_lock<std::mutex> lk(ptr->m_mutex);
		ptr->m_cv.wait(lk);

		if(ptr->m_logBuffer.size() > 0)
		{
			ptr->m_dbHandler.insertDataLogs(ptr->m_logBuffer);
		}
	}
}
