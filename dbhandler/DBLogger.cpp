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


bool DBLogger::m_working;


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
	m_working = false;

	// Wait for the mutex to be unlocked.
	{
		std::lock_guard<std::mutex> lk(m_mutex);
	}
	// instruct the worker thread to work
	m_cv.notify_one();

	m_thread->join();

	delete m_thread;
}

void DBLogger::startWorkerThread()
{
	m_working = true;
	m_thread = new std::thread(workerThread, this);
}

void DBLogger::log(const WindStateMsg* msg, double rudder, double sail, int sailServoPosition, int rudderServoPosition,
		double distanceToWaypoint, double bearingToWaypoint, double courseToSteer, bool tack, bool goingStarboard,
		int waypointId, double twd, bool routeStarted,std::string timestamp_str)
{
	LogItem item;

 /*
	*	NOTE: Some code block are been commented out because the properties used
  *	in them are not available in msg any longer.
	*/
	/*item.m_compassHeading = msg->compassHeading();
	item.m_compassPitch = msg->compassPitch();
	item.m_compassRoll = msg->compassRoll();
	item.m_gpsHasFix = msg->gpsHasFix();
	item.m_gpsOnline = msg->gpsOnline();
	item.m_gpsLat = setValue<double>(msg->latitude()); if(item.m_gpsLat == -1) item.m_gpsLat = -1000; //-1 is a valid number for latitude and longitude
	item.m_gpsLon = setValue<double>(msg->longitude()); if(item.m_gpsLon == -1) item.m_gpsLon = -1000;
	item.m_gpsUnixTime = setValue<double>(msg->unixTime());
	item.m_gpsSpeed = setValue<double>(msg->speed());
	item.m_gpsHeading = setValue<double>(msg->gpsHeading());
	item.m_gpsSatellite = msg->gpsSatellite();
	item.m_windDir = setValue<float>(msg->windDir());
	item.m_windSpeed = setValue<float>(msg->windSpeed());
	item.m_windTemp = setValue<float>(msg->windTemp()); if(item.m_windTemp == -1) item.m_windTemp = -1000;
	item.m_arduinoPressure = msg->arduinoPressure();
	item.m_arduinoRudder = msg->arduinoPressure();
	item.m_arduinoSheet = msg->arduinoSheet();
	item.m_arduinoBattery = msg->arduinoBattery();*/
	item.m_rudder = setValue<double>(rudder);
	item.m_sail = setValue<double>(sail);
	item.m_sailServoPosition = sailServoPosition;
	item.m_rudderServoPosition = rudderServoPosition;
	item.m_distanceToWaypoint = setValue<double>(distanceToWaypoint);
	item.m_bearingToWaypoint = setValue<double>(bearingToWaypoint);
	item.m_courseToSteer = setValue<double>(courseToSteer);
	item.m_tack = tack;
	item.m_goingStarboard = goingStarboard;
	item.m_waypointId = waypointId;
	item.m_twd = setValue<double>(twd);
	item.m_routeStarted = routeStarted;
	item.m_timestamp_str = timestamp_str;

	m_logBufferFront->push_back(item);

	// Kick off the worker thread
	if(m_logBufferFront->size() == m_bufferSize)
	{
		std::vector<LogItem>* tmp = m_logBufferBack;
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
	while(m_working)
	{
		std::unique_lock<std::mutex> lk(ptr->m_mutex);
		ptr->m_cv.wait(lk);

		if(ptr->m_logBufferBack->size() > 0)
		{
			ptr->m_dbHandler.insertDataLogs(*ptr->m_logBufferBack);
			ptr->m_logBufferBack->clear();
		}
	}
}
