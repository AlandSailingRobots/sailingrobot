/****************************************************************************************
 *
 * File:
 * 		GPSDNode.h
 *
 * Purpose:
 *		A GPSD node that uses the GPSD library to provide GPS data to the message bus.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once


#include "MessageBus/ActiveNode.h"
#include "DataBase/DBHandler.h"
#include <libgpsmm.h>


class GPSDNode : public ActiveNode {
public:
	GPSDNode(MessageBus& msgBus, DBHandler& dbhandler);

	virtual ~GPSDNode();


	///----------------------------------------------------------------------------------
	/// Initialises a connection gpsd.
	///
	///----------------------------------------------------------------------------------
	bool init();

	///----------------------------------------------------------------------------------
	/// Currently doesn't process any messages.
	///
	///----------------------------------------------------------------------------------
	void processMessage(const Message* msgPtr);

	///----------------------------------------------------------------------------------
	/// Update values from the database as the loop time of the thread and others parameters
	///----------------------------------------------------------------------------------
	void updateConfigsFromDB();

	///----------------------------------------------------------------------------------
 	/// This function starts the GPS thread
 	///
 	///----------------------------------------------------------------------------------
	void start();
private:

	///----------------------------------------------------------------------------------
	/// Attempts to retrieve data from the GPS and then sends a GPSDataMessage every x
	/// milliseconds, see GPS_SENSOR_SLEEP_MS in GPSDNode.cpp
	///
	///----------------------------------------------------------------------------------
	static void GPSThreadFunc(ActiveNode* nodePtr);

	bool 	m_Initialised;
	gpsmm* 	m_GpsConnection;

	double	m_Lat;		// North(+) or South(-) [0-90]
	double	m_Lon;		// East(+) or West(-)  [0-180]
	double	m_Speed;	//NOTE : m/s
	double	m_Course;	//NOTE : degree [0-360[

	int m_currentDay;
	double m_LoopTime;	// in seconds (ex: 0.5 s)
	DBHandler& m_db;

	const int GPS_TIMEOUT_MICRO_SECS = 50000000;

};
