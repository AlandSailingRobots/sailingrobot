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


#include "ActiveNode.h"
#include <libgpsmm.h>


class GPSDNode : public ActiveNode {
public:
	GPSDNode(MessageBus& msgBus);
	
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
	static void GPSThreadFunc(void* nodePtr);

	bool 	m_Initialised;
	gpsmm* 	m_GpsConnection;

	double	m_Lat;
	double	m_Lon;
	double	m_Speed;
	double	m_Heading;

	int m_currentDay;
};
