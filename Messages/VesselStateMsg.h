/****************************************************************************************
 *
 * File:
 * 		VesselStateMsg.h
 *
 * Purpose:
 *		A VesselStateMsg contains the state of the vessel at a given time.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


class VesselStateMsg : public Message {
public:
	VesselStateMsg(	NodeID destinationID, NodeID sourceID, int compassHeading, int compassPitch, 
					int compassRoll, bool gpsFix, double lat, double lon, double unixTime, 
					double gpsSpeed, double heading, float windDir, float windSpeed, float windTemp)
		:Message(MessageType::VesselState, sourceID, destinationID),
		m_CompassHeading(compassHeading), m_CompassPitch(compassPitch), m_CompassRoll(compassRoll), 
		m_GPSHasFix(gpsFix), m_GPSLat(lat), m_GPSLon(lon), m_GPSUnixTime(unixTime), m_GPSSpeed(gpsSpeed), 
		m_GPSHeading(heading), m_WindDir(windDir), m_WindSpeed(windSpeed), m_WindTemp(windTemp)
		{ }

	VesselStateMsg(	int compassHeading, int compassPitch, int compassRoll, bool gpsFix, double lat,
					double lon, double unixTime, double gpsSpeed, double heading, float windDir, 
					float windSpeed, float windTemp)
		:Message(MessageType::VesselState, NodeID::None, NodeID::None),
		m_CompassHeading(compassHeading), m_CompassPitch(compassPitch), m_CompassRoll(compassRoll), 
		m_GPSHasFix(gpsFix), m_GPSLat(lat), m_GPSLon(lon), m_GPSUnixTime(unixTime), m_GPSSpeed(gpsSpeed), 
		m_GPSHeading(heading), m_WindDir(windDir), m_WindSpeed(windSpeed), m_WindTemp(windTemp)
		{ }

	virtual ~VesselStateMsg() { }

	int compassHeading() { return m_CompassHeading; }
	int compassPitch() { return m_CompassPitch; }
	int compassRoll() { return m_CompassRoll; }
	
	bool gpsHasFix() { return m_GPSHasFix; }
	double latitude() { return m_GPSLat; }
	double longitude() { return m_GPSLon; }
	double unixTime() { return m_GPSUnixTime; }
	double speed() { return m_GPSSpeed; }
	double gpsHeading() { return m_GPSHeading; }
	
	float windDir() { return m_WindDir; }
	float windSpeed() { return m_WindSpeed; }
	float windTemp() { return m_WindTemp; }
private:
	int 	m_CompassHeading;
	int 	m_CompassPitch;
	int 	m_CompassRoll;
	bool	m_GPSHasFix;
	double	m_GPSLat;
	double	m_GPSLon;
	double	m_GPSUnixTime;
	double	m_GPSSpeed;
	double	m_GPSHeading;
	float	m_WindDir;
	float	m_WindSpeed;
	float 	m_WindTemp;
};
