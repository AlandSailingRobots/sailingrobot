/****************************************************************************************
 *
 * File:
 * 		GPSDataMsg.h
 *
 * Purpose:
 *		Contains GPS Data.
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"


enum class GPSMode
{
	NoUpdate = 0,	// Indicates that a update hasn't occured yet
	NoFix,			// Indicates that there is no GPS fix
	LatLonOk,		// Indicates that the latitude/longitude data is good
	LatLonAltOk		// Indicates that the latitude/longitude/altitude data is good
};


class GPSDataMsg : public Message {
public:
	GPSDataMsg(	NodeID destinationID, NodeID sourceID, bool hasFix, bool online, double lat,
				double lon, double unixTime, double speed, double heading, int satCount, GPSMode mode)

		:Message(MessageType::GPSData, sourceID, destinationID), m_HasFix(hasFix), m_Online(online),
		 m_Lat(lat), m_Lon(lon), m_UnixTime(unixTime), m_Speed(speed), m_Heading(heading),
		 m_SatCount(satCount), m_Mode(mode)
	{ }

	GPSDataMsg(	bool hasFix, bool online, double lat, double lon, double unixTime, double speed,
				double heading, int satCount, GPSMode mode)

		:Message(MessageType::GPSData, NodeID::None, NodeID::None), m_HasFix(hasFix), m_Online(online),
		 m_Lat(lat), m_Lon(lon), m_UnixTime(unixTime), m_Speed(speed), m_Heading(heading),
		 m_SatCount(satCount), m_Mode(mode)
	{ }

	virtual ~GPSDataMsg() { }

	bool hasFix() { return m_HasFix; }
	bool gpsOnline() { return m_Online; }
	double latitude() { return m_Lat; }
	double longitude() { return m_Lon; }
	double unixTime() { return m_UnixTime; }
	double speed() { return m_Speed; }
	double heading() { return m_Heading; }
	int satelliteCount() { return m_SatCount; }
	GPSMode gpsMode() { return m_Mode; }

private:
	bool	m_HasFix;
	bool	m_Online;
	double	m_Lat;
	double	m_Lon;
	double	m_UnixTime;
	double	m_Speed;
	double	m_Heading;
	int		m_SatCount;
	GPSMode	m_Mode;
};
