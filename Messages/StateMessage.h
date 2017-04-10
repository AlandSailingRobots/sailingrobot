/****************************************************************************************
 *
 * File:
 * 		StateMessage.h
 *
 * Purpose:
 *		A StateMessage contains the state of the StateEstimationNode at a given time.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "Message.h"

class StateMessage : public Message {
public:
  StateMessage (NodeID destinationID, NodeID sourceID, int compassHeading, int compassPitch,
					int compassRoll, bool gpsFix, bool gpsOnline, double lat, double lon, double unixTime,
					double gpsSpeed, int gpsSatellite)
		:Message(MessageType::StateMessage, sourceID, destinationID),
		m_CompassHeading(compassHeading), m_CompassPitch(compassPitch), m_CompassRoll(compassRoll),
		m_GPSHasFix(gpsFix), m_GPSOnline(gpsOnline), m_GPSLat(lat), m_GPSLon(lon), m_GPSUnixTime(unixTime), m_GPSSpeed(gpsSpeed),
		m_GPSHeading(heading), m_GPSSatellite(gpsSatellite)
		{ }
