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

    StateMessage(	int compassHeading, int compassPitch, int compassRoll, bool gpsFix, bool gpsOnline, double lat,
      double lon, double unixTime, double gpsSpeed, int gpsSatellite)
      :Message(MessageType::VesselState, NodeID::None, NodeID::None),
      m_CompassHeading(compassHeading), m_CompassPitch(compassPitch), m_CompassRoll(compassRoll),
      m_GPSHasFix(gpsFix), m_GPSOnline(gpsOnline), m_GPSLat(lat), m_GPSLon(lon), m_GPSUnixTime(unixTime), m_GPSSpeed(gpsSpeed),
      m_GPSHeading(heading), m_GPSSatellite(gpsSatellite)
      { }

      StateMessage(MessageDeserialiser deserialiser)
      :Message(deserialiser)
      {
        if(	!deserialiser.readInt(m_CompassHeading) ||
        !deserialiser.readInt(m_CompassPitch) ||
        !deserialiser.readInt(m_CompassRoll) ||
        !deserialiser.readBool(m_GPSHasFix) ||
        !deserialiser.readBool(m_GPSOnline) ||
        !deserialiser.readDouble(m_GPSLat) ||
        !deserialiser.readDouble(m_GPSLon) ||
        !deserialiser.readDouble(m_GPSUnixTime) ||
        !deserialiser.readDouble(m_GPSSpeed) ||
        !deserialiser.readDouble(m_GPSHeading) ||
        !deserialiser.readInt(m_GPSSatellite)
        {
          m_valid = false;
        }
      }

      virtual ~StateMessage() { }

      int compassHeading() { return m_CompassHeading; }
      int compassPitch() { return m_CompassPitch; }
      int compassRoll() { return m_CompassRoll; }

      bool gpsHasFix() { return m_GPSHasFix; }
      bool gpsOnline() { return m_GPSOnline; }
      double latitude() { return m_GPSLat; }
      double longitude() { return m_GPSLon; }
      double unixTime() { return m_GPSUnixTime; }
      double speed() { return m_GPSSpeed; }
      double gpsHeading() { return m_GPSHeading; }
      int gpsSatellite() { return m_GPSSatellite; }

      ///----------------------------------------------------------------------------------
      /// Serialises the message into a MessageSerialiser
      ///----------------------------------------------------------------------------------
      virtual void Serialise(MessageSerialiser& serialiser) const
      {
        Message::Serialise(serialiser);

        serialiser.serialise(m_CompassHeading);
        serialiser.serialise(m_CompassPitch);
        serialiser.serialise(m_CompassRoll);
        serialiser.serialise(m_GPSHasFix);
        serialiser.serialise(m_GPSOnline);
        serialiser.serialise(m_GPSLat);
        serialiser.serialise(m_GPSLon);
        serialiser.serialise(m_GPSUnixTime);
        serialiser.serialise(m_GPSSpeed);
        serialiser.serialise(m_GPSHeading);
        serialiser.serialise(m_GPSSatellite);
      }

    private:
      int 	m_CompassHeading;
      int 	m_CompassPitch;
      int 	m_CompassRoll;
      bool	m_GPSHasFix;
      bool	m_GPSOnline;
      double	m_GPSLat;
      double	m_GPSLon;
      double	m_GPSUnixTime;
      double	m_GPSSpeed;
      double	m_GPSHeading;
      int		m_GPSSatellite;
    };
