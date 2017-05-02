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
  StateMessage (NodeID destinationID, NodeID sourceID, float compassHeading,
    double lat, double lon, double gpsSpeed, double gpsCourse)
    :Message(MessageType::StateMessage, sourceID, destinationID),
    m_VesselHeading(compassHeading), m_VesselLat(lat), m_VesselLon(lon), m_VesselSpeed(gpsSpeed), m_VesselCourse(gpsCourse)
    { }


    StateMessage(float compassHeading, double lat, double lon, double gpsSpeed, double gpsCourse)
      :Message(MessageType::StateMessage, NodeID::None, NodeID::None),
      m_VesselHeading(compassHeading), m_VesselLat(lat), m_VesselLon(lon), m_VesselSpeed(gpsSpeed), m_VesselCourse(gpsCourse)
      { }

      float heading() const {  return m_VesselHeading; }
      double latitude() const {  return m_VesselLat; }
      double longitude() const {  return m_VesselLon; }
      double speed() const { return m_VesselSpeed; }
      double course() const {return m_VesselCourse; }

      virtual ~StateMessage() { }

      StateMessage(MessageDeserialiser deserialiser)
      :Message(deserialiser)
      {
        if(	!deserialiser.readFloat(m_VesselHeading) ||
        !deserialiser.readDouble(m_VesselLat) ||
        !deserialiser.readDouble(m_VesselLon) ||
        !deserialiser.readDouble(m_VesselCourse) ||
        !deserialiser.readDouble(m_VesselSpeed))
        {
          m_valid = false;
        }
      }

      ///----------------------------------------------------------------------------------
      /// Serialises the message into a MessageSerialiser
      ///----------------------------------------------------------------------------------
      virtual void Serialise(MessageSerialiser& serialiser) const
      {
        Message::Serialise(serialiser);

        serialiser.serialise(m_VesselHeading);
        serialiser.serialise(m_VesselLat);
        serialiser.serialise(m_VesselLon);
        serialiser.serialise(m_VesselSpeed);
        serialiser.serialise(m_VesselCourse);
      }

    private:
      float 	m_VesselHeading;
      double	m_VesselLat;
      double	m_VesselLon;
      double	m_VesselSpeed;
      double  m_VesselCourse;
    };
