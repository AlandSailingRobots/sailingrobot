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
    vesselHeading(compassHeading), vesselLat(lat), vesselLan(lon), vesselSpeed(gpsSpeed), vesselCourse(gpsCourse)
    { }


    StateMessage(float compassHeading, double lat, double lon, double gpsSpeed, double gpsCourse)
      :Message(MessageType::StateMessage, NodeID::None, NodeID::None),
      vesselHeading(compassHeading), vesselLat(lat), vesselLan(lon), vesselSpeed(gpsSpeed), vesselCourse(gpsCourse)
      { }

      float heading() const {  return vesselHeading; }
      double latitude() const {  return vesselLat; }
      double longitude() const {  return vesselLan; }
      double speed() const { return vesselSpeed; }

      virtual ~StateMessage() { }

      StateMessage(MessageDeserialiser deserialiser)
      :Message(deserialiser)
      {
        if(	!deserialiser.readFloat(vesselHeading) ||
        !deserialiser.readDouble(vesselLat) ||
        !deserialiser.readDouble(vesselLan) ||
        !deserialiser.readDouble(vesselCourse) ||
        !deserialiser.readDouble(vesselSpeed))
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

        serialiser.serialise(vesselHeading);
        serialiser.serialise(vesselLat);
        serialiser.serialise(vesselLan);
        serialiser.serialise(vesselSpeed);
        serialiser.serialise(vesselCourse);
      }

    private:
      float 	vesselHeading;
      double	vesselLat;
      double	vesselLan;
      double	vesselSpeed;
      double  vesselCourse;
    };
