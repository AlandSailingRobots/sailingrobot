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
    double lat, double lon, double gpsSpeed)
    :Message(MessageType::StateMessage, sourceID, destinationID),
    vesselHeading(compassHeading), vesselLat(lat), vesselLan(lon), vesselSpeed(gpsSpeed)
    { }


    StateMessage(float compassHeading, double lat, double lon, double gpsSpeed)
      :Message(MessageType::StateMessage, NodeID::None, NodeID::None),
      vesselHeading(compassHeading), vesselLat(lat), vesselLan(lon), vesselSpeed(gpsSpeed)
      { }

      float heading() { return vesselHeading; }
      double latitude() { return vesselLat; }
      double longitude() { return vesselLan; }
      double speed() { return vesselSpeed; }

      virtual ~StateMessage() { }

      StateMessage(MessageDeserialiser deserialiser)
      :Message(deserialiser)
      {
        if(	!deserialiser.readFloat(vesselHeading) ||
        !deserialiser.readDouble(vesselLat) ||
        !deserialiser.readDouble(vesselLan) ||
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
      }

    private:
      float 	vesselHeading;
      double	vesselLat;
      double	vesselLan;
      double	vesselSpeed;
    };
