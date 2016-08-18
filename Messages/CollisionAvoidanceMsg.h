/****************************************************************************************
 *
 * File:
 * 		CollisionAvoidanceMsg.h
 *
 * Purpose:
 *		Contains 3 waypoints.
 *
 * Developer Notes:
 *
 ***************************************************************************************/


#pragma once

#include "Message.h"


class CollisionAvoidanceMsg : public Message {
public:
    CollisionAvoidanceMsg(NodeID destinationID, NodeID sourceID,
                          double startWaypointLon,
                          double startWaypointLat,
                          double midWaypointLon,
                          double midWaypointLat,
                          double endWaypointLon,
                          double endWaypointLat)
            :Message(MessageType::CollisionAvoidance, sourceID, destinationID),
             m_startWaypointLon(startWaypointLon),
             m_startWaypointLat(startWaypointLat),
             m_midWaypointLon(midWaypointLon),
             m_midWaypointLat(midWaypointLat),
             m_endWaypointLon(endWaypointLon),
             m_endWaypointLat(endWaypointLat)
    { }

    CollisionAvoidanceMsg(double startWaypointLon,
                          double startWaypointLat,
                          double midWaypointLon,
                          double midWaypointLat,
                          double endWaypointLon,
                          double endWaypointLat)
            :Message(MessageType::CollisionAvoidance, NodeID::None, NodeID::None),
             m_startWaypointLon(startWaypointLon),
             m_startWaypointLat(startWaypointLat),
             m_midWaypointLon(midWaypointLon),
             m_midWaypointLat(midWaypointLat),
             m_endWaypointLon(endWaypointLon),
             m_endWaypointLat(endWaypointLat)
    { }

    CollisionAvoidanceMsg(MessageDeserialiser deserialiser)
            :Message(deserialiser)
    {
        if(     !deserialiser.readDouble(m_startWaypointLon) ||
                !deserialiser.readDouble(m_startWaypointLat) ||
                !deserialiser.readDouble(m_midWaypointLon  ) ||
                !deserialiser.readDouble(m_midWaypointLat  ) ||
                !deserialiser.readDouble(m_endWaypointLon  ) ||
                !deserialiser.readDouble(m_endWaypointLat  )
          )
        {
            m_valid = false;
        }
    }

    virtual ~CollisionAvoidanceMsg() { }

    double startWaypointLon() { return m_startWaypointLon; }
    double startWaypointLat() { return m_startWaypointLat; }
    double midWaypointLon()   { return m_midWaypointLon;   }
    double midWaypointLat()   { return m_midWaypointLat;   }
    double endWaypointLon()   { return m_endWaypointLon;   }
    double endWaypointLat()   { return m_endWaypointLat;   }

    ///----------------------------------------------------------------------------------
    /// Serialises the message into a MessageSerialiser
    ///----------------------------------------------------------------------------------
    virtual void Serialise(MessageSerialiser& serialiser) const
    {
        Message::Serialise(serialiser);

        serialiser.serialise(m_startWaypointLon);
        serialiser.serialise(m_startWaypointLat );
        serialiser.serialise(m_midWaypointLon  );
        serialiser.serialise(m_midWaypointLat   );
        serialiser.serialise(m_endWaypointLon  );
        serialiser.serialise(m_endWaypointLat   );
    }

private:
    double m_startWaypointLon;
    double m_startWaypointLat;
    double m_midWaypointLon;
    double m_midWaypointLat;
    double m_endWaypointLon;
    double m_endWaypointLat;
};
