#pragma once

#include "Messages/Message.h"
#include "Messages/MessageTypes.h"
#include "Nodes/NodeIDs.h"

enum class NavigationState {
    sailToWaypoint, stationKeeping, speedTarget
};

class NavigationControlMsg : public Message {
public:
    NavigationControlMsg(NodeID sourceID, NodeID destinationID, int courseToSteer, float targetSpeed, bool windvaneSelfSteeringOn, bool tack, bool starboard, NavigationState state) :
    Message(MessageType::NavigationControl, sourceID, destinationID), m_CourseToSteer(courseToSteer), m_TargetSpeed(targetSpeed),
        m_WindvaneSelfSteeringOn(windvaneSelfSteeringOn), m_tack(tack), m_starboard(starboard), m_NavigationState(state)
    { }

    NavigationControlMsg(int courseToSteer, float targetSpeed, bool windvaneSelfSteeringOn, bool tack, bool starboard, NavigationState state) :
    Message(MessageType::NavigationControl, NodeID::None, NodeID::None), m_CourseToSteer(courseToSteer), m_TargetSpeed(targetSpeed),
        m_WindvaneSelfSteeringOn(windvaneSelfSteeringOn), m_tack(tack), m_starboard(starboard), m_NavigationState(state)
    { }

    NavigationState navigationState() const { return m_NavigationState; }
    int courseToSteer() const { return m_CourseToSteer; }
    float targetSpeed() const { return m_TargetSpeed; }
    bool windvaneSelfSteeringOn() const { return m_WindvaneSelfSteeringOn; }
    bool tack() const { return m_tack; }
    bool starboard() const { return m_starboard; }

private:

    int     m_CourseToSteer;
    float   m_TargetSpeed;
    bool    m_WindvaneSelfSteeringOn;
    bool    m_tack;
    bool    m_starboard;
    NavigationState m_NavigationState;

};
