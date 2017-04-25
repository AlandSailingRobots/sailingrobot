#pragma once

#include "Messages/Message.h"
#include "Messages/MessageTypes.h"
#include "Nodes/NodeIDs.h"

class NavigationControlMsg : public Message {
public:

    enum class NavigationState {
        sailToWaypoint, stationKeeping, speedTarget
    };

    NavigationControlMsg(NodeID sourceID, NodeID destinationID, int courseToSteer, float targetSpeed, bool windvaneSelfSteeringOn, NavigationState state) :
    Message(MessageType::NavigationControl, sourceID, destinationID), m_CourseToSteer(courseToSteer), m_TargetSpeed(targetSpeed), 
        m_WindvaneSelfSteeringOn(windvaneSelfSteeringOn), m_NavigationState(state)
    { }

    NavigationControlMsg(int courseToSteer, float targetSpeed, bool windvaneSelfSteeringOn, NavigationState state) :
    Message(MessageType::NavigationControl, NodeID::None, NodeID::None), m_CourseToSteer(courseToSteer), m_TargetSpeed(targetSpeed), 
        m_WindvaneSelfSteeringOn(windvaneSelfSteeringOn), m_NavigationState(state)
    { }

    NavigationState navigationState() { return m_NavigationState; }
    int courseToSteer() { return m_CourseToSteer; }
    float targetSpeed() { return m_TargetSpeed; }
    bool windvaneSelfSteeringOn() { return m_WindvaneSelfSteeringOn; }

private:

    NavigationState m_NavigationState;

    int     m_CourseToSteer;
    float   m_TargetSpeed;
    bool    m_WindvaneSelfSteeringOn;

};