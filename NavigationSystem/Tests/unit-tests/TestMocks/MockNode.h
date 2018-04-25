/****************************************************************************************
 *
 * File:
 * 		MockNode.h
 *
 * Purpose:
 *		A mock node for testing whether message passing is working correctly.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include <atomic>

#include "MessageBus/Node.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/LocalNavigationMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/WindStateMsg.h"

class MockNode : public Node {
   public:
    MockNode(MessageBus& msgBus, bool& registered)
        : Node(NodeID::MessageLogger, msgBus),
          m_MessageReceived(false),
          m_HasFix(false),
          m_Online(false),
          m_WindDir(0),
          m_WindSpeed(0),
          m_TargetCourse(0.5),
          m_TargetSpeed(0) {
        if (msgBus.registerNode(*this, MessageType::GPSData) &&
            msgBus.registerNode(*this, MessageType::WindData) &&
            msgBus.registerNode(*this, MessageType::WindState) &&
            msgBus.registerNode(*this, MessageType::CompassData) &&
            msgBus.registerNode(*this, MessageType::WaypointData) &&
            msgBus.registerNode(*this, MessageType::StateMessage) &&
            msgBus.registerNode(*this, MessageType::ServerConfigsReceived) &&
            msgBus.registerNode(*this, MessageType::LocalNavigation) &&
            msgBus.registerNode(*this, MessageType::AISData)) {
            registered = true;
        }
    }

    virtual ~MockNode() {}

    bool init() { return true; }

    void processMessage(const Message* message) {
        MessageType type = message->messageType();
        // std::cerr << "MockNode processMessage" << std::endl;

        switch (type) {
            case MessageType::GPSData: {
                m_MessageReceived = true;
                const GPSDataMsg* gpsMsg = static_cast<const GPSDataMsg*>(message);
                m_HasFix = gpsMsg->hasFix();
                m_Online = gpsMsg->gpsOnline();
                m_Lat = gpsMsg->latitude();
                m_Lon = gpsMsg->longitude();
                m_UnixTime = gpsMsg->unixTime();
                m_Speed = gpsMsg->speed();
                m_Course = gpsMsg->course();
                m_SatCount = gpsMsg->satelliteCount();
                m_Mode = gpsMsg->gpsMode();
            } break;
            case MessageType::WindData: {
                m_MessageReceived = true;
                const WindDataMsg* windMsg = static_cast<const WindDataMsg*>(message);
                m_WindDir = windMsg->windDirection();
                m_WindSpeed = windMsg->windSpeed();
                m_WindTemp = windMsg->windTemp();
            } break;
            case MessageType::WindState: {
                m_MessageReceived = true;
                const WindStateMsg* windState = static_cast<const WindStateMsg*>(message);
                m_trueWindDir = windState->trueWindSpeed();
                m_trueWindSpeed = windState->trueWindDirection();
                m_apparentWindSpeed = windState->apparentWindSpeed();
                m_apparentWindDir = windState->apparentWindDirection();
            } break;
            case MessageType::CompassData: {
                m_MessageReceived = true;
                const CompassDataMsg* compassData = static_cast<const CompassDataMsg*>(message);
                m_compassHeading = compassData->heading();
                m_compassPitch = compassData->pitch();
                m_compassRoll = compassData->roll();
            } break;
            case MessageType::WaypointData: {
                m_MessageReceived = true;
                const WaypointDataMsg* waypointData = static_cast<const WaypointDataMsg*>(message);
                m_waypointNextId = waypointData->nextId();
                m_waypointNextLongitude = waypointData->nextLongitude();
                m_waypointNextLatitude = waypointData->nextLatitude();
                m_waypointNextDeclination = waypointData->nextDeclination();
                m_waypointNextRadius = waypointData->nextRadius();
                m_waypointStayTime = waypointData->stayTime();
                m_isCheckpoint = waypointData->isCheckpoint();
                m_waypointPrevId = waypointData->prevId();
                m_waypointPrevLongitude = waypointData->prevLongitude();
                m_waypointPrevLatitude = waypointData->prevLatitude();
                m_waypointPrevDeclination = waypointData->prevDeclination();
                m_waypointPrevRadius = waypointData->prevRadius();

            } break;
            case MessageType::StateMessage: {
                m_MessageReceived = true;
                const StateMessage* stateMsg = static_cast<const StateMessage*>(message);
                m_StateMsgHeading = stateMsg->heading();
                m_StateMsgLat = stateMsg->latitude();
                m_StateMsgLon = stateMsg->longitude();
                m_StateMsgSpeed = stateMsg->speed();
                m_StateMsgCourse = stateMsg->course();
            } break;
            case MessageType::LocalNavigation: {
                m_MessageReceived = true;
                const LocalNavigationMsg* localNavigationMsg =
                    static_cast<const LocalNavigationMsg*>(message);
                m_TargetCourse = localNavigationMsg->targetCourse();
                m_TargetSpeed = localNavigationMsg->targetSpeed();
                m_BeatingState = localNavigationMsg->beatingMode();
                m_TargetTackStarboard = localNavigationMsg->targetTackStarboard();
            } break;
            case MessageType::ServerConfigsReceived: {
                m_MessageReceived = true;
            } break;
            case MessageType::AISData: {
                m_MessageReceived = true;
            } break;
            default: { throw std::logic_error("Unknown message type in MockNode"); }
                return;
        }
    }

    void clearMessageReceived() { m_MessageReceived = false; }

    std::atomic<bool> m_MessageReceived;

    // GPSData variables
    //=========================
    bool m_HasFix;
    bool m_Online;
    double m_Lat;
    double m_Lon;
    double m_UnixTime;
    double m_Speed;
    double m_Course;
    int m_SatCount;
    GPSMode m_Mode;

    // WindData variables
    //=========================
    float m_WindDir;
    float m_WindSpeed;
    float m_WindTemp;
    float m_Heading;

    // WindState variables
    //=========================
    double m_trueWindDir;
    double m_trueWindSpeed;
    double m_apparentWindSpeed;
    double m_apparentWindDir;

    // CompassData variables
    //=========================
    double m_compassHeading;
    double m_compassPitch;
    double m_compassRoll;

    // WaypointData variables
    //=========================
    int m_waypointNextId;
    double m_waypointNextLongitude;
    double m_waypointNextLatitude;
    int m_waypointNextDeclination;
    int m_waypointNextRadius;
    int m_waypointStayTime;
    bool m_isCheckpoint;
    int m_waypointPrevId;
    double m_waypointPrevLongitude;
    double m_waypointPrevLatitude;
    int m_waypointPrevDeclination;
    int m_waypointPrevRadius;

    // StateMessage variables
    //=========================
    float m_StateMsgHeading;
    double m_StateMsgLat;
    double m_StateMsgLon;
    double m_StateMsgSpeed;
    double m_StateMsgCourse;

    // ActuatorPosition variables
    //=========================
    int m_rudderPosition;
    int m_sailPosition;

    // ActuatorPosition variables
    //=========================
    int16_t m_DesiredCourse;

    // NavigationControlMsg variables

    // LocalNavigationMsg variables
    //=========================
    float m_TargetCourse;
    float m_TargetSpeed;
    bool m_BeatingState;
    bool m_TargetTackStarboard;
};
