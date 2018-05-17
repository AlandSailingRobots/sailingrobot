/****************************************************************************************
 *
 * File:
 * 		MessageLogger.h
 *
 * Purpose:
 *		Catches all known message types.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once

#include "MessageBus/Node.h"

class MessageLogger : public Node {
   public:
    MessageLogger(MessageBus& msgBus)
        : Node(NodeID::MessageLogger, msgBus),
          m_DataRequest(false),
          m_WindData(false),
          m_CompassData(false),
          m_GPSData(false),
          m_waypointData(false),
          m_rudderCommand(false),
          m_sailCommand(false),
          m_wingSailCommand(false),
          m_WindState(false),
          m_StateData(false),
          m_NavigationData(false) {
        msgBus.registerNode(*this);
        msgBus.registerNode(*this, MessageType::WindData);
        msgBus.registerNode(*this, MessageType::CompassData);
        msgBus.registerNode(*this, MessageType::GPSData);
        msgBus.registerNode(*this, MessageType::WaypointData);
        msgBus.registerNode(*this, MessageType::RudderCommand);
        msgBus.registerNode(*this, MessageType::SailCommand);
        msgBus.registerNode(*this, MessageType::WingSailCommand);
        msgBus.registerNode(*this, MessageType::WindState);
        msgBus.registerNode(*this, MessageType::StateMessage);
        msgBus.registerNode(*this, MessageType::LocalNavigation);
    }

    virtual ~MessageLogger() {}

    bool init() { return true; }

    void processMessage(const Message* message) {
        MessageType msgType = message->messageType();

        switch (msgType) {
            case MessageType::DataRequest:
                m_DataRequest = true;
                break;
            case MessageType::WindData:
                m_WindData = true;
                break;
            case MessageType::CompassData:
                m_CompassData = true;
                break;
            case MessageType::GPSData:
                m_GPSData = true;
                break;
            case MessageType::WaypointData:
                m_waypointData = true;
                break;
            case MessageType::RudderCommand:
                m_rudderCommand = true;
                break;
            case MessageType::SailCommand:
                m_sailCommand = true;
                break;
            case MessageType::WingSailCommand:
                m_wingSailCommand = true;
                break;
            case MessageType::WindState:
                m_WindState = true;
                break;
            case MessageType::StateMessage:
                m_StateData = true;
                break;
            case MessageType::LocalNavigation:
                m_NavigationData = true;
                break;
            default:
                return;
        }
    }

    bool dataRequestReceived() { return m_DataRequest; }
    bool windDataReceived() { return m_WindData; }
    bool compassDataReceived() { return m_CompassData; }
    bool gpsDataReceived() { return m_GPSData; }
    bool waypointDataReceived() { return m_waypointData; }
    bool rudderCommandReceived() { return m_rudderCommand; }
    bool sailCommandReceived() { return m_sailCommand; }
    bool wingSailCommandReceived() { return m_wingSailCommand; }
    bool windStateReceived() { return m_WindState; }
    bool stateDataReceived() { return m_StateData; }
    bool navigationDataReceived() { return m_NavigationData; }

    void clearState() {
        m_DataRequest = false;
        m_WindData = false;
        m_CompassData = false;
        m_GPSData = false;
        m_waypointData = false;
        m_rudderCommand = false;
        m_sailCommand = false;
        m_wingSailCommand = false;
        m_WindState = false;
        m_StateData = false;
    }

   private:
    bool m_DataRequest;
    bool m_WindData;
    bool m_CompassData;
    bool m_GPSData;
    bool m_waypointData;
    bool m_rudderCommand;
    bool m_sailCommand;
    bool m_wingSailCommand;
    bool m_WindState;
    bool m_StateData;
    bool m_NavigationData;
};
