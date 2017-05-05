/****************************************************************************************
*
* File:
* 		StateMessageListener.h
*
* Purpose:
*		Catches a message of type StateMessage and can return the value in the message.
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "Nodes/Node.h"
#include "Messages/GPSDataMsg.h"
#include "Nodes/StateEstimationNode.h"


class StateMessageListener: public Node {
public:
  StateMessageListener(MessageBus& msgBus) : Node(NodeID::StateMessageListener, msgBus),
  m_stateData(false)
  {
    msgBus.registerNode(*this, MessageType::StateMessage);
  }

  bool init() { return true; }

  ~StateMessageListener(){}

  void processMessage(const Message* message)
  {
    MessageType msgType = message->messageType();

    if(msgType == MessageType::StateMessage ){
      const StateMessage* stateMsg = static_cast<const StateMessage*>(message);
      m_VesselHeading = stateMsg->heading();
      m_VesselLat = stateMsg->latitude();
      m_VesselLon = stateMsg->longitude();
      m_VesselSpeed = stateMsg->speed();
      m_VesselCourse = stateMsg->course();
    }

    switch(msgType)
    {
      case MessageType::StateMessage:
      m_stateData = true;
      break;
      default:
      return;
    }
  }

  bool stateDataReceived(){return m_stateData; }
  float getVesselheading(){return m_VesselHeading;}
  double getVesselLat(){return m_VesselLat;}
  double getVesselLon(){return m_VesselLon;}
  double getVesselSpeed(){return m_VesselSpeed;}
  double getVesselCourse(){return m_VesselCourse;}

  void resetStateDataReceived(){
    m_stateData = false;
    m_VesselHeading = 0;
    m_VesselLat = 0;
    m_VesselLon = 0;
    m_VesselSpeed = 0;
    m_VesselCourse = 0;
  }

private:
  bool m_stateData;
  float 	m_VesselHeading;
  double	m_VesselLat;
  double	m_VesselLon;
  double	m_VesselSpeed;
  double  m_VesselCourse;

};
