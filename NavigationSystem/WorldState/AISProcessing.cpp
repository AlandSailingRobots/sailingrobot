/****************************************************************************************
*
* File:
* 		AISProcessing.cpp
*
* Purpose:
*     Receives the data from the CANAISNode and processes it and sends the vessels
*     that are in a certain radius to the collidableMgr
*
* Developer Notes:
*
*
***************************************************************************************/

#include "AISProcessing.h"

AISProcessing::AISProcessing(MessageBus& msgBus, double loopTime)
  : ActiveNode(NodeID::AISProcessing, msgBus), m_LoopTime(loopTime) {
    msgBus.registerNode(*this, MessageType::AISData);
    msgBus.registerNode(*this, MessageType::StateMessage);
  }

  AISProcessing::~AISProcessing() {

  }

  bool init() {
    return true;
  }

  void processMessage(const Message* msg) {
    MessageType type = msg->MessageType();

    switch (type) {
      case MessageType::AISData :
        processAISMessage((AISDataMsg*) msg);
        break;
      case MessageType::StateMessage :
        processStateMessage((StadeMessage*)msg);
        break;
      default:
        return;
    }
  }
