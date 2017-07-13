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

  bool AISProcessing::init() {
    return true;
  }

  void AISProcessing::processMessage(const Message* msg) {
    MessageType type = msg->MessageType();

    switch (type) {
      case MessageType::AISData :
        processAISMessage(dynamic_cast<const AISDataMsg*> msg);
        break;
      case MessageType::StateMessage :
        processStateMessage(dynamic_cast<const StateMessage*> msg);
        break;
      default:
        return;
    }
  }

  void AISProcessing::processAISMessage(const AISDataMsg* msg) {
    std::vector<AISVessel> list = msg.vesselList();
    double dist;
    for (auto vessel: list) {
      dist = calculateDTW(m_latitude, m_longitude, vessel.latitude, vessel.longitude);
      if (dist < RADIUS) {
        m_VesselList.push_back(vessel);
      }
    }
  }

  void AISProcessing::processStateMessage(const StateMessage* msg) {
    m_latitude = msg.latitude();
    m_longitude = msg.longitude();
  }

  void AISProcessing::start() {
    runThread(AISProcessingThreadFunc);
  }

  void AISProcessing::AISProcessingThreadFunc(ActiveNode* nodePtr) {
    AISProcessing* node = dynamic_cast<AISProcessing*> nodePtr;

    Timer timer;
    timer.start();

    while(true) {
      node->m_lock.lock();
      // Send data to CollidableMgr

      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime*1.0f/1000);
      timer.reset();
    }
  }
