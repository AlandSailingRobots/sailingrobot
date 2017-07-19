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

AISProcessing::AISProcessing(MessageBus& msgBus, CollidableMgr* collidableMgr, int radius, double loopTime)
  : ActiveNode(NodeID::AISProcessing, msgBus), m_LoopTime(loopTime), m_Radius(radius), collidableMgr(collidableMgr) {
    msgBus.registerNode(*this, MessageType::AISData);
  }

  AISProcessing::~AISProcessing() {

  }

  bool AISProcessing::init() {
    return true;
  }

  void AISProcessing::processMessage(const Message* msg) {
    MessageType type = msg->messageType();

    switch (type) {
      case MessageType::AISData :
        processAISMessage((AISDataMsg*) msg);
        break;
      default:
        return;
    }
  }

  void AISProcessing::processAISMessage(AISDataMsg* msg) {
    std::vector<AISVessel> list = msg->vesselList();
    double dist;
    m_latitude = msg->posLat();
    m_longitude = msg->posLon();
    for (auto vessel: list) {
      std::cout << sizeof m_latitude << "\t" << sizeof m_longitude << "\t"
                << sizeof vessel.latitude << "\t" << sizeof vessel.longitude << std::endl;
      dist = CourseMath::calculateDTW(m_latitude, m_longitude, vessel.latitude, vessel.longitude);
      std::cout << dist << std::endl;
      if (dist < m_Radius) {
        m_Vessels.push_back(vessel);
      }
    }
  }

  void AISProcessing::sendAISData() {
    for (auto vessel: m_Vessels) {
      this->collidableMgr->addAISContact(vessel.MMSI, vessel.latitude, vessel.longitude, vessel.SOG, vessel.COG);
    }
  }

  void AISProcessing::start() {
    runThread(AISProcessingThreadFunc);
  }

  void AISProcessing::AISProcessingThreadFunc(ActiveNode* nodePtr) {
    AISProcessing* node = dynamic_cast<AISProcessing*> (nodePtr);

    Timer timer;
    timer.start();

    while(true) {
      node->m_lock.lock();
      node->sendAISData();
      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime*1.0f/1000);
      timer.reset();
    }
  }
