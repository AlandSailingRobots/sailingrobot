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

AISProcessing::AISProcessing(MessageBus& msgBus, CollidableMgr* collidableMgr, int radius, uint32_t mmsi, double loopTime)
  : ActiveNode(NodeID::AISProcessing, msgBus), m_LoopTime(loopTime), m_Radius(radius), m_MMSI(mmsi), collidableMgr(collidableMgr) {
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
      dist = CourseMath::calculateDTW(m_latitude, m_longitude, vessel.latitude, vessel.longitude);
      if (dist < m_Radius && vessel.MMSI != m_MMSI) {
        m_Vessels.push_back(vessel);
      }
    }
  for(auto& ves: m_Vessels) {
    if (ves.length > 1) {
        std::cout << "MMSI: " << ves.MMSI << ", Lat: " << ves.latitude << ", Lon: " << ves.longitude
            << ", COG: " << ves.COG << ", SOG: " << ves.SOG << ", length: " << ves.length << ", beam: " << ves.beam << std::endl;
    }
  }
  }

  void AISProcessing::sendAISData() {
    for (auto vessel: m_Vessels) {
      //  std::cout << "COG: " << vessel.COG << ",\t";
      this->collidableMgr->addAISContact(vessel.MMSI, vessel.latitude, vessel.longitude, vessel.SOG, vessel.COG);
    }
    //std::cout << std::endl;
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
      node->m_Vessels.clear();
      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime);
      timer.reset();
    }
  }
