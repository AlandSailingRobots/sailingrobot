/****************************************************************************************
*
* File:
* 		AISProcessing.cpp
*
* Purpose:
*     Receives the AIS data from the CANAISNode, processes it and adds the vessels
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
    std::vector<AISVesselInfo> tmp_info = msg->vesselInfoList();
    double dist;
    m_latitude = msg->posLat();
    m_longitude = msg->posLon();
    for (auto vessel: list) {
      dist = CourseMath::calculateDTW(m_latitude, m_longitude, vessel.latitude, vessel.longitude);
      if (dist < m_Radius && vessel.MMSI != m_MMSI) {
        m_Vessels.push_back(vessel);
      }
    }
    for (auto tmp: tmp_info) {
      for (auto info: m_InfoList) {
        if (tmp.MMSI == info.MMSI) {
          break;
        }
        m_InfoList.push_back(tmp);
      }
    }
  }

  void AISProcessing::addAISDataToCollidableMgr() {
    /*
    * First loop sends the position report to collidable manager
    * And the second sends the static report
    */
    std::vector<int> indexToRemove;
    for (auto vessel: m_Vessels) {
      this->collidableMgr->addAISContact(vessel.MMSI, vessel.latitude, vessel.longitude, vessel.SOG, vessel.COG);
      for (uint32_t i = 0; i<m_InfoList.size();i++) {
        if (vessel.MMSI == m_InfoList[i].MMSI) {
          this->collidableMgr->addAISContact(m_InfoList[i].MMSI, m_InfoList[i].length, m_InfoList[i].beam);
          indexToRemove.push_back(i);
        }
      }
    }
    for (auto i: indexToRemove) {
      m_InfoList.erase(m_InfoList.begin() + i);
    }
    indexToRemove.clear();
    m_Vessels.clear();
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
      node->addAISDataToCollidableMgr();
      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime);
      timer.reset();
    }
  }
