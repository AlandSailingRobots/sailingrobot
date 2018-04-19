/****************************************************************************************
*
* File:
* 		ThermalImagerProcessing.cpp
*
* Purpose:
*     Receives the Thermal Imager data (notice that the related codes work with any camera devices)
*     from the Thermal Imager Node, processes it and adds the amount of free space
*     in the visual field of the camera to the collidableMgr
*
* Developer Notes:
*
*
***************************************************************************************/

#include "ThermalImagerProcessing.h"

ThermalImagerProcessing::ThermalImagerProcessing(MessageBus& msgBus, DBHandler& dbhandler, CollidableMgr* collidableMgr)
  : ActiveNode(NodeID::ThermalImagerProcessing, msgBus), m_LoopTime(0.5),
  collidableMgr(collidableMgr), m_db(dbhandler), m_running(false) {
    msgBus.registerNode(*this, MessageType::ThermalImagerData);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
    updateConfigsFromDB();
  }

  ThermalImagerProcessing::~ThermalImagerProcessing() {

  }

  void ThermalImagerProcessing::updateConfigsFromDB(){
      m_LoopTime = m_db.retrieveCellAsDouble("config_thermalimager_processing","1","loop_time");
  }

  bool ThermalImagerProcessing::init() {
    updateConfigsFromDB();
    return true;
  }

  void ThermalImagerProcessing::processMessage(const Message* msg) {
    MessageType type = msg->messageType();
    switch (type) {
      case MessageType::ThermalImagerData :
        processThermalImagerMessage((ThermalImagerDataMsg*) msg);
        break;
      case MessageType::ServerConfigsReceived :
        updateConfigsFromDB();
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
    m_running = true;
    runThread(AISProcessingThreadFunc);
  }

  void AISProcessing::stop() {
    m_running = false;
    stopThread(this);
  }

  void AISProcessing::AISProcessingThreadFunc(ActiveNode* nodePtr) {
    AISProcessing* node = dynamic_cast<AISProcessing*> (nodePtr);

    Timer timer;
    timer.start();

    while(node->m_running) {
      node->m_lock.lock();
      node->addAISDataToCollidableMgr();
      node->m_lock.unlock();
      timer.sleepUntil(node->m_LoopTime);
      timer.reset();
    }
  }
