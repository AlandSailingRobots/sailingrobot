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

#pragma once

#include "Messages/AISDataMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Timer.h"
#include "MessageBus/Message.h"
#include "Math/CourseMath.h"
#include "MessageBus/ActiveNode.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"

#include <chrono>
#include <thread>
#include <vector>
#include <mutex>

// The radius in which we need to track all vessels, in meters
#define RADIUS 300e9

class AISProcessing : public ActiveNode {
public:
  AISProcessing(MessageBus& msgBus, CollidableMgr* collidableMgr, double loopTime);
  ~AISProcessing();

  bool init();

  void processMessage(const Message* msg);

  void processAISMessage(AISDataMsg* msg);

  void processStateMessage(StateMessage* msg);

  void start();

private:

  void sendAISData();

  static void AISProcessingThreadFunc(ActiveNode* nodePtr);
  bool AISDataReceived = false;
  bool stateMsgReceived = false;

  std::vector<AISVessel> m_Vessels;
  double m_latitude;
  double m_longitude;
  double m_LoopTime;
  std::mutex m_lock;
  CollidableMgr* collidableMgr;

};
