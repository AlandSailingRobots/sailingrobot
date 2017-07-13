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
#include "Utility/CourseMath.h"

#include <chrono>
#include <thread>

// The radius in which we need to track all vessels, in meters
#define RADIUS = 300;

class AISProcessing : public ActiveNode {
public:
  AISProcessing(MessageBus& msgBus, double loopTime);
  ~AISProcessing();

  bool init();

  void processMessage(const Message* msg);

  void processAISMessage(const AISDataMsg* msg);

  void processStateMessage(const StateMessage* msg);

  void start();

private:

  void AISProcessingThreadFunc(ActiveNode* nodePtr);
  bool AISDataReceived = false;
  bool stateMsgReceived = false;

  std::vector<AISVessel> m_Vessels;
  double m_latitude;
  double m_longitude;
  std::mutex m_lock;
};
