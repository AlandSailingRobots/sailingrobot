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
#include "SystemServices/Logger.h"

#include <chrono>
#include <thread>
#include <vector>
#include <mutex>

class AISProcessing : public ActiveNode {
public:
  /*
  * Constructor, pointer to messagebus and canservice
  * int radius, the distance [meter] from us in which a vessel is interesting
  * double loopTime, how often we send messages
  */
  AISProcessing(MessageBus& msgBus, CollidableMgr* collidableMgr, int radius, uint32_t mmsi, double loopTime);
  ~AISProcessing();

  bool init();

  /*
  * Processes the message received
  */
  void processMessage(const Message* msg);

  /*
  * Starts the worker thread
  */
  void start();

private:
  /*
  * Sends the data to the collidable manager
  */
  void sendAISData();

  /*
  * Gets to process the message if the message received is an AISDataMsg
  */
  void processAISMessage(AISDataMsg* msg);

  /*
  * The function that thread works on
  */
  static void AISProcessingThreadFunc(ActiveNode* nodePtr);

  /*
  * Private variables
  */
  bool AISDataReceived = false;
  bool stateMsgReceived = false;

  std::vector<AISVessel> m_Vessels;
  double m_latitude;
  double m_longitude;
  double m_LoopTime;
  int m_Radius;
  uint32_t m_MMSI;
  std::mutex m_lock;
  CollidableMgr* collidableMgr;
};
