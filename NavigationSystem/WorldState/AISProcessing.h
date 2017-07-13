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

class AISProcessing : public ActiveNode {
public:
  AISProcessing(MessageBus& msgBus, double loopTime);
  ~AISProcessing();

  bool init();

  void processMessage(const Message* msg);

  void processAISMessage(AISDataMsg* msg);

  void processStateMessage(StateMessage* msg);

  void start();

private:

  void AISProcessingThreadFunc();

  std::vector<AISVessel> m_Vessels;
  double m_latitude;
  double m_longitude;
}
