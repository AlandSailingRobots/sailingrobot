#pragma once

#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/Timer.h"
#include "Messages/StateMessage.h"

#include <mutex>
#include <vector>
#include <iostream>

class CANAISNode : public ActiveNode {
public:
  CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime);
  ~CANAISNode();

  bool init();
  void processMessage(const Message* message);
  void processFrame(CanMsg& msg);
  void processPGN(N2kMsg& nMsg);
  void parsePGN129038_129039();
  void start();

private:

  static void CANAISThreadFunc(ActiveNode* nodePtr);
  std::vector<AISVessel> m_VesselList;
  std::mutex m_lock;
  double m_LoopTime;
};
// #endif
