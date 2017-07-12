#pragma once

#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/Timer.h"
#include "Messages/AISDataMsg.h"
#include "Math/Utility.h"

#include <cstring>
#include <cstdint>
#include <mutex>
#include <vector>
#include <iostream>

class CANAISNode : public CANPGNReceiver, public ActiveNode {
public:
  CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime);
  ~CANAISNode();

  bool init();
  void processMessage(const Message* message);
  void processPGN(N2kMsg& nMsg);
  void parsePGN129038_129039(N2kMsg& nMsg, AISVessel& vessel);
  void start();

private:

  static void CANAISThreadFunc(ActiveNode* nodePtr);
  std::vector<AISVessel> m_VesselList;
  std::mutex m_lock;
  double m_LoopTime;
};
