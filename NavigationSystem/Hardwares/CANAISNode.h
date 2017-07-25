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
#include "SystemServices/Logger.h"

#include <fstream>
#include <cstring>
#include <cstdint>
#include <mutex>
#include <vector>
#include <iostream>

class CANAISNode : public CANPGNReceiver, public ActiveNode {
public:
  /*
  * Constructor, pointer to a message bus and canservice
  * double loopTime, how often we send messages
  */
  CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime);
  ~CANAISNode();

  bool init();

  /*
  * Processes any message we receive from the messagebus
  */
  void processMessage(const Message* message);

  /*
  * Receives the N2kMsg and processes it
  */
  void processPGN(N2kMsg& nMsg);


  /*
  * Starts the worker thread
  */
  void start();

private:
  /*
  * Processes and return the data of the message
  * if the PGN is 129038 or 129039
  */
  void parsePGN129038_129039(N2kMsg& nMsg);

  /*
  * Processes and return the data of the message
  * if the PGN is 129025
  */
  void parsePGN129025(N2kMsg& nMsg);

  /*
  * The function that the thread works on
  */
  static void CANAISThreadFunc(ActiveNode* nodePtr);

  /*
  * Private variables
  */
  std::vector<AISVessel> m_VesselList;
  double m_PosLat;
  double m_PosLon;
  std::mutex m_lock;
  double m_LoopTime;
  double res_pos = 1e-7;
  float res_cog = 1e-4;
  float res_sog = 1e-2;
};
