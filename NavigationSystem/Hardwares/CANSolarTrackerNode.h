#pragma once

#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/Timer.h"
#include "Messages/StateMessage.h"
#include "Messages/SolarDataMsg.h"

#include <time.h>
#include <mutex>
#include <vector>
#include <iostream>
#include <cstring>

class CANSolarTrackerNode : public ActiveNode {
public:
  CANSolarTrackerNode(MessageBus& msgBus, CANService& canService, double loopTime);
  ~CANSolarTrackerNode();

  bool init();
  void processMessage (const Message* message);
  void processFrame (CanMsg& Msg);
  void sendMsg (float lat, float lon, float head, uint16_t h, uint16_t m);
  void start();

private:

  static void CANSolarTrackerThreadFunc(ActiveNode* nodePtr);

  CANService* m_CANService;
  float	m_Lat;
  float	m_Lon;
  uint16_t m_Hour;
  uint16_t m_Minute;
  float	m_Heading;
  bool m_initialised;
  double m_LoopTime;

  std::mutex m_lock;

  const int DATA_OUT_OF_RANGE = -2000;
};
