#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/CANFrameReceiver.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/Message.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/Timer.h"

#include <mutex>
#include <vector>
#include <iostream>


class CANSolarTrackerNode : public ActiveNode {
public:

  CANSolarTrackerNode(MessageBus& msgBus, double loopTime);
  ~CANSolarTrackerNode();
  void processMessage (const Message* message);
  void processFrame (CanMsg& msg);
  void start();

private:

  void CANGPSDThreadFunc(ActiveNode* nodePtr);

  gpsmm* 	m_GpsConnection;

  float	m_Lat;
  float	m_Lon;
  float	m_Time;
  float	m_Heading;

  int m_currentDay;
  double m_LoopTime;

  const int GPS_TIMEOUT_MICRO_SECS = 50000000;
}
