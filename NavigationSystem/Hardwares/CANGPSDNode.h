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


class CANGPSDNode : public ActiveNode {
public:

  CANGPSDNode(MessageBus& msgBus, double loopTime);
  ~CANGPSDNode();
  void processMessage (const Message* message);
  void processFrame (CanMsg& msg);
  void start();

private:

  void CANGPSDThreadFunc(ActiveNode* nodePtr);

  gpsmm* 	m_GpsConnection;

  double	m_Lat;
  double	m_Lon;
  double	m_Speed;
  double	m_Heading;

  int m_currentDay;
  double m_LoopTime;

  const int GPS_TIMEOUT_MICRO_SECS = 50000000;
}
