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
  CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime)
  ~CANAISNode();
  
  bool init();
  void processMessage(const Message* message)
  void start();
private:
  void CANAISThreadFunc(ActiveNode* nodePtr);

  int m_MMSI;
  float m_Lat;
  float m_Lon;
  int m_COG;
  float m_SOG;
}
