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

struct AISVessel {
  int MMSI;
  float latitude;
  float longitude;
  uint16_t COG;
  uint16_t SOG;
}

class CANAISNode : public ActiveNode {
public:
  CANAISNode(MessageBus& msgBus, CANService& canService, double loopTime)
  ~CANAISNode();

  bool init();
  void processMessage(const Message* message)
  void processPGN(N2kMsg& nMsg);
  void parsePGN129038();
  void parsePGN129039();
  void start();

private:

  void CANAISThreadFunc(ActiveNode* nodePtr);
  std::vector<AISVessel> m_VesselList;
  double m_LoopTime;
}
