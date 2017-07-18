// #pragma once

#include "Hardwares/CANAISNode.h"
#include "Messages/AISDataMsg.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"

CANService canService;
MessageBus msgBus;
CANAISNode* aisNode;

class AISDataReceiver : public Node {
public:
  AISDataReceiver(MessageBus& msgBus, float timeBetweenPrints) :
  Node(NodeID::None, msgBus), m_TimeBetweenPrints(timeBetweenPrints) {
    msgBus.registerNode(*this, MessageType::AISData);
  }

  bool init() { return true; }

  void processMessage(const Message* message) {
    MessageType type = message->messageType();
    if (type == MessageType::AISData) {
      AISDataMsg* msg = (AISDataMsg*) message;
      m_VesselList = msg->vesselList();
      printData();
    }
  }

  void printData() {
    std::cout << std::endl << m_VesselList.size() << std::endl;
    for (auto ves: m_VesselList) {
      std::cout << ves.MMSI << std::endl;
    }
    if (m_VesselList.size()>0) {
      std::cout << m_VesselList[0].MMSI << std::endl;
      std::cout << m_VesselList[0].latitude << std::endl;
      std::cout << m_VesselList[0].longitude << std::endl;
      std::cout << m_VesselList[0].COG << std::endl;
      std::cout << m_VesselList[0].SOG << std::endl;
    }
  }

private:
  float m_TimeBetweenPrints;
  std::vector<AISVessel> m_VesselList;
};

void messageLoop() {
    msgBus.run();
}

int main() {
  auto future = canService.start();

  AISDataReceiver aisRec(msgBus, 300);
  aisNode = new CANAISNode(msgBus, canService, 500);
  aisNode->start();


  std::thread thr(messageLoop);
  thr.detach();

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    aisRec.printData();
  }

  std::cout << "MMMMMMMMMMMMMMMMMMM" << std::endl;
}
