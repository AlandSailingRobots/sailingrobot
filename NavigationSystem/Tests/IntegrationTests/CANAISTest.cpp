// #pragma once

#include "Hardwares/CANAISNode.h"
#include "Messages/AISDataMsg.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"

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
    Logger::info("Size: " + std::to_string(m_VesselList.size()));
    Logger::info("");
    for (auto ves: m_VesselList) {
      std::cout << ves.MMSI << std::endl;
      Logger::info("MMSI: " + std::to_string(ves.MMSI));
      Logger::info("Lat: " + std::to_string(ves.latitude));
      Logger::info("Lon: " + std::to_string(ves.longitude));
      Logger::info("COG: " + std::to_string(ves.COG));
      Logger::info("SOG: " + std::to_string(ves.SOG));// << std::endl;
      Logger::info("");
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
  Logger::init("AISTest.log");

  auto future = canService.start();


  AISDataReceiver aisRec(msgBus, 10000);
  aisNode = new CANAISNode(msgBus, canService, 50);
  aisNode->start();


  std::thread thr(messageLoop);
  thr.detach();

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    aisRec.printData();
  }
}
