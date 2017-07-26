// #pragma once

#include "Hardwares/CANAISNode.h"
#include "Messages/AISDataMsg.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
#include "WorldState/AISProcessing.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"

CANService canService;
MessageBus msgBus;
CANAISNode* aisNode;
AISProcessing* aisProc;
CollidableMgr cMgr;

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
      posLat = msg->posLat();
      posLon = msg->posLon();
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
    Logger::info("Lat: " + std::to_string(posLat));
    Logger::info("Lon: " + std::to_string(posLon));
    Logger::info("");
  }

private:
  double posLat;
  double posLon;
  float m_TimeBetweenPrints;
  std::vector<AISVessel> m_VesselList;
};

void messageLoop() {
    msgBus.run();
}

int main() {
  Logger::init("AISTest.log");

  auto future = canService.start();


 // AISDataReceiver aisRec(msgBus, 10000);
  aisNode = new CANAISNode(msgBus, canService, 1);
  aisNode->start();

  aisProc = new AISProcessing(msgBus, &cMgr, 300e6, 230082790, 1);
  aisProc->start();

  cMgr.startGC();

  std::thread thr(messageLoop);
  thr.detach();
    int now;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    now = SysClock::unixTime();
    Logger::info("Collidable manager size: " + std::to_string(cMgr.getAISContacts().length()));
    auto colList = cMgr.getAISContacts();
    for (int i = 0; i<cMgr.getAISContacts().length(); i++) {
        auto t = colList.next();
        Logger::info("MMSI: " + std::to_string(t.mmsi) + ", Lat: " + std::to_string(t.latitude) + ", Lon: " + std::to_string(t.longitude) +
                ", COG: " + std::to_string(t.course) + " (" + std::to_string(t.course*180/3.141592) + ")" + ", SOG: " + std::to_string(t.speed) +
                " (" + std::to_string(t.speed*1.9438) + ")" + ", Length: " + std::to_string(t.length) + ", Beam: " + std::to_string(t.beam) + 
                ", Report age: " + std::to_string(now-t.lastUpdated));
    }
  //  aisRec.printData();
  }
}
