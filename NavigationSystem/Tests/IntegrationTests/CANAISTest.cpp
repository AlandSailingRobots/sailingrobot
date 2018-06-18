/****************************************************************************************
*
* File:
* 		CANAISTest.cpp
*
* Purpose:
*		Integration test for the AIS, tests if we can receive and read the messages from the AIS
*
*
* Developer Notes:
*
*
***************************************************************************************/

#include "DataBase/DBHandler.h"
#include "Hardwares/CANAISNode.h"
#include "Messages/AISDataMsg.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
#include "WorldState/AISProcessing.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"

CANService canService;
DBHandler dbHandler("../asr.db");
MessageBus msgBus;
CANAISNode* aisNode;
AISProcessing* aisProc;
CollidableMgr cMgr;

void messageLoop() {
    msgBus.run();
}

int main() {
  Logger::init("AISTest.log");

  auto future = canService.start();

  aisNode = new CANAISNode(msgBus, dbHandler, canService);
  aisNode->start();

  aisProc = new AISProcessing(msgBus,dbHandler, &cMgr);
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
  }
}
