/****************************************************************************************
*
* File:
* 		ObstacleDetectionTest.cpp
*
* Purpose:
*		Integration test for obstacle detection with the camera.
*
*
* Developer Notes:
*
*
***************************************************************************************/
#include <thread>
#include <chrono>

#include "DataBase/DBHandler.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "SystemServices/Logger.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"

DBHandler dbHandler("../asr.db");
MessageBus msgBus;
CollidableMgr cMgr;

void messageLoop() {
    msgBus.run();
}

int main() {
  Logger::init("ObstacleDetectionTest.log");

  cMgr.startGC();

  std::thread thr(messageLoop);
  thr.detach();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    int now = SysClock::unixTime();
    Logger::info("unixTime: " + std::to_string(now));
    // Logger::info("Collidable manager size: " + std::to_string(cMgr.getAISContacts().length()));
    // auto colList = cMgr.getAISContacts();
    // for (int i = 0; i<cMgr.getAISContacts().length(); i++) {
    //     auto t = colList.next();
    //     Logger::info("MMSI: " + std::to_string(t.mmsi) + ", Lat: " + std::to_string(t.latitude) + ", Lon: " + std::to_string(t.longitude) +
    //             ", COG: " + std::to_string(t.course) + " (" + std::to_string(t.course*180/3.141592) + ")" + ", SOG: " + std::to_string(t.speed) +
    //             " (" + std::to_string(t.speed*1.9438) + ")" + ", Length: " + std::to_string(t.length) + ", Beam: " + std::to_string(t.beam) +
    //             ", Report age: " + std::to_string(now-t.lastUpdated));
    // }
  }
}