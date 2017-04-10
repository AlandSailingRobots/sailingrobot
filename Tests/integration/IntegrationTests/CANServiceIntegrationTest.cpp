#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "MessageBus/MessageBus.h"
#include "Nodes/CANWindsensorNode.h"
#include "Tests/unit-tests/TestMocks/MessageLogger.h"

#include <thread>
#include <chrono>
#include <future>

#define WAIT_TIME 5

static MessageBus& msgBus(){
  static MessageBus* mbus = new MessageBus();
  return *mbus;
}

void startMsgBus(){
  msgBus().run();
}

int main(int argc, char const *argv[]) {

  CANService service;

  CANWindsensorNode windNode(msgBus(), service);
  MessageLogger logger(msgBus());

  auto future = service.start();
  std::thread msgThread (startMsgBus);
  std::this_thread::sleep_for(std::chrono::seconds(WAIT_TIME));
  service.stop();
  future.get();

  if(logger.windDataReceived()) {
    std::cout << "Test passed with flying colors." << std::endl;
  }

  else {
    std::cout << "Logger did not receive wind data from the CAN Node" << std::endl;
    std::cout << "Test not passed" << std::endl;
  }

  return 0;
}