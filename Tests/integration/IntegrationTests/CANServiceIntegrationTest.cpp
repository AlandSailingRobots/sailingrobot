#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "MessageBus/MessageBus.h"
#include "Nodes/CANWindsensorNode.h"
#include "Tests/integration/TestMocks/MessagePrinter.h"

#include <thread>
#include <chrono>
#include <future>

#define WAIT_TIME 750



void startMsgBus(MessageBus& mBus){
  mBus.run();
}

int main(int argc, char const *argv[]) {

  CANService service;
  MessageBus mBus;

  CANWindsensorNode windNode(mBus, service, 500);
  MessagePrinter printer(mBus);
  windNode.start();

  auto future = service.start();
  
  std::async (std::launch::async, startMsgBus, std::ref(mBus));

  std::this_thread::sleep_for(std::chrono::seconds(WAIT_TIME));

  service.stop();
  future.get();

  return 0;
}