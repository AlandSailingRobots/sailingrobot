#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "MessageBus/MessageBus.h"
#include "Nodes/CANWindsensorNode.h"
#include "Tests/integration/TestMocks/MessagePrinter.h"

#include <thread>
#include <chrono>
#include <future>

#define WAIT_TIME 750

MessageBus mBus;

void startMsgBus(){
  mBus.run();
}

int main(int argc, char const *argv[]) {

  CANService service;


  CANWindsensorNode windNode(mBus, service, 500);
  MessagePrinter printer(mBus);

  auto future = service.start();
  std::thread msgThread (startMsgBus);
  msgThread.detach();

  std::this_thread::sleep_for(std::chrono::seconds(WAIT_TIME));

  service.stop();
  future.get();

  // Expected output:
  // Only first and last messages should be 
  // printed out

  return 0;
}