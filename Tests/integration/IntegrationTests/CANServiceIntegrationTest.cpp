#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "MessageBus/MessageBus.h"
#include "Nodes/CANWindsensorNode.h"
#include "Tests/integration/TestMocks/MessagePrinter.h"
#include "Messages/WindDataMsg.h"

#include <thread>
#include <chrono>
#include <future>

#define WAIT_TIME 5
/*
static MessageBus& msgBus(){
  static MessageBus* mbus = new MessageBus();
  return *mbus;
}
*/
void startMsgBus(MessageBus& msgBus){
  msgBus.run();
}

int main(int argc, char const *argv[]) {

  CANService service;
  MessageBus msgBus;
  std::cout << &msgBus << std::endl;
  CANWindsensorNode windNode(msgBus, service, 500);
  MessagePrinter printer(msgBus);

  auto future = service.start();
  
  windNode.start();
  std::thread t1(startMsgBus, std::ref(msgBus));
  t1.detach();
  msgBus.sendMessage(std::make_unique<WindDataMsg>(1,2,3));

  std::this_thread::sleep_for(std::chrono::seconds(WAIT_TIME));

  service.stop();
  future.get();

  return 0;
}