#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"

#include <thread>
#include <chrono>
#include <future>

#define WAIT_TIME 15

int main(int argc, char const *argv[]) {

  CANService service;
  auto future = service.start();
  std::this_thread::sleep_for(std::chrono::seconds(WAIT_TIME));
  service.stop();
  future.get();

  return 0;
}