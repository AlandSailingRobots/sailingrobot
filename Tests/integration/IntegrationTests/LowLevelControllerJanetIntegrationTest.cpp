#include "Nodes/LowLevelControllerNodeJanet.h"
#include "MessageBus/MessageBus.h"
#include "SystemServices/Logger.h"
#include "HardwareServices/CAN_Services/CANService.h"
#include "Tests/unit-tests/TestMocks/MockCANReceiver.h"
#include "Tests/unit-tests/TestMocks/MessageLogger.h"

#define WAIT_FOR_MESSAGE		300


MessageBus msgBus;

void msgBusLoop() {
    msgBus.run();
}

int main() {
  Logger::DisableLogging();
    LowLevelControllerNodeJanet node(msgBus);
    MessageLogger logger(msgBus);

    std::thread t (msgBusLoop);
    t.detach();

    msgBus.sendMessage(std::make_unique<NavigationControlMsg>(1.2, 2.3, false, NavigationState::sailToWaypoint));
    msgBus.sendMessage(std::make_unique<StateMessage>(1,2,3,4,5));
    msgBus.sendMessage(std::make_unique<WindStateMsg>(6,7,8,9));
    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_MESSAGE));

    if(logger.actuatorPositionReceived() == true){
      std::cout << "ActuatorPosition Message Received" << std::endl;
    }


}
