#include "Nodes/LowLevelControllerNodeASPire.h"
#include "MessageBus/MessageBus.h"
#include "HardwareServices/CAN_Services/CANService.h"
#include "Tests/unit-tests/TestMocks/MockCANReceiver.h"

int main() {
    MessageBus msgBus;
    CANService canService;
    LowLevelControllerNodeASPire node(msgBus, canService);
    std::vector<uint32_t> IDs = { 700 };
    MockCANReceiver mock (canService, IDs);

    canService.start();
    canService.SetLoopBackMode();
    msgBus.run();

    NavigationControlMsg::NavigationState state =
                    NavigationControlMsg::NavigationState::sailToWaypoint;
    msgBus.sendMessage(std::make_unique<NavigationControlMsg>(1.2, 2.3, false, state));
    msgBus.sendMessage(std::make_unique<StateMessage>(1,2,3,4,5));
    msgBus.sendMessage(std::make_unique<WindStateMsg>(6,7,8,9));

    std::cout << mock.message_received() << std::endl;

}