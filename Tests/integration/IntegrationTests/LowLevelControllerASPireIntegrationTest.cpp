#include "Nodes/LowLevelControllerNodeASPire.h"
#include "MessageBus/MessageBus.h"
#include "HardwareServices/CAN_Services/CANService.h"

int main() {
    MessageBus msgBus;
    CANService canService;
    LowLevelControllerNodeASPire node(msgBus, canService);
}