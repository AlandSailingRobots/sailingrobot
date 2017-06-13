


#pragma once

#include "MessageBus/MessageBus.h"
#include "Messages/MessageTypes.h"
#include "Messages/ActuatorControlASPireMessage.h"
#include "Hardwares/CAN_Services/CANService.h"

class ActuatorNodeASPire : public Node {
public:
    ActuatorNodeASPire(MessageBus& msgBus, CANService& canService);
    ~ActuatorNodeASPire();
    bool init();
    void processMessage(const Message* message);

private:
    CANService* m_CANService;

};