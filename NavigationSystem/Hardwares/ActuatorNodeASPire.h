


#pragma once

#include "MessageBus/MessageBus.h"
#include "MessageBus/MessageTypes.h"
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
    const float MAX_RUDDER_ANGLE = 30;
	const float MAX_WINGSAIL_ANGLE = 13;
	const float INT16_SIZE = 65535;
	double rudderAngle;
	double wingsailAngle;
	bool WindvaneSelfSteeringOn;
	
};